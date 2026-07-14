```from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from contextlib import suppress
from dataclasses import dataclass
from enum import Enum, auto
from typing import Iterable, Sequence

os.environ.setdefault("GPIOZERO_PIN_FACTORY", "lgpio")

from gpiozero import DigitalInputDevice, DigitalOutputDevice
from rplidar import RPLidar, RPLidarException

PORT = "/dev/ttyUSB0"
LEFT_FORWARD_PIN = 12
RIGHT_FORWARD_PIN = 19
IR_PIN = 26


class DriveState(Enum):
    STOP = auto()
    FORWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    U_TURN_LEFT = auto()
    U_TURN_RIGHT = auto()


@dataclass(frozen=True, slots=True)
class ScanView:
    visible_points: int
    front: tuple[float, ...]
    left: tuple[float, ...]
    right: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class Decision:
    state: DriveState
    reason: str
    view: ScanView
    ir_blocked: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--left-forward-pin", "--left-pin", "--left-in1", dest="left_forward_pin", type=int, default=LEFT_FORWARD_PIN)
    parser.add_argument("--right-forward-pin", "--right-pin", "--right-in3", dest="right_forward_pin", type=int, default=RIGHT_FORWARD_PIN)
    parser.add_argument("--left-low-pin", type=int, default=-1)
    parser.add_argument("--right-low-pin", type=int, default=-1)
    parser.add_argument("--ir-pin", type=int, default=IR_PIN)
    parser.add_argument("--angle-offset", type=float, default=0.0)
    parser.add_argument("--fov", type=float, default=180.0)
    parser.add_argument("--front-angle", type=float, default=25.0)
    parser.add_argument("--min-quality", type=int, default=5)
    parser.add_argument("--min-distance", "--min-dist", dest="min_distance", type=float, default=50.0)
    parser.add_argument("--max-distance", "--max-dist", dest="max_distance", type=float, default=8000.0)
    parser.add_argument("--wall-stop-distance", type=float, default=150.0)
    parser.add_argument("--obstacle-distance", "--obstacle-dist", dest="obstacle_distance", type=float, default=230.0)
    parser.add_argument("--clear-distance", type=float, default=320.0)
    parser.add_argument("--path-distance", type=float, default=700.0)
    parser.add_argument("--side-guard-distance", type=float, default=180.0)
    parser.add_argument("--minimum-hits", type=int, default=2)
    parser.add_argument("--minimum-visible-points", type=int, default=16)
    parser.add_argument("--minimum-sector-points", type=int, default=3)
    parser.add_argument("--decision-scans", type=int, default=2)
    parser.add_argument("--clear-scans", type=int, default=2)
    parser.add_argument("--resume-scans", type=int, default=2)
    parser.add_argument("--lost-path-scans", type=int, default=2)
    parser.add_argument("--minimum-turn-time", type=float, default=0.25)
    parser.add_argument("--maximum-turn-time", type=float, default=1.50)
    parser.add_argument("--u-turn-time", type=float, default=2.10)
    parser.add_argument("--maximum-u-turn-time", type=float, default=4.00)
    parser.add_argument("--turn-score-margin", type=float, default=60.0)
    parser.add_argument("--max-buffer", type=int, default=500)
    parser.add_argument("--min-scan-length", "--min-scan-len", dest="min_scan_length", type=int, default=5)
    parser.add_argument("--warmup", type=float, default=1.5)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--log-interval", type=float, default=1.0)
    return parser.parse_args()


def signed_angle(angle: float) -> float:
    return ((angle + 180.0) % 360.0) - 180.0


def analyze_scan(scan: Iterable[Sequence[float]], args: argparse.Namespace) -> ScanView:
    front: list[float] = []
    left: list[float] = []
    right: list[float] = []
    visible_points = 0
    half_fov = args.fov * 0.5

    for measurement in scan:
        if len(measurement) < 3:
            continue
        quality = measurement[0]
        raw_angle = measurement[1]
        distance = measurement[2]
        if quality < args.min_quality:
            continue
        if distance < args.min_distance or distance > args.max_distance:
            continue
        angle = signed_angle(raw_angle + args.angle_offset)
        if abs(angle) > half_fov:
            continue
        visible_points += 1
        if abs(angle) <= args.front_angle:
            front.append(float(distance))
        elif angle < 0.0:
            left.append(float(distance))
        else:
            right.append(float(distance))

    return ScanView(visible_points, tuple(front), tuple(left), tuple(right))


def has_hits(readings: Sequence[float], threshold: float, required: int) -> bool:
    hits = 0
    for distance in readings:
        if distance <= threshold:
            hits += 1
            if hits >= required:
                return True
    return False


def count_at_least(readings: Sequence[float], threshold: float) -> int:
    return sum(1 for distance in readings if distance >= threshold)


def count_at_most(readings: Sequence[float], threshold: float) -> int:
    return sum(1 for distance in readings if distance <= threshold)


def percentile(readings: Sequence[float], fraction: float, fallback: float) -> float:
    if not readings:
        return fallback
    ordered = sorted(readings)
    index = min(len(ordered) - 1, max(0, int((len(ordered) - 1) * fraction)))
    return ordered[index]


def clearance(readings: Sequence[float], fallback: float) -> float:
    return percentile(readings, 0.25, fallback)


class MotionController:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.state = DriveState.STOP
        self.reason = "startup"
        self.turn_started = 0.0
        self.clear_streak = 0
        self.lost_path_streak = 0
        self.pending_turn: DriveState | None = None
        self.pending_streak = 0
        self.last_turn = DriveState.TURN_RIGHT
        self.turn_locked = False

    def known(self, readings: Sequence[float]) -> bool:
        return len(readings) >= self.args.minimum_sector_points

    def path_score(self, readings: Sequence[float]) -> float:
        if not self.known(readings):
            return -math.inf
        far_points = count_at_least(readings, self.args.path_distance)
        close_points = count_at_most(readings, self.args.side_guard_distance)
        low = clearance(readings, 0.0)
        median = percentile(readings, 0.50, 0.0)
        high = percentile(readings, 0.75, 0.0)
        return high + (median * 0.50) + (low * 0.25) + (far_points * 35.0) - (close_points * 120.0)

    def open_path(self, readings: Sequence[float]) -> bool:
        return (
            self.known(readings)
            and count_at_least(readings, self.args.path_distance) >= self.args.minimum_hits
            and count_at_most(readings, self.args.side_guard_distance) < self.args.minimum_hits
        )

    def front_clear(self, view: ScanView) -> bool:
        return self.known(view.front) and not has_hits(view.front, self.args.clear_distance, self.args.minimum_hits)

    def front_blocked(self, view: ScanView) -> bool:
        return self.known(view.front) and has_hits(view.front, self.args.obstacle_distance, self.args.minimum_hits)

    def front_wall(self, view: ScanView) -> bool:
        return self.known(view.front) and has_hits(view.front, self.args.wall_stop_distance, self.args.minimum_hits)

    def choose_turn(self, view: ScanView) -> DriveState | None:
        left_open = self.open_path(view.left)
        right_open = self.open_path(view.right)
        if not left_open and not right_open:
            return None
        if left_open and not right_open:
            return DriveState.TURN_LEFT
        if right_open and not left_open:
            return DriveState.TURN_RIGHT
        left_score = self.path_score(view.left)
        right_score = self.path_score(view.right)
        if left_score > right_score + self.args.turn_score_margin:
            return DriveState.TURN_LEFT
        if right_score > left_score + self.args.turn_score_margin:
            return DriveState.TURN_RIGHT
        return DriveState.TURN_LEFT if self.last_turn == DriveState.TURN_RIGHT else DriveState.TURN_RIGHT

    def choose_u_turn(self, view: ScanView) -> DriveState:
        left_score = self.path_score(view.left)
        right_score = self.path_score(view.right)
        if left_score > right_score + self.args.turn_score_margin:
            return DriveState.U_TURN_LEFT
        if right_score > left_score + self.args.turn_score_margin:
            return DriveState.U_TURN_RIGHT
        return DriveState.U_TURN_LEFT if self.last_turn == DriveState.TURN_RIGHT else DriveState.U_TURN_RIGHT

    def stop(self, reason: str, lock_turning: bool = False) -> None:
        self.state = DriveState.STOP
        self.reason = reason
        self.turn_started = 0.0
        self.clear_streak = 0
        self.lost_path_streak = 0
        self.pending_turn = None
        self.pending_streak = 0
        if lock_turning:
            self.turn_locked = True

    def forward(self, reason: str) -> None:
        self.state = DriveState.FORWARD
        self.reason = reason
        self.turn_started = 0.0
        self.clear_streak = 0
        self.lost_path_streak = 0
        self.pending_turn = None
        self.pending_streak = 0
        self.turn_locked = False

    def turn(self, direction: DriveState, now: float, reason: str) -> None:
        self.state = direction
        self.reason = reason
        self.turn_started = now
        self.clear_streak = 0
        self.lost_path_streak = 0
        self.pending_turn = None
        self.pending_streak = 0
        if direction in (DriveState.TURN_LEFT, DriveState.TURN_RIGHT):
            self.last_turn = direction

    def u_turn(self, view: ScanView, now: float, reason: str) -> None:
        direction = self.choose_u_turn(view)
        self.state = direction
        self.reason = reason
        self.turn_started = now
        self.clear_streak = 0
        self.lost_path_streak = 0
        self.pending_turn = None
        self.pending_streak = 0
        self.last_turn = DriveState.TURN_LEFT if direction == DriveState.U_TURN_LEFT else DriveState.TURN_RIGHT

    def confirm_turn(self, direction: DriveState | None, now: float, reason: str) -> None:
        if direction is None:
            self.pending_turn = None
            self.pending_streak = 0
            self.reason = "no visible path"
            return
        if direction == self.pending_turn:
            self.pending_streak += 1
        else:
            self.pending_turn = direction
            self.pending_streak = 1
        self.reason = reason
        if self.pending_streak >= self.args.decision_scans:
            self.turn(direction, now, reason)

    def decide(self, view: ScanView, ir_blocked: bool, now: float) -> Decision:
        if ir_blocked:
            self.stop("IR obstacle")
            return Decision(self.state, self.reason, view, True)

        if view.visible_points < self.args.minimum_visible_points or not self.known(view.front):
            self.stop(f"insufficient {self.args.fov:.0f}-degree scan")
            return Decision(self.state, self.reason, view, False)

        front_wall = self.front_wall(view)
        front_blocked = self.front_blocked(view)
        front_clear = self.front_clear(view)
        left_open = self.open_path(view.left)
        right_open = self.open_path(view.right)
        left_guard = self.known(view.left) and has_hits(view.left, self.args.side_guard_distance, self.args.minimum_hits)
        right_guard = self.known(view.right) and has_hits(view.right, self.args.side_guard_distance, self.args.minimum_hits)

        if self.state in (DriveState.U_TURN_LEFT, DriveState.U_TURN_RIGHT):
            elapsed = now - self.turn_started
            if elapsed >= self.args.u_turn_time and front_clear:
                self.clear_streak += 1
                if self.clear_streak >= self.args.clear_scans:
                    self.forward("u-turn complete")
            else:
                self.clear_streak = 0
            if elapsed >= self.args.maximum_u_turn_time and self.state in (DriveState.U_TURN_LEFT, DriveState.U_TURN_RIGHT):
                if front_clear:
                    self.forward("u-turn complete")
                else:
                    direction = self.choose_turn(view)
                    if direction is None:
                        self.stop("u-turn timeout", True)
                    else:
                        self.turn(direction, now, "u-turn found path")
            return Decision(self.state, self.reason, view, False)

        if self.state in (DriveState.TURN_LEFT, DriveState.TURN_RIGHT):
            elapsed = now - self.turn_started
            target_open = left_open if self.state == DriveState.TURN_LEFT else right_open
            if target_open:
                self.lost_path_streak = 0
            else:
                self.lost_path_streak += 1
            if self.lost_path_streak >= self.args.lost_path_scans:
                if front_wall:
                    self.u_turn(view, now, "turn path lost near wall")
                else:
                    self.stop("turn path lost", True)
                return Decision(self.state, self.reason, view, False)
            if elapsed >= self.args.maximum_turn_time:
                self.u_turn(view, now, "turn timeout; reversing course")
                return Decision(self.state, self.reason, view, False)
            if elapsed >= self.args.minimum_turn_time and front_clear:
                self.clear_streak += 1
                if self.clear_streak >= self.args.clear_scans:
                    self.forward("turn complete")
            else:
                self.clear_streak = 0
            return Decision(self.state, self.reason, view, False)

        if self.state == DriveState.FORWARD:
            if front_wall:
                direction = self.choose_turn(view)
                if direction is None:
                    self.u_turn(view, now, "wall at 15cm; reversing course")
                else:
                    self.turn(direction, now, "wall close; taking best path")
                return Decision(self.state, self.reason, view, False)
            if front_blocked:
                direction = self.choose_turn(view)
                if direction is None:
                    self.reason = "approaching obstacle to find path"
                else:
                    self.turn(direction, now, "front obstacle; best path")
                return Decision(self.state, self.reason, view, False)
            if left_guard and right_guard:
                self.stop("corridor too narrow")
                return Decision(self.state, self.reason, view, False)
            if left_guard:
                if right_open:
                    self.turn(DriveState.TURN_RIGHT, now, "left side too close")
                else:
                    self.stop("left blocked and no right path")
                return Decision(self.state, self.reason, view, False)
            if right_guard:
                if left_open:
                    self.turn(DriveState.TURN_LEFT, now, "right side too close")
                else:
                    self.stop("right blocked and no left path")
                return Decision(self.state, self.reason, view, False)
            self.reason = "path clear"
            return Decision(self.state, self.reason, view, False)

        if front_clear:
            self.clear_streak += 1
            self.pending_turn = None
            self.pending_streak = 0
            self.reason = "confirming straight path"
            if self.clear_streak >= self.args.resume_scans:
                self.forward("straight path clear")
            return Decision(self.state, self.reason, view, False)

        self.clear_streak = 0
        if front_wall:
            direction = self.choose_turn(view)
            if direction is None:
                self.u_turn(view, now, "wall at 15cm; reversing course")
            else:
                self.confirm_turn(direction, now, "confirming side path")
            return Decision(self.state, self.reason, view, False)
        if front_blocked and not self.turn_locked:
            self.forward("approaching obstacle to find path")
            return Decision(self.state, self.reason, view, False)
        if self.turn_locked:
            self.pending_turn = None
            self.pending_streak = 0
            self.reason = "stopped until straight path is clear"
            return Decision(self.state, self.reason, view, False)
        self.forward("edging closer to scan")
        return Decision(self.state, self.reason, view, False)


class Motors:
    def __init__(self, left_forward_pin: int, right_forward_pin: int, left_low_pin: int, right_low_pin: int, interlock: threading.Event):
        self.interlock = interlock
        self.lock = threading.Lock()
        self.left_forward = DigitalOutputDevice(left_forward_pin, initial_value=False)
        self.right_forward = DigitalOutputDevice(right_forward_pin, initial_value=False)
        self.left_low = DigitalOutputDevice(left_low_pin, initial_value=False) if left_low_pin >= 0 else None
        self.right_low = DigitalOutputDevice(right_low_pin, initial_value=False) if right_low_pin >= 0 else None
        self.state: DriveState | None = None
        self.stop()

    def set_wheel(self, forward_pin: DigitalOutputDevice, reverse_pin: DigitalOutputDevice | None, direction: int) -> None:
        if direction > 0:
            forward_pin.on()
            if reverse_pin is not None:
                reverse_pin.off()
        elif direction < 0 and reverse_pin is not None:
            forward_pin.off()
            reverse_pin.on()
        else:
            forward_pin.off()
            if reverse_pin is not None:
                reverse_pin.off()

    def set_outputs(self, left: int, right: int) -> None:
        self.set_wheel(self.left_forward, self.left_low, left)
        self.set_wheel(self.right_forward, self.right_low, right)

    def apply(self, state: DriveState) -> None:
        with self.lock:
            if self.interlock.is_set():
                state = DriveState.STOP
            if state == self.state:
                return
            if state == DriveState.FORWARD:
                self.set_outputs(1, 1)
            elif state == DriveState.TURN_LEFT:
                self.set_outputs(0, 1)
            elif state == DriveState.TURN_RIGHT:
                self.set_outputs(1, 0)
            elif state == DriveState.U_TURN_LEFT:
                self.set_outputs(-1, 1)
            elif state == DriveState.U_TURN_RIGHT:
                self.set_outputs(1, -1)
            else:
                self.set_outputs(0, 0)
            self.state = state

    def stop(self) -> None:
        with self.lock:
            self.set_outputs(0, 0)
            self.state = DriveState.STOP

    def close(self) -> None:
        self.stop()
        with suppress(Exception):
            self.left_forward.close()
        with suppress(Exception):
            self.right_forward.close()
        if self.left_low is not None:
            with suppress(Exception):
                self.left_low.close()
        if self.right_low is not None:
            with suppress(Exception):
                self.right_low.close()


def close_lidar(lidar: RPLidar | None) -> None:
    if lidar is None:
        return
    with suppress(Exception):
        lidar.stop()
    with suppress(Exception):
        lidar.stop_motor()
    with suppress(Exception):
        lidar.disconnect()


def connect_lidar(args: argparse.Namespace) -> RPLidar:
    lidar = RPLidar(args.port, timeout=args.timeout)
    try:
        with suppress(Exception):
            lidar.clear_input()
        health = lidar.get_health()
        if not isinstance(health, tuple) or len(health) < 2:
            with suppress(Exception):
                lidar.clear_input()
            health = lidar.get_health()
        if not isinstance(health, tuple) or len(health) < 2:
            raise RPLidarException(f"invalid health response: {health!r}")
        if str(health[0]).lower() == "error":
            raise RPLidarException(f"health error: {health!r}")
        lidar.start_motor()
        time.sleep(args.warmup)
        return lidar
    except Exception:
        close_lidar(lidar)
        raise


def format_clearance(readings: Sequence[float]) -> str:
    value = clearance(readings, math.inf)
    return "unknown" if math.isinf(value) else f"{value:.0f}mm"


def status_line(decision: Decision) -> str:
    view = decision.view
    return (
        f"{decision.state.name:10s} {decision.reason:30s} "
        f"front={format_clearance(view.front):>8s} "
        f"left={format_clearance(view.left):>8s} "
        f"right={format_clearance(view.right):>8s} "
        f"IR={'BLOCKED' if decision.ir_blocked else 'clear':7s} "
        f"points={view.visible_points}"
    )


def validate(args: argparse.Namespace) -> str | None:
    if not 0.0 < args.fov <= 180.0:
        return "fov must be between 0 and 180 degrees"
    if not 0.0 < args.front_angle < args.fov * 0.5:
        return "front-angle must be smaller than half the fov"
    if args.min_distance < 0.0 or args.max_distance <= args.min_distance:
        return "invalid distance range"
    if args.wall_stop_distance <= args.min_distance:
        return "wall-stop-distance must exceed min-distance"
    if args.obstacle_distance <= args.wall_stop_distance:
        return "obstacle-distance must exceed wall-stop-distance"
    if args.clear_distance <= args.obstacle_distance:
        return "clear-distance must exceed obstacle-distance"
    if args.path_distance <= args.obstacle_distance:
        return "path-distance must exceed obstacle-distance"
    if args.minimum_hits < 1 or args.minimum_visible_points < 1 or args.minimum_sector_points < 1:
        return "point and hit limits must be positive"
    if args.decision_scans < 1 or args.clear_scans < 1 or args.resume_scans < 1 or args.lost_path_scans < 1:
        return "scan confirmation values must be positive"
    if args.minimum_turn_time <= 0.0 or args.maximum_turn_time <= args.minimum_turn_time:
        return "maximum-turn-time must exceed minimum-turn-time"
    if args.u_turn_time <= 0.0 or args.maximum_u_turn_time <= args.u_turn_time:
        return "maximum-u-turn-time must exceed u-turn-time"
    if args.warmup < 0.0 or args.timeout <= 0.0 or args.reconnect_delay < 0.0:
        return "invalid timing value"
    return None


def main() -> int:
    args = parse_args()
    error = validate(args)
    if error is not None:
        print(error, file=sys.stderr)
        return 2

    interlock = threading.Event()
    motors: Motors | None = None
    ir: DigitalInputDevice | None = None
    lidar: RPLidar | None = None

    try:
        motors = Motors(args.left_forward_pin, args.right_forward_pin, args.left_low_pin, args.right_low_pin, interlock)
        ir = DigitalInputDevice(args.ir_pin, pull_up=True, bounce_time=0.01)

        def ir_on() -> None:
            interlock.set()
            if motors is not None:
                motors.stop()

        def ir_off() -> None:
            interlock.clear()

        ir.when_activated = ir_on
        ir.when_deactivated = ir_off
        if ir.is_active:
            ir_on()
        else:
            ir_off()

        controller = MotionController(args)
        previous_state: DriveState | None = None
        previous_reason = ""
        next_log = 0.0

        print(
            f"L298N forward-only controller ready; FOV={args.fov:.0f} degrees; "
            f"left GPIO{args.left_forward_pin}; right GPIO{args.right_forward_pin}; IR GPIO{args.ir_pin}",
            flush=True,
        )

        while True:
            try:
                lidar = connect_lidar(args)
                print("LIDAR connected", flush=True)

                for scan in lidar.iter_scans(max_buf_meas=args.max_buffer, min_len=args.min_scan_length):
                    now = time.monotonic()
                    view = analyze_scan(scan, args)
                    ir_blocked = interlock.is_set() or ir.is_active
                    decision = controller.decide(view, ir_blocked, now)
                    motors.apply(decision.state)

                    changed = decision.state != previous_state or decision.reason != previous_reason
                    if changed or now >= next_log:
                        print(status_line(decision), flush=True)
                        previous_state = decision.state
                        previous_reason = decision.reason
                        next_log = now + max(0.1, args.log_interval)

                motors.stop()
                close_lidar(lidar)
                lidar = None

            except KeyboardInterrupt:
                break
            except (RPLidarException, OSError, ValueError) as exc:
                motors.stop()
                close_lidar(lidar)
                lidar = None
                print(f"LIDAR error: {exc}; reconnecting", file=sys.stderr, flush=True)
                time.sleep(max(0.1, args.reconnect_delay))

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Fatal error: {exc}", file=sys.stderr)
        return 1
    finally:
        close_lidar(lidar)
        if ir is not None:
            with suppress(Exception):
                ir.when_activated = None
            with suppress(Exception):
                ir.when_deactivated = None
            with suppress(Exception):
                ir.close()
        if motors is not None:
            motors.close()
        print("Motors stopped", flush=True)

    return 0


if __name__ == "__main__":```
    raise SystemExit(main())
