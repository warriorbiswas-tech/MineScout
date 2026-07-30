# 🤖 MineScout

> **An Autonomous LiDAR-Based Rover for Intelligent Terrain Exploration & Hazard Detection**
>
> **"Explore Beyond. Detect Before. Navigate Autonomously."**

![Version](https://img.shields.io/badge/version-2.0-blue)
![Platform](https://img.shields.io/badge/platform-Raspberry%20Pi%205-red)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E)
![Python](https://img.shields.io/badge/Python-3.11-yellow)
![License](https://img.shields.io/badge/license-MIT-green)

---

# 🚀 Overview

**MineScout** is an autonomous ground rover designed for intelligent terrain exploration, obstacle avoidance, and environmental mapping using **LiDAR-based perception**. The project focuses on creating a low-cost, scalable robotic platform capable of navigating unknown environments without human intervention.

Built around the **Raspberry Pi 5** and **RPLIDAR A1**, MineScout generates a real-time 2D map of its surroundings, detects obstacles, plans safe paths, and autonomously traverses challenging environments.

Originally developed as a student research project, MineScout demonstrates the integration of robotics, computer vision, embedded systems, and autonomous navigation into a compact exploration platform.

---

# ✨ Features

- 🛰 Autonomous Navigation
- 📡 360° LiDAR Mapping
- 🚧 Real-Time Obstacle Detection & Avoidance
- 🗺 Live Environment Mapping
- 🤖 Autonomous Decision Making
- ⚡ Raspberry Pi Powered
- 🔄 Differential Drive Motion
- 📈 Real-Time Sensor Visualization
- 🔋 Modular & Expandable Architecture
- 🧩 ROS 2 Compatible

---

# 🎯 Applications

- Educational Robotics
- Indoor Autonomous Navigation
- Smart Warehouse Automation
- Research in SLAM
- Disaster Response Robotics
- Infrastructure Inspection
- Industrial Monitoring
- Search & Reconnaissance
- Autonomous Exploration

---

# 🛠 Hardware

| Component | Purpose |
|------------|----------|
| Raspberry Pi 5 | Main Computer |
| RPLIDAR A1 | 360° LiDAR Scanner |
| L298N Motor Driver | Motor Control |
| DC Geared Motors | Locomotion |
| Robot Chassis | Mobile Platform |
| Caster Wheel | Stability |
| Li-ion Battery Pack | Power Supply |
| Buck Converter | Voltage Regulation |

---

# 💻 Software Stack

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Python 3
- OpenCV
- NumPy
- RPLIDAR SDK
- RViz2
- Linux

---

# 📂 Repository Structure

```
MineScout/
│
├── README.md
├── src/
│   ├── navigation/
│   ├── mapping/
│   ├── motor_control/
│   └── lidar/
│
├── launch/
│
├── config/
│
├── images/
│   ├── rover.jpg
│   ├── rviz.png
│   ├── mapping.png
│   └── prototype.jpg
│
├── docs/
│   ├── Presentation.pdf
│   ├── Report.pdf
│   └── Circuit.pdf
│
└── LICENSE
```

---

# 🧠 System Architecture

```
              +----------------+
              |  RPLIDAR A1    |
              +----------------+
                      |
                      |
             Laser Scan Data
                      |
                      ▼
             +----------------+
             | Raspberry Pi 5 |
             +----------------+
             | ROS2 Nodes     |
             | Navigation     |
             | Mapping        |
             | Obstacle Avoid |
             +----------------+
                      |
              Motor Commands
                      |
                      ▼
              +----------------+
              | L298N Driver   |
              +----------------+
                      |
          +-----------+-----------+
          |                       |
      Left Motor             Right Motor
```

---

# ⚙ Working Principle

1. The LiDAR continuously scans the surrounding environment.
2. Distance measurements are converted into a 2D point cloud.
3. Mapping and localization algorithms analyze the environment.
4. Obstacles are identified in real time.
5. The navigation controller computes a collision-free path.
6. Motor commands are generated and sent to the rover.
7. The rover autonomously explores while continuously updating its map.

---

# 📊 Capabilities

- Real-time LiDAR scanning
- Autonomous obstacle avoidance
- 2D environmental mapping
- Differential drive control
- Live visualization using RViz
- Modular software architecture
- Expandable sensor support

---

# 📈 Technologies Used

- ROS 2 Jazzy
- Raspberry Pi 5
- Python
- Linux
- OpenCV
- NumPy
- RPLIDAR SDK
- RViz2

---

# 🖼 Images

Add your project images here.

```
images/rover.jpg
images/prototype.jpg
images/mapping.png
images/rviz.png
```

---

# 🔄 Workflow

```
LiDAR Scan
      │
      ▼
Point Cloud Generation
      │
      ▼
Obstacle Detection
      │
      ▼
Mapping & Localization
      │
      ▼
Path Planning
      │
      ▼
Motor Controller
      │
      ▼
Autonomous Navigation
```

---

# 📌 Future Roadmap

- 📷 Camera-Based Object Detection
- 🛰 GPS Waypoint Navigation
- 🧭 IMU Sensor Fusion
- 🗺 3D SLAM
- 🤖 AI-Based Terrain Classification
- ☁ Remote Monitoring Dashboard
- 📡 Wireless Teleoperation
- 🔋 Autonomous Charging Dock
- 🌐 Multi-Robot Coordination
- 🎮 Mobile Control Application

---

# 📊 Performance Goals

| Metric | Target |
|---------|--------|
| Navigation | Autonomous |
| Mapping | Real-Time |
| Obstacle Detection | 360° |
| Operating System | Ubuntu + ROS 2 |
| Control Frequency | 20–30 Hz |
| Mapping Accuracy | High Indoor Precision |

---

# 🔬 Future Research

MineScout is envisioned as a foundation for future autonomous robotic systems capable of operating in complex environments. Planned research areas include:

- AI-assisted navigation
- Multi-sensor fusion
- Semantic mapping
- Edge AI deployment
- Human-robot collaboration
- Autonomous exploration algorithms

---

# 📚 References

The project draws inspiration from modern autonomous rover architectures, LiDAR-based mapping systems, and open-source robotics frameworks such as ROS 2. :contentReference[oaicite:0]{index=0}

---

# 📄 License

Licensed under the **MIT License**.

---

# 👨‍💻 Author

**Arpan Biswas**

Student Researcher • Robotics Enthusiast • Embedded Systems Developer

Passionate about autonomous robotics, artificial intelligence, computer vision, and next-generation intelligent machines.

---

# 🙏 Acknowledgements

Special thanks to:

- Raspberry Pi Foundation
- ROS Community
- Slamtec (RPLIDAR)
- OpenCV Community
- Open Source Robotics Foundation
- Teachers and mentors who supported the project

---

# ⭐ Support

If you found this project interesting, consider giving it a ⭐ on GitHub.

*"The future belongs to machines that can perceive, reason, and explore autonomously."*
