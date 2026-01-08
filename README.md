# Fastbot ROS2 Docker

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Docker Hub](https://img.shields.io/badge/Docker%20Hub-legalaspro%2Ffastbot-2496ED?logo=docker)](https://hub.docker.com/r/legalaspro/fastbot)
[![Nav2](https://img.shields.io/badge/Nav2-Navigation-green)](https://navigation.ros.org/)
[![Cartographer](https://img.shields.io/badge/Cartographer-SLAM-orange)](https://google-cartographer-ros.readthedocs.io/)

Complete ROS2 development environment for the Fastbot differential drive robot. Develop in simulation, deploy to real hardware — all containerized with Docker and VS Code devcontainers.

<table align="center" width="100%">
  <tr>
    <td align="center" width="50%">
      <img src="docs/images/fastbot_real.jpeg" alt="Fastbot Real Robot" width="100%">
      <br><strong>Real Robot</strong>
    </td>
    <td align="center" width="50%">
      <img src="docs/images/fastbot_sim.png" alt="Fastbot Simulation" width="100%">
      <br><strong>Gazebo Simulation</strong>
    </td>
  </tr>
</table>

---

## Quick Start

### Option A: Simulation (No Hardware)

```bash
# 1. Clone the repo
git clone https://github.com/legalaspro/fastbot-ros2-docker.git
cd fastbot-ros2-docker

# 2. Open in VS Code → "Reopen in Container" → "Fastbot ROS2 Simulation"

# 3. In devcontainer terminal:
ros2 launch fastbot_gazebo one_fastbot_warehouse.launch.py

# 4. Open http://localhost:6080 for noVNC desktop with RViz/Gazebo
```

### Option B: Real Robot

```bash
# On Raspberry Pi:
cd ~/fastbot/docker/real
docker compose up -d robot

# On Dev Machine: Open VS Code → "Reopen in Container" → "Fastbot Real Robot (Remote)"
# Requires VPN setup - see docker/real/README-REMOTE.md
```

---

## Features

| Feature           | Simulation               | Real Robot                |
| ----------------- | ------------------------ | ------------------------- |
| **Gazebo World**  | ✅ Warehouse environment | —                         |
| **SLAM**          | ✅ Cartographer          | ✅ Cartographer           |
| **Navigation**    | ✅ Nav2                  | ✅ Nav2                   |
| **Web Interface** | ✅ ROSBridge + WebApp    | ✅ ROSBridge + WebApp     |
| **RViz**          | ✅ via noVNC             | ✅ via noVNC              |
| **Camera**        | ✅ Simulated             | ✅ Raspberry Pi Camera    |
| **LiDAR**         | ✅ Simulated             | ✅ Leishen N10            |
| **Remote Access** | —                        | ✅ Tailscale/Husarnet VPN |

---

## Repository Structure

```
fastbot-ros2-docker/
├── .devcontainer/           # VS Code devcontainer configs
│   ├── dev/                 # Simple development
│   ├── simulation/          # Gazebo simulation
│   ├── real/                # Real robot (Tailscale)
│   └── real-husarnet/       # Real robot (Husarnet)
│
├── docker/                  # Docker files
│   ├── real/                # Pi + remote containers
│   └── simulation/          # Gazebo containers
│
├── fastbot_description/     # URDF, meshes, robot model
├── fastbot_bringup/         # Real robot launch files
├── fastbot_gazebo/          # Simulation worlds & launch
├── fastbot_slam/            # Cartographer & Nav2 configs
├── fastbot_webapp/          # Web UI (roslibjs, ros3d)
├── fastbot_gripper/         # Gripper control package
│
├── serial_motor/            # Motor driver (Arduino bridge)
├── serial_motor_msgs/       # Motor message definitions
├── Lslidar_ROS2_driver/     # LiDAR driver
├── tf2_web_republisher_py/  # TF for web clients
│
├── docs/                    # Documentation
└── Makefile                 # Build shortcuts
```

---

## Devcontainer Options

| Devcontainer                      | Purpose                     | Use Case                       |
| --------------------------------- | --------------------------- | ------------------------------ |
| **Fastbot Dev (Simple)**          | Basic ROS2 development      | Package development, testing   |
| **Fastbot ROS2 Simulation**       | Full Gazebo simulation      | SLAM, Nav2 testing, WebApp dev |
| **Fastbot Real Robot (Remote)**   | Connect to Pi via Tailscale | Control real robot remotely    |
| **Fastbot Real Robot (Husarnet)** | Connect to Pi via Husarnet  | Self-hosted VPN option         |

### Ports

| Port  | Service                      |
| ----- | ---------------------------- |
| 6080  | noVNC Desktop (RViz, Gazebo) |
| 8000  | WebApp                       |
| 9090  | ROSBridge WebSocket          |
| 11315 | Web Video Server             |

---

## Documentation

| Guide                                                | Description                    |
| ---------------------------------------------------- | ------------------------------ |
| [Simulation README](docker/simulation/README.md)     | Gazebo setup and usage         |
| [Real Robot README](docker/real/README.md)           | Pi setup, device mapping       |
| [Remote Access README](docker/real/README-REMOTE.md) | VPN setup (Tailscale/Husarnet) |
| [SLAM README](fastbot_slam/README.md)                | Mapping and navigation         |

---

## Hardware

The Fastbot is a differential drive robot with:

- **Raspberry Pi 4/5** — Main computer
- **Arduino Nano** — Motor controller (ros_arduino_bridge)
- **Leishen N10 LiDAR** — 360° laser scanner
- **Raspberry Pi Camera** — Vision
- **DC Motors with Encoders** — Differential drive

See [fastbot_ros2 docs](https://github.com/legalaspro/fastbot_ros2/tree/main/docs) for build guides and 3D printing files.

---

## Credits

This project is inspired by [The Construct's FastBot](https://www.theconstructsim.com/) — an open-source ROS 2 robot kit for hands-on robotics learning.

---

## License

MIT License — Copyright (c) 2025 Dmitri Manajev
