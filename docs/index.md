# Multi-Vehicle AV Framework
*A Distributed Simulation Framework for Autonomous Vehicles*

![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-orange?logo=zenoh)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

This repository provides a setup for running **Autoware** and **AWSIM Labs** in a multi-host, multi-vehicle simulation environment. It includes installation and configuration instructions for:
- Autoware Universe
- AWSIM Labs (Unity-based simulation)
- Zenoh middleware for distributed ROS 2 topic synchronization

The framework allows simulation of multiple autonomous vehicles across different physical machines while maintaining synchronized perception, localization, planning, and control.

## Docs Map
- **Framework**
  - [System Architecture](framework/system-architecture.md)
  - [Software Installation](framework/software-installation.md)
  - [Multi-Vehicle Simulation](framework/multi-vehicle-simulation.md)
- **AVP (Automated Valet Parking)**
  - [Overview](../avp/overview.md)
  - [Workspace & Launch](../avp/workspace-launch.md)
  - [Scenarios](../avp/scenarios.md)
  - [Troubleshooting](../avp/troubleshooting.md)
- **Extensions**
  - [Future Work & Extensions](extensions/future-work.md)
- **About**
  - [License](about/license.md)
  - [Acknowledgements](about/acknowledgements.md)
