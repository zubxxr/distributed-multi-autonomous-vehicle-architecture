# Multi-Vehicle AV Framework
*A Distributed Simulation Framework for Autonomous Vehicles*

![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-orange?logo=zenoh)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

The **Multi-Vehicle AV Framework** provides a complete setup for running **Autoware** and **AWSIM Labs** in a multi-host, multi-vehicle simulation environment.  

It enables simulation of multiple autonomous vehicles across different physical machines while maintaining synchronized perception, localization, planning, and control.

## Features
- Distributed simulation across multiple hosts
- Multi-vehicle Automated Valet Parking (AVP) coordination
- Zenoh-based ROS 2 topic synchronization
- Supports Ubuntu 22.04 (ROS 2 Humble) and Unity-based AWSIM Labs

---

## Getting Started

Before installing or running anything, start with the **[System Architecture](GettingStarted/SystemArchitecture/index.md)** to understand how the components fit together, what hardware is recommended, and the roles of each host in the distributed setup.

Once you’re familiar with the architecture, continue with the **[Software Installation](GettingStarted/SoftwareInstallation/index.md)** guide to set up Autoware Universe, AWSIM Labs, and Zenoh on your machines.