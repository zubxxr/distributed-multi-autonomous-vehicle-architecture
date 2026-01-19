# DMAVA: Distributed Multi-Autonomous Vehicle Architecture
*A distributed simulation architecture for autonomous vehicles integrating Autoware Universe, AWSIM Labs, and Zenoh for synchronized multi-vehicle operation.*

![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-orange?logo=zenoh)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

The **DMAVA** provides a complete setup for running **Autoware** and **AWSIM Labs** in a multi-host, multi-vehicle simulation environment.  

It enables simulation of multiple autonomous vehicles across different physical machines while maintaining synchronized perception, localization, planning, and control.

## Features

- Distributed simulation across multiple hosts
- Zenoh-based ROS 2 topic synchronization
- Multi-vehicle coordination with namespaced topics for collision avoidance
- Modular design enabling integration of new perception, planning, or control components
- Scalable to larger fleets and more complex simulation scenarios

---

## Extensions

This framework has already been extended to support **[Automated Valet Parking (AVP)](Extensions/Multi-VehicleAVP/index.md)** as a working implementation.  

In addition, several other ideas are explored as possible directions for future **[Multi-Agent Implementations](Extensions/FutureWork/index.md)**. While not yet validated, these highlight the broader potential of the framework for distributed autonomous driving research.  


## Getting Started

To get started with running the framework, see the [System Architecture](GettingStarted/SystemArchitecture/index.md) page.

For a condensed list of frequently used commands, see the [Developer Quick Commands](DeveloperGuide/QuickCommands/index.md) page.

## Troubleshooting 

Refer to [Issues](https://github.com/zubxxr/distributed-multi-autonomous-vehicle-architecture/issues) to see if the issue has been addressed. Otherwise, feel free to open one.
