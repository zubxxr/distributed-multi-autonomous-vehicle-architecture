# DMAVA: Distributed Multi-Autonomous Vehicle Architecture
*A distributed simulation architecture for autonomous vehicles integrating Autoware Universe, AWSIM Labs, and Zenoh for synchronized multi-vehicle operation.*

![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.7.2-orange?logo=zenoh)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

The **DMAVA** provides a complete setup for running **Autoware** and **AWSIM Labs** in a multi-host, multi-vehicle simulation environment for **autonomous vehicles**.  

It enables simulation of multiple autonomous vehicles across different physical machines while maintaining synchronized perception, localization, planning, and control pipelines.

## Features

- Distributed simulation across multiple hosts
- Zenoh-based ROS 2 topic synchronization
- Multi-vehicle coordination with namespaced topics for collision avoidance
- Modular design enabling integration of new perception, planning, or control components
- Scalable to larger fleets and more complex simulation scenarios

---

## Related Publication
This repository is part of the following paper:

**DMAVA: Distributed Multi-Autonomous Vehicle Architecture Using Autoware**  
Zubair Islam, Mohamed El-Darieby  
*Accepted at IEEE Intelligent Vehicles Symposium (IV) 2026, awaiting publication*  

Preprint: https://arxiv.org/abs/2601.16336

---


## Demo Videos

A playlist of recorded demonstrations showcasing multi-host, multi-vehicle execution using DMAVA is available below:

**DMAVA Demo Playlist**  
https://youtube.com/playlist?list=PL4MADLjXmDi2MWOM2Ulagf3Bk2z1ABV28

The videos include distributed execution across multiple hosts, synchronized vehicle behavior, and end-to-end system operation under different configurations.

---


## Getting Started and Documentation

[https://zubxxr.github.io/distributed-multi-autonomous-vehicle-architecture/](https://zubxxr.github.io/distributed-multi-autonomous-vehicle-architecture/)

---

## License
This project is licensed under the Apache License 2.0.

## Contact
For questions, suggestions, or collaboration opportunities, feel free to reach out:

- **Author:** Zubair Islam  
- **Email:** zubxxr@gmail.com
- **LinkedIn:** [linkedin.com/in/zubairislam02](https://www.linkedin.com/in/zubairislam02/)
