# Multi-Vehicle Autonomous Valet Parking
*Distributed Autonomous Valet Parking Across Multiple Hosts*

![Autoware](https://img.shields.io/badge/Autoware-2024.11-blue?logo=autoware)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green?logo=unity)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple?logo=ros)
![Zenoh](https://img.shields.io/badge/Zenoh-1.4.0-orange?logo=zenoh)
![YOLOv5](https://img.shields.io/badge/YOLO-v5-red?logo=github)
![AVP Node](https://img.shields.io/badge/AVP_Node-custom-black?logo=robotframework)
![License](https://img.shields.io/badge/License-Apache%202.0-blue?logo=apache)

The **Multi-Vehicle AVP** system extends the **Multi-Vehicle Framework** to support **multiple vehicles operating in a shared simulation environment across different physical hosts**.  

This setup leverages **Zenoh** for ROS 2 topic synchronization, enabling each vehicle to operate its own Autoware stack while coordinating parking decisions in real-time.

---

## Features

- Multi-host, multi-vehicle Automated Valet Parking (AVP) simulation
- Zenoh-based distributed ROS 2 topic bridging for synchronized operation
- Parking spot detection and reservation mechanisms
- Namespace-aware orchestration for individual Autoware stacks
- Custom RViz panel for streamlined AVP testing and monitoring
- Scalable to larger fleets and more complex simulation scenarios

---

## Getting Started

See [https://zubxxr.github.io/multi-vehicle-avp](https://zubxxr.github.io/multi-vehicle-avp).