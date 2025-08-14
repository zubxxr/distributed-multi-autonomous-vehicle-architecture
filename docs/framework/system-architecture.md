# 1. System Architecture
This section outlines the software stack, hardware specifications, and machine roles. The architecture is built around a distributed, multi-host setup where each host handles specific tasks such as simulation, perception, control, or coordination.

## 1.1 Software Stack and Version Overview

| **Component**        | **Name**                  | **Version / Branch**                  |
|----------------------|---------------------------|---------------------------------------|
| Operating System     | Ubuntu                    | 22.04 LTS                             |
| ROS 2 Distribution   | ROS 2                     | Humble Hawksbill                      |
| Autonomy Stack       | Autoware Universe         | `release/2024.11` (forked & modified) |
| Simulation Engine    | AWSIM Labs                | Internal version (modified since Nov 2024) |
| Middleware Bridge    | Zenoh Bridge for ROS 2    | `release/1.4.0`                       |

## 1.2 Hardware Specifications of Host Machines Used During Development

| **Host**   | **Model**                     | **CPU**              | **GPU**                 | **RAM** | **OS**        | **NVIDIA Driver** |
|------------|-------------------------------|----------------------|-------------------------|---------|---------------|-------------------|
| Nitro PC   | Acer Nitro N50-640            | Intel Core i7-12700F | GeForce RTX 3060        | 24 GB   | Ubuntu 22.04  | 575               |
| ROG Laptop | ASUS ROG Zephyrus G15 GA502IV | AMD Ryzen 7 4800HS   | GeForce RTX 2060 Max-Q  | 24 GB   | Ubuntu 22.04  | 575               |

<img width="700" height="500" alt="image" src="https://github.com/user-attachments/assets/f329880a-004f-4e7c-bf47-3b05aeceb701" />

## 1.3 Design Considerations
- **Host 1**: Runs AWSIM Labs and Autoware for `EgoVehicle_1` (no namespace).  
- **Host 2**: Runs Autoware for `EgoVehicle_2` (namespaced as `/vehicle2`) and communicates with Host 1 via Zenoh.

Additional hosts can be added following the Host 2 pattern.

If your machine is powerful enough, you may run both AWSIM Labs and Autoware on a single host. Otherwise, distribute the workload: e.g., **AWSIM Labs** on one host; each additional host runs Autoware for one vehicle.

In this project, the **Nitro PC** ran AWSIM Labs + Autoware for `EgoVehicle_1`, while the **ROG Laptop** ran Autoware for `EgoVehicle_2` (two-host, two-vehicle).

> If your machine matches or exceeds the Nitro PC, you should be able to run both components together.
