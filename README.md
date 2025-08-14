# Multi-Host Autonomous Vehicle Simulation Framework

![Autoware](https://img.shields.io/badge/Autoware-2024.09-blue)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple)
![Zenoh](https://img.shields.io/badge/Zenoh-Middleware-orange)

This repository provides a setup for running **Autoware** and **AWSIM Labs** in a multi-host, multi-vehicle simulation environment. It includes installation and configuration instructions for:
- Autoware Universe
- AWSIM Labs (Unity-based simulation)
- Zenoh middleware for distributed ROS 2 topic synchronization

The framework allows for the simulation of multiple autonomous vehicles across different physical machines while maintaining synchronized perception, localization, planning, and control.

---

## Table of Contents
1. **[System Architecture](#1-system-architecture)**  
   &nbsp;&nbsp;1.1 [Software Stack and Version Overview](#11-software-stack-and-version-overview)  
   &nbsp;&nbsp;1.2 [Hardware Specifications](#12-hardware-specifications-of-host-machines-used-during-development)  
   &nbsp;&nbsp;1.3 [Design Considerations](#13-design-considerations)  

2. **[Software Installation](#2-software-installation)**  
   &nbsp;&nbsp;2.1 [Repository](#21-repository)  
   &nbsp;&nbsp;2.2 [Autoware Universe](#22-autoware-universe)  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.2.1 [Hardware Requirements](#221-hardware-requirements)*  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.2.2 [Version Used](#222-version-used)*  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.2.3 [Increase Swap Memory (Optional but Recommended)](#223-increase-swap-memory-optional-but-recommended)*  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.2.4 [Installation Steps](#224-installation-steps)*  
   &nbsp;&nbsp;2.3 [AWSIM Labs](#23-awsim-labs)  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.3.1 [Networking Configurations](#231-networking-configurations)*  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.3.2 [Preparation](#232-preparation)*  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.3.3 [Unity Installation Steps](#233-unity-installation-steps)*  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.3.4 [AWSIM Labs Setup](#234-awsim-labs-setup)*  
   &nbsp;&nbsp;2.4 [Zenoh Middleware](#24-zenoh-middleware)  
   &nbsp;&nbsp;&nbsp;&nbsp;*2.4.1 [Installation Steps](#241-installation-steps)*  

3. **[Multi-Vehicle Simulation](#3-multi-vehicle-simulation)**  
   &nbsp;&nbsp;3.1 [Prerequisites](#31-prerequisites)  
   &nbsp;&nbsp;3.2 [Launch Sequence](#32-launch-sequence)  

---

## 1. System Architecture
This section outlines the software stack, hardware specifications, and machine roles used throughout the project. The architecture is built around a distributed, multi-host setup where each host is responsible for specific tasks such as simulation, perception, control, or coordination.

### 1.1 Software Stack and Version Overview

| **Component**              | **Name**                                | **Version / Branch**                               |
|---------------------------|-----------------------------------------|-----------------------------------------------------|
| Operating System           | Ubuntu                                  | 22.04 LTS                                          |
| ROS 2 Distribution         | ROS 2                                   | Humble Hawksbill                                   |
| Autonomy Stack             | Autoware Universe                       | `release/2024.11` (forked and modified)            |
| Simulation Engine          | AWSIM Labs                              | Internal version (modified since Nov 2024)         |
| Middleware Bridge          | Zenoh Bridge for ROS 2                  | `release/1.4.0`                                    |

### 1.2 Hardware Specifications of Host Machines Used During Development

| **Host**        | **Model**                      | **CPU**                    | **GPU**                   | **RAM**  | **OS**          | **NVIDIA Driver** |
|-----------------|--------------------------------|----------------------------|---------------------------|----------|-----------------|-------------------|
| Nitro PC        | Acer Nitro N50-640             | Intel Core i7-12700F       | GeForce RTX 3060          | 24 GB    | Ubuntu 22.04    | 575               |
| ROG Laptop      | ASUS ROG Zephyrus G15 GA502IV  | AMD Ryzen 7 4800HS         | GeForce RTX 2060 Max-Q    | 24 GB    | Ubuntu 22.04    | 575               |

<img width="700" height="500" alt="image" src="https://github.com/user-attachments/assets/f329880a-004f-4e7c-bf47-3b05aeceb701" />

---

### 1.3 Design Considerations
This framework is designed for a **multi-host setup** to distribute computational load and enable coordinated operation of multiple ego vehicles in AWSIM Labs.  

- **Host 1**: Runs AWSIM Labs and Autoware for `EgoVehicle_1` (no namespace).  
- **Host 2**: Runs Autoware for `EgoVehicle_2` (namespaced as `/vehicle2`) and communicates with Host 1 via Zenoh.

Additional hosts may be added for more ego vehicles, following the same pattern as Host 2.  

If your machine is powerful enough, you may choose to run both AWSIM Labs and Autoware on a single host. Otherwise, it's recommended to distribute the workload across multiple machines, especially if you're using laptops or other resource-constrained systems, **which means that it's good to have AWSIM Labs on its own host, and have each other host only run Autoware**.

In this project, the **Nitro PC** (see hardware specs) was powerful enough to run both AWSIM Labs and Autoware for `EgoVehicle_1`, while the ROG Laptop was used to run Autoware for `EgoVehicle_2`, effectively creating a two-host, two-vehicle architecture (multi-vehicle setup).

> If your machine has similar or better specifications than the Nitro PC, you should be able to run both components together without major issues.  

---

## 2. Software Installation

### 2.1 Repository
Clone the main repository for this framework on **all hosts**.
It contains:
- Zenoh configuration files (`zenoh_configs/`) for different host setups.
- A `cyclonedds.xml` configuration file for cross-host ROS 2 communication.
- Map and localization configuration files for Autoware.

```bash
cd ~
git clone https://github.com/zubxxr/multi-vehicle-framework.git
```

---

### 2.2 Autoware Universe
Autoware is an open-source autonomous driving stack designed for self-driving vehicles. It provides core modules for localization, perception, planning, and control.
Autoware must be installed on each host that is responsible for controlling a vehicle.

#### 2.2.1 Hardware Requirements
Before starting, review [Autoware’s official hardware requirements](https://autowarefoundation.github.io/autoware-documentation/main/installation/).

#### 2.2.2 Version Used
This guide uses the Autoware branch [release/2024.11](https://github.com/autowarefoundation/autoware/tree/release/2024.11), with a forked and customized version available here: [Customized Autoware Repository](https://github.com/zubxxr/autoware)

#### 2.2.3 Increase Swap Memory (Optional but Recommended)
If you encounter memory issues when building Autoware, increase your swap size:

```bash
# Check current swap usage
free -h

# Remove the existing swapfile
sudo swapoff /swapfile
sudo rm /swapfile

# Create a new 32GB swapfile
sudo fallocate -l 32G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Verify changes
free -h
```
> More info: [Autoware Build Troubleshooting](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/#build-issues).

#### 2.2.4 Installation Steps

The following installation steps are adapted from the [Autoware Universe Source Installation Guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

1. **Install Git**
    ```bash
    sudo apt-get -y update
    sudo apt-get -y install git
    ```

2. **Clone Autoware**
    ```bash
    cd ~
    git clone https://github.com/zubxxr/autoware.git
    cd ~/autoware
    ```

3. **Install Development Environment**
    ```bash
    ./setup-dev-env.sh
    ```
    > If any build issues are encountered, see the [Autoware Troubleshooting Guide](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/#build-issues) and [this issue thread](https://github.com/zubxxr/multi-vehicle-framework/issues/24).

4. **Import Source Code**
    ```bash
    cd ~/autoware
    mkdir src
    vcs import src < autoware.repos
    ```

5. **Install ROS 2 Dependencies**
    ```bash
    source /opt/ros/humble/setup.bash
    sudo apt update && sudo apt upgrade
    rosdep update
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    ```

6. **Setup Ccache**
    ```bash
    sudo apt update && sudo apt install ccache
    mkdir -p ~/.cache/ccache
    touch ~/.cache/ccache/ccache.conf
    echo "max_size = 60G" >> ~/.cache/ccache/ccache.conf
    ```

7. **Integrate Ccache into Environment**

    Open the .bashrc file using the Text Editor:
    ```bash
    gedit ~/.bashrc
    ```
    Add the following lines:
    ```bash
    export CC="/usr/lib/ccache/gcc"
    export CXX="/usr/lib/ccache/g++"
    export CCACHE_DIR="$HOME/.cache/ccache/"
    ```
    Save the file and source it:
    ```bash
    source ~/.bashrc
    ```
    Verify:
    ```bash
    ccache -s
    ```

9. **Build Autoware**
    ```bash
    cd ~/autoware
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
    
---

### 2.3 AWSIM Labs
[AWSIM Labs](https://autowarefoundation.github.io/AWSIM-Labs/main/) is a Unity-based 3D simulation environment tailored for testing autonomous vehicles using Autoware. It provides realistic visuals, physics, and ROS 2 integration to simulate ego vehicle behavior in structured environments like parking lots.
> **Recommendation:** Install AWSIM Labs on the most powerful host in your setup (e.g., Nitro PC), as it is the most resource-intensive component in the simulation pipeline.

This section is adapted from the official [AWSIM Labs Unity Setup Guide](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/).

---

#### 2.3.1 Networking Configurations

1. **Add the following lines to your `~/.bashrc` file**
    ```bash
    if [ ! -e /tmp/cycloneDDS_configured ]; then
        sudo sysctl -w net.core.rmem_max=2147483647
        sudo sysctl -w net.ipv4.ipfrag_time=3
        sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728     # (128 MB)
        sudo ip link set lo multicast on
        touch /tmp/cycloneDDS_configured
    fi
    ```
    
2. **Set up CycloneDDS**

    Copy the configuration file:
    ```bash
    cp ~/multi-vehicle-framework/cyclonedds.xml ~/cyclonedds.xml
    ```

    Add these lines to your `~/.bashrc`:
    ```bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=/home/your_username/cyclonedds.xml
    ```
    > Replace `your_username` with your actual Linux username.

---
    
#### 2.3.2 Preparation
Follow the [Environment preparation](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#environment-preparation) and [ROS 2](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#ros-2) sections.

> The **"ROS 2"** section recommends that ROS 2 should **not** be sourced in your environment when running Unity.  
> **Recommendation:** It is best to **remove ROS 2 sourcing lines from `~/.bashrc`** and manually source ROS 2 only when needed.

#### 2.3.3 Unity Installation Steps
Due to authentication requirements, Unity Hub is first installed in Step 1 via the package manager to allow account sign-in. 
In Step 2, the Unity Hub AppImage is installed and used for all subsequent project work.

1. **Install Unity Hub from the Package Manager**

    Follow the **"Install the Unity Hub on Linux"** section in [this page](https://docs.unity3d.com/hub/manual/InstallHub.html).

    After installation, launch Unity Hub with:
    ```bash
    unityhub
    ```
    Sign in or create a Unity account as prompted.

2. **Install Unity Editor Binary**
   
    Run the following commands to install the Unity Editor:
    ```bash
    
    # Create a directory to store Unity-related files
    mkdir ~/Unity
    cd ~/Unity

    # Install and configure required libraries
    sudo apt install fuse libfuse2
    sudo modprobe fuse
    sudo groupadd fuse
    sudo usermod -a -G fuse $USER

    # Download the Unity Hub AppImage
    wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

    # Make the Unity Hub AppImage executable
    chmod +x UnityHub.AppImage
    
    # Launch Unity Hub and directly install Unity Editor version 2022.3.62f1
    ./UnityHub.AppImage unityhub://2022.3.62f1/d91830b65d9b
    ```
    
    > The final command installs Unity version `2022.3.62f1`, which at the time of writing is the current version.

    For future use, to launch Unity Hub later, run the following command in a terminal that does not have ROS 2 sourced:
    ```bash
    ~/Unity/UnityHub.AppImage
    ```
#### 2.3.4 AWSIM Labs Setup
1. **Open the AWSIM Labs Project**
   
    Follow the [Open AWSIM Labs project](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#open-awsim-project) section.
    > **Note:** Replace the following command:
    > 
    > ```bash
    > git clone https://github.com/autowarefoundation/AWSIM-Labs.git
    > ```
    > 
    > with:
    > 
    > ```bash
    > git clone https://github.com/zubxxr/AWSIM-Labs.git
    > ```

2. **Import the Environment**
   
    In the [Import external packages](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#import-external-packages) section, **do not** use the green “Download Map Package” button shown in the docs.
    
    Instead, **download the map package from this link**: [Download SIRC_MultiVehicle_URP_v1.0.0.unitypackage](https://drive.google.com/file/d/1s5RB18-M80A2iE-m0wdnAcY-OZj2qmlt/view?usp=sharing)
    
    Then, follow the remaining steps in the section to import the `.unitypackage` file into Unity.

3. **Running the Simulation**
   
    Lastly, follow the [Run the Demo in Editor](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#run-the-demo-in-editor) section.
    
    After successful completion, the simulation will be running (see image below), which will simulates two ego vehicles. These vehicles will later be controlled by their own Autoware clients. 
    
    <img width="1850" height="973" alt="image" src="https://github.com/user-attachments/assets/7b8835b0-19ca-42c7-8366-dfd3499369a2" />


    
    > The game view is expanded by double clicking on the **Game** tab.

---

### 2.4 Zenoh Middleware

[Zenoh](github.com/eclipse-zenoh/zenoh-plugin-ros2dds) is a lightweight communication middleware designed for data routing across networks. 
In this framework, Zenoh bridges ROS 2 topics between multiple hosts, enabling real-time communication between AWSIM Labs and their respective Autoware instances running on separate machines.

#### Design Considerations
In this setup, AWSIM Labs simulates **two ego vehicles**, both publishing the same ROS 2 topics. To avoid collisions, one vehicle is assigned a topic namespace:

- **EgoVehicle_1** — connects locally to AWSIM Labs on the same host (no namespace)
- **EgoVehicle_2** — connects from the second host to AWSIM Labs on the first host with the `/vehicle2` namespace

This ensures all topics are isolated. For the EgoVehicle_2 GameObject, open each relevant child GameObject in Unity and add the `/vehicle2` prefix to all topic names (see example below).

<img width="1852" height="617" alt="image" src="https://github.com/user-attachments/assets/06b181bd-b86b-4d8a-addf-943e4a6d1da8" />

#### 2.4.1 Installation Steps
1. **Install Rust**  
   Follow the [official installation guide](https://www.rust-lang.org/tools/install).
   
2. **Clone the release/1.4.0 version of Zenoh Bridge and Build it on All Hosts**

    ```bash
    cd ~
    git clone https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds -b release/1.4.0
    cd ~/zenoh-plugin-ros2dds
    rustup update
    rosdep install --from-paths . --ignore-src -r -y
    colcon build --packages-select zenoh_bridge_ros2dds --cmake-args -DCMAKE_BUILD_TYPE=Release
    source ~/zenoh-plugin-ros2dds/install/setup.bash
    ```
3. **Find the IP Address for Host 1**

    You will need the **IP address of Host 1's active network interface** (usually Wi-Fi or Ethernet).
   
    Run the following:
    ```bash
    ip a
    ```
    Look for your active interface:
    - **Wi-Fi** names usually start with `wlp` (e.g., `wlp3s0`)  
    - **Ethernet** names usually start with `enp` (e.g., `enp2s0`)  
    - Ignore `lo` (loopback), `docker0`, and `br-...` (Docker bridges)
  
    Example output:
    ```
    inet 10.0.0.172/24 ...
    ```
    
    In the example output above, the IP before the slash (`10.0.0.172`) would be used for Zenoh connections:
    ```bash
    zenoh_bridge_ros2dds -e tcp/<IP-address>:7447
    # Example:
    zenoh_bridge_ros2dds -e tcp/10.0.0.172:7447
    ```
    > **Tip:** If unsure, choose the interface with an `inet` address in the `10.x.x.x` or `192.168.x.x` range and `state UP`.

4. **Set up CycloneDDS for cross-host communication**

    This step is identical to the CycloneDDS setup performed for AWSIM Labs, but it must also be completed on any additional hosts participating in the Zenoh network.

    Copy the CycloneDDS configuration file to the home directory.
    ```bash
    cp ~/multi-vehicle-avp/cyclonedds.xml ~/cyclonedds.xml
    ```
    
    Add the following to the .bashrc file.
    ```bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=/home/your_username/cyclonedds.xml
    ```
    > Replace `your_username` with your actual Linux username.

5. **Launch the Zenoh bridge with host-specific configuration**
   
   Both configs are available in the repository previously cloned.

   **Host 1** (run this first):
    ```bash
    source ~/zenoh-plugin-ros2dds/install/setup.bash
    zenoh_bridge_ros2dds -c ~/multi-vehicle-framework/zenoh_configs/zenoh-bridge-awsim.json5
    ```

   **Host 2** (run after Host 1 is running):
    ```bash
    source ~/zenoh-plugin-ros2dds/install/setup.bash
    zenoh_bridge_ros2dds -c ~/multi-vehicle-framework/zenoh_configs/zenoh-bridge-vehicle2.json5 -e tcp/10.0.0.172:7447
    ```

    > **Note:** The IP `10.0.0.172` above is from an example network and is different for all machines.  
    > Replace it with the Host 1 IP obtained via `ip a`.

---

## 3. Multi-Vehicle Simulation

### 3.1 Prerequisites
Ensure the following components are set up on each host:

- **Host 1:** Runs AWSIM Labs, Autoware (for Vehicle 1), and Zenoh.  
- **Host 2:** Runs Autoware (for Vehicle 2) and Zenoh.

---

### 3.2 Launch Sequence
1. **Launch AWSIM Labs** (Host 1)
   
   See [Section 2.3.3 – Step 3](https://github.com/zubxxr/multi-vehicle-framework/tree/main?tab=readme-ov-file#233-awsim-labs-setup) for detailed instructions.

---

2. **Launch Autoware**

    **Host 1**  
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/autoware/install/setup.bash
    ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=$HOME/autoware_map/sirc/ launch_vehicle_interface:=true
    ```
    
    On Host 1, Autoware automatically connects to AWSIM Labs because both components are running on the same machine.
    <img width="1472" height="802" alt="image" src="https://github.com/user-attachments/assets/247377c5-0288-40a9-8a21-5c3458788c24" />


    **Host 2**  
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/autoware/install/setup.bash
    ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=$HOME/autoware_map/sirc/ launch_vehicle_interface:=true
    ```
    On Host 2, Autoware will remain in a waiting state until it receives an initial pose via Zenoh.
    <img width="1600" height="880" alt="image" src="https://github.com/user-attachments/assets/b0792fa0-c63d-4c0f-96d8-d63f6aa3e0b0" />

---

3. **Run the Zenoh Bridges**

   **Host 1**  
   ```bash
   source ~/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c ~/multi-vehicle-avp/zenoh_configs/zenoh-bridge-awsim.json5
   ```

   **Host 2**  
   ```bash
   source ~/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c ~/multi-vehicle-avp/zenoh_configs/zenoh-bridge-vehicle2.json5 -e tcp/<IP-address>:7447
   ```
   > Replace `<IP-address>` with the Host 1 IP address found in Step 3 of the [Zenoh Installation Steps](https://github.com/zubxxr/multi-vehicle-framework/blob/main/README_2.md#installation-steps-1).

   Once both Zenoh Bridges are connected, Autoware on Host 2 immediately receives the initial pose and localizes successfully.
   <img width="1600" height="877" alt="image" src="https://github.com/user-attachments/assets/fc7cf011-eab7-4749-848c-466394413f09" />

---

## License
This project is licensed under the Apache License 2.0.
