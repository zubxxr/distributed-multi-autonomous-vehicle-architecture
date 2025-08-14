# Multi-Host Autonomous Vehicle Simulation Framework

![Autoware](https://img.shields.io/badge/Autoware-2024.09-blue)
![AWSIM Labs](https://img.shields.io/badge/AWSIM%20Labs-Unity-green)
![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-purple)
![Zenoh](https://img.shields.io/badge/Zenoh-Middleware-orange)

This repository provides a setup for running **Autoware** and **AWSIM Labs** in a multi-host, multi-vehicle simulation environment. It includes installation and configuration instructions for:
- Autoware Universe
- AWSIM Labs (Unity-based simulation)
- Zenoh middleware for distributed ROS 2 topic synchronization

The framework allows you to simulate multiple autonomous vehicles across different physical machines while maintaining synchronized perception, localization, planning, and control.

---

## 1. Software Installation and Setup

### 1.1 Repository


### 1.2 Autoware Universe
Autoware is an open-source autonomous driving stack designed for full-sized self-driving vehicles. It provides core modules for localization, perception, planning, and control.
Autoware must be installed on each device that is responsible for controlling a vehicle.

#### Hardware Requirements
Before starting, review [Autoware’s official hardware requirements](https://autowarefoundation.github.io/autoware-documentation/main/installation/).

#### Version Used
This guide uses the Autoware branch [release/2024.11](https://github.com/autowarefoundation/autoware/tree/release/2024.11), with a forked and customized version available here: [Customized Autoware Repository](https://github.com/zubxxr/autoware)

---

#### Increase Swap Memory (Optional but Recommended)
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

#### Installation Steps

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

### 1.3 AWSIM Labs
[AWSIM Labs](https://autowarefoundation.github.io/AWSIM-Labs/main/) is a Unity-based 3D simulation environment tailored for testing autonomous vehicles using Autoware. It provides realistic visuals, physics, and ROS 2 integration to simulate ego vehicle behavior in structured environments like parking lots.
> **Recommendation:** Install AWSIM Labs on the most powerful host in your setup (e.g., Nitro PC), as it is the most resource-intensive component in the simulation pipeline.

This section is adapted from the official [AWSIM Labs Unity Setup Guide](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/).

---

#### Networking Configurations

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
    
#### Preparation
Follow the [Environment preparation](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#environment-preparation) and [ROS 2](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#ros-2) sections.

> The **"ROS 2"** section recommends that ROS 2 should **not** be sourced in your environment when running Unity.  
> **Recommendation:** It is best to **remove ROS 2 sourcing lines from `~/.bashrc`** and manually source ROS 2 only when needed.

#### Unity Installation Steps
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
#### AWSIM Labs Setup
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
    
    Instead, **download the map package from this link**: [Download Zenoh-AWSIM-Labs-SIRC-June-4-2025.unitypackage](https://drive.google.com/file/d/1JXPlB_EWzItpGQwsTVuQIvlqlbNDCXrp/view?usp=sharing)
    
    Then, follow the remaining steps in the section to import the `.unitypackage` file into Unity.

3. **Running the Simulation**
   
    Lastly, follow the [Run the Demo in Editor](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#run-the-demo-in-editor) section.
    
    After successful completion, the simulation will be running (see image below), which will simulates two ego vehicles. These vehicles will later be controlled by their own Autoware clients. 
    
    ![image](https://github.com/user-attachments/assets/fffc4994-3622-4f69-a574-68b61ac352b1)
    
    > The game view is expanded by double clicking on the **Game** tab.

---

### 1.4 Zenoh Middleware

Zenoh is a lightweight communication middleware that bridges ROS 2 topics across machines. In this AVP system, Zenoh enables real-time communication between AWSIM Labs (simulating two ego vehicles) and their respective Autoware instances, which may be running on separate hosts.

Zenoh will be used to bridge ROS 2 topics between hosts.
```bash
# Download Zenoh
wget https://github.com/eclipse-zenoh/zenoh/releases/download/0.11.0-rc/zenoh-0.11.0-rc-x86_64-unknown-linux-gnu.zip
unzip zenoh-0.11.0-rc-x86_64-unknown-linux-gnu.zip -d ~/zenoh
```
Create a Zenoh configuration file (`config.json5`) for each host, defining routers or clients depending on your network topology.

---

## 2. Multi-Host Setup

### 2.1 Host Roles
- **Host 1:** Runs AWSIM Labs and Autoware for vehicle1.
- **Host 2:** Runs Autoware for vehicle2.

### 2.2 Zenoh Bridging
1. On Host 1 (Router mode):
```bash
~/zenoh/zenohd --config config_router.json5
```
2. On Host 2 (Client mode):
```bash
~/zenoh/zenohd --config config_client.json5
```
3. Ensure that namespaced topics (e.g., `/vehicle2/`) are bridged correctly.

### 2.3 Launch Sequence
1. **Host 1:** Start AWSIM Labs.
2. **Host 1:** Launch Autoware for vehicle1:
```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulation.launch.xml vehicle_model:=sample_vehicle_1 map_path:=<map_path>
```
3. **Host 2:** Launch Autoware for vehicle2:
```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulation.launch.xml vehicle_model:=sample_vehicle_2 map_path:=<map_path> vehicle_id:=vehicle2
```

---

## 3. Notes
- Use **ROS 2 namespaces** for each vehicle to prevent topic collisions.
- Ensure all hosts are on the same ROS 2 DDS domain.
- Test Zenoh connectivity before launching Autoware.

---

## 4. License
This project is licensed under the MIT License.
