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

### 1.1 Autoware Universe

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

### 1.2 AWSIM Labs
1. Install Unity Hub and Unity Editor version **2023.2.6f1**.
2. Clone the AWSIM Labs repository:
```bash
git clone https://github.com/tier4/AWSIM.git
```
3. Open the project in Unity Hub and allow packages to load.
4. Configure your simulation scene with desired map and ego vehicle setup.

### 1.3 Zenoh Middleware
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
