# Software Setup

This section covers the setup process for all required software components.  

Except for **AWSIM Labs**, which is installed only on **Host 1**, **Autoware**, **Zenoh**, and tools like **Barrier** and **AnyDesk** are installed on **every host** in the system.

### Barrier & AnyDesk (Multi-Host Control and File Access)

When managing multiple machines in a distributed simulation, efficient control and file transfer tools are essential.

#### Barrier
Barrier allows one keyboard and mouse to control multiple systems by moving the cursor between screens, as if they were a single extended desktop. This is especially useful when running Autoware and related tools across hosts.

#### AnyDesk
AnyDesk provides lightweight remote access and file transfer between hosts.  

- **Remote Access:**
    Enables headless operation of Host 1 (Nitro PC), allowing Host 2 (ROG Laptop) to restart Barrier or terminals after reboots.  

- **File Transfer:**
    Quick sharing without USB drives or external cloud services.

#### Installation

1. **Initial Hardware Setup**

    Connect a keyboard and mouse to each host for the first-time configuration.  

2. **AnyDesk**

    Install [AnyDesk](https://anydesk.com/en) on all hosts and set a password for unattended access to enable quick remote connections.  

3. **Barrier**

    Install [Barrier](https://github.com/debauchee/barrier) on all hosts.  

    - **Host 1:** Run Barrier in **server** mode.  
    - **Host 2:** Run Barrier in **client** mode and connect to Host 1.  

4. **Post-Reboot Recovery**

    If Barrier does not auto-launch after a reboot, use AnyDesk to remotely access the host and restart Barrier without physically reconnecting peripherals.  

    > Barrier and AnyDesk significantly improved productivity by reducing downtime, avoiding physical reconnections, and streamlining debugging during multi-machine development.


### Repository Cloning

Clone the main repository for this framework on **all hosts**.

```bash
cd ~
git clone https://github.com/zubxxr/distributed-multi-autonomous-vehicle-architecture.git
```

The repository contains:

- Zenoh configuration files (`zenoh_configs/`) for different host setups.
- A `cyclonedds.xml` configuration file for cross-host ROS 2 communication.
- Map and localization configuration files for Autoware.

---

### Autoware Universe
Autoware is an open-source autonomous driving stack designed for self-driving vehicles. It provides core modules for localization, perception, planning, and control.
Autoware must be installed on each host that is responsible for controlling a vehicle.

#### Hardware Requirements
Before starting, review [Autoware’s official hardware requirements](https://autowarefoundation.github.io/autoware-documentation/main/installation/).

#### Version
This guide uses the Autoware branch [release/2024.11](https://github.com/autowarefoundation/autoware/tree/release/2024.11), with a forked and customized version available here: [Customized Autoware Repository](https://github.com/zubxxr/autoware/tree/avp-release/2025.08)

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

    **Recommended**: use the latest stable release

    ```bash
    cd ~
    git clone https://github.com/zubxxr/autoware.git -b avp-release/2025.08
    cd ~/autoware
    ```

    **Alternatively**: use the main branch if you want the newest (possibly experimental) changes

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

    > Building Autoware can take about 1-3 hours.

    > If any build issues occur, refer to [issues](https://github.com/zubxxr/multi-vehicle-framework/issues) or the Autoware community for possible solutions.
    
---

#### Running Autoware

Once Autoware is successfully built, it should be tested to ensure the installation is functioning correctly.

1. Download the Map

    ```bash
    gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1prmoy4UBgT_J-tJ7SELsL7m9alWGzD71'
    unzip -d ~/autoware_map ~/autoware_map/sirc.zip
    ```

2. Source the Environment

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/autoware/install/setup.bash
    ```

3. Launch Autoware using the Planning Simulation

    ```bash
    ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sirc vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
    ```

    Follow the [Basic Simulations](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/ad-hoc-simulation/planning-simulation/#basic-simulations) to run different scenarios in Autoware.


### AWSIM Labs
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

    Copy the configuration file from the repository to the home directory:
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

#### AWSIM Labs Configuration Steps

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
    
    Instead, **download the map package from this link**: [Download SIRC_MultiVehicle_URP_v1.0.0.unitypackage](https://drive.google.com/file/d/1AqETq-FrH9ayitxePGYnGe8IsjkAjyqq/view?usp=sharing)
    
    Then, follow the remaining steps in the section to import the `.unitypackage` file into Unity.

3. **Import Assets**

    Follow the [Import Vehicle Physics Pro Community Edition Asset](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#import-vehicle-physics-pro-community-edition-asset) and [Import Graphy Asset](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#import-graphy-asset).

#### Running AWSIM Labs

To run the simulation, follow the [Run the Demo in Editor](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/#run-the-demo-in-editor) section.

After successful completion, the simulation will be running with two ego vehicles. These vehicles will later be controlled by their own Autoware clients. 
    
![Multi-Vehicle Simulation Diagram](multi_vehicle_simulation.png)

> The game view is expanded by double clicking on the **Game** tab.

---

### Zenoh Middleware

[Zenoh](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) is a lightweight communication middleware designed for data routing across networks. 
In this framework, Zenoh bridges ROS 2 topics between multiple hosts, enabling real-time communication between AWSIM Labs and their respective Autoware instances running on separate machines.

#### Design Considerations
In this setup, AWSIM Labs simulates **two ego vehicles**, both publishing the same ROS 2 topics. To avoid collisions, one vehicle is assigned a topic namespace:

- **EgoVehicle_1** — connects locally to AWSIM Labs on the same host (no namespace)
- **EgoVehicle_2** — connects from the second host to AWSIM Labs on the first host with the `/vehicle2` namespace

This ensures all topics are isolated. For the EgoVehicle_2 GameObject, open each relevant child GameObject in Unity and add the `/vehicle2` prefix to all topic names.

![Topics Prefixing](topic_prefixing.png)

#### Installation Steps

1. **Install Rust**  

    Follow the [official installation guide](https://www.rust-lang.org/tools/install).
   

2. **Clone Zenoh Bridge Version *release/1.4.0* and Build on All Hosts**

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


4. **Set Up CycloneDDS for Cross-Host Communication**

    This step is identical to the CycloneDDS setup performed for AWSIM Labs, but it must also be completed on any additional hosts participating in the Zenoh network.

    Copy the configuration file from the repository to the home directory:

    ```bash
    cp ~/distributed-multi-autonomous-vehicle-architecture/cyclonedds.xml ~/cyclonedds.xml
    ```
    
    Add the following to the .bashrc file:

    ```bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=/home/your_username/cyclonedds.xml
    ```
    > Replace `your_username` with your actual Linux username.

---

#### Running Zenoh Bridges  

Each Zenoh bridge must be launched with host-specific configurations, already available in the cloned repository.  

**Host 1** (start this first)

```bash
source ~/zenoh-plugin-ros2dds/install/setup.bash
zenoh_bridge_ros2dds -c ~/distributed-multi-autonomous-vehicle-architecture/zenoh_configs/zenoh-bridge-awsim.json5  
```

**Host 2** (start after Host 1 is running):  

```bash
source ~/zenoh-plugin-ros2dds/install/setup.bash
zenoh_bridge_ros2dds -c ~/distributed-multi-autonomous-vehicle-architecture/zenoh_configs/zenoh-bridge-vehicle2.json5 -e tcp/10.0.0.172:7447  
```

> **Note:** The IP `10.0.0.172` is only an example.  

If done correctly, both bridges will be connected.

**Next Steps:** Proceed to [Multi-Vehicle Simulation](../Multi-VehicleSimulation/index.md) to start the simulation.
