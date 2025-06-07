
# Multi Vehicle Autonomous Valet Parking Using Three Machines

[Include summary]

---

## Table of Contents

1. [System Setup](#system-setup)  
   - [Host 1](#host-1)  
   - [Host 2](#host-2)  
   - [Host 3](#host-3)  
2. [Software Installation and Setup](#software-installation-and-setup)  
   - [Installing Autoware](#installing-autoware)  
   - [AWSIM Setup](#awsim-setup)  
     - [Unity Installation](#unity-installation)  
     - [Open AWSIM Project](#open-awsim-project)  
     - [Replace the Map Package Link](#replace-the-map-package-link)  
   - [Installing Zenoh](#installing-zenoh)
   - [YOLOv5 Server Setup](#yolov5-server-setup)
3. [Launching the Full System](#launching-the-full-system-awsim-autoware-zenoh-and-yolo)  
   - [Step 1: Launching AWSIM](#step-1-launching-awsim)  
   - [Step 2: Start the YOLO Server](#step-2-start-the-yolo-server)  
   - [Step 3: Launching Autoware](#step-3-launching-autoware)  
   - [Step 4: Running Zenoh Bridge](#step-4-running-zenoh-bridge)  
     - [Finding Your IP Address](#finding-your-ip-address)  
   - [Step 5: Start the Automated Valet Parking Node](#step-5-start-the-automated-valet-parking-node)
  
## System Setup
The image below shows all the machines involved in this project. All systems are running Ubuntu 22.04.

![image](https://github.com/user-attachments/assets/466b95f2-8b8e-4e33-910b-87cc5aba8b10)

### Host 1
Host 1 is the laptop on the far left. It has a RTX 2060 Max-Q GPU and 24 GB of RAM.

Its display is extended to the two adjacent monitors:

- **Laptop screen**: runs terminal commands for the Zenoh Bridge, YOLOv5 server, and related processes.
- **Center monitor**: displays the AWSIM simulation.
- **Right monitor**: used to remotely access Host 2 and Host 3 via AnyDesk.

### Host 2
Host 2 is the laptop placed on top of the PC tower. It has a RTX 4050 GPU and 16 GB of RAM. It runs Autoware and the Zenoh Bridge independently. 

### Host 3
Host 3 is the PC tower itself. It has a RTX 3060 GPU and 24 GB of RAM. Its output is shown on the far-right monitor and runs Autoware and the Zenoh Bridge for a second ego vehicle.

---

## Software Installation and Setup

### Installing Autoware
Read the [following](https://autowarefoundation.github.io/autoware-documentation/main/installation/) to see the hardware requirements. 

The version of [Autoware](https://github.com/autowarefoundation/autoware/tree/release/2024.11) being used is `release/2024.11`. This version was forked and updated to better support the custom parking simulation use case.

To install Autoware, follow the instructions on [this page](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

> **Note:** Replace the following command:
> 
> ```bash
> git clone https://github.com/autowarefoundation/autoware.git
> ```
> 
> with:
> 
> ```bash
> git clone https://github.com/zubxxr/autoware.git
> ```

---

### AWSIM Setup

Setting up AWSIM requires the installation of Unity. Follow the **"Environment preparation"** section and carefully read the **"ROS 2"** section on [this page](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/) to get started.  

> The **"ROS 2"** section mentions that your environment should not have ROS 2 sourced.  
> It is recommended to **remove any ROS 2 sourcing lines from your `~/.bashrc`**, and instead **manually source ROS 2 only when needed**, to avoid environment conflicts.

#### Unity Installation


##### **1. Install Unity Hub from the Package Manager**

Start by installing Unity Hub to enable login. Follow the **"Install the Unity Hub on Linux"** section on [this page](https://docs.unity3d.com/hub/manual/InstallHub.html).

After installation, launch Unity Hub with:

```bash
unityhub
```
Sign in or create a Unity account as prompted.

##### **2. Install Unity Editor Binary**
Run the following commands to install the Unity Editor:
```bash
mkdir ~/Unity
cd ~/Unity
sudo apt install fuse libfuse2
sudo modprobe fuse
sudo groupadd fuse
sudo usermod -a -G fuse $USER
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage
chmod +x UnityHub.AppImage
./UnityHub.AppImage unityhub://2022.3.62f1/d91830b65d9b
```
The final command installs Unity version `2022.3.62f1`, which at the time of writing is the current version.

For future use, to launch Unity Hub later, run the following commands in a terminal that does not have ROS 2 sourced:
```bash
cd ~/Unity
./UnityHub.AppImage
```

These two steps complete the **"Unity installation**" section in [this page](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/).


#### Open AWSIM Project 

Follow the **"Open AWSIM project**" step in [this page](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/).  

> **Note:** At the time of writing, the documentation incorrectly tells you to clone:
> ```bash
> git clone git@github.com:autowarefoundation/AWSIM.git
> ```
> The correct repository is:
> ```bash
> git clone ~/https://github.com/autowarefoundation/AWSIM-Labs.git
> ```

#### Replace the Map Package Link
In the **"Import external packages"** section, **do not** use the green “Download Map Package” button shown in the docs.

Instead, **download the map package from this link**: [Download Zenoh-AWSIM-Labs-SIRC-June-4-2025.unitypackage](https://drive.google.com/file/d/1JXPlB_EWzItpGQwsTVuQIvlqlbNDCXrp/view?usp=sharing)

#### Final Steps
Then, follow the remaining steps in that section to import the `.unitypackage` file into Unity.

After the final step **Run the demo in Editor**, you will see the simulation running (see image below), which currently simulates both ego vehicles, which will later be controlled by their own Autoware clients. 

![image](https://github.com/user-attachments/assets/fffc4994-3622-4f69-a574-68b61ac352b1)

---

### Zenoh
This step covers setting up the Zenoh bridge on both hosts using their respective config files. This enables communication between the two ego vehicles simulated in AWSIM and their corresponding Autoware clients.

AWSIM is configured to simulate two ego vehicles, both publishing the same set of ROS 2 topics. To prevent conflicts, each vehicle's topics are manually namespaced:
- `Vehicle1_EgoVehicle` uses the `/vehicle1` prefix
- `Vehicle2_EgoVehicle` uses the `/vehicle2` prefix

This isolates their data and avoids topic collisions.


**Example of Host 2 Ego Vehicle (Vehicle1_EgoVehicle):**

![image](https://github.com/user-attachments/assets/39639116-1c8b-48a2-88e1-d3f7cd109e05)

**Example of Host 3 Ego Vehicle (Vehicle2_EgoVehicle):**

![image](https://github.com/user-attachments/assets/c5a7c99a-d0b0-42b4-86f2-ea0ef9b76d84)


#### How Zenoh Bridges Work

The Zenoh bridge is launched on each host to enable bidirectional communication of ROS 2 topics across machines.

- **Host 1 (AWSIM machine)** starts the bridge **first**, using this [config](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-awsim.json5). It **does not include a namespace**, so it sends *all* topics (for both vehicles) without filtering.

- **Host 2 (Vehicle 1)** starts the bridge **second**, using this [config](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-vehicle1.json5), which includes the namespace `/vehicle1`. This means:
  - It **only receives topics** under `/vehicle1`
  - It can still “see” other topics (like `/vehicle2`), but **cannot subscribe to or use them**
  - Topics appear without the `/vehicle1` prefix locally, due to Zenoh stripping it automatically

- **Host 3 (Vehicle 2)** follows the same process using the [config](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-vehicle2.json5) with the `/vehicle2` namespace.

This behavior is intentional and convenient — changing topic names in Autoware directly would be far more complex. Zenoh handles the mapping by stripping the namespace from incoming Zenoh topics before exposing them to the ROS 2 layer.

![image](https://github.com/user-attachments/assets/c12b7d85-80ef-450e-b460-aab95cfb1995)

#### Installing Zenoh ROS 2 Bridge

1. Install [Rust](https://www.rust-lang.org/tools/install)

2. Clone and build the Zenoh bridge on all hosts:

```bash
git clone ~/https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds -b release/1.4.0
cd ~/zenoh-plugin-ros2dds
rustup update
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select zenoh_bridge_ros2dds --cmake-args -DCMAKE_BUILD_TYPE=Release
source $HOME/zenoh-plugin-ros2dds/install/setup.bash
```

3. Find IP Address for Host 1
To set up Zenoh properly, you need the **IP address of the Host 1 active network interface** (usually Wi-Fi or Ethernet).

Run the following in your terminal:
```bash
ip a
```

Look for your active network interface:
- For **Wi-Fi**, the name usually starts with `wlp` (e.g., `wlp3s0`, `wlp0s20f3`)
- For **Ethernet**, it usually starts with `enp` (e.g., `enp2s0`)
- Ignore interfaces like `lo` (loopback), `docker0`, or `br-...` (Docker bridges)

Find the line that looks like this:
```
inet 10.0.0.22/24 ...
```

The IP address before the slash (`10.0.0.22`) is what you’ll use to connect from the other hosts:

```json
zenoh_bridge_ros2dds -e tcp/<IP-address>:7447
zenoh_bridge_ros2dds -e tcp/10.0.0.22:7447
```

> **Tip:** If you’re unsure which interface is active, check which one shows an `inet` address in the `10.x.x.x` or `192.168.x.x` range and says `state UP`.


3. **Launch the Zenoh bridge with host-specific configuration:**
   ##### A) Host 1 (Running AWSIM)
   
   Download the config file: [zenoh-bridge-awsim.json5](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-awsim.json5)
   
   ```bash
   cd ~/zenoh-plugin-ros2dds
   mv ~/Downloads/zenoh-bridge-awsim.json5 .
   source $HOME/zenoh-plugin-ros2dds/install/setup.bash

   # Test to see if it works
   zenoh_bridge_ros2dds -c zenoh-bridge-awsim.json5 # This should be executed first
   ```
   
   ##### B) Host 2 (Vehicle 1 / Autoware 1)
   
   Download the config file: [zenoh-bridge-vehicle1.json5](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-vehicle1.json5)
   
   ```bash
   cd ~/zenoh-plugin-ros2dds
   mv ~/Downloads/zenoh-bridge-vehicle1.json5 .
   source $HOME/zenoh-plugin-ros2dds/install/setup.bash

   # Test to see if it works
   zenoh_bridge_ros2dds -c zenoh-bridge-vehicle1.json5 -e tcp/<IP-address>:7447
   # Replace <IP-address> with Host 1's IP (e.g., 10.0.0.22)
   # This should be executed after Host 1 is running
   ```
   
   ##### C) Host 3 (Vehicle 2 / Autoware 2)
   
   Download the config file: [zenoh-bridge-vehicle2.json5](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-vehicle2.json5)
   
   ```bash
   cd ~/zenoh-plugin-ros2dds
   mv ~/Downloads/zenoh-bridge-vehicle2.json5 .
   source $HOME/zenoh-plugin-ros2dds/install/setup.bash

   # Test to see if it works
   zenoh_bridge_ros2dds -c zenoh-bridge-vehicle2.json5 -e tcp/<IP-address>:7447
   # Replace <IP-address> with Host 1's IP (e.g., 10.0.0.22)
   # This should be executed after Host 1 is running
   ```
---

### YOLOv5 Server Setup

The YOLOv5 server runs locally on the same machine as AWSIM. It captures frames from the overhead camera in the simulation, performs vehicle detection using YOLOv5, extracts the bounding box coordinates, and sends them to Unity for further processing.

In Unity, custom scripts receive these bounding boxes, draws them on the overhead view, and compares them with predefined parking spot coordinates. If there is any overlap, the spot is marked as **occupied**.

As a result, a ROS 2 topic is published containing a list of currently **empty parking spots**, which can be visualized in the bottom-left corner of the simulation (see image below):

![image](https://github.com/user-attachments/assets/fd8fad9a-dfba-4936-b6e1-dfc06943eb2d)


#### Download Required Files

Download the following files from this repository:

- [Weight File](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/best.pt)
- [YOLO Server Script](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/yolo_server.py)
- [Requirements File](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/requirements.txt)


#### Setup Instructions
Run the following commands in your terminal to set up the YOLO server environment:
```bash
mkdir ~/YOLO_Server
cd ~/YOLO_Server

# Move the downloaded files
mv ~/Downloads/best.pt .
mv ~/Downloads/yolo_server.py .
mv ~/Downloads/requirements.txt .

# Create and activate virtual environment
python3 -m venv yolo_server_env
source yolo_server_env/bin/activate

# Install dependencies
pip install -r requirements.txt
```

To start the server:

```bash
cd ~/YOLO_Server
source yolo_server_env/bin/activate
python3 yolo_server.py
```

If everything is working correctly, you will see output similar to the image below:

![image](https://github.com/user-attachments/assets/346d98c2-df20-48df-8cc1-311367c3021b)

---

## Launching the Full System: AWSIM, Autoware, Zenoh, and YOLO
Follow the steps below to launch all components required for the simulation.

### Step 1: Launching AWSIM
This step covers running AWSIM on Host 1.

#### Host 1 (ROG Laptop)
**1. Launch UnityHub**
  ```bash
  cd ~/Unity
  ./UnityHub.AppImage
  ```

**2. Launch AWSIM**
See [Final Steps](final-steps).

### Step 2: Start the YOLO Server
```cmd
cd ~/YOLO_Server
source yolo_server_env/bin/activate
python3 yolo_server.py
```

### Step 3: Launching Autoware
This step covers running Autoware on Host 2, and similarly, another separate Autoware client on Host 3.

#### Host 2 (Victus Laptop)
1. Launch Autoware on Host 2  
     ```bash
     cd ~/autoware
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/Zubair/autoware_map/sirc/ launch_vehicle_interface:=true
     ```
     
#### Host 3 (Nitro PC)
1. Launch Autoware
     ```bash
     cd ~/autoware
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/ovin/autoware_map/sirc/ launch_vehicle_interface:=true
     ```
---

### Step 4: Running Zenoh Bridge
To recap, after the previous steps, the current setup is:
- AWSIM running on Host 1
- Autoware running on Host 2
- Autoware running on Host 3

### Host 1 (ROG Laptop)
**1. Run Zenoh Bridge**
   ``` bash
   cd $HOME/zenoh-plugin-ros2dds
   source $HOME/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-awsim.json5
   ```

### Host 2 (Victus Laptop)
**1. Run Zenoh Bridge and Connect to Host 1**

Use the IP address retrieved from [Find IP Address for Host 1](find-ip-address-for-host-1) step. In this case, its 10.0.0.22.
   ``` bash
   cd $HOME/zenoh-plugin-ros2dds
   source $HOME/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-vehicle1.json5 -e tcp/10.0.0.22:7447
   ```
### Host 3 (Nitro PC)
1. Run Zenoh Bridge and Connect to Host 1
     ``` bash
     cd $HOME/zenoh-plugin-ros2dds
     source $HOME/zenoh-plugin-ros2dds/install/setup.bash
     zenoh_bridge_ros2dds -c zenoh-bridge-vehicle2.json5 -e tcp/10.0.0.22:7447
     ```

### Step 5: Start the Automated Valet Parking Node
#### Launch Script Sending Available Parking Spots to Autoware
```cmd
cd $HOME/Multi-AVP
source $HOME/autoware/install/setup.bash
source /opt/ros/humble/setup.bash
source env/bin/activate
python3 avp_sirc.py
```
This script can be run separately on each host. It subscribes to the empty parking spot ROS2 topic, takes the first parking spot from the list, sets a destination in that parking spot, and parks.


**[Include Picture Here]**


## Future Work
[]
