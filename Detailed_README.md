
# Multi Vehicle Autonomous Valet Parking Using Three Machines

[Include summary]

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

**[Include Steps on Downloading Autoware, AWSIM, UnityHub]**

## Installing Autoware
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

## Installing UnityHub/AWSIM

Follow the steps on [this page](https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/SetupUnityProject/).  
> **Note:** At the time of writing, the documentation incorrectly tells you to clone:
> ```bash
> git clone git@github.com:autowarefoundation/AWSIM.git
> ```
> The correct repository is:
> ```bash
> git clone https://github.com/autowarefoundation/AWSIM-Labs.git
> ```

### ⚠️ Replace the Map Package Link
In the **"Import external packages"** section, **do not** use the green “Download Map Package” button shown in the docs.

Instead, **download the map package from the link below**:

[Download Zenoh-AWSIM-Labs-SIRC-June-4-2025.unitypackage](https://drive.google.com/file/d/1JXPlB_EWzItpGQwsTVuQIvlqlbNDCXrp/view?usp=sharing)

Then, follow the remaining steps in that section to import the `.unitypackage` file into Unity.

---

## Step 1: Launching AWSIM
This step covers running AWSIM on Host 1.

### Host 1 (ROG Laptop)
**1. Launch UnityHub**
  ```bash
  cd ~/Unity
  ./UnityHub.AppImage
  ```

@@

**2. Launch AWSIM**

After launching UnityHub, open the project named `AWSIM-Labs-Zenoh` and click play to run the scene. This simulates both ego vehicles, which will run their own Autoware clients. 

![image](https://github.com/user-attachments/assets/fffc4994-3622-4f69-a574-68b61ac352b1)


## Step 2: Start the Parking Spot Detection Node
[Include description here]
### Launch YOLO Server
```cmd
cd $HOME/Multi-AVP
source env/bin/activate
python3 yolo_server.py
```

![image](https://github.com/user-attachments/assets/346d98c2-df20-48df-8cc1-311367c3021b)

This python script takes frames from the overhead camera in the AWSIM simulation, runs YOLO car detection on them, and extracts the bounding box coordinates.

In Unity, there are scripts that retrieve the bounding box coordinates. maps and draws them on to the overhead camera view in the simulation, and compares them with the parking spot coordinates. If there is overlap, then it is classified as occupied. 

A ROS2 topic is published as a result of the parking spot detection, which publishes a list of empty parking spots. The list can be seen in the bottom left of the image below.

![image](https://github.com/user-attachments/assets/fd8fad9a-dfba-4936-b6e1-dfc06943eb2d)

## Step 3: Launching Autoware
This step covers running Autoware on Host 2, and similarly, another separate Autoware client on Host 3.

### Host 2 (Victus Laptop)
1. Launch Autoware on Host 2  
     ```bash
     cd ~/autoware
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/Zubair/autoware_map/sirc/ launch_vehicle_interface:=true
     ```
     
### Host 3 (Nitro PC)
1. Launch Autoware
     ```bash
     cd ~/autoware
     source install/setup.bash
     ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit map_path:=/home/ovin/autoware_map/sirc/ launch_vehicle_interface:=true
     ```
---

## Step 4: Running Zenoh Bridge To Connect Both Ego Vehicles in AWSIM to Both Autoware Clients
This step covers running the Zenoh bridge on both hosts with their respective config files. 

To recap, after the previous steps, the current setup is:
- AWSIM running on Host 1
- Autoware running on Host 2
- Autoware running on Host 3

AWSIM has been configured to simulate two ego vehicles, both publishing the same set of topics. The first vehicle, `Vehicle1_EgoVehicle`, has `/vehicle1` manually prefixed to each of its topic names, while the second vehicle, `Vehicle2_EgoVehicle`, has `/vehicle2` prefixed. This prevents conflicts between the two ego vehicles by isolating their data under separate namespaces.

**Example of Host 2 Ego Vehicle (Vehicle1_EgoVehicle):**

![image](https://github.com/user-attachments/assets/39639116-1c8b-48a2-88e1-d3f7cd109e05)

**Example of Host 3 Ego Vehicle (Vehicle2_EgoVehicle):**

![image](https://github.com/user-attachments/assets/c5a7c99a-d0b0-42b4-86f2-ea0ef9b76d84)


The Zenoh bridge is run on both hosts to enable communication and data exchange. On host 1, the [config](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-awsim.json5) file has no namespace set, and the bridge is started first. This means it sends all ROS 2 topics to host 2 and host 3 without any filtering. On host 2, the bridge is started shortly after, using a [config](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-vehicle1.json5) that includes the namespace `/vehicle1`. As a result, it only receives data from the topics under that namespace. However, it can "see" other topics, but not recieve data from them. When these topics are listed on host 2, they appear without the `/vehicle1` prefix. The same thing applies to host 3. The bridge on host 3 can be started after, using the [config](https://github.com/zubxxr/Multi-Vehicle-Autonomous-Valet-Parking/blob/main/Zenoh-Setup/zenoh-bridge-vehicle2.json5) file that includes the namespace `/vehicle2`.

This behavior works as expected and for convienience, as changing all the topics in Autoware would be challenging. Essentially, the Zenoh bridge on host 2 and host 3 respectively maps namespaced Zenoh topics to local ROS 2 topics by stripping the namespace defined in their configs.

![image](https://github.com/user-attachments/assets/c12b7d85-80ef-450e-b460-aab95cfb1995)


## Host 1 (ROG Laptop)
**1. Run Zenoh Bridge**
   ``` bash
   cd $HOME/ZENOH/zenoh-plugin-ros2dds
   source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-awsim.json5
   ```

## Finding Your IP Address
To set up Zenoh properly, you need the **IP address of your machine’s active network interface** (usually Wi-Fi or Ethernet).

### Run this in your terminal:
```bash
ip a
```

### Look for your active network interface:
- For **Wi-Fi**, the name usually starts with `wlp` (e.g., `wlp3s0`, `wlp0s20f3`)
- For **Ethernet**, it usually starts with `enp` (e.g., `enp2s0`)
- Ignore interfaces like `lo` (loopback), `docker0`, or `br-...` (Docker bridges)

### Find the line that looks like this:
```
inet 10.0.0.22/24 ...
```

The part before the slash (`10.0.0.22`) is your **IP address**. Use this as a command line argument when connecting to Host 1. For example:

```json
zenoh_bridge_ros2dds -e tcp/<IP-address>:7447
zenoh_bridge_ros2dds -e tcp/10.0.0.22:7447
```

> **Tip:** If you’re unsure which interface is active, check which one shows an `inet` address in the `10.x.x.x` or `192.168.x.x` range and says `state UP`.

## Host 2 (Victus Laptop)
**1. Run Zenoh Bridge and Connect to Host 1**

Use the IP address retrieved from the above step. In this case, its 10.0.0.22.
   ``` bash
   cd $HOME/ZENOH/zenoh-plugin-ros2dds
   source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
   zenoh_bridge_ros2dds -c zenoh-bridge-vehicle1.json5 -e tcp/10.0.0.22:7447
   ```
## Host 3 (Nitro PC)
1. Run Zenoh Bridge and Connect to Host 1
     ``` bash
     cd $HOME/ZENOH/zenoh-plugin-ros2dds
     source $HOME/ZENOH/zenoh-plugin-ros2dds/install/setup.bash
     zenoh_bridge_ros2dds -c zenoh-bridge-vehicle2.json5 -e tcp/10.0.0.22:7447
     ```

## Step 5: Start the Automated Valet Parking Node
### Launch Script Sending Available Parking Spots to Autoware
```cmd
cd $HOME/Multi-AVP
source $HOME/autoware/install/setup.bash
source /opt/ros/humble/setup.bash
source env/bin/activate
python3 avp_sirc.py
```
This script can be run separately on each host. It subscribes to the empty parking spot ROS2 topic, takes the first parking spot from the list, sets a destination in that parking spot, and parks.


**[Include Picture Here]**
