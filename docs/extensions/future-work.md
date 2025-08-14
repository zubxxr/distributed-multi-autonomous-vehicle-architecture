## 4. Future Work & Extensions

This framework was designed to be extended into additional autonomous driving capabilities beyond the current scope. Below are some potential directions for expansion.

### 4.1 Autonomous Valet Parking (AVP) — Implemented
This framework was already extended to develop a complete multi-vehicle AVP implementation, using namespaced Autoware stacks, Zenoh bridging, and parking goal execution.  
Source code and documentation: [Multi-Vehicle AVP Repository](https://github.com/zubxxr/multi-vehicle-avp)

### 4.2 Adaptive Cruise Control (ACC)
Enable vehicles to maintain safe distances from others by automatically adjusting their speed based on a detected lead vehicle.

### 4.3 Cooperative Maneuvers
Implement coordinated behaviors such as merging, overtaking, and intersection handling, where vehicles communicate to negotiate priority and avoid conflicts.

### 4.4 Occlusion Handling
Integrate strategies to detect and address occluded objects, using multiple viewpoints or predictive modeling to reduce blind spots and improve safety in complex environments.

### 4.5 Distributed Image Stitching & Sensor Fusion
Enable multiple vehicles or infrastructure-mounted cameras to share and combine visual or LiDAR data in real-time, creating a unified, high-coverage perception view that enhances detection accuracy and environmental awareness.

### 4.6 Additional Ideas
Further potential extensions, implementation strategies, and detailed discussions can be found in the *Future Work* section of the thesis paper. *(Reference: Thesis Section X.X)*
