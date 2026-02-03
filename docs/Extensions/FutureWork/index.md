This architecture has the potential to be extended into additional autonomous driving capabilities beyond the current AVP implementation. Below are some possible directions for future expansion.

### Autonomous Valet Parking (AVP) — Implemented
This architecture was already extended to develop a complete multi-vehicle AVP implementation, using namespaced Autoware stacks, Zenoh bridging, and parking goal execution.  

See [Multi-Vehicle AVP](https://zubxxr.github.io/multi-vehicle-avp).

### Adaptive Cruise Control (ACC)
Enable vehicles to maintain safe distances from others by automatically adjusting their speed based on a detected lead vehicle.

### Cooperative Maneuvers
Implement coordinated behaviors such as merging, overtaking, and intersection handling, where vehicles communicate to negotiate priority and avoid conflicts.

### Occlusion Handling
Integrate strategies to detect and address occluded objects, using multiple viewpoints or predictive modeling to reduce blind spots and improve safety in complex environments.

### Distributed Image Stitching & Sensor Fusion
Enable multiple vehicles or infrastructure-mounted cameras to share and combine visual or LiDAR data in real-time, creating a unified, high-coverage perception view that enhances detection accuracy and environmental awareness.

### Additional Directions

- **Improved network robustness and transport tuning**, including exploration of alternative Zenoh configurations, transport backends, and message scheduling strategies to reduce timing variability under multi-host operation.  
- **High-bandwidth sensor distribution**, such as sharing camera or LiDAR streams across hosts to enable distributed perception and infrastructure-assisted sensing.  
- **Cloud or edge-assisted execution**, where selected components are offloaded to external compute resources to reduce per-host load and improve scalability.  
- **Deployment on embedded robotic platforms**, enabling validation under real-world wireless conditions and hardware constraints.  
- **Scalability beyond three hosts**, using dedicated networking infrastructure or more homogeneous compute platforms to evaluate behavior under higher vehicle counts.  


