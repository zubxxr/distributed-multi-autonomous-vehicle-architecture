# Scaling to More Vehicles/Hosts

The framework can be extended beyond two vehicles by adding more hosts and namespaces. However, during testing with the available machines, certain issues were encountered that impacted performance.  

Despite these constraints, the framework remained fully operational. Instructions are given below, as well as an example using a third host.

## General Scaling Procedure

To extend the system, replicate the following for each additional vehicle/host:

1. Install **Autoware** and **Zenoh** on the new host.
2. Add a new ego vehicle in AWSIM Labs with a **unique namespace** (e.g., `/vehicle3`, `/vehicle4`, etc.).
3. Update Zenoh configuration to bridge the new namespace.
4. Connect the host to **Host 1** via Zenoh.

---

## Scalability Limitations

- Requires multiple physical machines (one Autoware instance per vehicle)
- Increasing the number of vehicles increases memory demands across the network and all machines.

---

## Third Host Example

To evaluate the scalability of the proposed architecture, a **third host (Victus Laptop)** was introduced alongside the existing **Nitro** and **ROG** hosts.

### Hardware Specification of Third Host

| **Host**   | **Model**                     | **CPU**              | **GPU**                 | **RAM** | **OS**        | **NVIDIA Driver** |
|------------|-------------------------------|----------------------|-------------------------|---------|---------------|-------------------|
| Victus Laptop   | HP Victus 15-fa1xxx            | Intel Core i5-12500H | GeForce RTX 4050        | 16 GB   | Ubuntu 22.04  | 575               |


A **third ego vehicle** was also added to the Unity simulation.

**Figure 1:** *Vehicle3 setup in Unity* (placeholder)

---

### Configuration Steps

1. Added a new ego vehicle GameObject in Unity, with all published topics namespaced under `/vehicle3`.
2. Created a dedicated clock publisher at `/vehicle3/clock` via the `Vehicle3_ClockPublisher` script.

   **Figure 2:** *Parking spot publisher for Vehicle 3* (placeholder)

3. Modified the UI camera bridge (`UICameraBridge` script) to adjust coordinate offsets for Vehicle 3 visualization.

   **Figure 3:** *Camera adjustments for Vehicle 3 visualization* (placeholder)

---

### Results

After launching the simulation, all three vehicles were visible and initialized.

**Figure 4:** *Simulation started with all three vehicles active* (placeholder)

Each vehicle localized successfully:

- **Figure 5:** Vehicle 1 Localized (Nitro PC) *(placeholder)*
- **Figure 6:** Vehicle 2 Localized (ROG Laptop) *(placeholder)*
- **Figure 7:** Vehicle 3 Localized (Victus Laptop) *(placeholder)*

---


### DO A TEST

### Remaining Issue: Emergency State on Victus

- Host 3’s vehicle remained in an **emergency state** despite restored communications.
- Likely cause: **system memory constraints**.
- *Table 7.1* from the thesis showed that Victus (16 GB RAM) had far less free memory than Nitro/ROG (24 GB each), even with 32 GB swap configured.

| Setup       | Host        | Free Memory After Launch |
|-------------|-------------|--------------------------|
| Three-Host  | ROG Laptop  | 500 MB                   |
|             | Nitro PC    | 833 MB                   |
|             | Victus      | 316 MB                   |
| Two-Host    | ROG Laptop  | 820 MB                   |
|             | Nitro PC    | 2.0 GB                   |

This strongly suggests low available RAM on Victus caused the emergency-state failure.

---

## Communication Optimization

Before this test, all hosts ran Zenoh in **router mode**.  
- Occasionally, Host 2 would connect but fail to localize.  
- Adding a third host made this failure more frequent.

**Optimization:**  
- Keep **Host 1** in **router mode**.  
- Switch **Host 2** and **Host 3** to **client mode**.  

This change:
- Resolved the rare connection failures.
- Achieved **100% reliable message delivery** for all three vehicles.
- Was permanently adopted in the final AVP architecture.

---
