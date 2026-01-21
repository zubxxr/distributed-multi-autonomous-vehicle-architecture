# DMAVA RTT Tools

Lightweight ROS 2 tools for measuring **round-trip time (RTT)** between distributed hosts in a DMAVA deployment.  
These tools are used to evaluate inter-host communication latency via Zenoh during multi-host operation.

---

## Overview

The RTT measurement follows a **ping–pong model**:

- **Host 1 (sender)** publishes timestamped RTT probe messages
- **Remote hosts (receivers)** immediately echo the message back
- RTT is computed on Host 1 using a single time reference

This avoids assumptions about clock synchronization between hosts.

---

## Nodes

- **RTT Ping Node**  
  Publishes RTT probe messages and computes latency statistics.

- **RTT Pong Node**  
  Subscribes to RTT probes and immediately echoes them back.

---

## Execution Order

### 1. Start Zenoh on all hosts

Ensure Zenoh bridges are running and ROS 2 topic discovery is functional across hosts.

---

### 2. Launch pong nodes on remote hosts first

On **each remote host** (e.g., vehicle2, vehicle3):

```bash
ros2 run rtt_test rtt_pong
```

If using namespaces:

```bash
ros2 run rtt_test rtt_pong --ros-args -r __ns:=/vehicle2
```

Repeat this step for each additional remote host, using the appropriate namespace (e.g., `/vehicle3`).

---

### 3. Launch ping node on Host 1

On **Host 1** (the RTT origin):

```bash
ros2 run rtt_test rtt_ping
```

The ping node publishes RTT probes at a fixed rate and computes round-trip latency statistics based on echoed responses.

---

## Output Metrics

The ping node reports the following metrics:

- **RTT (ms)** reported as mean ± standard deviation  
- **Maximum RTT (ms)**  
- **Sample count**

The standard deviation represents latency jitter.

---

## Namespaces

The RTT tools are namespace-aware and are intended to operate under the same vehicle namespaces used by DMAVA.

Example namespaces:
- `/vehicle1`
- `/vehicle2`
- `/vehicle3`

Each pong node responds within its own namespace, allowing RTT to be measured independently for each remote host.

---

### Optional: Logging RTT to File

The ping node supports logging RTT measurements to a CSV file for offline analysis.

Example:

~~~bash
ros2 run rtt_test ping --ros-args \
  -p namespace:=vehicle2 \
  -p output_file:=rtt_phase1_baseline.csv
~~~

This will write RTT samples to the specified file on Host 1, including:
- Timestamp
- RTT (ms)
- Sequence ID

The output file can be used for post-processing and visualization of latency statistics.

## Notes

- RTT measurements are end-to-end and include middleware, serialization, and transport overhead.
- All RTT computation is performed on Host 1 using a single time reference.
- No clock synchronization between hosts is assumed.
- These tools are intended for evaluation and diagnostics, not for real-time enforcement.
