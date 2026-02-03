This section summarizes system-level measurements collected during two-host and three-host DMAVA execution. Memory availability, CPU utilization, and inter-host communication latency were monitored to characterize system behavior under distributed operation and to identify factors affecting scalability beyond two vehicles.

### Available System Memory for Two- and Three-Host Configurations

| Setup      | Host           | Available Memory |
|------------|----------------|------------------|
| Two-Host   | ROG Laptop     | 11.0 GiB         |
|            | Nitro PC       | 9.8 GiB          |
| Three-Host | ROG Laptop     | 8.0 GiB         |
|            | Nitro PC       | 8.3 GiB          |
|            | Victus Laptop  | 5.4 GiB          |


---

### CPU Utilization for Two-Host and Three-Host Configurations

| Configuration | Host           | Mean CPU (%) | Peak CPU (%) |
|---------------|----------------|--------------|--------------|
| Two-Host      | ROG Laptop     | 57           | < 90         |
|               | Nitro PC       | 44           | < 50         |
| Three-Host    | ROG Laptop     | 67           | < 90         |
|               | Nitro PC       | 50           | < 55         |
|               | Victus Laptop  | 42           | < 60         |


### RTT Measurements During Three-Host Active Operation Under Dedicated Local Access Point

| Configuration        | RTT (ms)        | Max RTT (ms) | Samples |
|----------------------|-----------------|--------------|---------|
| Two-Host             | 41.03 ± 61.16   | 352.49       | 1134    |
| Three-Host (ROG)     | 61.26 ± 87.95   | 625.10       | 1430    |
| Three-Host (Victus)  | 32.5 ± 44.64    | 309.41       | 1481    |


### Observations

During three-host operation, a delayed start was observed for the vehicle running on the Victus laptop, while the ROG and Nitro hosts executed without noticeable startup lag. This behavior did not occur in any two-host configuration.

Analysis of system metrics showed that memory availability remained sufficient across all hosts, with no swapping observed, and CPU utilization stayed within stable operating limits throughout execution. These results indicate that neither memory nor CPU constraints were the primary contributors to the observed delay.

Further investigation identified inter-host communication behavior as the dominant factor. Under three-host execution, network contention and instability affected message delivery over the Zenoh bridge, leading to intermittent delays despite adequate computational resources on each host. This highlights that network stability and contention characteristics, rather than nominal CPU load or memory availability, are the critical factors influencing reliable multi-host execution in DMAVA.

---




