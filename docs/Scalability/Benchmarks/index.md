To evaluate the performance of running three vehicles across separate hosts, memory usage was recorded using `free -h` during execution. The results provide insight into system load and limitations when scaling beyond two vehicles.

### Raw Memory Snapshots (Three-Host Setup)

**Nitro PC (Host 1)**  

```bash
ovin@ovin-Nitro-N50-640:~$ free -h
                total        used        free      shared  buff/cache   available
Mem:            23Gi        14Gi       721Mi       130Mi       8.0Gi       8.2Gi
Swap:           31Gi          0B        31Gi
```

**ROG Laptop (Host 2)**  

```bash
zubair@zubair-ROG-Zephyrus-G15:~$ free -h
                total        used        free      shared  buff/cache   available
Mem:            22Gi        11Gi       4.6Gi       147Mi       6.4Gi        11Gi
Swap:           31Gi          0B        31Gi
```

**Victus Laptop (Host 3)**  

```bash
zubair@AVLab:~$ free -h
                total        used        free      shared  buff/cache   available
Mem:            15Gi       9.0Gi       209Mi       523Mi       6.1Gi       5.4Gi
Swap:           31Gi       0.0Ki        31Gi
```

---

### Condensed Comparison (Three-Host Setup)

| Host          | Total RAM | Used RAM | **Free RAM** | Available RAM | 
|---------------|-----------|----------|--------------|---------------|
| Nitro PC      | 23 Gi     | 14 Gi    | **721 Mi**   | 8.2 Gi        |
| ROG Laptop    | 22 Gi     | 11 Gi    | **4.6 Gi**   | 11 Gi         |
| Victus Laptop | 15 Gi     | 9.0 Gi   | **209 Mi**   | 5.4 Gi        |

---

### Observations

The **Victus Laptop** host showed significant limitations:  

  - It delayed vehicle execution by ~30 seconds before the car began driving.  
  - This lag occurred after **Vehicle 1 had already completed its goal** and **Vehicle 2 was only ~4 seconds from finishing**.  
  - The **low free memory (209 Mi)** and the **smallest total memory (15 Gi)** among the three hosts suggest that resource constraints were the key bottleneck.   

Both the **ROG** and **Nitro** machines executed without noticeable startup lag, maintaining smoother operation.  

Two-host, two-vehicle scenarios never exhibited these delays, confirming that scalability issues begin to emerge with three vehicles across distributed hosts. To better illustrate this contrast, baseline memory snapshots from the Nitro and ROG hosts during two-vehicle execution are provided below.  

---


### Baseline Comparison (Two-Host Setup)

**Nitro PC (Host 1)**  

```bash
ovin@ovin-Nitro-N50-640:~$ free -h
               total        used        free      shared  buff/cache   available
Mem:           23Gi        13Gi       4.5Gi       108Mi       5.7Gi       9.7Gi
Swap:          31Gi          0B       31Gi
```

**ROG Laptop (Host 2)**  

```bash
zubair@zubair-ROG-Zephyrus-G15:~$ free -h
               total        used        free      shared  buff/cache   available
Mem:           22Gi       9.5Gi       7.7Gi       110Mi       5.7Gi        13Gi
Swap:          31Gi          0B       31Gi
```


### Condensed Comparison (Two-Host Setup)

| Host          | Total RAM | Used RAM | **Free RAM** | Available RAM | 
|---------------|-----------|----------|--------------|---------------|
| Nitro PC      | 23 Gi     | 13 Gi    | **4.5 Gi**   | 9.7 Gi        |
| ROG Laptop    | 22 Gi     | 9.5 Gi   | **7.7 Gi**   | 13 Gi         |


## Summary Across Setups

To provide a high-level comparison, the following table consolidates free memory measurements across both the three-host and two-host experiments. This highlights how resource availability directly influenced system scalability.  

| Setup       | Host        | Free Memory After Launch |
|-------------|-------------|--------------------------|
| **Three-Host**  | ROG Laptop  | 4.6 GiB  |
|               | Nitro PC    | 721 MiB  |
|               | Victus      | 209 MiB  |
| **Two-Host**   | ROG Laptop  | 7.7 GiB  |
|               | Nitro PC    | 4.5 GiB  |