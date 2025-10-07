# rmp-eval

[![CI](https://github.com/roboticsys/rmp-eval/actions/workflows/ci.yml/badge.svg)](https://github.com/roboticsys/rmp-eval/actions/workflows/ci.yml)
[![CodeQL](https://github.com/roboticsys/rmp-eval/actions/workflows/codeql.yml/badge.svg)](https://github.com/roboticsys/rmp-eval/actions/workflows/codeql.yml)
[![Release](https://github.com/roboticsys/rmp-eval/actions/workflows/release.yml/badge.svg)](https://github.com/roboticsys/rmp-eval/actions/workflows/release.yml)

A Linux utility for evaluating PC hardware capability to run the [RMP EtherCAT Motion Controller](https://www.roboticsys.com/rmp-ethercat-motion-controller).

## Purpose

This tool helps determine if your [PC hardware meets the latency requirements](https://support.roboticsys.com/rmp/guide-pc-latency-jitter-bios.html) for running the RMP EtherCAT Motion Controller. RMP requires consistent real-time performance with specific latency thresholds that depend on your sample rate (e.g., 1000Hz requires <125µs latency).

Use this utility during the PC hardware evaluation phase to:

- Test cyclic timing performance of your isolated CPU
- Evaluate network interface card (NIC) performance with hardware timestamps
- Identify timing variability before deploying RMP

## Download

Pre-built packages are available to download for amd64 and arm64 from the [GitHub Releases](https://github.com/roboticsys/rmp-eval/releases) page.

**Debian/Ubuntu Installation:**

```bash
# Install
sudo dpkg -i rmp-eval_*.deb
```

## Usage

**Important:** This tool must be run as root to access raw sockets and set real-time thread priorities. A PREEMPT_RT kernel is required for RMP.

**Basic cyclic test (no NIC):**

```bash
sudo rmp-eval
```

**NIC test with EtherCAT drive (replace `enp2s0` with your NIC name):**

```bash
sudo rmp-eval --nic enp2s0
```

**Note:** The NIC must have an EtherCAT drive connected for this test.

Press `Ctrl+C` to stop the test and view final results.

## Example Output

```bash
sudo ./rmp-eval --nic enp2s0

CPU: 13th Gen Intel(R) Core(TM) i3-13100TE (4 logical, 4 physical)
Kernel: Linux 6.12.43+deb13-rt-amd64 #1 SMP PREEMPT_RT Debian 6.12.43-1 (2025-08-27) x86_64

System Checks
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PREEMPT_RT active                   ✔️    /sys/kernel/realtime=1
Swap disabled                       ✔️    /proc/swaps empty
Timer Migration disabled            ✔️    timer_migration=0
RT throttling disabled              ✔️    sched_rt_runtime_us=-1
Clocksource stable                  ✔️    tsc

Core 3 Checks
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
RT core isolated                    ✔️    isolated list: 3
nohz_full on RT core                ✔️    nohz_full list: 3
rcu_nocbs includes RT core          ✔️    3
CPU governor = performance          ✔️    governor=performance
CPU current frequency               ❌   cur=800000 kHz, min=800000 kHz, max=4100000 kHz
irqaffinity excludes RT core        ✔️    0,1
No unrelated IRQs on RT core        ✔️    clean
SMT sibling isolated/disabled       ✔️    no sibling
Deep C-states capped                ✔️    intel_idle.max_cstate=0
Turbo/boost disabled                ❌   intel_pstate/no_turbo=0

NIC enp2s0 Checks
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
NIC interface present               ✔️    exists
NIC link is UP                      ✔️    operstate=up
NIC is quiet                        ❌   v4=0, v6=1, def4=no, def6=no
NIC IRQs pinned to RT core          ✔️    all pinned to CPU3
RPS disabled on NIC                 ✔️    all zero masks

Target period: 1000 us

|          |       |  Great  |  Good   |  Poor   |   Bad    | Pathetic  | Max Latency  |
| Label    | Count | < 125us | < 250us | < 500us | < 1000us | >= 1000us |   us | index |
|----------+-------+---------+---------+---------+----------+-----------+------+-------+
| Sender   | 62367 |   62367 |       0 |       0 |        0 |         0 |    1 |   161 |
| Receiver | 62367 |   62367 |       0 |       0 |        0 |         0 |   10 | 61567 |
Duration: 00:01:02.445
  Sender max period: 1001µs at index 161 which is Great.
Receiver max period: 1010µs at index 61567 which is Great.
```

Columns:

- Label: Sender or Receiver thread (or Cyclic if no NIC)
- Count: Total observations (cycles)
- < 125us, < 250us, < 500us, < 1000us, >= 1000us: Latency deviation buckets
  - Colored (green → red) based on severity
  - Calculated as deviation from target period
- Max us: Worst-case latency deviation in microseconds (line 208: max - target)
- Max Index: Which iteration had the worst case

## Command-Line Options

```bash
./rmp-eval --help

Options:
--nic, -n                Network interface card name
--iterations, -i         Number of iterations
--send-sleep, -s         Send sleep duration in microseconds
--send-priority, -sp     Send thread priority
--receive-priority, -rp  Receive thread priority
--send-cpu, -sc          CPU core to use for the sender thread
--receive-cpu, -rc       CPU core to use for the receiver thread
--verbose, -v            Enable verbose output
--no-config, -nc         Skip system configuration checks
--bucket-width, -b       Bucket width in microseconds for counting occurrences.
--help, -h               Show this help message
--version                Show version information
```

## Building from Source

**Prerequisites:**

- CMake 3.15 or later
- GCC 12 or later
- C++20 support

**Build steps:**

```bash
# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_C_COMPILER=gcc-12 \
      -DCMAKE_CXX_COMPILER=g++-12

# Build
cmake --build build

# Run
sudo ./build/rmp-eval
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
