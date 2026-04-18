# raw_vs_cpp_rate_monitor — mewtwo 2026-04-17

Head-to-head CPU benchmark of the `helix_topic_rate_monitor` subscription
path at 250 Hz `sensor_msgs/Imu`. Three variants, same publisher
(`publisher_utlidar.py`), same BEST_EFFORT QoS, two 60 s trials per
variant, psutil sampling the subscriber process's CPU at 1 Hz.

Variants:
1. **Python cooked** — rclpy subscription, full message object construction
   per callback (`raw=False`).
2. **Python raw** — rclpy subscription with `raw=True`, callback receives
   serialized CDR bytes, skips Python message deserialization.
3. **C++** — `rclcpp_lifecycle::LifecycleNode` from the new
   `helix_adapter_cpp` package. Typed subscription, callback just records
   a monotonic timestamp into a `std::deque`.

## Results

| Variant | Trial A | Trial B | Mean | Δ vs cooked |
|---|---:|---:|---:|---:|
| Python cooked | 4.47% | 4.73% | **4.60%** | baseline |
| Python raw    | 3.58% | 3.53% | **3.56%** | −23% |
| C++           | 1.04% | 1.21% | **1.13%** | **−75%** |

All trials received 15014–15016 messages over 60.06 s (≈ 250.00 Hz) with
zero drops.

## Headline

- `raw=True` alone saves ~20% — deserialization isn't the dominant cost;
  most of it is rclpy executor / GIL overhead per dispatch.
- C++ is **4× cheaper than Python cooked**, **3.1× cheaper than Python raw**.
- Extrapolated to Jetson Session 7 numbers (Python cooked IMU subscription
  ~42 pp of one A78AE core): a C++ equivalent should land near **~10 pp**
  on the Jetson, bringing the 6-node adapter from ~48 % to roughly **~16 %**
  of one core without dropping /utlidar/imu from the topics list.

## Reproducing

    cd ~/workspace/helix
    colcon build --packages-select helix_adapter_cpp --symlink-install
    source install/setup.bash
    experiments/raw_vs_cpp_rate_monitor/run_cpp_vs_py.sh

Uses `ROS_DOMAIN_ID=99` to stay isolated from any live HELIX / GO2 stack.

## Caveats

- Mewtwo Ryzen is much faster per-core than Jetson A78AE. The
  C++/Python ratio may not be constant across architectures. Needs a
  real-hardware re-run on the Jetson to confirm the extrapolation.
- One active subscription tested; the full deployment has six, five of
  them low-rate. Total C++ 6-node cost on Jetson needs direct
  measurement during a future lab session.
- Publisher is a synthetic in-process timer, not the GO2 utlidar driver.
  DDS path characteristics (middleware, QoS deadline, actual timing
  jitter) on hardware will differ.
