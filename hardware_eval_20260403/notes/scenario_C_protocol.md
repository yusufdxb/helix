# Scenario C: Topic Throttling Observation (PC-Side Only)

## Protocol
**Objective**: Establish baseline topic rates and bandwidth for HELIX monitoring calibration.

**Safety classification**: OBSERVATION ONLY -- passive bag recording, no modifications.

### Steps Executed
1. Recorded 30-second bag of key standard-type topics from PC
2. Attempted `ros2 topic hz` and `ros2 topic bw` live measurements (failed due to DDS
   discovery latency -- commands did not produce output within 6-second timeouts)
3. Computed rates from bag message counts and duration

### Results: Baseline Topic Rates

| Topic | Type | Rate (Hz) | Messages/30s | Notes |
|-------|------|-----------|--------------|-------|
| /utlidar/imu | sensor_msgs/Imu | 249 | 9887-9918 | Highest rate, most sensitive indicator |
| /utlidar/robot_odom | nav_msgs/Odometry | 149-150 | 5921-5960 | Primary odometry |
| /utlidar/robot_pose | geometry_msgs/PoseStamped | 18.7 | 741-744 | SLAM output |
| /gnss | std_msgs/String | 1.0 | 40 | GPS status |
| /multiplestate | std_msgs/String | 1.0 | 39-40 | Robot state |
| /rosout | rcl_interfaces/Log | 0.4-0.5 | 16-21 | Log stream |
| /parameter_events | rcl_interfaces/ParameterEvent | 0.1 | 3 | Rare |

### Bandwidth Estimate
- Total bag size: 8.1-8.2 MiB for ~40 seconds
- Effective data rate: ~1.6 Mbps for standard-type topics only
- The full GO2 topic set (including PointCloud2, custom types) is much higher

### Rate Stability
Two independent 30-second captures showed:
- /utlidar/imu: 9918 vs 9887 msgs (0.3% variation)
- /utlidar/robot_odom: 5960 vs 5921 msgs (0.7% variation)
- /utlidar/robot_pose: 744 vs 741 msgs (0.4% variation)

This sub-1% variation establishes a reliable baseline for anomaly detection.

### Implications for HELIX
- HELIX should monitor /utlidar/imu rate (249 Hz) as the most sensitive indicator of
  LiDAR node health -- any significant rate drop would indicate a problem
- Rate anomaly thresholds: suggest >5% sustained deviation over 5 seconds as alert trigger
- `ros2 topic hz` is unreliable from external machines; bag-based or custom subscriber-based
  rate computation is more robust

### Evidence
- Bag: `bags/helix_topic_rates/`
- Sidecar: `bags/helix_topic_rates/metadata_sidecar.yaml`
