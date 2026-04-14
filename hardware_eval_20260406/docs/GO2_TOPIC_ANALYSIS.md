# GO2 Topic Gap Analysis for HELIX

## Data Provenance

All data was captured on **2026-04-02** from a live Unitree GO2 robot.

- **Observer**: Jetson Orin NX at `192.168.123.18` (Ubuntu 22.04, kernel 5.15.148-tegra)
- **Robot**: GO2 at `192.168.123.161`
- **RMW implementation**: FastRTPS (default)
- **ROS 2 distribution**: Humble (inferred from package versions)
- **Source files**: `ros2_topic_list.txt`, `topic_info.txt`, `system_info.txt`, `helix_relevant_bag/`

This is real hardware data, not simulation output.

## Topic Landscape

The GO2 publishes **121 active ROS 2 topics** (122 lines in the topic list, one blank). They fall into the following categories:

### Robot State and Control (19 topics)

Low-level and high-level robot state, motor commands, and odometry.

| Topic | Type | Notes |
|-------|------|-------|
| `/sportmodestate` | `unitree_go/msg/SportModeState` | Custom type |
| `/lowstate` | `unitree_go/msg/LowState` | Custom type |
| `/lf/lowstate` | (custom) | Filtered variant |
| `/lf/sportmodestate` | (custom) | Filtered variant |
| `/lowcmd` | (custom) | Motor commands |
| `/imu` | `go2_interfaces/msg/IMU` | Custom type |
| `/joint_states` | `sensor_msgs/msg/JointState` | Standard type |
| `/odom` | `nav_msgs/msg/Odometry` | Standard type |
| `/go2_states` | (custom) | Aggregate state |
| `/multiplestate` | (custom) | Multi-robot state |
| `/cmd_vel_out` | (likely `geometry_msgs/msg/Twist`) | Velocity output |
| `/joy` | (likely `sensor_msgs/msg/Joy`) | Joystick input |
| `/arm_Command` | (custom) | Arm control |
| `/arm_Feedback` | (custom) | Arm feedback |
| `/selftest` | (custom) | Self-test status |
| `/servicestate` | (custom) | Service state |
| `/servicestateactivate` | (custom) | Service activation |
| `/wirelesscontroller` | (custom) | Controller input |
| `/wirelesscontroller_unprocessed` | (custom) | Raw controller input |

### Perception (16 topics)

Camera, LiDAR, and point cloud data.

| Topic | Notes |
|-------|-------|
| `/camera/camera_info` | Camera intrinsics |
| `/camera/image_raw` | Raw camera image |
| `/frontvideostream` | Front video stream |
| `/point_cloud2` | Fused point cloud |
| `/utlidar/cloud` | LiDAR cloud |
| `/utlidar/cloud_base` | Base frame cloud |
| `/utlidar/cloud_deskewed` | Motion-compensated cloud |
| `/utlidar/grid_map` | 2D grid map |
| `/utlidar/height_map` | Height map |
| `/utlidar/height_map_array` | Height map array |
| `/utlidar/imu` | LiDAR IMU |
| `/utlidar/lidar_state` | LiDAR health |
| `/utlidar/range_info` | Range data |
| `/utlidar/range_map` | Range map |
| `/utlidar/voxel_map` | Voxel map |
| `/utlidar/voxel_map_compressed` | Compressed voxel map |

### Navigation and SLAM (10 topics)

Mapping, localization, and path planning.

| Topic | Notes |
|-------|-------|
| `/uslam/client_command` | SLAM commands |
| `/uslam/cloud_map` | SLAM map |
| `/uslam/frontend/cloud_world_ds` | Downsampled world cloud |
| `/uslam/frontend/odom` | SLAM frontend odometry |
| `/uslam/localization/cloud_world` | Localization world cloud |
| `/uslam/localization/odom` | Localization odometry |
| `/uslam/navigation/global_path` | Global path |
| `/uslam/server_log` | SLAM server log |
| `/lio_sam_ros2/mapping/odometry` | LIO-SAM odometry |
| `/pctoimage_local` | Point cloud to image |

### API Services (34 topics)

Unitree proprietary request/response service topics (17 service pairs).

All follow the pattern `/api/<service>/request` and `/api/<service>/response`. Services include: `sport`, `sport_lease`, `motion_switcher`, `obstacles_avoid`, `robot_state`, `config`, `videohub`, `audiohub`, `vui`, `gpt`, `bashrunner`, `pet`, `fourg_agent`, `gas_sensor`, `gesture`, `programming_actuator`, `assistant_recorder`, `uwbswitch`, `rm_con`.

### Communication and Networking (10 topics)

WebRTC, audio, and network status.

| Topic |
|-------|
| `/audio/raw` |
| `/audiohub/player/state` |
| `/audioreceiver` |
| `/audiosender` |
| `/webrtc_req` |
| `/webrtcreq` |
| `/webrtcres` |
| `/xfk_webrtcreq` |
| `/xfk_webrtcres` |
| `/public_network_status` |

### Graph and Mapping Infrastructure (7 topics)

Graph-based mapping commands and queries.

| Topic |
|-------|
| `/qt_add_edge` |
| `/qt_add_node` |
| `/qt_command` |
| `/qt_notice` |
| `/query_result_edge` |
| `/query_result_node` |
| `/gpt_cmd` |

### System (5 topics)

Standard ROS 2 infrastructure.

| Topic | Type |
|-------|------|
| `/rosout` | `rcl_interfaces/msg/Log` |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` |
| `/tf` | `tf2_msgs/msg/TFMessage` |
| `/clock` | (not observed, but standard) |
| `/config_change_status` | (custom) |

### Other (remaining topics)

`/gas_sensor`, `/gesture/result`, `/gnss`, `/gptflowfeedback`, `/lf/battery_alarm`, `/pet/flowfeedback`, `/programming_actuator/command`, `/programming_actuator/feedback`, `/rtc/state`, `/rtc_status`, `/utlidar/client_cmd`, `/utlidar/mapping_cmd`, `/utlidar/robot_odom`, `/utlidar/robot_pose`, `/utlidar/server_log`, `/utlidar/switch`, `/uwbstate`, `/uwbswitch`, `/videohub/inner`.

## HELIX Input Mapping

HELIX subscribes to four topics. Their availability on the GO2:

| HELIX Topic | Message Type | GO2 Status | Source File |
|-------------|-------------|------------|-------------|
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | **NOT PUBLISHED** | `topic_info.txt`: "Unknown topic '/diagnostics'" |
| `/helix/heartbeat` | `std_msgs/msg/String` | **NOT PUBLISHED** | Not in `ros2_topic_list.txt` |
| `/helix/metrics` | `std_msgs/msg/Float64MultiArray` | **NOT PUBLISHED** | Not in `ros2_topic_list.txt` |
| `/rosout` | `rcl_interfaces/msg/Log` | **AVAILABLE** | `topic_info.txt`: 5 publishers, 0 subscribers |

**Result: 1 of 4 HELIX input channels has a data source on the GO2.**

- The `/rosout` subscription in `helix_core/log_parser.py` will receive data immediately.
- The `/diagnostics` subscription in `helix_core/anomaly_detector.py` will receive nothing.
- The `/helix/heartbeat` subscription in `helix_core/heartbeat_monitor.py` requires bridge nodes that publish heartbeats on behalf of monitored GO2 nodes.
- The `/helix/metrics` subscription in `helix_core/anomaly_detector.py` requires adapter nodes that extract numeric values from GO2 state topics (e.g., battery voltage from `/lowstate`, joint temperatures from `/joint_states`).

## Custom Message Barrier

Several high-value GO2 topics use custom message types not available in standard ROS 2:

| Topic | Type | Impact |
|-------|------|--------|
| `/sportmodestate` | `unitree_go/msg/SportModeState` | Cannot deserialize without `unitree_go` package |
| `/lowstate` | `unitree_go/msg/LowState` | Cannot deserialize without `unitree_go` package |
| `/imu` | `go2_interfaces/msg/IMU` | Cannot deserialize without `go2_interfaces` package |

Without these packages installed:
- `ros2 topic echo` cannot display the data
- `ros2 bag record` cannot serialize the messages
- Any subscriber node will fail to create the subscription

The `unitree_go` and `go2_interfaces` packages must be built from Unitree's SDK sources and installed in the same ROS 2 workspace.

## Bag Capture Limitation

The bag file in `helix_relevant_bag/` contains only topics with standard message types:

- **Size**: 24 KB
- **Duration**: 7.2 seconds
- **Messages**: 13 total
- **Topics**: `/joint_states` (sensor_msgs/msg/JointState), `/odom` (nav_msgs/msg/Odometry), `/rosout` (rcl_interfaces/msg/Log)

Topics using `unitree_go` and `go2_interfaces` types were excluded because `ros2 bag record` cannot serialize messages without the type support libraries installed.

## Conclusion

Of the four HELIX input channels, only `/rosout` is natively available on the GO2. The remaining three (`/diagnostics`, `/helix/heartbeat`, `/helix/metrics`) require purpose-built bridge or adapter nodes.

Deploying HELIX on the GO2 requires:

1. **Installing Unitree message packages** (`unitree_go`, `go2_interfaces`) to access custom-typed topics.
2. **Building adapter nodes** to translate GO2 state data into HELIX's expected `/helix/metrics` format.
3. **Building heartbeat publishers** for each GO2 node that HELIX should monitor.
4. **Creating hardware-specific detection rules** in the log parser and anomaly detector for GO2 fault patterns.

This is an engineering integration task, not a fundamental architectural incompatibility. The GO2 publishes rich state and sensor data; HELIX's monitoring architecture can consume it once the bridging layer exists.
