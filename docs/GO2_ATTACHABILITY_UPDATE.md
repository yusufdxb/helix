# GO2 Attachability Update — Hardware-Grounded

Based on hardware evidence collected 2026-04-03 from a live Unitree GO2 quadruped.

## Current State

HELIX subscribes to 4 input channels. On the GO2:

| Channel | Required Type | GO2 Status | Evidence |
|---------|--------------|------------|---------|
| `/diagnostics` | DiagnosticArray | NOT PUBLISHED | Confirmed via `ros2 topic info /diagnostics` — "Unknown topic" |
| `/helix/heartbeat` | String | NOT PUBLISHED | Not in topic list (103 topics enumerated) |
| `/helix/metrics` | Float64MultiArray | NOT PUBLISHED | Not in topic list |
| `/rosout` | Log | AVAILABLE | 5+ publishers observed; messages captured in bags |

**1 of 4 channels operational. HELIX in current form cannot observe meaningful GO2 state.**

## Why This Gap Exists

HELIX was designed for a generic ROS 2 graph where:
- Nodes publish heartbeats on a shared topic
- Numeric metrics are available as Float64MultiArray
- The standard /diagnostics aggregation pipeline exists

The GO2 follows none of these conventions:
- Nodes are internal to the robot's DDS domain; `ros2 node list` returns empty from external machines
- Numeric state data is published in custom Unitree message types (unitree_go/msg/LowState, SportModeState)
- No DiagnosticArray publisher exists
- Rich data exists but in non-standard formats (JSON strings, custom messages)

## What the GO2 Actually Offers (Measured)

### High-Rate Sensor Streams (Standard ROS 2 Types)
| Topic | Rate | Type | Monitoring Potential |
|-------|------|------|---------------------|
| /utlidar/imu | 250 Hz | sensor_msgs/Imu | Rate anomaly, drift detection |
| /utlidar/robot_odom | 150 Hz | nav_msgs/Odometry | Position/velocity drift |
| /utlidar/robot_pose | 18.7 Hz | geometry_msgs/PoseStamped | Pose tracking |
| /utlidar/cloud | 15.4 Hz | sensor_msgs/PointCloud2 | LiDAR health (point count) |
| /uslam/frontend/odom | Quiescent | nav_msgs/Odometry | SLAM health |

### Status Streams (JSON Strings)
| Topic | Rate | Sample Content |
|-------|------|---------------|
| /gnss | 1 Hz | `{"fixed":0,"hdop":0,"longitude":120.21,...,"satellite_total":13}` |
| /multiplestate | 1 Hz | `{"brightness":0,"obstaclesAvoidSwitch":true,"volume":7}` |
| /audiohub/player/state | 4 Hz | `{"play_state":"not_in_use","is_playing":false}` |
| /rtc/state | 1 Hz | RTC status |

### Custom-Type Streams (Require unitree_go Package)
| Topic | Rate | Content | Monitoring Value |
|-------|------|---------|------------------|
| /lowstate | ~500 Hz | Motor currents, joint temperatures, battery | HIGH — critical fault signals |
| /sportmodestate | ~500 Hz | Gait mode, foot forces, body state | HIGH — locomotion health |
| /lf/lowstate | ~50 Hz | Low-frequency variant | MODERATE |
| /lf/sportmodestate | ~50 Hz | Low-frequency variant | MODERATE |

## Bridging the Gap: Realistic Engineering Path

### Phase 1: Passive monitoring with zero GO2 changes (2-3 days)

1. **Topic rate monitor node** — Subscribe to IMU, odom, lidar topics. Compute 5-second rolling rate. Deviation >10% from baseline = fault event. This is the lowest-effort, highest-value addition.

2. **JSON state parser node** — Parse /multiplestate, /gnss, /servicestate strings. Extract numeric values. Publish as Float64MultiArray on /helix/metrics. HELIX anomaly detector works immediately.

3. **Pose drift monitor** — Subscribe to /utlidar/robot_pose. Compute rolling displacement rate. Unexpected acceleration/stationarity = fault signal.

### Phase 2: Custom message integration (1-2 weeks)

4. **Build unitree_go packages** on Jetson/PC from `~/go2_ws/src/unitree_ros2/`
5. **LowState adapter** — Extract battery voltage, joint temperatures, motor currents → /helix/metrics
6. **SportModeState adapter** — Extract gait mode transitions → log events on /rosout

### Phase 3: Full HELIX deployment (2-4 weeks)

7. **Heartbeat publisher per GO2 node** — Infer node liveness from topic activity
8. **Hardware-specific log rules** — Regex patterns for GO2-specific errors
9. **DDS-aware monitoring** — Participant discovery, QoS health
10. **On-device profiling** — Full HELIX stack on Jetson under thermal stress

## Design Implications for Paper

The gap between HELIX's current interfaces and the GO2's topic landscape is not an architectural failure — it reveals a design assumption:

> HELIX assumes the monitored graph follows ROS 2 conventions (/diagnostics, standard types, heartbeat protocols). Real robots like the GO2 use proprietary message types and non-standard communication patterns.

This motivates the paper's argument for **adapter-based fault sensing**: a monitoring layer that translates platform-specific state into generic observability channels. The GO2 hardware evidence shows this is feasible (rich data exists) but requires meaningful engineering work.

## Evidence Strength

| Claim | Evidence | Strength |
|-------|----------|----------|
| GO2 publishes rich ROS 2 data | 103 topics enumerated, 6 bags captured | Strong |
| Standard-type topics have stable rates | Two 30s captures, <1% rate variation | Moderate |
| Custom types carry critical fault data | Known from GO2 documentation + topic info | Moderate |
| HELIX can observe /rosout on GO2 | Bag captures include GO2-side /rosout messages | Strong |
| Adapter approach is feasible | JSON strings already parseable, IMU/odom available | Moderate |
| Full deployment works on hardware (≥30 min on Jetson vs live GO2) | EXP-15 (Session 5, 2026-04-15) | Strong |
| `/diagnostics` is not natively published — across all sport-API states under motion_switcher=normal | EXP-16 (Session 5) | Strong |
| LiDAR occlusion is invisible to HELIX without an adapter | EXP-17 inject 2 (Session 5) | Strong |
| USB device disconnect is invisible to HELIX without an adapter | EXP-17 inject 4 (Session 5) | Strong |
| Log-pattern detection works end-to-end on hardware | EXP-17 inject 1 (Session 5) | Strong |
| `/diagnostics` remains absent — re-verified clean in Session 6 (2026-04-16) post full HELIX teardown | Session 6 `attachability_matrix_clean_20260416.json` → 1/4 native | Strong |
| ANOMALY end-to-end on live GO2 — 60 real FaultEvents from the `helix_adapter` path | Session 6 (20 min) | Strong |

**Current reading:** native HELIX-input coverage on the GO2 is **1–2 / 4,
mode-dependent**. `/rosout` is always-on; `/diagnostics` has only ever been
observed under a specific `twist_mux` configuration in Session 1 (2026-04-03)
and has not been reproduced under `motion_switcher = normal` in Sessions 2,
5, or 6. The conservative reading for any claim written today is **1/4** until
`/diagnostics` is re-verified in `ai` / `advanced` modes. Session 7
(2026-04-17) did not re-probe attachability — it focused on adapter CPU /
stability — so the Session-6 1/4 reading stands.
