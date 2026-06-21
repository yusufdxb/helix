#!/usr/bin/env bash
pkill -f helix_ 2>/dev/null
pkill -f 'ros2 bag record' 2>/dev/null
sleep 3
rm -rf /tmp/helix_t6 /tmp/helix_t6.log /tmp/helix_t6_done /tmp/helix_t6_bag
nohup setsid bash -c '
  source /opt/ros/humble/setup.bash
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
  source ~/helix_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
  export PYTHONUNBUFFERED=1
  # Launch HELIX (renames OUT to /tmp/helix_t6 via env override is not supported by script,
  # so we run script verbatim and copy out at the end). Use 720s duration.
  DURATION_S=720 SAMPLE_PERIOD_S=15 /tmp/run_jetson_persistent_30min_v3.sh > /tmp/helix_t6_main.log 2>&1
  # After main script ends, mark done.
  mv /tmp/helix_t3 /tmp/helix_t6 2>/dev/null
  touch /tmp/helix_t6_done
' </dev/null >/dev/null 2>&1 &
disown
echo "T6_HELIX_LAUNCHED at $(date)"
exit 0
