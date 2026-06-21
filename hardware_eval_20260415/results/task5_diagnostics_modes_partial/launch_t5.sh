#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
export PYTHONUNBUFFERED=1
ros2 daemon stop >/dev/null 2>&1; sleep 2; ros2 daemon start >/dev/null 2>&1; sleep 4
rm -rf /tmp/helix_t5 /tmp/helix_t5_done
nohup setsid bash -c 'python3 /tmp/task5_diagnostics_modes.py > /tmp/helix_t5.log 2>&1; touch /tmp/helix_t5_done' </dev/null >/dev/null 2>&1 &
disown
echo "T5_LAUNCHED at $(date)"
exit 0
