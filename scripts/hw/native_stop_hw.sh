#!/bin/bash
# HELIX native stop test on the GO2, ARMED: helix_native.launch.py with the C++
# sink armed, supervisor actuation enabled, arbiter on /helix_hw/*, then one
# native_stop_test.py stage. Records the sport request/response, robot state
# (/sportmodestate, /lowstate with foot forces), odometry, /wirelesscontroller
# and the HELIX topics. The 2026-10-17 runs in hardware_eval_20261017/native_stop
# predate the last two (/lowstate, /wirelesscontroller).
# Usage: native_stop_hw.sh <build dir> <ack|motion> [native_stop_test.py args]
# motion: robot standing, area clear, operator on the remote.
REPO=$1; STAGE=$2; shift 2
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source "$REPO/install/setup.bash"
# after sourcing: the ROS setup scripts read unset variables
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
export PYTHONUNBUFFERED=1
D=~/helix_hw/$(date -u +%Y%m%dT%H%M%S)_stop_$STAGE
mkdir -p "$D"; cd "$D"
exec > >(tee -a run.log) 2>&1
echo "start $(date -u +%FT%TZ) sha=$(cat "$REPO/GIT_SHA") stage=$STAGE args=$*"
ros2 daemon stop >/dev/null 2>&1
SHARE=$(ros2 pkg prefix helix_arbiter)/share/helix_arbiter
setsid ros2 bag record -s sqlite3 -o bag /api/sport/request /api/sport/response /sportmodestate \
  /lf/sportmodestate /utlidar/robot_odom /helix/faults /helix/fault_clear /helix/hold \
  /helix/supervisor/status /helix/sink/status /helix/arbiter/status /helix_hw/cmd_vel \
  /helix_hw/nav/cmd_vel /wirelesscontroller /lowstate > bag.log 2>&1 &
TB=$!
setsid ros2 launch helix_bringup helix_native.launch.py sink_mode:=armed actuation_enabled:=true \
  arbiter_config:="$SHARE/config/arbiter_hw.yaml" cmd_vel_out:=/helix_hw/cmd_vel > stack.log 2>&1 &
TL=$!
sleep 15
python3 "$REPO/scripts/hw/native_stop_test.py" --stage "$STAGE" --out "$D/evidence" "$@"
RC=$?
echo "stage rc=$RC"
kill -INT -$TL; sleep 4; kill -INT -$TB; sleep 3
kill -TERM -$TL -$TB 2>/dev/null
echo "DONE $(date -u +%FT%TZ) rc=$RC dir=$D"
exit $RC
