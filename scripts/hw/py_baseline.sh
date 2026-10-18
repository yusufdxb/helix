#!/bin/bash
# HELIX Python-stack cost baseline on the GO2 payload, non-actuating: arbiter
# output and sources on /helix_dry/*, recovery active but enabled=false, sport
# sink in dry_run (no /api/sport/request publisher). Produced
# hardware_eval_20261017/pybaseline (build 3f648a8).
# Usage: py_baseline.sh <build dir> <seconds>
REPO=$1
SECS=${2:-1800}
TOOLS=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source "$REPO/install/setup.bash"
# after sourcing: the ROS setup scripts read unset variables
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
export PYTHONUNBUFFERED=1
D=~/helix_hw/$(date -u +%Y%m%dT%H%M)_pybaseline
mkdir -p "$D"; cd "$D"
exec > >(tee -a run.log) 2>&1
echo "start $(date -u +%FT%TZ) sha=$(git -C "$REPO" rev-parse --short HEAD) dirty=$(git -C "$REPO" status --porcelain | wc -l) secs=$SECS"
env | grep -E "ROS_DOMAIN_ID|RMW_IMPLEMENTATION" > env.txt
ros2 daemon stop >/dev/null 2>&1
SHARE=$(ros2 pkg prefix helix_arbiter)/share/helix_arbiter
setsid ros2 launch helix_bringup helix_closedloop.launch.py \
  auto_activate_recovery:=true recovery_enabled:=false enable_twist_mux:=false \
  arbiter_config:="$SHARE/config/arbiter_dry.yaml" cmd_vel_out:=/helix_dry/cmd_vel > stack.log 2>&1 &
T1=$!
setsid ros2 run helix_arbiter helix_go2_sport_sink --ros-args -p mode:=dry_run \
  -p input_topic:=/helix_dry/cmd_vel > sink.log 2>&1 &
T2=$!
sleep 20
for n in helix_heartbeat_monitor helix_anomaly_detector helix_log_parser helix_topic_rate_monitor \
    helix_json_state_parser helix_pose_drift_monitor helix_context_buffer helix_diagnosis_node \
    helix_recovery_node helix_arbiter; do
  echo "$n $(ros2 lifecycle get /$n 2>&1)" >> lifecycle_state.txt
done
cat lifecycle_state.txt
setsid ros2 bag record -s sqlite3 -o bag /helix/metrics /helix/faults /helix/recovery_hints \
  /helix/recovery_actions /helix/hold /helix/node_health > bag.log 2>&1 &
T3=$!
python3 "$TOOLS/proc_sampler.py" --out procs.jsonl --duration "$SECS" --period 5 \
  --warmup 30 --match 'helix_' &
T4=$!
( while kill -0 $T4 2>/dev/null; do
    echo "$(date -u +%T) $(cut -d' ' -f1-3 /proc/loadavg) $(cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | head -1)"
    sleep 30
  done ) > load.txt &
wait $T4
tail -1 procs.jsonl
kill -INT -$T3; sleep 3; kill -INT -$T2; sleep 2; kill -INT -$T1; sleep 8
kill -TERM -$T1 -$T2 -$T3 2>/dev/null
echo "DONE $(date -u +%FT%TZ)"
