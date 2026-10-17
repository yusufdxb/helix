#!/bin/bash
# HELIX native runtime on the GO2 payload, non-actuating: helix_native.launch.py
# defaults (arbiter on /helix_dry/*, supervisor actuation disabled, C++ sink
# in dry_run so no /api/sport/request publisher exists). Records per-process
# CPU/RSS and the HELIX topics for the same comparison as the Python baseline.
# Usage: native_run.sh <build dir> <seconds> [label]
set -u
REPO=$1
SECS=${2:-600}
LABEL=${3:-native}
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source "$REPO/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
export PYTHONUNBUFFERED=1
D=~/helix_hw/$(date -u +%Y%m%dT%H%M)_$LABEL
mkdir -p "$D"; cd "$D"
exec > >(tee -a run.log) 2>&1
echo "start $(date -u +%FT%TZ) sha=$(cat "$REPO/GIT_SHA") secs=$SECS"
env | grep -E "CYCLONEDDS_URI|ROS_DOMAIN_ID|RMW_IMPLEMENTATION" > env.txt
ros2 daemon stop >/dev/null 2>&1
setsid ros2 launch helix_bringup helix_native.launch.py > stack.log 2>&1 &
T1=$!
sleep 20
for n in helix_topic_rate_monitor helix_anomaly_detector helix_log_parser helix_supervisor helix_arbiter; do
  echo "$n $(ros2 lifecycle get /$n 2>&1)" >> lifecycle_state.txt
done
cat lifecycle_state.txt
setsid ros2 bag record -s sqlite3 -o bag /helix/metrics /helix/faults /helix/fault_clear \
  /helix/recovery_hints /helix/recovery_actions /helix/hold /helix/supervisor/status \
  /helix/sink/status /helix/arbiter/status /helix/heartbeat > bag.log 2>&1 &
T3=$!
python3 "$REPO/scripts/hw/proc_sampler.py" --out procs.jsonl --duration "$SECS" --period 5 \
  --warmup 30 --match 'helix_' &
T4=$!
( while kill -0 $T4 2>/dev/null; do
    echo "$(date -u +%T) $(cut -d' ' -f1-3 /proc/loadavg) $(cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | head -1)"
    sleep 30
  done ) > load.txt &
wait $T4
tail -1 procs.jsonl
kill -INT -$T3; sleep 3; kill -INT -$T1; sleep 8
kill -TERM -$T1 -$T3 2>/dev/null
echo "DONE $(date -u +%FT%TZ)"
