#!/usr/bin/env bash
# Off-robot rehearsal of the GO2 hardware stages A-F (docs/HW_MOTION_TEST.md).
#
# Runs the SAME commands as the hardware procedure, with helix_fake_go2 in
# place of the robot (it speaks the real unitree_api Request/Response types).
# Every evidence file is labelled REHEARSAL; the stage runner refuses to let a
# rehearsal session unlock a hardware stage. This is NOT hardware evidence.
#
# Needs: helix workspace built, and a unitree_ros2 workspace providing
# unitree_api (UNITREE_WS, default ~/workspace/unitree_ros2/cyclonedds_ws).
set -uo pipefail
HELIX=$(cd "$(dirname "$0")/.." && pwd)
UNITREE_WS=${UNITREE_WS:-$HOME/workspace/unitree_ros2/cyclonedds_ws}
SESSION=${1:-$HELIX/results/hw_rehearsal_$(date +%Y%m%d_%H%M%S)}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-85}
set +u
source /opt/ros/humble/setup.bash
source "$UNITREE_WS/install/setup.bash"
source "$HELIX/install/setup.bash"
set -u
mkdir -p "$SESSION/logs"
PIDS=()
start() { local name=$1; shift; setsid "$@" >"$SESSION/logs/$name.log" 2>&1 & PIDS+=($!); echo "$!" >"$SESSION/logs/$name.pid"; }
stop_pid() { local p; p=$(cat "$SESSION/logs/$1.pid"); kill -INT -- "-$p" 2>/dev/null; sleep 1.5; kill -KILL -- "-$p" 2>/dev/null; true; }
cleanup() { for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done; sleep 2; for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done; }
trap cleanup EXIT

start fake_go2 ros2 run helix_arbiter helix_fake_go2
start stack ros2 launch helix_bringup helix_closedloop.launch.py \
      auto_activate_recovery:=true recovery_enabled:=true
sink() { stop_pid sink 2>/dev/null; start sink ros2 run helix_arbiter helix_go2_sport_sink --ros-args -p mode:="$1"; sleep 3; }
stage() {
  local s=$1 phrase=$2
  echo "=================== STAGE $s ==================="
  ros2 run helix_arbiter helix_hw_stage --stage "$s" --session-dir "$SESSION" \
      --repo "$HELIX" --rehearsal --confirm "$phrase"
  local rc=$?
  if [ $rc -ne 0 ]; then echo "STAGE $s did not PASS (rc=$rc); stopping. No later stage runs."; exit $rc; fi
}
sleep 8
start sink ros2 run helix_arbiter helix_go2_sport_sink --ros-args -p mode:=dry_run; sleep 3
stage A "MOTORS OFF"
sink stop_only;  stage B "ZERO ONLY"
sink dry_run;    stage C "ROBOT STAGED"
sink armed;      stage D "AREA CLEAR MOVE"
stage E "AREA CLEAR FAULT"
stage F "OPERATOR REARM"
echo "REHEARSAL COMPLETE: $SESSION"
