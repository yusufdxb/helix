#!/usr/bin/env bash
# Live-path smoke test for helix_monitor_node: replays real GO2 bags through DDS into the node.
# Isolated: localhost-only DDS on a private domain, and the replayed /api/sport/request is remapped
# so no robot on any network can receive it. The node itself has no command publisher.
# usage: eval/node_smoke.sh <clean_bag> <native_stop_bag>   (needs the unitree_ros2 overlay sourced)
set -u
NODE=${NODE:-$(ros2 pkg prefix helix_monitor)/lib/helix_monitor/helix_monitor_node}
stop_node() { kill -INT "$1" 2>/dev/null; for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$1" 2>/dev/null; }
CLEAN=$1; STOP=$2; OUT=${OUT:-$(mktemp -d)}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
REQ=/helix_smoke/api/sport/request
run() {  # name, bag args...
  local name=$1; shift
  "$NODE" --ros-args -r /api/sport/request:=$REQ >"$OUT/$name.log" 2>&1 &
  local pid=$!; sleep 2
  ros2 bag play "$@" >/dev/null 2>&1
  sleep 3; stop_node $pid; wait $pid 2>/dev/null
  printf '%-8s raised=%s cleared-or-info=%s stop_verdicts=[%s]\n' "$name" \
    "$(grep -c ' raised' "$OUT/$name.log")" "$(grep -c 'INFO' "$OUT/$name.log")" \
    "$(grep -o 'stop at [0-9.]*: [A-Z]*' "$OUT/$name.log" | sed 's/.*: //' | tr '\n' ' ')"
  grep ' raised' "$OUT/$name.log" | sed 's/^.*\] //' | sort | uniq -c | head -10
}
run clean "$CLEAN" --topics /utlidar/imu /sportmodestate /lowstate
run stop "$STOP" --topics /sportmodestate /api/sport/request --remap /api/sport/request:=$REQ
# Positive control: a constant SportModeState payload at 300 Hz must raise frozen and stamp on sport.
"$NODE" >"$OUT/frozen.log" 2>&1 &
pid=$!; sleep 2
timeout 3 ros2 topic pub -r 300 /sportmodestate unitree_go/msg/SportModeState '{}' >/dev/null 2>&1
sleep 1; stop_node $pid; wait $pid 2>/dev/null
printf 'frozen   raised: %s\n' "$(grep ' raised' "$OUT/frozen.log" | sed 's/^.*\] //' | sed 's/ (value.*//' | sort -u | tr '\n' ';')"
echo "logs: $OUT"
