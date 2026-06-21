#!/usr/bin/env bash
# Task 3 — 30-min Jetson persistent deployment vs live GO2.
# Run on the Jetson. Outputs:
#   /tmp/helix_t3/launch.log
#   /tmp/helix_t3/tegrastats.log
#   /tmp/helix_t3/sampler.jsonl
#   /tmp/helix_t3/jetson_persistent_30min.json   (final)
DURATION_S=${DURATION_S:-1800}        # 30 minutes
SAMPLE_PERIOD_S=${SAMPLE_PERIOD_S:-10}
TEGRA_PERIOD_MS=${TEGRA_PERIOD_MS:-1000}
OUT=/tmp/helix_t3
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/helix_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml

# Refresh discovery
ros2 daemon stop >/dev/null 2>&1; sleep 2; ros2 daemon start >/dev/null 2>&1; sleep 5

# 1) Launch lifecycle nodes in background
nohup ros2 launch helix_bringup helix_sensing.launch.py \
  > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "launch_pid=$LAUNCH_PID" | tee "$OUT/pids.txt"

# Wait for nodes to register
sleep 12

# 2) Configure + activate the 3 lifecycle nodes
for n in helix_heartbeat_monitor helix_anomaly_detector helix_log_parser; do
  ros2 lifecycle set "/$n" configure >> "$OUT/lifecycle.log" 2>&1 || true
  sleep 1
  ros2 lifecycle set "/$n" activate  >> "$OUT/lifecycle.log" 2>&1 || true
  sleep 1
done

# Snapshot lifecycle states
{
  echo "--- lifecycle states at t0 ---"
  for n in helix_heartbeat_monitor helix_anomaly_detector helix_log_parser; do
    echo -n "$n: "; ros2 lifecycle get "/$n" 2>&1
  done
} >> "$OUT/lifecycle.log"

# 3) tegrastats in background
( tegrastats --interval "$TEGRA_PERIOD_MS" > "$OUT/tegrastats.log" 2>&1 ) &
TEGRA_PID=$!
echo "tegra_pid=$TEGRA_PID" >> "$OUT/pids.txt"

# 4) /helix/faults counter in background (count-only, no payload)
nohup bash -c "
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/helix_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
ros2 topic echo --no-arr --no-str /helix/faults helix_msgs/msg/FaultEvent 2>&1
" > "$OUT/faults_echo.log" 2>&1 &
FAULTS_PID=$!
echo "faults_pid=$FAULTS_PID" >> "$OUT/pids.txt"

# Also start a /helix/node_health monitor (count messages)
nohup bash -c "
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/helix_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
ros2 topic hz /helix/node_health 2>&1
" > "$OUT/node_health_hz.log" 2>&1 &
HEALTH_PID=$!
echo "health_pid=$HEALTH_PID" >> "$OUT/pids.txt"

# 5) Sampler loop: per-node RSS + CPU%, fault count, time
START_TS=$(date +%s)
END_TS=$(( START_TS + DURATION_S ))
> "$OUT/sampler.jsonl"

sample_one() {
  local ts elapsed
  ts=$(date +%s)
  elapsed=$(( ts - START_TS ))
  local hb_pid ad_pid lp_pid hb_rss ad_rss lp_rss hb_cpu ad_cpu lp_cpu
  hb_pid=$(pgrep -f "lib/helix_core/helix_heartbeat_monitor" | head -1)
  ad_pid=$(pgrep -f "lib/helix_core/helix_anomaly_detector"  | head -1)
  lp_pid=$(pgrep -f "lib/helix_core/helix_log_parser"        | head -1)
  hb_pid=${hb_pid:-0}; ad_pid=${ad_pid:-0}; lp_pid=${lp_pid:-0}

  read -r hb_rss hb_cpu < <(ps -p "$hb_pid" -o rss=,%cpu= 2>/dev/null | awk '{print $1, $2}')
  read -r ad_rss ad_cpu < <(ps -p "$ad_pid" -o rss=,%cpu= 2>/dev/null | awk '{print $1, $2}')
  read -r lp_rss lp_cpu < <(ps -p "$lp_pid" -o rss=,%cpu= 2>/dev/null | awk '{print $1, $2}')
  hb_rss=${hb_rss:-null}; ad_rss=${ad_rss:-null}; lp_rss=${lp_rss:-null}
  hb_cpu=${hb_cpu:-null}; ad_cpu=${ad_cpu:-null}; lp_cpu=${lp_cpu:-null}

  local fault_n
  fault_n=$(awk '/^---/{c++} END{print c+0}' "$OUT/faults_echo.log" 2>/dev/null)
  fault_n=${fault_n:-0}

  local hb_state ad_state lp_state
  hb_state=$(timeout 4 ros2 lifecycle get /helix_heartbeat_monitor 2>/dev/null | tr -d '\n\r')
  ad_state=$(timeout 4 ros2 lifecycle get /helix_anomaly_detector 2>/dev/null | tr -d '\n\r')
  lp_state=$(timeout 4 ros2 lifecycle get /helix_log_parser 2>/dev/null | tr -d '\n\r')

  printf '{"ts":%s,"elapsed_s":%s,"fault_count_total":%s,' \
    "$ts" "$elapsed" "$fault_n" >> "$OUT/sampler.jsonl"
  printf '"hb":{"pid":%s,"state":"%s","rss_kb":%s,"cpu_pct":%s},' \
    "$hb_pid" "$hb_state" "$hb_rss" "$hb_cpu" >> "$OUT/sampler.jsonl"
  printf '"ad":{"pid":%s,"state":"%s","rss_kb":%s,"cpu_pct":%s},' \
    "$ad_pid" "$ad_state" "$ad_rss" "$ad_cpu" >> "$OUT/sampler.jsonl"
  printf '"lp":{"pid":%s,"state":"%s","rss_kb":%s,"cpu_pct":%s}}\n' \
    "$lp_pid" "$lp_state" "$lp_rss" "$lp_cpu" >> "$OUT/sampler.jsonl"
}

while [ "$(date +%s)" -lt "$END_TS" ]; do
  sample_one
  sleep "$SAMPLE_PERIOD_S"
done

# 6) Teardown
kill "$TEGRA_PID" 2>/dev/null
kill "$FAULTS_PID" 2>/dev/null
kill "$HEALTH_PID" 2>/dev/null
sleep 1

# Final lifecycle snapshot
{
  echo "--- lifecycle states at end ---"
  for n in helix_heartbeat_monitor helix_anomaly_detector helix_log_parser; do
    echo -n "$n: "; timeout 4 ros2 lifecycle get "/$n" 2>&1
  done
} >> "$OUT/lifecycle.log"

# Stop launch (lifecycle nodes & launch process tree)
kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 3
pkill -f "helix_heartbeat_monitor" 2>/dev/null
pkill -f "helix_anomaly_detector"  2>/dev/null
pkill -f "helix_log_parser"        2>/dev/null
sleep 1

# 7) Aggregate JSON summary
python3 - "$OUT" <<'PY'
import json, os, sys, statistics, re
out=sys.argv[1]
recs=[json.loads(l) for l in open(os.path.join(out,"sampler.jsonl")) if l.strip()]
def s(node, key):
    v=[r[node][key] for r in recs if r[node][key] is not None]
    if not v: return None
    return {"min":min(v),"max":max(v),"mean":sum(v)/len(v),"n":len(v)}

# tegrastats: capture max temps and any throttling (case-insensitive — log uses lowercase)
tegra=open(os.path.join(out,"tegrastats.log")).read()
temps=[float(m.group(1)) for m in re.finditer(r"cpu@([\d.]+)C", tegra, re.IGNORECASE)]
gpu=[float(m.group(1)) for m in re.finditer(r"gpu@([\d.]+)C", tegra, re.IGNORECASE)]
soc=[float(m.group(1)) for m in re.finditer(r"soc0@([\d.]+)C", tegra, re.IGNORECASE)]
tj=[float(m.group(1)) for m in re.finditer(r"tj@([\d.]+)C", tegra, re.IGNORECASE)]
throttle = ("throttle" in tegra.lower()) or ("thermal" in tegra.lower() and "limit" in tegra.lower())

# fault count = max
fault_max=max((r["fault_count_total"] for r in recs), default=0)

# state continuity per node: was it 'active' the whole time?
def state_ok(node):
    states=[r[node]["state"] for r in recs]
    return all(("active" in (st or "")) for st in states), states[:3], states[-3:]

hb_ok,hb_first,hb_last=state_ok("hb")
ad_ok,ad_first,ad_last=state_ok("ad")
lp_ok,lp_first,lp_last=state_ok("lp")

summary={
 "task": "Task 3 — 30-min Jetson persistent deployment vs live GO2",
 "duration_target_s": 1800,
 "duration_actual_s": recs[-1]["elapsed_s"] if recs else 0,
 "samples": len(recs),
 "sample_period_s": 10,
 "fault_count_total": fault_max,
 "thermal": {
   "cpu_max_c": max(temps) if temps else None,
   "gpu_max_c": max(gpu) if gpu else None,
   "soc_max_c": max(soc) if soc else None,
   "tj_max_c":  max(tj) if tj else None,
   "throttle_observed": bool(throttle),
   "tegra_samples": len(temps),
 },
 "nodes": {
   "helix_heartbeat_monitor": {"rss_kb":s("hb","rss_kb"), "cpu_pct":s("hb","cpu_pct"),
                                "state_active_throughout": hb_ok, "first":hb_first, "last":hb_last},
   "helix_anomaly_detector":  {"rss_kb":s("ad","rss_kb"), "cpu_pct":s("ad","cpu_pct"),
                                "state_active_throughout": ad_ok, "first":ad_first, "last":ad_last},
   "helix_log_parser":        {"rss_kb":s("lp","rss_kb"), "cpu_pct":s("lp","cpu_pct"),
                                "state_active_throughout": lp_ok, "first":lp_first, "last":lp_last},
 },
 "success_criteria": {
   "no_oom": all(r["hb"]["pid"] and r["ad"]["pid"] and r["lp"]["pid"] for r in recs),
   "no_thermal_throttle": not bool(throttle),
   "lifecycle_active_throughout": hb_ok and ad_ok and lp_ok,
 },
}
with open(os.path.join(out,"jetson_persistent_30min.json"),"w") as f:
    json.dump(summary,f,indent=2)
print(json.dumps(summary,indent=2))
PY

echo "DONE: $OUT"
