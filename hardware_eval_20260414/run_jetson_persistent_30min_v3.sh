#!/usr/bin/env bash
# Task 3 v3 — launch each node via direct `ros2 run` in its own setsid+nohup
# session. No `ros2 launch` wrapper. Per-node stderr capture so any silent
# crash leaves a traceback. Uses helix_node_runner.sh which handles a single
# node + its log redirect, called once per node.
DURATION_S=${DURATION_S:-1800}
SAMPLE_PERIOD_S=${SAMPLE_PERIOD_S:-15}
TEGRA_PERIOD_MS=${TEGRA_PERIOD_MS:-1000}
OUT=/tmp/helix_t3
mkdir -p "$OUT"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/helix_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
export PYTHONUNBUFFERED=1

ros2 daemon stop >/dev/null 2>&1; sleep 2; ros2 daemon start >/dev/null 2>&1; sleep 5

PARAMS=$(ros2 pkg prefix helix_bringup)/share/helix_bringup/config/helix_params.yaml
RULES=$(ros2 pkg prefix helix_bringup)/share/helix_bringup/config/log_rules.yaml

# Per-node launcher — keeps env, redirects stdout+stderr, line-buffered.
NODE_RUNNER=/tmp/helix_node_runner.sh
cat > "$NODE_RUNNER" <<'INNER'
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/helix_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
export PYTHONUNBUFFERED=1
exec stdbuf -oL -eL ros2 run "$@"
INNER
chmod +x "$NODE_RUNNER"

# Launch each node detached with its own log file.
setsid nohup "$NODE_RUNNER" helix_core helix_heartbeat_monitor \
    --ros-args -r __node:=helix_heartbeat_monitor --params-file "$PARAMS" \
    > "$OUT/hb.log" 2>&1 < /dev/null &
HB_PID=$!
disown $HB_PID 2>/dev/null

setsid nohup "$NODE_RUNNER" helix_core helix_anomaly_detector \
    --ros-args -r __node:=helix_anomaly_detector --params-file "$PARAMS" \
    > "$OUT/ad.log" 2>&1 < /dev/null &
AD_PID=$!
disown $AD_PID 2>/dev/null

setsid nohup "$NODE_RUNNER" helix_core helix_log_parser \
    --ros-args -r __node:=helix_log_parser --params-file "$PARAMS" \
                -p "rules_file_path:=$RULES" \
    > "$OUT/lp.log" 2>&1 < /dev/null &
LP_PID=$!
disown $LP_PID 2>/dev/null

echo "hb_starter=$HB_PID ad_starter=$AD_PID lp_starter=$LP_PID" > "$OUT/pids.txt"
sleep 12

for n in helix_heartbeat_monitor helix_anomaly_detector helix_log_parser; do
  ros2 lifecycle set "/$n" configure >> "$OUT/lifecycle.log" 2>&1 || true
  sleep 1
  ros2 lifecycle set "/$n" activate  >> "$OUT/lifecycle.log" 2>&1 || true
  sleep 1
done
{ echo "--- t0 lifecycle ---";
  for n in helix_heartbeat_monitor helix_anomaly_detector helix_log_parser; do
    echo -n "$n: "; ros2 lifecycle get "/$n" 2>&1
  done
} >> "$OUT/lifecycle.log"

# Watchers
setsid nohup tegrastats --interval "$TEGRA_PERIOD_MS" \
    > "$OUT/tegrastats.log" 2>&1 < /dev/null &
TEGRA_PID=$!; disown $TEGRA_PID 2>/dev/null

setsid nohup bash -c "
source /opt/ros/humble/setup.bash; source ~/unitree_ros2/cyclonedds_ws/install/setup.bash; source ~/helix_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unitree/unitree_ros2/cyclonedds_ws/src/cyclonedds.xml
ros2 topic echo --no-arr --no-str /helix/faults helix_msgs/msg/FaultEvent" \
    > "$OUT/faults_echo.log" 2>&1 < /dev/null &
FAULTS_PID=$!; disown $FAULTS_PID 2>/dev/null

echo "tegra_pid=$TEGRA_PID faults_pid=$FAULTS_PID" >> "$OUT/pids.txt"

START_TS=$(date +%s)
END_TS=$(( START_TS + DURATION_S ))
> "$OUT/sampler.jsonl"

sample_one() {
  local ts elapsed
  ts=$(date +%s); elapsed=$(( ts - START_TS ))
  local hb_pid ad_pid lp_pid
  hb_pid=$(pgrep -f "lib/helix_core/helix_heartbeat_monitor" | head -1); hb_pid=${hb_pid:-0}
  ad_pid=$(pgrep -f "lib/helix_core/helix_anomaly_detector"  | head -1); ad_pid=${ad_pid:-0}
  lp_pid=$(pgrep -f "lib/helix_core/helix_log_parser"        | head -1); lp_pid=${lp_pid:-0}

  local hb_rss hb_cpu ad_rss ad_cpu lp_rss lp_cpu
  read -r hb_rss hb_cpu < <(ps -p "$hb_pid" -o rss=,%cpu= 2>/dev/null | awk '{print $1, $2}')
  read -r ad_rss ad_cpu < <(ps -p "$ad_pid" -o rss=,%cpu= 2>/dev/null | awk '{print $1, $2}')
  read -r lp_rss lp_cpu < <(ps -p "$lp_pid" -o rss=,%cpu= 2>/dev/null | awk '{print $1, $2}')
  hb_rss=${hb_rss:-null}; ad_rss=${ad_rss:-null}; lp_rss=${lp_rss:-null}
  hb_cpu=${hb_cpu:-null}; ad_cpu=${ad_cpu:-null}; lp_cpu=${lp_cpu:-null}

  local fault_n
  fault_n=$(awk '/^---/{c++} END{print c+0}' "$OUT/faults_echo.log" 2>/dev/null); fault_n=${fault_n:-0}

  local hb_state ad_state lp_state
  hb_state=$(timeout 4 ros2 lifecycle get /helix_heartbeat_monitor 2>/dev/null | tr -d '\n\r')
  ad_state=$(timeout 4 ros2 lifecycle get /helix_anomaly_detector 2>/dev/null | tr -d '\n\r')
  lp_state=$(timeout 4 ros2 lifecycle get /helix_log_parser 2>/dev/null | tr -d '\n\r')

  printf '{"ts":%s,"elapsed_s":%s,"fault_count_total":%s,"hb":{"pid":%s,"state":"%s","rss_kb":%s,"cpu_pct":%s},"ad":{"pid":%s,"state":"%s","rss_kb":%s,"cpu_pct":%s},"lp":{"pid":%s,"state":"%s","rss_kb":%s,"cpu_pct":%s}}\n' \
    "$ts" "$elapsed" "$fault_n" \
    "$hb_pid" "$hb_state" "$hb_rss" "$hb_cpu" \
    "$ad_pid" "$ad_state" "$ad_rss" "$ad_cpu" \
    "$lp_pid" "$lp_state" "$lp_rss" "$lp_cpu" >> "$OUT/sampler.jsonl"
}

while [ "$(date +%s)" -lt "$END_TS" ]; do
  sample_one
  sleep "$SAMPLE_PERIOD_S"
done

# Teardown
kill "$TEGRA_PID" "$FAULTS_PID" 2>/dev/null
sleep 1
{ echo "--- end lifecycle ---";
  for n in helix_heartbeat_monitor helix_anomaly_detector helix_log_parser; do
    echo -n "$n: "; timeout 4 ros2 lifecycle get "/$n" 2>&1
  done
} >> "$OUT/lifecycle.log"
pkill -f "lib/helix_core/helix_heartbeat_monitor" 2>/dev/null
pkill -f "lib/helix_core/helix_anomaly_detector"  2>/dev/null
pkill -f "lib/helix_core/helix_log_parser"        2>/dev/null

# Aggregate
python3 - "$OUT" <<'PY'
import json, os, sys, re
out=sys.argv[1]
recs=[json.loads(l) for l in open(os.path.join(out,"sampler.jsonl")) if l.strip()]
def s(node, key):
    v=[r[node][key] for r in recs if r[node][key] is not None]
    if not v: return None
    return {"min":min(v),"max":max(v),"mean":sum(v)/len(v),"n":len(v)}
tegra=open(os.path.join(out,"tegrastats.log")).read()
temps=[float(m.group(1)) for m in re.finditer(r"cpu@([\d.]+)C", tegra, re.IGNORECASE)]
gpu=[float(m.group(1)) for m in re.finditer(r"gpu@([\d.]+)C", tegra, re.IGNORECASE)]
soc=[float(m.group(1)) for m in re.finditer(r"soc0@([\d.]+)C", tegra, re.IGNORECASE)]
tj =[float(m.group(1)) for m in re.finditer(r"tj@([\d.]+)C",  tegra, re.IGNORECASE)]
throttle = ("throttle" in tegra.lower()) or ("thermal" in tegra.lower() and "limit" in tegra.lower())
fault_max=max((r["fault_count_total"] for r in recs), default=0)
def state_ok(node):
    states=[r[node]["state"] for r in recs]
    return all(("active" in (st or "")) for st in states), states[:3], states[-3:]
hb_ok,hb_first,hb_last=state_ok("hb"); ad_ok,ad_first,ad_last=state_ok("ad"); lp_ok,lp_first,lp_last=state_ok("lp")
def first_death(node):
    for r in recs:
        if not r[node]["pid"]: return r["elapsed_s"]
    return None
summary={
 "task":"Task 3 — 30-min Jetson persistent deployment vs live GO2",
 "duration_target_s": 1800,
 "duration_actual_s": recs[-1]["elapsed_s"] if recs else 0,
 "samples": len(recs),
 "fault_count_total": fault_max,
 "thermal":{"cpu_max_c":max(temps) if temps else None,
            "gpu_max_c":max(gpu) if gpu else None,
            "soc_max_c":max(soc) if soc else None,
            "tj_max_c": max(tj) if tj else None,
            "throttle_observed":bool(throttle),
            "tegra_samples":len(temps)},
 "nodes":{
  "helix_heartbeat_monitor":{"rss_kb":s("hb","rss_kb"),"cpu_pct":s("hb","cpu_pct"),
                              "first_death_elapsed_s":first_death("hb"),
                              "state_active_throughout":hb_ok,"first":hb_first,"last":hb_last},
  "helix_anomaly_detector": {"rss_kb":s("ad","rss_kb"),"cpu_pct":s("ad","cpu_pct"),
                              "first_death_elapsed_s":first_death("ad"),
                              "state_active_throughout":ad_ok,"first":ad_first,"last":ad_last},
  "helix_log_parser":       {"rss_kb":s("lp","rss_kb"),"cpu_pct":s("lp","cpu_pct"),
                              "first_death_elapsed_s":first_death("lp"),
                              "state_active_throughout":lp_ok,"first":lp_first,"last":lp_last},
 },
 "success_criteria":{
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
