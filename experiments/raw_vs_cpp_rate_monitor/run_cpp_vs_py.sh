#!/usr/bin/env bash
# Apples-to-apples: 250 Hz Imu publisher on /utlidar/imu; measure the
# subscriber process's CPU for Python cooked, Python raw, and C++.
# Two 60 s trials per variant. Total wall ≈ 5 min.
set -eo pipefail

export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source ~/workspace/helix/install/setup.bash

cd "$(dirname "$0")"
mkdir -p results
exec > >(tee -a "results/run_cpp_vs_py.log") 2>&1
echo "=== bench start $(date -u +%FT%TZ) ==="

start_pub() {
    python3 publisher_utlidar.py > "results/pub_${1}.log" 2>&1 &
    PUB_PID=$!
    sleep 2
}
stop_pub() {
    kill "${PUB_PID}" 2>/dev/null || true
    wait "${PUB_PID}" 2>/dev/null || true
}

trial_py() {
    local raw="$1" tag="$2"
    echo "--- trial py raw=${raw} (${tag}) ---"
    start_pub "${tag}"
    python3 subscriber.py --raw "${raw}" --duration 60 --warmup 5 \
        --topic /utlidar/imu > "results/sub_${tag}.log" 2>&1 || true
    stop_pub
    tail -n 1 "results/sub_${tag}.log"
    sleep 3
}

trial_cpp() {
    local tag="$1"
    echo "--- trial cpp (${tag}) ---"
    start_pub "${tag}"
    python3 bench_cpp.py --duration 60 --warmup 5 \
        > "results/sub_${tag}.log" 2>&1 || true
    stop_pub
    tail -n 1 "results/sub_${tag}.log"
    sleep 3
}

trial_py 0 "py_cooked_a"
trial_py 1 "py_raw_a"
trial_cpp "cpp_a"
trial_py 0 "py_cooked_b"
trial_py 1 "py_raw_b"
trial_cpp "cpp_b"

echo "=== bench end $(date -u +%FT%TZ) ==="
