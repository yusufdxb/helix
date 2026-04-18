#!/usr/bin/env bash
# Run the raw-subscription benchmark. Two trials per variant, 60 s each,
# with a 5 s warmup that excludes DDS discovery from the measurement.
set -eo pipefail

# Isolate from any running helix DDS domain.
export ROS_DOMAIN_ID=99
source /opt/ros/humble/setup.bash

cd "$(dirname "$0")"
mkdir -p results
exec > >(tee -a "results/run.log") 2>&1
echo "=== bench start $(date -u +%FT%TZ) ==="

run_trial() {
    local raw="$1"
    local tag="$2"

    echo "--- trial raw=${raw} (${tag}) ---"
    # Start publisher fresh per trial so DDS state is clean.
    python3 publisher.py > "results/pub_${tag}.log" 2>&1 &
    local pub_pid=$!
    trap "kill ${pub_pid} 2>/dev/null || true" RETURN
    sleep 2
    python3 subscriber.py --raw "${raw}" --duration 60 --warmup 5 \
        > "results/sub_${tag}.log" 2>&1
    kill "${pub_pid}" 2>/dev/null || true
    wait "${pub_pid}" 2>/dev/null || true
    trap - RETURN
    echo "sub log:"
    tail -n 2 "results/sub_${tag}.log"
}

run_trial 0 "cooked_a"
sleep 3
run_trial 1 "raw_a"
sleep 3
run_trial 0 "cooked_b"
sleep 3
run_trial 1 "raw_b"

echo "=== bench end $(date -u +%FT%TZ) ==="
