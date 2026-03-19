#!/bin/bash
# HELIX environment setup — source this before any helix terminal session.
# Usage: source ~/helix_ws/helix_env.sh

source /opt/ros/humble/setup.bash
source ~/helix_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI

echo "HELIX env ready — ROS2 Humble + helix_ws sourced, localhost only"
