#!/bin/bash
# HELIX environment setup — source this before any helix terminal session.
# Usage: source ~/helix_ws/helix_env.sh

source /opt/ros/humble/setup.bash
source ~/helix_ws/install/setup.bash
export CYCLONEDDS_URI=file:///home/carebear/helix_ws/cyclonedds_loopback.xml
unset ROS_LOCALHOST_ONLY

echo "HELIX env ready — ROS2 Humble + helix_ws sourced, CycloneDDS loopback with unicast peer discovery"
