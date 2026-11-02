# Public BeyondMimic-style G1 bag replayed in a loop; nothing is injected, so every decision is a false alarm.
healer startup_grace_s=3 verify_s=5

node bag
out bag /joint_states 500
out bag /odom 500
out bag /tf 1000
exec bag /opt/ros/humble/bin/ros2 bag play BAG_DIR --loop
