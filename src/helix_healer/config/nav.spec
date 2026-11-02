# Example: a small navigation pipeline built from helix_demo stages.
#   lidar (lifecycle) -> /scan ----\
#   odom              -> /odom ----+-> localizer -> /pose -> planner -> /plan -> controller -> /cmd_vel
# lidar falls back to lidar_backup; controller is a lifecycle node.
# lidar publishes best-effort (sensor QoS); a reliable subscriber to /scan is a QoS mismatch.
healer startup_grace_s=3 verify_s=5

node lidar lifecycle=1 fallback=lidar_backup
out lidar /scan 20
exec lidar @helix_demo/demo_node --ros-args -p lifecycle:=true -p rate_hz:=20.0 -p outputs:=[/scan] -p payload:=4096 -p pub_best_effort:=true

node lidar_backup autostart=0
out lidar_backup /scan 10
exec lidar_backup @helix_demo/demo_node --ros-args -p rate_hz:=10.0 -p outputs:=[/scan] -p payload:=2048 -p pub_best_effort:=true

node odom
out odom /odom 50
exec odom @helix_demo/demo_node --ros-args -p rate_hz:=50.0 -p outputs:=[/odom]

node localizer
in localizer /scan
in localizer /odom
out localizer /pose 20
exec localizer @helix_demo/demo_node --ros-args -p rate_hz:=20.0 -p inputs:=[/scan,/odom] -p outputs:=[/pose] -p input_timeout_s:=0.3

node planner critical=0
in planner /pose
out planner /plan 5
exec planner @helix_demo/demo_node --ros-args -p rate_hz:=5.0 -p inputs:=[/pose] -p outputs:=[/plan] -p input_timeout_s:=0.5

node controller lifecycle=1
in controller /plan
in controller /pose
out controller /cmd_vel 20
exec controller @helix_demo/demo_node --ros-args -p lifecycle:=true -p rate_hz:=20.0 -p inputs:=[/pose] -p outputs:=[/cmd_vel] -p input_timeout_s:=0.3
