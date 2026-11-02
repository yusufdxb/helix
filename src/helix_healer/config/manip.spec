# Held-out evaluation topology: a small manipulation pipeline built from helix_demo
# stages. It was written before any held-out episode ran and was never used to tune
# the healer (see docs/v3/method.md).
#   camera (lifecycle) -> /image -> detector -> /detections --\
#   joints             -> /joint_states -----------------------+-> tracker -> /targets -> arm_planner -> /trajectory
#   joints             -> /joint_states -> arm_controller -> /joint_cmd
#   gripper            -> /gripper_state
# camera falls back to camera_backup. detector latches: once its input has been
# gone for 1 s it stops publishing until it is restarted, even after the input
# returns (a common real failure: a node that does not recover on its own).
# camera publishes best-effort (sensor QoS).
healer startup_grace_s=3 verify_s=5

node camera lifecycle=1 fallback=camera_backup
out camera /image 30
exec camera @helix_demo/demo_node --ros-args -p lifecycle:=true -p rate_hz:=30.0 -p outputs:=[/image] -p payload:=16384 -p pub_best_effort:=true

node camera_backup autostart=0
out camera_backup /image 15
exec camera_backup @helix_demo/demo_node --ros-args -p rate_hz:=15.0 -p outputs:=[/image] -p payload:=8192 -p pub_best_effort:=true

node joints
out joints /joint_states 100
exec joints @helix_demo/demo_node --ros-args -p rate_hz:=100.0 -p outputs:=[/joint_states] -p payload:=256

node detector
in detector /image
out detector /detections 15
exec detector @helix_demo/demo_node --ros-args -p rate_hz:=15.0 -p inputs:=[/image] -p outputs:=[/detections] -p input_timeout_s:=0.3 -p latch_on_input_loss_s:=1.0 -p work_us:=2000

node tracker
in tracker /detections
in tracker /joint_states
out tracker /targets 15
exec tracker @helix_demo/demo_node --ros-args -p rate_hz:=15.0 -p inputs:=[/detections,/joint_states] -p outputs:=[/targets] -p input_timeout_s:=0.4

node arm_planner lifecycle=1 critical=0
in arm_planner /targets
out arm_planner /trajectory 10
exec arm_planner @helix_demo/demo_node --ros-args -p lifecycle:=true -p rate_hz:=10.0 -p inputs:=[/targets] -p outputs:=[/trajectory] -p input_timeout_s:=0.5

node arm_controller
in arm_controller /joint_states
out arm_controller /joint_cmd 100
exec arm_controller @helix_demo/demo_node --ros-args -p rate_hz:=100.0 -p inputs:=[/joint_states] -p outputs:=[/joint_cmd] -p input_timeout_s:=0.2

node gripper critical=0
out gripper /gripper_state 5
exec gripper @helix_demo/demo_node --ros-args -p rate_hz:=5.0 -p outputs:=[/gripper_state]
