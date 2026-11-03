# GO2 Orin stack (MOSAIC runtime), Stage 0: OBSERVE ONLY. Run with observe_only:=true as well.
# Every node is act=0. HELIX adopts processes that the MOSAIC launch started (attach=, a cmdline
# substring) and only diagnoses them; see docs/go2/observe.md.
#
# MOTION CHAIN, act=0 PERMANENTLY (never lifted, also not in Stage 1): go2_bridge, hardware_bridge,
# mosaic_guardian, safety_arbiter, sport_stop_watchdog, candidate_stamper, mosaic (go2_primitives
# motion topics), velocity_smoother, controller_server, and anything that publishes /cmd_vel*,
# /api/sport/request or a stop/estop topic. A restart of these can replay a command or drop a stop.
#
# Unverified until Stage 0 (lab card in the vault):
#   - each attach= pattern must match exactly one pid on the Orin (patterns are "/<executable>",
#     inferred from the standard lib/<pkg>/<exe> install layout, not read from a live cmdline);
#   - topic names below come from source reads (mosaic cloud-mosaic launch, guidedog-x, nav-rc), not
#     from the live graph; nodes with no out line are checked for liveness only until Stage 0 fills
#     them in from `ros2 node info`;
#   - no rates: every out line is learned (no hz) over learn_s, because no Orin rates are measured;
#   - HELIX subscribes to each out topic to measure its rate. /camera/front/image_raw is a raw image
#     stream; its CPU cost on the Orin is a Stage 0 measurement.
# Nav2 is off since 10-31: its nodes are listed but never adopted, so they are never judged.
healer startup_grace_s=20 verify_s=10 learn_s=20

# ---- drivers ----
node go2_bridge act=0 attach=/go2_bridge_node
node hardware_bridge act=0 attach=/hardware_bridge_node
node go2_state_relay act=0 attach=/go2_state_relay_node
node front_video act=0 attach=/go2_front_video
out front_video /camera/front/image_raw

# ---- perception ----
node scan act=0 attach=/pointcloud_to_laserscan_node
out scan /scan
node slam act=0 attach=/async_slam_toolbox_node
in slam /scan
node lidar_hazard act=0 attach=/lidar_hazard_node
node traversability act=0 attach=/mosaic_traversability
out traversability /mosaic/traversability/status
node posture act=0 attach=/mosaic_posture
out posture /mosaic/posture
node doa act=0 attach=/mosaic_doa
out doa /mosaic/doa/health
node snapshot act=0 attach=/snapshot_server
node vlm act=0 attach=/vlm_server
node intent_grounding act=0 attach=/intent_grounding_node

# ---- voice ----
node stt act=0 attach=/stt_node
out stt /whip/diag/stt
node tts act=0 attach=/tts_node
out tts /whip/diag/tts

# ---- MOSAIC core and recording ----
node mosaic act=0 attach=/mosaic_node
node recorder act=0 critical=0 attach=/mosaic_recorder/

# ---- safety (motion chain) ----
node guardian act=0 attach=/mosaic_guardian
out guardian /mosaic/guardian/status
node safety_arbiter act=0 attach=/safety_arbiter_node
node stop_watchdog act=0 attach=/sport_stop_watchdog_node
out stop_watchdog /go2/stop_watchdog/status
node candidate_stamper act=0 attach=/candidate_stamper_node

# ---- Nav2 (off since 10-31; unadopted, never judged) ----
node controller_server act=0 attach=/controller_server
node planner_server act=0 attach=/planner_server
node behavior_server act=0 attach=/behavior_server
node bt_navigator act=0 attach=/bt_navigator
node smoother_server act=0 attach=/smoother_server
node waypoint_follower act=0 attach=/waypoint_follower
node velocity_smoother act=0 attach=/velocity_smoother
