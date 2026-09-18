# HelixHold: HELIX recovery hold state for the motion arbiter.
#
# Published by RecoveryNode at a fixed rate for as long as it is active,
# whether or not it is holding, so the arbiter can tell a live "not holding"
# from a dead publisher. This is a STATE, not a velocity command: HELIX never
# commands motion, it only asks the arbiter to force its output to zero.
#
# Freshness is judged on the arbiter's own receipt clock. stamp and
# asserted_stamp are the publisher's clock and are used for tracing only
# (the robot and payload clocks are known to be skewed).
#
# (epoch, seq) orders messages: a message that is not newer than the last
# accepted one is dropped as a DDS reorder/duplicate. epoch changes when the
# publisher restarts.

bool hold
string fault_id          # fault that asserted the current hold ('' if none)
string reason            # human-readable cause of the current state
uint64 epoch             # publisher start time in ns, changes on restart
uint64 seq               # strictly increasing within an epoch
float64 stamp            # publisher clock, seconds (tracing only)
float64 asserted_stamp   # publisher clock when the hold was asserted, 0 if not holding
