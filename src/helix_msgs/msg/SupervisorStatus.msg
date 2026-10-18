# SupervisorStatus: the HELIX supervisor's single fault-state machine.
#
# Published on every state change and at a fixed rate while active. One
# process owns fault tracking, diagnosis and the hold decision, so there is
# no second state machine that can disagree with this one.
#
# state values:
#   STARTING           supervision just started; hold asserted until every
#                      critical expected node has sent a heartbeat
#   NOMINAL            no stop-worthy fault active, hold released
#   HOLD_REQUESTED     stop-worthy fault active, hold asserted, arbiter has not
#                      yet confirmed it forced its output to zero
#   HOLD_APPLIED       arbiter confirmed HELIX_HOLD for this hold_id; waiting
#                      for the robot controller to acknowledge StopMove
#   STOP_ACKED         controller acknowledged StopMove; waiting for measured
#                      robot motion to fall below the stop threshold
#   STOP_CONFIRMED     measured motion is below the stop threshold
#   STOP_UNCONFIRMED   an acknowledgment or the measured stop did not arrive in
#                      time; the hold stays asserted, operator attention needed
#   CLEARING           every stop-worthy fault has cleared; the hold stays
#                      until they have stayed clear for clear_hold_s
#   AWAIT_RELEASE      faults cleared, release needs an operator (auto_resume
#                      false, or a latched fault was involved)
#
# hold is what is published on /helix/hold. When actuation_enabled is false
# the same state machine runs (would_hold) but hold stays false.
# The native arbiter owns a separate protective latch. NOMINAL or hold=false
# here does not release that latch; inspect ArbiterStatus.protective_latched.

string state
bool hold
bool would_hold
bool actuation_enabled
bool auto_resume
string hold_id               # '' when not holding; echoed by the arbiter
string hold_cause            # fault key that opened the current hold episode
string reason                # why the last transition happened
string[] active_faults       # keys of active faults, stop-worthy first
uint8[] active_stop_worthy   # 1 if the fault at the same index is stop-worthy
string[] missing_nodes       # expected heartbeats never seen or lost
float64 hold_requested_at    # steady clock, seconds; 0 if not in a hold episode
float64 arbiter_ack_at
float64 controller_ack_at
float64 motion_stopped_at
float64 cleared_at
float64 stamp                # steady clock, seconds
uint64 seq
uint64 revision             # changes with faults, health/state and operator decisions
uint32 hold_episodes
