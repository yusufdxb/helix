# ArbiterStatus: what the motion arbiter selected on one output tick.
#
# reason values:
#   SOURCE               an upstream input won; its command was published
#   HELIX_HOLD           HELIX hold asserted; output forced to zero
#   HELIX_STATE_STALE    HELIX hold state stopped arriving; fail safe to zero
#   HELIX_STATE_MISSING  no HELIX hold state received yet; fail safe to zero
#   PROTECTIVE_HOLD      native arbiter latch awaits explicit operator release
#   NO_LIVE_INPUT        HELIX released but no valid fresh input; zero
#   SHUTDOWN             arbiter deactivating or exiting; zero

string selected_source   # winning input name, '' when output is forced to zero
string reason
bool hold_active         # true whenever the zero is forced by HELIX (hold, stale, missing)
string hold_fault_id
float64 out_linear_x
float64 out_linear_y
float64 out_angular_z
float64 stamp            # arbiter clock, seconds
uint64 seq
uint32 rejected_total    # malformed input messages rejected since activation
int32 sink_subscribers   # subscribers on the output topic at this tick

# Native arbiter-owned latch, independent of the supervisor's hold state.
bool protective_latched
string release_challenge  # unique arbiter session plus protection generation
uint64 supervisor_epoch   # accepted hold publisher epoch, 0 before first state
