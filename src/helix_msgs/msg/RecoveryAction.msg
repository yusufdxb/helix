# RecoveryAction — audit log event from RecoveryNode.
#
# status values:
#   ACCEPTED              — hint passed all safety checks; action fired
#   SUPPRESSED_COOLDOWN   — same-type action fired within cooldown window
#   SUPPRESSED_ALLOWLIST  — hint requested action outside ALLOWED_ACTIONS
#   SUPPRESSED_DISABLED   — recovery.enabled param is false
#
# reason is always human-readable and never empty.

string fault_id
string action
string status
float64 timestamp
string reason
