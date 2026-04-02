# RecoveryHint — forward-looking interface definition
#
# This message type is defined for future use by a recovery planning
# subsystem. It is NOT published, subscribed to, or processed by any
# node in the current codebase. Its presence does not indicate that
# recovery functionality is implemented.

string fault_id
string suggested_action
float32 confidence
string reasoning
