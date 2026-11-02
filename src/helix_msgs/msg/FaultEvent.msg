# One alarm transition from helix_monitor_node.
string node_name
string fault_type        # check name: RATE | FROZEN | STAMP | RANGE | RESIDUAL
uint8  severity          # 2 = raised, 0 = cleared
string detail
float64 timestamp        # monitor clock time of the transition, s
string[] context_keys    # stream, state, value
string[] context_values
