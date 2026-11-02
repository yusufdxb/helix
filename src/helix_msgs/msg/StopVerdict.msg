# Independent-evidence verdict on one commanded stop (published by helix_monitor_node).
float64 t_cmd        # monitor clock time when the stop command was seen, s
float64 t            # decision time, s
string verdict       # VERIFIED | FAILED | UNVERIFIED
string[] groups      # evidence group names, config order
float64[] metrics    # per group at decision time (NaN = no fresh data)
