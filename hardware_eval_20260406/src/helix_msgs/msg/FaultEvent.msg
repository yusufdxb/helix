string node_name
string fault_type        # CRASH | ANOMALY | LOG_PATTERN | NETWORK
uint8  severity          # 1=WARN 2=ERROR 3=CRITICAL
string detail
float64 timestamp
string[] context_keys
string[] context_values
