string fault_id           # uuid4 string tied to the originating FaultEvent
string node_name          # node that faulted
string fault_type         # CRASH | ANOMALY | LOG_PATTERN | NETWORK
uint8  tier               # 1 | 2 | 3
string action             # action key from policy (e.g. "restart_node")
bool   success
string outcome_detail
float64 duration_sec
float64 timestamp
