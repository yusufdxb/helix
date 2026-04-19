# RecoveryHint — emitted by DiagnosisNode, consumed by RecoveryNode.
#
# fault_id       correlates with the FaultEvent that triggered this hint
# suggested_action  STOP_AND_HOLD | RESUME | LOG_ONLY (enforced by RecoveryNode allowlist)
# confidence     0.0 - 1.0
# reasoning      short human-readable rationale
# rule_matched   rule identifier from rules.py (R1, R2, R3, R4, ...)

string fault_id
string suggested_action
float32 confidence
string reasoning
string rule_matched
