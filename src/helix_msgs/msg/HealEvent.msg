# One self-healing event from helix_healer.
float64 t          # seconds since the healer started (steady clock)
string node        # managed node name
string kind        # act, given_up, victim, escalate, verified, verify_failed, fallback, action_failed, ...
string fault       # diagnosed fault class
string action      # repair action from the closed set
int32 attempt      # repair attempt within the budget window
string detail
