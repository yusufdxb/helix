# Scenario D: Network Latency Perturbation (Documentation Only)

## Protocol
**Objective**: Document what a network perturbation experiment would look like and what
HELIX signals it would produce.

**Safety classification**: DOCUMENTATION ONLY -- no network perturbation was executed.

**Reason for not executing**: Network perturbation between PC and the GO2 subnet could
disrupt the robot's connectivity to its Jetson and any active control loops. Even though
we would only apply tc netem on the PC's interface (enp0s31f6), this could cause DDS
participant timeout and topic disappearance, which might trigger unexpected behavior if
any GO2 node depends on external DDS acknowledgments. The risk is LOW but NOT ZERO, and
the robot is live on a physical floor -- we err on the side of caution.

## Proposed Experiment Design

### Setup
```bash
# Identify the interface to the GO2 subnet
# PC uses enp0s31f6 for 192.168.123.0/24

# Add 50ms latency with 10ms jitter (moderate perturbation)
sudo tc qdisc add dev enp0s31f6 root netem delay 50ms 10ms

# Verify
tc qdisc show dev enp0s31f6
```

### Experiment Steps
1. Start baseline bag recording (30s, no perturbation) -- ALREADY DONE (baseline_001)
2. Apply tc netem delay (50ms +/- 10ms) on enp0s31f6
3. Start perturbed bag recording (30s)
4. Remove perturbation: `sudo tc qdisc del dev enp0s31f6 root`
5. Start recovery bag recording (30s)
6. Compare message counts, rates, and inter-message timing across all three bags

### Perturbation Levels (escalating)
| Level | tc netem Command | Expected Effect |
|-------|-----------------|-----------------|
| 1 (mild) | delay 20ms 5ms | Slightly increased inter-message jitter |
| 2 (moderate) | delay 50ms 10ms | Noticeable jitter, possible DDS heartbeat warnings |
| 3 (significant) | delay 200ms 50ms | Possible topic dropout, DDS participant timeout |
| 4 (severe) | delay 500ms 100ms loss 5% | Topic loss, participant expiry, potential /rosout errors |

### Reversal
All perturbations are instantly reversible:
```bash
sudo tc qdisc del dev enp0s31f6 root
```
This immediately restores normal network behavior. No persistent configuration change.

### Expected HELIX Observables

**With current HELIX instrumentation:**
- /rosout: Would likely show NO change at levels 1-2 (robot nodes don't log about
  external DDS latency). At levels 3-4, might see DDS participant expiry warnings.
- Topic rate deviation: IMU rate (249 Hz) would show increased jitter and potentially
  dropped messages at levels 2+. This is the most sensitive indicator.
- Message gap analysis: Time between consecutive messages on /utlidar/robot_odom would
  show bimodal distribution (normal ~6.7ms gaps shifting to ~56.7ms at level 2).

**With additional instrumentation HELIX would need:**
- DDS-level heartbeat monitoring (libddsc hooks or cyclonedds log parsing)
- Network-level RTT measurement (ping or custom UDP probe to 192.168.123.161)
- Topic staleness detector: alert when max(time_since_last_msg) exceeds 2x expected period
- Jitter histogram: track inter-message time distribution and alert on distribution shift

### Risk Assessment
| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Robot loses DDS connectivity | Low (levels 1-2), Medium (3-4) | Robot continues autonomously, just invisible to us | Remove netem immediately |
| Control loop disrupted | Very Low (GO2 control is onboard) | None -- locomotion is local | N/A |
| Jetson loses connectivity to robot | None (netem is on PC, not on 192.168.123.18 path) | N/A | N/A |
| tc netem persists after experiment | Low | Degraded monitoring | Always verify removal with `tc qdisc show` |

### Recommendation
This experiment is FEASIBLE and LOW RISK for levels 1-2, but should be executed:
1. With a human operator physically near the robot
2. With the reversal command pre-typed and ready to execute
3. During a controlled session (not during other active experiments)
4. After verifying that the Jetson-to-robot path (192.168.123.18 to .161) is NOT
   affected by the PC's netem (it should not be, since they are different interfaces)
