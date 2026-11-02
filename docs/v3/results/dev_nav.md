# DEV results: NAV topology, 3 reps (development set, not held out)

Run: `python3 eval/heal/run_episodes.py --set nav --reps 3 --out eval/heal/out/dev_nav` (156 episodes, all healer rc=0, 0 stray processes). Scored with `python3 eval/heal/score.py eval/heal/out/dev_nav`; per-episode rows in `dev_nav.csv`. R = restored, S = SAFE_STOP, F = failed; uN = unnecessary decisions, bN = episodes ending with a runaway or leak still burning.

```
scenario                  helix           none            respawn         watchdog        
cpu_runaway               RRR             RRR b3          RRR b3          RRR b3          
crash_exit                RRR             FFF             RRR             RRR             
crash_loop                RRR             FFF             FFF             RRR u18         
crash_segv                RRR             FFF             RRR             RRR             
degrade                   RRR             FFF             FFF             FFF             
degrade_lifecycle         RRR             FFF             FFF             FFF             
endpoint_loss             RRR             FFF             FFF             RRR             
endpoint_loss_lifecycle   RRR             FFF             FFF             RRR u3          
hang                      RRR             FFF             FFF             RRR u6          
hang_lifecycle            RRR             FFF             FFF             RRR             
leak                      RRR             RRR b3          RRR b3          RRR             
nominal                   RRR             RRR             RRR             RRR             
qos_mismatch              SSS             FFF             FFF             SSS u24         

policy    n  restored  safe  failed  diag_ok  decisions  unneeded  futile  burning  lat_med_s  outage_mean_s  cpu%  rss_mb
none      39         9     0      30    0/36           0        0       0        6       0.00          20.55   0.4    14.2
respawn   39        15     0      24    6/36          36        0       0        6       0.67          16.61   0.5    14.2
watchdog  39        30     3       6   15/36          96       51       9        3       2.05           7.97   0.5    14.2
helix     39        36     3       0   36/36          42        0       0        0       2.15           3.80   0.5    14.2

PRIMARY helix - watchdog: RESTORED diff +6 over 12 fault scenarios (2 non-tied blocks, p-value floor 0.25); exact sign-flip p = 0.25; unnecessary helix 0 vs watchdog 51; dropped pairs 0
KILL CRITERION (diff < 3 or more unnecessary): not met, claim survives
```

Learned-diagnosis positive control (train = test = DEV, so this checks the pipeline, not generalization): `python3 eval/heal/learned.py --train eval/heal/out/dev_nav --test eval/heal/out/dev_nav`

```
A decision point (n=36): rules 36/36  gbdt 36/36
B false alarms: gbdt flags a node in 4/3160 fault-free snapshots (0.13%)
C cpu_runaway              gbdt detects root at ['1.06', '0.65', '0.80'] s
C crash_exit               gbdt detects root at ['0.41', '0.50', '0.41'] s
C crash_loop               gbdt detects root at ['0.50', '0.45', '0.35'] s
C crash_segv               gbdt detects root at ['0.45', '0.46', '0.45'] s
C degrade                  gbdt detects root at ['0.80', '0.61', '0.80'] s
C degrade_lifecycle        gbdt detects root at ['0.75', '0.80', '0.80'] s
C endpoint_loss            gbdt detects root at ['0.46', '0.60', '0.50'] s
C endpoint_loss_lifecycle  gbdt detects root at ['0.40', '0.35', '0.55'] s
C hang                     gbdt detects root at ['0.71', '0.80', '0.80'] s
C hang_lifecycle           gbdt detects root at ['0.76', '0.71', '0.86'] s
C leak                     gbdt detects root at ['0.81', '0.70', '0.81'] s
C qos_mismatch             gbdt detects root at ['0.60', '0.36', '0.30'] s
```

These numbers drove development (see A9 in the pre-registration) and are not evidence for any claim. The held-out result is in `heldout_manip.md`.
