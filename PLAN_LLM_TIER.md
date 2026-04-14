# HELIX Structured Diagnosis Tier -- Implementation Plan

**Date:** 2026-04-06
**Author:** Yusuf Guenena
**Status:** Under audit
**Timeline:** Summer 2026 (April-August)

---

## 1. Current State

HELIX is a deterministic ROS 2 fault sensing system with three lifecycle-managed detection nodes:

- **AnomalyDetector** -- Z-score on rolling window, publishes `FaultEvent` with `fault_type=ANOMALY`
- **HeartbeatMonitor** -- timeout-based liveness, publishes `FaultEvent` with `fault_type=CRASH` and health status on `/helix/node_health` (DiagnosticArray)
- **LogParser** -- regex pattern matching on /rosout, publishes `FaultEvent` with configurable `fault_type` per rule

All three publish to `/helix/faults`. Each `FaultEvent` already carries structured context via `context_keys[]` / `context_values[]` (metric name, Z-score, window stats, matched rule, log source, etc.).

### Existing message contracts:

**FaultEvent.msg** (actively used):
```
string   node_name
string   fault_type       # CRASH | ANOMALY | LOG_PATTERN | NETWORK
uint8    severity          # 1=WARN, 2=ERROR, 3=CRITICAL
string   detail
float64  timestamp
string[] context_keys
string[] context_values
```

**RecoveryHint.msg** (defined, unused):
```
string   fault_id
string   suggested_action
float32  confidence
string   reasoning
```

### Hardware evidence (2 sessions, April 3 + 6):
- 11 ROS 2 bags, 138K messages from live Unitree GO2
- 6 real FaultEvents across 2 sessions (2 different anomaly sources)
- Jetson benchmarks reproduced within 1% across 3-day gap
- Zero measurable overhead on robot from monitoring
- Artifacts on T7 SSD at `/media/yusuf/T7 Storage/LABWORK/HELIX/`

### What does NOT exist:
- Any interpretation of FaultEvents (no diagnosis)
- Context buffers (no /rosout history, no metric snapshots beyond what AnomalyDetector holds internally)
- Recovery execution
- Training data

---

## 2. Goal

Add a fourth lifecycle node that converts `FaultEvent` signals plus bounded recent context into a structured `RecoveryHint`.

1. Subscribe to `/helix/faults`
2. Gather bounded recent context (/rosout history, metric rate summaries, node health from `/helix/node_health`)
3. Produce structured `RecoveryHint` output
4. Start with a deterministic rule-based implementation
5. Optionally evaluate a local small LLM as a comparative add-on

The core system must work without any model. If a local LLM adds measurable value over the rule-based baseline, it is reported as an enhancement. If not, it is reported as a negative result.

### Hardware:
- **Deployment:** Jetson Orin NX (16 GB unified RAM). Full stack must fit.
- **Training (only if LLM path pursued):** RTX 5070 (12 GB VRAM) on mewtwo.

---

## 3. Architecture

```
[GO2 Robot]
    |
    ├── topic rates ──→ [AnomalyDetector] ──┐
    ├── heartbeats  ──→ [HeartbeatMonitor] ──┼──→ /helix/faults (FaultEvent)
    └── /rosout     ──→ [LogParser]        ──┘          |
                                                         |
    /helix/node_health (DiagnosticArray) ────────┐       |
    /rosout (rcl_interfaces/Log) ────────────────┤       |
    /helix/metrics (Float64MultiArray) ──────────┤       |
                                                  v       v
                                          [Context Buffers]
                                          - /rosout ring buffer (new)
                                          - metric rate cache (new)
                                          - node health from existing topic
                                                  |
                                                  v
                                          [DiagnosisNode]
                                          - default: rule-based lookup
                                          - optional: local LLM
                                                  |
                                                  v
                                          /helix/recovery (RecoveryHint)
```

The diagnosis node only fires when a FaultEvent arrives. Between events, the context buffers passively accumulate state. The rule-based path has zero model dependency and near-zero latency.

---

## 4. Fault Taxonomy

Restricted to categories that have a confirmed, reproducible signal path through existing HELIX detectors. Each category maps to a `FaultEvent.fault_type` value that the detectors already emit.

### Active categories:

| Category | fault_type | Detector | Signal | Reproduced? |
|----------|-----------|----------|--------|-------------|
| Rate anomaly | ANOMALY | AnomalyDetector | Z-score spike on topic rate metric | Yes -- 6 FaultEvents across 2 sessions, 2 sources |
| Sensor dropout | CRASH | HeartbeatMonitor | Monitored topic goes silent beyond timeout | Yes -- unit tests + fault injector |
| Log fault | LOG_PATTERN | LogParser | /rosout matches configured regex rule | Yes -- 22/22 accuracy on test corpus |

Three categories. These are the only fault types the system can currently detect and reproduce.

### Deferred (no signal path or not reproducible):

| Category | Why deferred |
|----------|-------------|
| NETWORK faults | fault_type exists in FaultEvent but no detector emits it today |
| Rate degradation (partial) | Subcategory of ANOMALY; not a separate detector signal. May be distinguished by Z-score magnitude in rules, but not a separate category. |
| DDS partition | No signal wired |
| Thermal throttle | Jetson thermal data not bridged to /helix/metrics |
| Actuator fault | GO2 motor topics use custom msgs, not bridged |
| Firmware/state error | /multiplestate parsing not implemented |

Categories can be promoted during development if their signal path is wired and tested. Do not add categories without a working detector.

---

## 5. Phases

### Phase 1: Diagnosis Scope & Interface (Week 1-2)

**Objective:** Lock down the diagnosis contract, verify it against existing message definitions, write reference examples.

#### 1a. Verify fault taxonomy against code
- Confirm each active category maps to actual FaultEvent output from a real detector
- For each: document the exact `fault_type`, `context_keys`, and `detail` format the detector produces
- If a category has no confirmed detector output, demote it

#### 1b. Define diagnosis input/output

The diagnosis input is a FaultEvent (already structured) plus a context snapshot (new). The output is a RecoveryHint.

**Context snapshot schema (assembled by buffers, not carried in FaultEvent):**
```json
{
  "rosout_recent": [
    {"level": 40, "name": "twist_mux", "msg": "...", "stamp": 1775499700.0}
  ],
  "metric_rates": {
    "rate_hz/utlidar_robot_pose": {"current_hz": 18.75, "mean_hz_10s": 18.74},
    "rate_hz/utlidar_cloud": {"current_hz": 15.1, "mean_hz_10s": 15.3}
  },
  "node_health": {
    "anomaly_detector": "GREEN",
    "heartbeat_monitor": "GREEN"
  }
}
```

This is combined with the triggering FaultEvent (which already carries fault_type, severity, detail, and context_keys/values) to form the full diagnosis input.

#### 1c. Decide on RecoveryHint.msg changes

Current RecoveryHint has: `fault_id`, `suggested_action`, `confidence`, `reasoning`.

Proposed additions:
```
string fault_category      # from taxonomy (rate_anomaly | sensor_dropout | log_fault)
string severity_label      # critical | warning | info (human-readable mirror of uint8)
string method              # "rules" | "llm:<model_id>"
```

Keep existing fields. Add only what's needed. Do not redesign.

#### 1d. Gold-standard examples
- Write 15-20 hand-crafted (FaultEvent + context snapshot → RecoveryHint) pairs
- At least 5 per active category
- These are the specification, not the test set. A separate held-out set is needed for evaluation (see Phase 3d).
- Store in `data/gold_examples.jsonl`

#### Deliverables
- [ ] Verified fault taxonomy document (with actual detector output samples)
- [ ] Context snapshot schema
- [ ] RecoveryHint.msg changes (minimal delta from current)
- [ ] `data/gold_examples.jsonl` (15-20 examples)

---

### Phase 2: Context Infrastructure (Week 2-4)

**Objective:** Build bounded context buffers and the serialization pipeline that assembles a diagnosis-ready snapshot when a FaultEvent arrives.

This is prerequisite infrastructure. Without it, diagnosis has no input beyond the FaultEvent itself.

#### 2a. /rosout ring buffer
- New subscriber to /rosout (rcl_interfaces/msg/Log)
- Bounded: last N messages (default 20) or last T seconds (default 30s), whichever is smaller
- Stores: level, name, msg, stamp
- Memory bound: ~50 KB worst case for 20 messages

#### 2b. Metric rate cache
- New subscriber to /helix/metrics (Float64MultiArray from passive_adapter.py)
- Maintains per-metric rolling rate: current value, mean over last window
- Bounded: last 10 samples per metric
- **Dependency:** Requires passive_adapter.py to be running. If adapter is absent, cache is empty. Diagnosis node must handle this gracefully (context field is empty, not an error).

#### 2c. Node health snapshot
- Subscribe to `/helix/node_health` (already published by HeartbeatMonitor as DiagnosticArray)
- Cache latest health status per node (GREEN/YELLOW/RED)
- No new publisher needed -- this topic already exists

#### 2d. Context serializer
- On FaultEvent arrival: query all three buffers, assemble JSON context snapshot
- Attach to the FaultEvent to form the full diagnosis input
- Must complete in <1ms (buffers are in-memory, no I/O)

#### 2e. Testing
- Unit tests: buffer capacity limits, overflow behavior, empty-buffer handling
- Unit tests: serializer produces valid JSON matching schema
- Integration test: publish a FaultEvent, verify context snapshot is assembled with data from all three buffers

#### Deliverables
- [ ] `src/helix_core/helix_core/context_buffer.py`
- [ ] `src/helix_core/helix_core/context_serializer.py`
- [ ] Unit tests for both
- [ ] Integration test: FaultEvent → assembled context snapshot

---

### Phase 3: Baseline Diagnosis Node (Week 4-6)

**Objective:** Implement `diagnosis_node.py` with rule-based diagnosis. No model.

#### 3a. Rule-based diagnosis

Lookup table mapping (fault_type, context pattern) → (fault_category, severity_label, suggested_action, reasoning).

Rules are hand-written from the taxonomy:

| fault_type | Context signal | → fault_category | → suggested_action |
|-----------|---------------|-----------------|-------------------|
| ANOMALY | Z-score > 10 | rate_anomaly | "Check sensor connection; review DDS QoS for [topic]" |
| ANOMALY | Z-score 3-10 | rate_anomaly | "Monitor [topic] rate; may be transient jitter" |
| CRASH | node in RED | sensor_dropout | "Verify node is running; check topic publisher" |
| LOG_PATTERN | matched rule | log_fault | Action from log parser rule config or default |

Deterministic. Zero latency. No model. Confidence = 1.0. Method = "rules".

#### 3b. diagnosis_node.py
- ROS 2 lifecycle node (consistent with existing HELIX nodes)
- Subscribes to `/helix/faults`
- On FaultEvent: context_serializer.assemble() → rule lookup → publish RecoveryHint to `/helix/recovery`
- Configurable via YAML: diagnosis method, buffer sizes
- If context buffers are empty (no adapter, no /rosout traffic): still produces diagnosis from FaultEvent fields alone, with reduced context

#### 3c. Launch integration
- Add to `helix_sensing.launch.py` as optional node (disabled by default)
- YAML config in `config/diagnosis.yaml`

#### 3d. Evaluation
- **Gold example validation:** All 15-20 gold examples must produce correct fault_category and severity_label.
- **Held-out evaluation:** Write 30 additional synthetic FaultEvent+context cases (10 per category) that were NOT used to write the rules. Measure:
  - Fault category accuracy (exact match)
  - Suggested action correctness (manual binary judgment: useful or not)
  - Schema compliance (RecoveryHint parses correctly)
- Store in `data/held_out_eval.jsonl` and `results/diagnosis_baseline_eval.json`

#### Deliverables
- [ ] `src/helix_core/helix_core/diagnosis_node.py`
- [ ] `config/diagnosis.yaml`
- [ ] Updated launch file
- [ ] `data/held_out_eval.jsonl` (30 held-out cases)
- [ ] `results/diagnosis_baseline_eval.json`
- [ ] Unit + integration tests

---

### Phase 4: Hardware Validation (Week 6-9)

**Objective:** Run the full HELIX stack (3 sensing nodes + context buffers + diagnosis node) on Jetson. Validate on live GO2.

#### 4a. Prerequisites
- Sync T7 repo_clone artifacts to local repo and commit
- Build helix_msgs on Jetson (currently not built there)
- Build helix_core and helix_bringup on Jetson

#### 4b. Jetson deployment
1. Build all packages from source on Jetson
2. Launch full stack with rule-based diagnosis enabled
3. Run passive_adapter.py bridging GO2 topics
4. Verify `/helix/recovery` publishes RecoveryHints

#### 4c. Evaluation experiments

| ID | Experiment | Method | Success criterion |
|----|------------|--------|-------------------|
| EXP-20 | Diagnosis correctness | Inject 3+ controlled perturbations (kill topic, inject /rosout error, cause rate anomaly). Check RecoveryHint output. | Correct fault_category for all injected faults |
| EXP-21 | End-to-end latency | Measure FaultEvent.timestamp → RecoveryHint.stamp delta | <50ms for rule-based (should be <5ms) |
| EXP-22 | Resource overhead | Measure full-stack RSS + CPU on Jetson | Sensing + diagnosis < 100 MB RSS total |
| EXP-23 | Sustained run | 10-minute run with periodic fault injection every 60s | No memory growth, no missed FaultEvents, no crash |
| EXP-24 | Context quality | Inspect serialized context for 5+ real FaultEvents | Buffers contain relevant /rosout lines and rate data |

#### 4d. Collect labeled hardware examples
- Record all (FaultEvent, context snapshot, RecoveryHint, ground truth label) tuples from perturbation experiments
- These become the real-hardware evaluation set for Phase 5

#### Deliverables
- [ ] Jetson build confirmed
- [ ] EXP-20 through EXP-24 results with artifacts on T7
- [ ] Labeled hardware examples
- [ ] End-to-end demo on live GO2

---

### Phase 5: Optional LLM Comparison (Week 9-12)

**Objective:** Evaluate whether a local small LLM produces better diagnoses than the rule-based baseline. This phase is optional. If skipped, the paper reports the rule-based system only.

#### 5a. Approach: start cheap, stop early if no signal

**Step 1: Prompt-only (no training, no fine-tuning)**
- Run a small model via Ollama on mewtwo (Llama 3.2 3B, Phi-3-mini, Qwen 2.5 3B)
- System prompt: fault taxonomy + 5 few-shot gold examples
- Input: same serialized FaultEvent + context as the rule-based path
- Evaluate on the same held-out set (30 cases) + hardware examples from Phase 4
- Compare fault_category accuracy and action correctness to baseline
- **Gate:** If prompt-only does not beat rules on category accuracy, stop. Report as negative result.

**Step 2: Minimal fine-tuning (only if Step 1 shows signal)**
- Generate synthetic corpus (1,000-3,000 examples) from taxonomy definitions
- QLoRA on mewtwo RTX 5070 (4-bit, LoRA rank 16)
- Evaluate on held-out + hardware sets
- Quantize to 4-bit GGUF for Jetson
- **Gate:** If fine-tuned model does not beat prompt-only, stop.

**Step 3: Jetson deployment (only if Step 2 succeeds)**
- Run diagnosis_node with `method: "llm"` on Jetson
- Measure: latency, memory, total stack resource usage
- **Gate:** If total stack exceeds Jetson budget, stop.

#### 5b. Evaluation metrics (same as baseline, apples-to-apples)

| Metric | Method |
|--------|--------|
| Fault category accuracy | Exact match on held-out + hardware sets |
| Action correctness | Manual binary: useful or not (50 outputs) |
| Schema compliance | RecoveryHint parses, all fields populated |
| Operator usefulness | Manual 1-5 rating on 20 outputs |
| Latency (Jetson) | FaultEvent → RecoveryHint wall clock |
| Memory (Jetson) | Full-stack RSS |

No ROUGE. No text similarity metrics. Correctness and usefulness only.

#### 5c. Deliverables
- [ ] Step 1 results (prompt-only vs rules)
- [ ] Go/no-go decision documented
- [ ] If proceeding: corpus, training script, model, quantized export, comparative eval
- [ ] If stopping: negative result documented with data

---

### Phase 6: Paper (Week 12-14)

#### Thesis

> **HELIX: Tiered Fault Sensing and Structured Diagnosis for ROS 2 Robots**

Core claim: a lightweight, deterministic tiered architecture that detects faults via statistical/pattern methods and produces structured diagnoses, deployable on edge hardware alongside a live robot.

If Phase 5 produced positive results: "We additionally evaluate a local small language model as a diagnosis enhancement and report [improvement / no improvement] over the rule-based baseline."

#### Contributions
1. Tiered sensing-to-diagnosis architecture with explicit context assembly
2. Attachability framework quantifying monitoring portability on non-standard platforms
3. Hardware-grounded evaluation on Unitree GO2 + Jetson Orin NX (detection + diagnosis + resource overhead)
4. (If applicable) Comparative evaluation of rule-based vs LLM diagnosis on edge hardware

#### Key sections
- Architecture (tiered design, context infrastructure, lifecycle nodes)
- Fault taxonomy (categories, signal paths, what is and isn't observable)
- Baseline diagnosis (rules, gold examples, held-out accuracy)
- Hardware evaluation (sensing benchmarks, diagnosis correctness, end-to-end latency, resource overhead)
- Attachability (GO2 topic landscape, adapter approach, measured coverage)
- (If applicable) LLM comparison
- Limitations (short observation windows, 3 fault categories, single platform, no confirmed real faults)

#### Target venue
Decide after Phase 4 results. Candidates: IROS 2026, pedestrian workshop, RSS/ICRA workshop.

---

## 6. Risk Register

| Risk | Impact | Mitigation |
|------|--------|------------|
| Rule-based diagnosis is trivial | Weak contribution | The contribution is the architecture + context pipeline + hardware eval, not the rules themselves. Rules are the baseline, not the thesis. |
| Context buffers empty when adapter not running | Diagnosis degrades | Handle gracefully: diagnose from FaultEvent alone, note reduced confidence. Document as deployment requirement. |
| helix_msgs fails to build on Jetson | Phase 4 blocked | Address early (week 1). Known risk from Session 1+2 notes. |
| Only 3 fault categories | Narrow scope | Honest about it. Paper contribution is the architecture pattern, not breadth of fault coverage. |
| LLM adds nothing over rules | Phase 5 is negative | Fine. Paper doesn't depend on it. Negative result is still publishable data. |
| No confirmed real faults on GO2 | Can't claim fault detection accuracy | Use controlled perturbation (proven safe in Session 2). Be explicit: these are injected, not naturally occurring. |

---

## 7. Hardware & Software

### Deployment (Jetson Orin NX)
- JetPack R36.4.7, 16 GB unified RAM
- ROS 2 Humble
- HELIX packages built from source
- (Phase 5 only) Ollama or llama.cpp

### Training (mewtwo, Phase 5 only)
- RTX 5070, 12 GB VRAM, CUDA (open kernel modules)
- Python 3.10+, PyTorch, transformers, peft/unsloth

### Data collection
- Unitree GO2 at 192.168.123.161
- Wired Ethernet 192.168.123.0/24
- CycloneDDS (PC) / FastRTPS (Jetson/GO2)
- passive_adapter.py bridging GO2 topics to /helix/metrics

---

## 8. Proposed File Additions

```
src/helix_core/helix_core/
  context_buffer.py            # ring buffers for /rosout, rates, health
  context_serializer.py        # fault-to-context snapshot assembly
  diagnosis_node.py            # lifecycle diagnosis node (rules + optional LLM)

src/helix_msgs/msg/
  RecoveryHint.msg             # add fault_category, severity_label, method fields

data/
  fault_taxonomy.yaml          # categories, signal paths, detector mappings
  gold_examples.jsonl          # 15-20 reference pairs (specification)
  held_out_eval.jsonl          # 30 held-out evaluation cases

config/
  diagnosis.yaml               # method, buffer sizes, thresholds

results/
  diagnosis_baseline_eval.json # rule-based accuracy on held-out set
  diagnosis_llm_eval.json      # LLM accuracy (Phase 5 only)
  jetson_diagnosis_bench.json  # latency + memory on Jetson

scripts/                       # Phase 5 only
  generate_fault_corpus.py
  train_diagnosis_model.py
  eval_diagnosis_model.py
```

---

## 9. Success Criteria

The project succeeds when:

1. HELIX publishes structured `RecoveryHint` messages from real FaultEvents on the GO2
2. The full stack (sensing + context + diagnosis) runs on a Jetson Orin NX within resource budget
3. Rule-based diagnosis is correct on the held-out evaluation set (30 cases, target >90% category accuracy)
4. At least one end-to-end demo on live GO2: controlled perturbation → FaultEvent → context → RecoveryHint
5. Paper draft complete with hardware evaluation results

**The project does not require an LLM to succeed.** Phase 5 is a comparative experiment. A negative LLM result is a valid outcome.
