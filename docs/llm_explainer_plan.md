# Local LLM Plan for `LLMExplainer`

**Status:** Decision document. No model was downloaded and no code was changed for this plan.
**Scope:** pick the model, pick the runtime, fix the three prompt roles, fix the interface boundary
that keeps the layer advisory, and set the latency budget.
**Related:** `docs/LLAMA_SERVER_JETSON_SETUP.md` (deployment runbook),
`src/helix_explanation/helix_explanation/llm_client.py` (the client this plan configures),
the author's private engineering notes (local LLM survey, 2026-04-18, not in this repo; the survey
this plan revises).
**Date:** 2026-08-24

---

## 0. Correction to the project's own description of this task

The task list describes an "`LLMExplainer` v1 stub that must become real local inference". That is out
of date. The current tree has:

| Component | State | File |
|---|---|---|
| `LLMExplainer` | **v2**, not a stub. Deterministic template path plus an optional LLM advisory path behind `llm_enabled` | `src/helix_explanation/helix_explanation/llm_explainer.py` |
| `LLMClient` / `AsyncLLMClient` | Implemented: OpenAI-compatible HTTP, three role enums, per-role JSON schemas, a dependency-free schema validator, typed error hierarchy, single-worker thread pool | `src/helix_explanation/helix_explanation/llm_client.py` (418 lines) |
| Three role prompts | Implemented as `_SYSTEM_PROMPTS` | same file |
| Jetson deployment runbook | Written: JetPack checks, source build, systemd unit with `MemoryMax`, health check, model swap procedure | `docs/LLAMA_SERVER_JETSON_SETUP.md` (396 lines) |
| Ship default | `llm_enabled` defaults to `False` (`llm_explainer.py`, "ship OFF") | |

So the remaining work is not "write the integration". It is: **choose and verify the model, choose and
verify the runtime on real arm64, and produce the measurements that justify turning `llm_enabled` on.**
This document covers the choosing and specifies the measuring. Nothing here has been measured on a
Jetson, because the Jetson was not available; every Jetson number below is labelled as an estimate or
a gate, never as a result.

---

## 1. Recommendation summary

| Decision | Choice | Evidence |
|---|---|---|
| Model | **Qwen2.5-1.5B-Instruct, Q4_K_M GGUF** | 1.117 GB, read from the HuggingFace API (section 2.1) |
| Backup model | **Gemma-3-1B-it, Q4_K_M GGUF** | 0.806 GB, same method |
| Runtime on the Jetson | **Ollama**, JetPack 6 build | first-class prebuilt arm64/JetPack6 artifact, 268 MB (section 3.1) |
| Runtime fallback | **llama.cpp `llama-server`**, built from source | no prebuilt arm64 binary in the current release (section 3.1) |
| Structured output | JSON schema constraint, **verified working** on the OpenAI-compatible endpoint | measured locally, section 3.3 |
| Client changes needed | **None** for either runtime | both speak `/v1/chat/completions`; section 3.4 |
| Roles | one model, three system prompts, three schemas | section 4 |
| Safety boundary | advisory only, three independent gates | section 5 |
| Budget | < 2 GB RSS for the whole subsystem; p95 first token < 500 ms; p95 total < 5 s | section 6 |

This **changes the April survey's runtime recommendation** (llama.cpp primary, Ollama backup) to the
reverse, for the reason in section 3.1. It does not change the model recommendation.

---

## 2. Model

### 2.1 Verified sizes

Read from the HuggingFace model API (`GET /api/models/<repo>?blobs=true`, `siblings[].size`) on
2026-08-24. These are on-disk GGUF file sizes, not RSS.

| Model | Repo | Q4_K_M file | Size |
|---|---|---|---|
| **Qwen2.5-1.5B-Instruct** | `Qwen/Qwen2.5-1.5B-Instruct-GGUF` | `qwen2.5-1.5b-instruct-q4_k_m.gguf` | **1.117 GB** |
| Gemma-3-1B-it | `unsloth/gemma-3-1b-it-GGUF` | `gemma-3-1b-it-Q4_K_M.gguf` | **0.806 GB** |
| Llama-3.2-1B-Instruct | `bartowski/Llama-3.2-1B-Instruct-GGUF` | `Llama-3.2-1B-Instruct-Q4_K_M.gguf` | **0.808 GB** |
| Qwen3-1.7B | `Qwen/Qwen3-1.7B-GGUF` | no Q4_K_M published in that repo | only `Qwen3-1.7B-Q8_0.gguf`, 1.834 GB |

Note on the last row: the April survey listed Qwen3-1.7B Q4_K_M at "~1.2 GB" as the escalation path.
The official Qwen repo publishes only Q8_0 (1.834 GB), which leaves no room for KV cache and runtime
inside a 2 GB cap. If Qwen3-1.7B is ever needed, it must come from a third-party quantiser and the
size must be re-verified before it is planned around.

### 2.2 Why Qwen2.5-1.5B-Instruct Q4_K_M

1. **It fits with margin.** 1.117 GB of weights. With `--ctx-size 1024` the KV cache for a 1.5B model
   with 28 layers is a few tens of MB, and the server process adds a couple of hundred. ASSUMPTION:
   total subsystem RSS lands in the 1.5 to 1.8 GB band. This is an estimate from file size plus
   typical runtime overhead, **not a measurement**, and it is the single number the Jetson gate in
   section 6.3 exists to check.
2. **Apache-2.0.** No Gemma prohibited-use policy to review, no Llama monthly-active-user threshold to
   track. HELIX is a public repo with a public demo video; a licence with no conditions attached is
   worth 300 MB.
3. **The task is small.** The Healer picks one of three enum values and writes one sentence. The
   Flagger returns a boolean and a sentence. The Predictor returns a boolean, a string and a number.
   None of these is a chain-of-thought problem. Instruction-following and schema adherence are what
   matter, and those are what a 1.5B instruct model at Q4_K_M can do.
4. **One model, three roles.** The roles differ only by system prompt and schema
   (`llm_client.py:_SYSTEM_PROMPTS`, `_SCHEMAS`). Loading two models to save 300 MB would cost a second
   warm-load, a second eval axis, and a second failure mode, in exchange for headroom that section 6.3
   will tell us whether we need.

**Fallback trigger.** If the Jetson RSS gate fails, drop to Gemma-3-1B-it Q4_K_M (0.806 GB, a 311 MB
saving). `docs/LLAMA_SERVER_JETSON_SETUP.md` section 7 already documents the swap. Flag the Gemma
terms in the repo's licence notes if it ships in the deliverable.

### 2.3 What is actually available locally right now

Standing rule: check before pulling. Command run on 2026-08-24, output verbatim:

```
$ ollama list
NAME                          ID              SIZE      MODIFIED
qwen2.5-coder:32b-instruct    b92d6a0bd47e    19 GB     3 months ago
qwen2.5-coder:32b             b92d6a0bd47e    19 GB     3 months ago
qwen3.6:27b                   a50eda8ed977    17 GB     3 months ago
deepseek-coder-v2:16b         63fb193b3a9b    8.9 GB    4 months ago
llama3.1:8b                   46e0c10c039e    4.9 GB    4 months ago
qwen2.5-coder:14b             9ec8897f747e    9.0 GB    4 months ago
qwen2.5-coder:latest          dae161e27b0e    4.7 GB    4 months ago
mistral:latest                6577803aa9a0    4.4 GB    4 months ago
```

**Nothing under 2 GB is present.** The smallest local model is `mistral:latest` at 4.4 GB, which is
over twice the Jetson budget. Every model here is a coding or general workstation model; none is a
candidate for the robot.

So the workstation evaluation in section 7 needs exactly one pull. Manifest size read from the Ollama
registry API without pulling:

| Tag | Model layer size | Pull command (**not run**) |
|---|---|---|
| `qwen2.5:1.5b-instruct-q4_K_M` | **0.986 GB** | `ollama pull qwen2.5:1.5b-instruct-q4_K_M` |
| `gemma3:1b` | 0.815 GB | `ollama pull gemma3:1b` |
| `llama3.2:1b` | 1.321 GB | `ollama pull llama3.2:1b` |

Under 1 GB each. Pull the first when the evaluation actually starts, not before.

Local Ollama version is **0.18.3**; current release is **0.32.15** (2026-08-19). Upgrade the
workstation before the evaluation so the workstation and the robot run the same generation of the
runtime, otherwise a prompt that works in one place proves nothing about the other.

---

## 3. Runtime

### 3.1 The arm64 reality check, actually checked

The April survey chose llama.cpp over Ollama on the grounds that Ollama costs "~150 MB more RSS" and a
"7 GB disk image". The disk figure in particular does not survive contact with the current releases.
Read from the GitHub releases API on 2026-08-24:

**Ollama, release `v0.32.15` (published 2026-08-19), arm64 assets:**

```
ollama-linux-arm64-jetpack5.tar.zst   0.296 GB
ollama-linux-arm64-jetpack6.tar.zst   0.268 GB
ollama-linux-arm64.tar.zst            1.543 GB
```

There is a **dedicated JetPack 6 build, 268 MB compressed**. Not a generic arm64 tarball with CUDA
detection, a JetPack-specific artifact. The Jetson is exactly the target this asset exists for.

**llama.cpp, latest release `v0.2.0` (published 2026-08-21):**

```
assets: ['nightly-tag.txt']
arm64/aarch64 assets: []
```

**One asset, and it is a text file.** There is no prebuilt binary of any architecture in that release,
so a Jetson deployment means a source build with `-DGGML_CUDA=on` against the JetPack CUDA toolkit,
which is precisely what `docs/LLAMA_SERVER_JETSON_SETUP.md` section 2 walks through. That build works,
but it is a build: a compiler toolchain on the robot, a pinned tag to track, and a rebuild every time
CUDA or the tag moves.

**Therefore: Ollama is the primary runtime for the Jetson, llama.cpp is the fallback.** A 268 MB
vendor-built artifact that the vendor tests on this exact hardware beats a source build for something
that is, by design, an optional advisory sidecar. Operational surface is the cost that matters here,
not 150 MB.

ASSUMPTION, and it is the one that could reverse this: the ~150 to 300 MB of extra Ollama daemon RSS
is real and unmeasured on this hardware. If the Jetson gate in 6.3 fails by less than ~300 MB,
switching to a source-built `llama-server` is the first remedy to try, ahead of shrinking the model.
The runbook for that path is already written.

### 3.2 Why this reversal is cheap

Both runtimes serve an OpenAI-compatible `/v1/chat/completions`. `llm_client.py` targets that
endpoint and takes its base URL from a ROS parameter (`llama_server_url`, default
`http://localhost:8080`). Switching runtimes is a parameter change plus a port number. No client code
changes either way, and the decision stays reversible after the Jetson measurement, which is exactly
the property a not-yet-measured decision should have.

### 3.3 Structured output: measured, not assumed

The whole design rests on the model being unable to emit an action outside the allowlist. That is a
claim about the runtime's constrained decoding, so it was tested rather than assumed.

Test 1, the real Healer schema, against the local Ollama 0.18.3 OpenAI-compatible endpoint:

```
$ curl -s http://localhost:11434/v1/chat/completions -H "Content-Type: application/json" -d '{
   "model":"mistral:latest",
   "messages":[{"role":"system","content":"Respond ONLY with JSON matching the schema."},
               {"role":"user","content":"Fault: rate_hz/utlidar_cloud Z=8.4, dropped 15.3 to 3.1 Hz."}],
   "response_format":{"type":"json_schema","json_schema":{"name":"healer","schema":{
     "type":"object","additionalProperties":false,
     "properties":{"action":{"type":"string","enum":["STOP_AND_HOLD","RESUME","LOG_ONLY"]},
                   "confidence":{"type":"number"},"reasoning":{"type":"string"}},
     "required":["action","confidence","reasoning"]}}},
   "temperature":0.2,"max_tokens":192}'
```

Returned, schema-valid:

```json
{"action": "RESUME", "confidence": 0.9, "reasoning": "The Fault report indicates a significant decrease in the rate of data from the utlidar_cloud device ..."}
```

Test 2, adversarial, to prove the constraint is genuinely enforced rather than the model merely
cooperating. Prompt asks for prose about the capital of France; schema demands an enum of two
nonsense tokens:

```
"messages":[{"role":"user","content":"What is the capital of France? Answer in plain English prose."}],
"response_format": ... "properties":{"zzq":{"type":"string","enum":["QQQ_ALPHA","QQQ_BETA"]}} ...
```

Returned: `'{\n\n  "zzq" : "QQQ_ALPHA"\n\n}\n\n \t\t...'`

The model could not escape the grammar even when the prompt pulled hard the other way. **Constrained
decoding is real on this path.**

Three caveats, stated because they bound what the test proves:

- Measured with `mistral:latest` on the workstation, not with Qwen2.5-1.5B on a Jetson. It validates
  the **runtime**, not the model choice.
- Ollama's documented structured-output surface is the `format` field on its native `/api/chat` and
  `/api/generate` endpoints (`docs/api.md`, "Structured outputs are supported by providing a JSON
  schema in the `format` parameter"). The OpenAI-compat `response_format` path is not documented in
  the file that was read; it is verified empirically above. Re-run test 2 after any Ollama upgrade,
  as a pinned regression test. It costs one second.
- Test 2's trailing whitespace run is a cosmetic artefact of grammar-constrained generation. The
  client already does `json.loads` on the content, which tolerates it. Worth a test case so it stays
  tolerated.

**Action:** add both calls to the repo as `tests/test_llm_runtime_contract.py`, skipped when no server
is reachable. Test 2 is the load-bearing one and it must be an explicit test, not a comment.

### 3.4 Deployment shape (unchanged from the runbook)

Sidecar daemon on loopback, never in-process. The three reasons in the April survey all hold and none
depends on which runtime is chosen: crash isolation (an OOM kills the sidecar, not the ROS node),
warm load (a 1.1 GB GGUF cannot be paged in per call inside a 500 ms budget), and visible, cappable
memory accounting (`systemd MemoryMax=`, already in the runbook).

If Ollama is primary, the systemd unit in the runbook section 5 becomes an `ollama serve` unit on port
11434, `OLLAMA_KEEP_ALIVE=-1` to hold the model resident, `OLLAMA_MAX_LOADED_MODELS=1` and
`OLLAMA_NUM_PARALLEL=1` so a second model can never be loaded behind our back and blow the cap. Set
`llama_server_url` to `http://127.0.0.1:11434`. `MemoryMax=` stays, sized from the measurement in 6.3
rather than guessed.

---

## 4. Prompt designs

All three share one loaded model, one endpoint, `temperature: 0.2`, `max_tokens: 192`. They differ by
system prompt and output schema only. The implemented versions live in `llm_client.py`
(`_SYSTEM_PROMPTS`, `HEALER_SCHEMA`, `FLAGGER_SCHEMA`, `PREDICTOR_SCHEMA`); this section is the design
of record and states where the implementation should change.

### 4.1 Healer: suggest a recovery action from fault context

**Trigger.** A `RecoveryHint` arrives on `/helix/recovery_hints`, joined against the most recent
`FaultEvent` (`llm_explainer.py::_on_hint`). The rule-based hint has **already been produced and
published** by the time the LLM is asked. The Healer is therefore a second opinion on a decision
already taken, not the decision itself. That ordering is a safety property, not an implementation
detail, and it must not be inverted for latency reasons.

**Input.** `fault_event` serialised by `_fault_to_dict` (node_name, fault_type, severity, detail,
timestamp, context_keys, context_values) plus a `context_snapshot`.

**System prompt** (implemented): names the robot and middleware, enumerates the allowlist with a
one-line gloss each, forbids inventing actions, specifies `LOG_ONLY` with low confidence as the
"unsure" answer, and states in-prompt that the output is advisory and a deterministic enforcer is the
hard gate. Telling the model it is not the decider is cheap and measurably reduces confident
overreach.

**Output schema** (implemented, `HEALER_SCHEMA`): `action` enum over `RECOVERY_ALLOWLIST`,
`confidence` in [0, 1], `reasoning` 1 to 240 chars, `additionalProperties: false`.

**Three changes this plan asks for:**

1. **Add the rule-based decision to the input and ask the model to agree or dissent.** The interesting
   signal is not "what would you do", it is "the rules said STOP_AND_HOLD, do you disagree and why".
   Dissent is a reviewable event; a duplicate opinion is noise. This is also the only framing under
   which the layer can eventually be evaluated against the rules on real data.
2. **Fill in the context snapshot.** It is currently `{'hint': ...}` with a `# Future:` comment
   (`llm_explainer.py::_on_hint`). Wire in a `/rosout` ring buffer and a per-topic metric summary. A
   Healer that sees only the hint it is grading is close to useless. Note the P2 task
   "Task 7: /rosout ring buffer node" in the project backlog: it is a dependency of this, not a
   separate nice-to-have.
3. **Bound the snapshot size.** Cap the serialised snapshot at 800 tokens so `--ctx-size 1024` cannot
   be overrun. Enforce it in the serialiser with a test, not by hoping.

### 4.2 Flagger: novel fault pattern detection

**Trigger.** Not fault-driven. A timer, at a low rate (start at 0.1 Hz, one call per 10 s), so the
Flagger can speak during apparently healthy operation, which is the whole point.

**Input.** The same `context_snapshot` with no `fault_event`. `LLMRequest.fault_event` is already
`Optional`, so the client supports this today.

**System prompt** (implemented): asks whether anything in a window of `/rosout` lines, per-topic metric
rates and node health looks unusual versus healthy steady state of a GO2 plus Nav2 stack.

**Output schema** (implemented, `FLAGGER_SCHEMA`): `novel` boolean, `why` 1 to 240 chars.

**Changes this plan asks for:**

1. **Add a `confidence` number.** The gate the survey describes is `novel && confidence > 0.5`, but
   `FLAGGER_SCHEMA` has no confidence field, so the gate is currently unimplementable. Add it.
2. **Give the model a baseline.** "Unusual compared to healthy steady state" is unanswerable without
   knowing what healthy looks like. Inject the measured steady-state rates for the six configured
   topics from the analysis artifacts (`analysis/data/`), as a compact table in the user message.
   Without this the Flagger is guessing at nominal Hz for a robot it has never seen.
3. **Rate-limit the output, not just the input.** Suppress repeat flags with identical `why` inside a
   60 s window. A 0.1 Hz timer that flags the same thing 360 times an hour trains the operator to
   ignore it.
4. **Publish to a dedicated topic**, `/helix/flags` (`std_msgs/String` JSON, `QoS(10).reliable()`), not
   onto `/helix/explanations`. Explanations are per-fault; flags are proactive and have no fault to
   attach to.

### 4.3 Predictor: precursor-state warning

**Trigger.** Same timer as the Flagger, same window, one extra call. Do not merge the two into one
prompt: distinct schemas keep the two questions separable, and a merged prompt makes a "no novelty"
answer contaminate the precursor answer.

**System prompt** (implemented): names three known failure modes explicitly, `sensor_dropout`
(topic silence), `rate_degradation` (sustained sub-nominal rate), `dds_partition` (QoS or discovery
warnings in `/rosout`), and instructs `precursor_detected=false` when none apply.

**Output schema** (implemented, `PREDICTOR_SCHEMA`): `precursor_detected` boolean, `what` 1 to 240
chars, `confidence` in [0, 1].

**Changes this plan asks for:**

1. **Add `mode` as an enum** over the three named failure modes plus `none`. A free-text `what` cannot
   be scored. An enum can, which is what makes the Predictor evaluable at all.
2. **Add `horizon_seconds`** as a bounded integer (1 to 60). A precursor warning with no time horizon
   cannot be checked against what happened next, so it cannot be shown to be right or wrong.
3. **Anchor the failure modes to real events.** All three named modes correspond to things HELIX has
   already recorded: LiDAR rate collapse, stale-topic NaN, DDS QoS mismatch. Put one concrete example
   of each in the system prompt, drawn from the evidence bags. A 1.5B model does much better matching
   against examples than against category names.
4. **Expect this role to fail first.** Prediction is genuinely harder than the other two and the
   evaluation in section 7 should be prepared to report that the Predictor does not beat "always say
   none". That is a publishable result, not a failure of the work.

---

## 5. The interface boundary that keeps this advisory

Four independent properties, each of which alone would be sufficient. Any change that weakens one of
them is a safety change and needs an explicit decision record.

### 5.1 There is no path from LLM output to actuation

The Healer's `action` is published as JSON on `/helix/llm_diagnostics` and rendered into a human string
on `/helix/explanations`. **Neither topic is subscribed by `helix_recovery`.** The only input to
actuation is `/helix/recovery_hints`, produced by the rule engine. The LLM does not publish there and
must never be given permission to. This is the primary gate and it is structural: it holds even if
every other gate is removed, because the wire does not exist.

Regression test: assert that `helix_recovery` subscribes to no topic the explanation tier publishes.
Cheap, static, and it catches the exact refactor that would quietly break this.

### 5.2 The rule-based answer is computed and published first

`_on_hint` renders and publishes the deterministic template **before** submitting anything to the LLM
(`llm_explainer.py`, step 1 then step 2). If the LLM path is disabled, times out, errors, or returns
garbage, the operator-facing output has already been published. The LLM cannot delay, block, or
suppress the deterministic answer.

### 5.3 The allowlist is enforced twice, independently

- Grammar-level, at generation: the enum in `HEALER_SCHEMA`, verified enforced in section 3.3.
- Code-level, downstream: `helix_recovery`'s envelope returns `SUPPRESSED_ALLOWLIST` for any action
  outside `{STOP_AND_HOLD, RESUME, LOG_ONLY}` (`recovery_node.py:53`).

The second gate does not trust the first. The parity test the survey specifies is right and should be
implemented: strip the grammar constraint, run 100 adversarial cases, confirm the downstream gate
still rejects every invalid action. That test proves the second gate works **without** the first,
which is the only version of the claim worth making.

### 5.4 The explanation node is not on the critical path at all

`LLMExplainer` is a plain `rclpy.node.Node`, not a lifecycle node, and it subscribes only. If the
process dies, `helix_diagnosis` and `helix_recovery` continue unaffected: the closed loop loses its
narration, not its behaviour. The module docstring states this ("If this node crashes, the robot still
recovers") and the topology honours it.

The LLM call runs on a single-worker `ThreadPoolExecutor` and never on the ROS executor
(`AsyncLLMClient`), so a slow or hung sidecar cannot apply back-pressure to callback dispatch. `max_workers=1` also caps concurrent inferences at one, which is what keeps the memory budget a
budget: two concurrent calls would double the KV cache.

### 5.5 Failure is logged, never silent

Every failure mode maps to a typed exception (`LLMTimeoutError`, `LLMHTTPError`, `LLMTransportError`,
`LLMInvalidResponseError`) and publishes a diagnostic JSON on `/helix/llm_diagnostics` with the error
type. The layer's own failure rate is therefore observable in a bag, which is what turns "the LLM
sometimes times out" from an anecdote into a number.

**One gap:** `llm_enabled` is read once in `__init__` and never re-read, so the layer cannot be turned
off at runtime without restarting the node. For an advisory subsystem sharing memory with a safety
stack, a runtime kill switch is worth having. Add a parameter callback, or a
`/helix/llm_enabled` `std_msgs/Bool` subscription, so an operator can shed the load in one command.

---

## 6. Latency and memory budget

### 6.1 What the budget has to satisfy

The Healer is not in the loop, so its latency does not gate any recovery action. The binding
constraint is different: **the advisory must arrive while the fault it describes is still the current
situation.** The diagnosis state machine's anomaly clear window is 3.0 s
(`ANOMALY_CLEAR_WINDOW_SECONDS` in `helix_diagnosis/rules.py`, referenced in
`docs/CPP_PORT_DESIGN_ANOMALY_DETECTOR.md` section 9). An advisory that lands after the state has
already cleared is describing history.

### 6.2 Budget

| Quantity | Budget | Rationale |
|---|---|---|
| First token, p95 | < 500 ms | prompt processing must not dominate |
| Total response, p95 | < 5 s | client timeout is 6.0 s (`llm_timeout_s` default), so a p95 at 5 s leaves the timeout as a genuine outlier catcher rather than a routine path |
| Total response, p50 | < 2.5 s | keeps the median advisory inside the 3.0 s clear window |
| Output length | 192 tokens max | set by `max_tokens`; the schemas cap `reasoning` at 240 characters anyway |
| Context | 1024 tokens | fixes the KV cache size; the snapshot serialiser must be capped at 800 tokens to guarantee it |
| Subsystem RSS | **< 1.8 GB** | 200 MB of headroom under the stated 2 GB cap |
| Concurrency | 1 | `max_workers=1` plus `OLLAMA_NUM_PARALLEL=1` |
| Calls per second | <= 0.3 | Healer is fault-driven (0.55/min measured idle: 33 faults in 30 min, `docs/cpp_parity_summary.md`), Flagger and Predictor at 0.1 Hz each |

### 6.3 The gates that turn these into results

None of the following has been run. Each is a gate with a defined failure action.

| # | Gate | Where | Fail action |
|---|---|---|---|
| G1 | Schema validity 100% over 20 gold examples x 10 runs | workstation | fix prompt or grammar; do not proceed |
| G2 | Healer category accuracy versus the rule baseline on a held-out set | workstation | if the LLM does not beat rules, that is the result. Report it and keep `llm_enabled: false` |
| G3 | Peak RSS of the sidecar under sustained load < 1.8 GB | Jetson, `systemd-cgtop` on the unit's cgroup | try `llama-server` (saves the daemon overhead), then Gemma-3-1B (saves 311 MB), in that order |
| G4 | p95 first token < 500 ms, p95 total < 5 s, 100 Healer calls | Jetson, Super Mode confirmed via `nvpmodel -q` first | drop to Gemma-3-1B, then reduce `--ctx-size` |
| G5 | HELIX stack RSS and CPU delta with the sidecar running versus not, 30 min | Jetson | if the sidecar degrades the sensing tier, it does not ship |
| G6 | Adversarial allowlist test: grammar off, 100 invalid actions, downstream gate rejects 100/100 | workstation | blocking. This one is not negotiable |

G5 deserves emphasis. This plan adds a ~1.5 GB resident process to the same Jetson that the C++ port
plan is trying to free memory on. The two efforts must be measured together, not separately, or
neither number means anything. Run G5 against the same baseline the C++ re-benchmark protocol
establishes (`docs/cpp_port_plan.md` section 7.1).

---

## 7. Sequenced next steps

Nothing here requires the robot until step 4.

1. **Upgrade the workstation Ollama** from 0.18.3 to current, so workstation and robot match.
2. **Pull one model**, `qwen2.5:1.5b-instruct-q4_K_M` (0.986 GB). Not yet done, deliberately.
3. **Build the gold set**: 20 examples drawn from real `FaultEvent` records in the existing evidence
   bags, each labelled with the action the rule engine chose and the action a human considers right.
   Run G1, G2, G6 on the workstation. This is the step that decides whether the layer is worth
   deploying, and it needs no Jetson.
4. **Only if G2 passes**: install the JetPack 6 Ollama build on the Jetson, run G3, G4, G5.
5. **Only if G3 to G5 pass**: flip `llm_enabled` to true in a launch profile, never in the default
   `helix_params.yaml`.

Steps 1 to 3 are desk work. Step 4 is one lab session and should be attached to an existing hardware
session rather than justifying its own.

---

## 8. Standing caveat

A 1.5B model at 4-bit will hallucinate and will miscategorise. Every gate in section 5 exists because
that is assumed, not feared. The layer's value is a second opinion that is logged, compared, and
eventually scored against the rule engine. It is never a control input, and the day it becomes one is
the day HELIX stops being able to make any claim about its own safety.

---

*Decision document. No model downloaded, no code changed.*
