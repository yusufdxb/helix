# HELIX agent: what the literature says about learned diagnosis and repair

Scope: fault diagnosis and repair-action selection for ROS 2 software, done
on-robot by a small model. Reviewed 2026-11-02. "V" means the number was read
from the paper's abstract or HTML. "UNVERIFIED" means it came from a search
snippet or third-party summary and must be opened before it is quoted.

## ROS and robotics self-adaptation

- **MROS**, arXiv 2010.09145 (V, abstract). An ontology and MAPE-K metacontrol
  framework for ROS. It reasons over a hand-built model, so it adapts only to
  faults the model already names.
- **SUAVE**, arXiv 2303.09220, SEAMS 2023 (V). A ROS 2 underwater exemplar for
  metacontrol. It is a benchmark harness, not a learned method.
- **ROSMonitoring 2.0**, arXiv 2411.14367, FMAS 2024 (V). Runtime verification of
  topics and services. It detects; it does not repair.
- **INPROVF**, arXiv 2503.13660 (UNVERIFIED beyond the abstract). An LLM proposes
  controller repairs and formal verification accepts or rejects them. The
  verify-before-accept pattern is the part HELIX adopts.
- No shared ROS 2 fault-injection-plus-repair benchmark was found. Telemetry
  anomaly-detection work on ROS (autoencoder/LSTM on TurtleBot3, MAVFI) trains on
  nominal data and reports detection, not repair (UNVERIFIED).

## LLM agents for incident root cause and remediation (cloud AIOps)

- **AIOpsLab**, arXiv 2501.06706 (V). Mitigation success: GPT-4 with a shell 27%,
  GPT-3.5 0%, ReAct 36%, Flash 55%, on about 13 cases per cell. Reported failure
  modes: invented APIs, looping calls, context overflow from raw telemetry, false
  positives on no-fault cases. No unsafe-action rate is reported.
- **ITBench**, arXiv 2502.05352 (V, v1 abstract). 13.8% of SRE scenarios solved.
- **Cloud-OpsBench**, arXiv 2603.00468 (V). Best agent 0.76 root-cause accuracy
  but 0.38 evidence closure: right answers without grounded evidence.
- **RCACopilot**, arXiv 2305.15778, EuroSys 2024 (V). Up to 0.766 accuracy; the
  evidence is collected by hand-written handlers and the LLM only classifies and
  summarizes.
- **RCAgent**, arXiv 2310.16340 (V). Self-hosted 13B model. Invalid output 22.8%
  for ReAct, 7.9% with RCAgent's filters; an XGBoost baseline reached 77.7%
  against RCAgent's 80.7% responsibility precision.
- **OpenRCA**, ICLR 2025: best agent 11.34% (UNVERIFIED, third-party summary).
- **Nissist**, arXiv 2402.17531 (UNVERIFIED): retrieval over runbooks, not free
  action generation. **E2E-REME**, arXiv 2604.11094 (V, abstract): RL fine-tuned
  remediation LLM; the abstract gives no numbers.
- No result was found for a model under 3B parameters doing diagnosis or
  remediation, and none measures unsafe-action rates for LLM remediation.

## Learned policies for recovery selection

- **Safe Remediation as Risk-Constrained Intervention**, arXiv 2607.20005 (V,
  abstract). Offline constrained MDP with a risk gate. Over a runbook baseline it
  cut false remediations by 39% and raised repair success by 2.5 points. This is
  the closest match to a closed, safety-gated action set, and the gain is modest.
- Contextual bandits are reported to adapt in few interactions; DQN-style agents
  for microservice healing need on the order of a thousand fault episodes
  (UNVERIFIED). No robotics-specific study was found.

## Jetson Orin NX

- No verified Orin NX 16GB measurement for 0.5 to 3B models was found. Proxies:
  Qwen3-8B Q4 about 13 tok/s with llama.cpp (single forum post); Qwen2.5-1.5B
  15 to 17 tok/s and Llama-3.2-3B 10 to 12 tok/s on the 8GB part (vendor table,
  UNVERIFIED). A 3B Q4 model is therefore expected at roughly 10 to 40 tok/s, an
  estimate, not a measurement.

## What this implies for HELIX

1. Evidence collection stays deterministic (the RCACopilot and AIOpsLab lesson:
   raw-telemetry agents overflow, loop and hallucinate). The agent gets named,
   structured diagnostics, not a shell.
2. Actions stay a closed enum with a per-incident budget, transactional rollback
   and an independent verifier (INPROVF, Safe Remediation).
3. The open question is the *ranker* over feasible repairs: hand-written rules, a
   repair-history posterior (bandit-style, learns online in few interactions), a
   small MLP distilled from exploration data, or a small LLM. The literature does
   not settle it for ROS 2, so HELIX measures it (see
   `docs/preregistration/agent-withheld.md`). Deep RL is not tested: the
   evidence says it needs far more fault episodes than a robot can afford, and a
   bandit is the sample-efficient version of the same idea.
