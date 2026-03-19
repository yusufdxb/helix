# HELIX — Hierarchical Execution and Lifecycle Intelligence eXecutor

> Self-healing middleware for ROS 2 robotic systems. Detects failures, recovers
> autonomously, and explains root causes using an on-device LLM — all without
> human intervention.

**Built for:** Unitree GO2 quadruped · ROS 2 Humble · Jetson Orin
**Status:** Active development — Phase 4 complete, paper in preparation

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10](https://img.shields.io/badge/Python-3.10-blue)](https://python.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-green)](LICENSE)

---

## What HELIX Does

Robots fail in the field. Nodes crash, SLAM diverges, motors overheat, networks
drop. Most systems let these failures propagate until a human intervenes.

HELIX catches failures in under 300ms and works through a tiered recovery ladder
automatically:
```
Fault detected → Tier 1: rule-based fix (restart, relocalize, reduce speed)
                       ↓ if failed
              → Tier 2: adaptive fallback (switch sensors, standalone mode)
                       ↓ if failed
              → Tier 3: on-device LLM diagnosis (Ollama / Phi-3 Mini)
                       ↓
              → Recovery action executed + verified + logged
```

All decisions are logged to SQLite and streamed to a live web dashboard.

---

## Architecture
```
┌─────────────────────────────────────────────────────┐
│                  ROS 2 Node Graph                   │
│   Navigation │ Perception │ Control │ Comms/DDS     │
└──────────────────────┬──────────────────────────────┘
                       │ heartbeats + metrics + /rosout
┌──────────────────────▼──────────────────────────────┐
│              Phase 1 — Fault Sensing                │
│  HeartbeatMonitor │ AnomalyDetector │ LogParser     │
│                   → /helix/faults                   │
└──────────────────────┬──────────────────────────────┘
                       │ FaultEvent
┌──────────────────────▼──────────────────────────────┐
│             Phase 2 — Recovery Engine               │
│  RecoveryPlanner → ActionExecutor → VerificationLoop│
│  8 recovery actions │ SQLite state │ 3-tier policy  │
│                   → /helix/recovery_actions         │
└──────────┬───────────────────────────┬──────────────┘
           │ tier 3 escalation         │ metrics
┌──────────▼──────────┐   ┌───────────▼──────────────┐
│  Phase 3 — LLM      │   │  Phase 4 — Dashboard     │
│  Ollama / Phi-3 Mini│   │  FastAPI + React          │
│  ContextBuilder     │   │  WebSocket broadcast      │
│  Confidence-gated   │   │  http://localhost:8080    │
│  auto-execution     │   │                          │
└─────────────────────┘   └──────────────────────────┘
```

---

## Fault Coverage

| Fault Class | Detection Method | Tier 1 | Tier 2 | Tier 3 |
|---|---|---|---|---|
| Node crash | Heartbeat timeout | Lifecycle restart | Fallback topic | LLM diagnosis |
| SLAM drift | Covariance Z-score | Global relocalize | Reinit from waypoint | LLM diagnosis |
| Motor anomaly | Torque/temp Z-score | Reduce velocity | Safe mode + E-stop | LLM diagnosis |
| Network drop | DDS discovery loss | DDS reconfigure | Standalone mode | LLM diagnosis |
| Log pattern | Regex rule match | Restart node | Safe mode | LLM diagnosis |

Detection latency: **< 300ms**
Tier 1 recovery latency: **< 2s**
LLM inference (Phi-3 Mini, CPU): **15–45s**

---

## Packages

| Package | Description |
|---|---|
| `helix_msgs` | Custom ROS 2 messages: FaultEvent, RecoveryAction, LLMDiagnosis, RecoveryHint |
| `helix_core` | Fault sensing: HeartbeatMonitor, AnomalyDetector, LogParser |
| `helix_recovery` | Recovery engine: RecoveryPlanner, ActionExecutor, VerificationLoop, StateDB |
| `helix_llm` | LLM advisor: OllamaClient, ContextBuilder, LLMAdvisor node |
| `helix_dashboard` | Web dashboard: FastAPI backend + React frontend |
| `helix_bringup` | Launch files, configs, fault injector demo node |

---

## Requirements

- ROS 2 Humble on Ubuntu 22.04
- Python 3.10+
- Ollama (for Phase 3 LLM features): https://ollama.com
```bash
# Install Ollama and pull the model
curl -fsSL https://ollama.com/install.sh | sh
ollama pull phi3:mini
```

---

## Build & Run
```bash
# Clone and build
git clone https://github.com/yusufdxb/helix.git
cd helix
mkdir -p ../helix_ws/src && cp -r . ../helix_ws/src/helix
cd ../helix_ws
source /opt/ros/humble/setup.bash
pip install fastapi uvicorn websockets --break-system-packages
colcon build --symlink-install
source install/setup.bash

# Start Ollama (separate terminal)
ollama serve

# Launch full system
ros2 launch helix_bringup helix_complete.launch.py
# Dashboard: http://localhost:8080

# Run fault injection demo (separate terminal)
ros2 run helix_bringup helix_fault_injector
```

---

## Demo

The fault injector simulates 5 scenarios:

| Scenario | What it injects | What HELIX does |
|---|---|---|
| 1 | Node stops heartbeating | Detects crash in <300ms, restarts via lifecycle |
| 2 | Metric spike (Z-score > 3) | Detects anomaly, reduces velocity limits |
| 3 | SLAM log error pattern | Detects pattern, triggers relocalization |
| 4 | Full recovery chain | Crash → detect → restart → verify → log |
| 5 | LLM tier escalation | All tiers fail → Ollama diagnoses → suggests action |

Dashboard updates in real time across all scenarios.

---

## Tests
```bash
colcon test --packages-select helix_core helix_recovery helix_llm helix_dashboard
colcon test-result --verbose
```

68 tests across 4 packages. All offline — no robot or Ollama instance required.

---

## Roadmap

- [ ] Jetson Orin deployment and hardware validation on GO2
- [ ] Prometheus metrics export + Grafana dashboard
- [ ] IROS 2026 paper submission
- [ ] Chaos engineering test suite (automated fault injection battery)
- [ ] Multi-robot HELIX federation (Phase 5 concept)

---

## Author

**Yusuf Guenena**
M.S. Robotics Engineering — Wayne State University
[linkedin.com/in/yusuf-guenena](https://linkedin.com/in/yusuf-guenena) · [github.com/yusufdxb](https://github.com/yusufdxb)

---

## License

MIT License — see [LICENSE](LICENSE) for details.
