# Agent freeze

Frozen inputs for the one-time withheld run registered in
`docs/preregistration/agent-withheld.md`. Nothing here may change after this commit
until that run is scored.

| File | sha256 | Source |
|---|---|---|
| `history_warm.log` | `18cc7947fd66f49ab35bf8c76b2f795b2f9757a459b38dbfa29b2e44da8ed93f` | 110 verified repair outcomes from the dev pool (`humanoid` set only). All are `restart`: 60 crash, 17 degraded, 16 endpoint_loss, 17 hang. |
| `mlp_weights.txt` | `888b74ac786cc85ad3225189889a82ea64f435823aa2118b0b86566ae5821b1a` | `train_ranker.py` on the same dev episodes, `--hidden 16 --epochs 400 --seed 0`. |

## LLM ranker

Chosen model: **`qwen2.5:1.5b`**, served by a private single-slot Ollama server on the
evaluation workstation GPU (`--agent-llm qwen2.5:1.5b@127.0.0.1:11435`).

Selection followed the amendment rule: a 2-rep `--set humanoid` screen per model on the
same host, keep models with zero LLM errors or fallbacks, then pick the lowest median
ranker wall time.

| Model | `ranker=llm` decisions | errors / fallbacks | ms p50 | ms p90 | ms max | resident on GPU |
|---|---|---|---|---|---|---|
| qwen2.5:1.5b | 27 | 0 | 211 | 225 | 263 | 1.35 GB, fully |
| llama3.2:3b | 27 | 0 | 230 | 251 | 255 | 2.52 GB, fully |
| qwen2.5:3b | 27 | 0 | 264 | 285 | 300 | 2.31 GB, fully |
| qwen3:4b-instruct | 27 | 0 | 292 | 315 | 333 | 3.26 GB, fully |

All four models were valid on every decision, so latency decided. Every screen decision
had a single candidate (`restart`), as the amendment predicted, so the screen says nothing
about the ranking quality of any model. Latency was measured on a desktop GPU. Jetson
Orin latency was not measured.
