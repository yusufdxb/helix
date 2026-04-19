# llama-server on Jetson Orin NX — HELIX Deployment Runbook

**Target hardware:** NVIDIA Jetson Orin NX 16 GB (GO2 onboard compute).
**JetPack:** 6.2 with Super Mode enabled (verify before running).
**Purpose:** Run the HELIX advisory LLM sidecar as a systemd-managed
`llama-server` daemon on 127.0.0.1:8080, serving
`Qwen2.5-1.5B-Instruct Q4_K_M` (primary) or `Gemma-3-1B-it Q4_K_M`
(backup) to the `helix_explanation` ROS 2 node.

This is a runbook, not a quickstart. The steps here produce a
repeatable deployment that auto-starts on boot and survives ROS 2
restarts. Source of record: `local_llm_survey_2026-04-18.md`.

> **Safety reminder:** the LLM is advisory only. The RecoveryNode
> allowlist (`STOP_AND_HOLD / RESUME / LOG_ONLY`) is the hard gate.
> Nothing in this runbook changes that contract.

---

## 1. Prerequisites

Verify each before you start.

### 1.1 JetPack 6.2 with Super Mode

```bash
# Jetson OS + L4T version
cat /etc/nv_tegra_release
# Expect something like: R36 (release), REVISION: 4.0, ...

# Current power model — 25W / MAXN_SUPER is what we want
sudo /usr/sbin/nvpmodel -q
sudo /usr/sbin/nvpmodel -m 0        # MAXN / MAXN_SUPER depending on device
sudo /usr/bin/jetson_clocks         # pin clocks high
```

Super Mode roughly doubles inference throughput on Orin NX. If you skip
it, expect the <500 ms first-token budget to fail.

### 1.2 CUDA toolkit (shipped with JetPack 6.2)

```bash
nvcc --version
# Expect CUDA 12.6 or 12.8 (JetPack 6.2 ships 12.6; Super Mode OTA may add 12.8)

ls /usr/local/cuda
```

### 1.3 Build dependencies

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git ccache \
                        libcurl4-openssl-dev pkg-config
```

### 1.4 Disk + RAM budget check

```bash
df -h /home                     # need ~1.5 GB free for the gguf + 150 MB for the binary
free -h                         # confirm we're not already over 14 GB RSS
```

HELIX's LLM budget is **< 2 GB RSS** for the whole sidecar.

---

## 2. Build llama.cpp with CUDA for Orin NX

We build from source rather than using a container so the binary lives
under `/opt/helix` and can be managed by a dedicated systemd unit
without Docker-in-the-loop.

```bash
# Pick a stable release tag; bump as needed
export LLAMA_CPP_TAG=b4260           # adjust to the current stable tag

sudo mkdir -p /opt/helix
sudo chown $USER:$USER /opt/helix
cd /opt/helix

git clone https://github.com/ggml-org/llama.cpp.git
cd llama.cpp
git fetch --tags
git checkout ${LLAMA_CPP_TAG}

cmake -B build \
  -DGGML_CUDA=on \
  -DCMAKE_CUDA_ARCHITECTURES=87 \
  -DLLAMA_CURL=on \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --config Release -j$(nproc)
```

Notes:

- **`CMAKE_CUDA_ARCHITECTURES=87`** is Ampere/Orin (SM 8.7). Do not
  leave this unset — the default `native` has been unreliable on
  Jetson.
- **`LLAMA_CURL=on`** lets `llama-server` pull HF URLs directly; we
  don't use that for HELIX (we pre-download the model), but it keeps
  the feature consistent with upstream docs.
- Build takes ~10–15 minutes on Orin NX. Use `ccache` (already
  installed) to speed up re-builds.
- If the build fails with a missing CUDA math header, re-run with
  `-DGGML_CUDA_FORCE_CUBLAS=on` and report upstream.

Smoke test the binary:

```bash
/opt/helix/llama.cpp/build/bin/llama-server --help | head -30
```

---

## 3. Download the primary model

Qwen2.5-1.5B-Instruct Q4_K_M from the official Qwen GGUF repo
(Apache-2.0).

```bash
mkdir -p /opt/helix/models
cd /opt/helix/models

# Primary — Qwen2.5-1.5B-Instruct Q4_K_M
curl -L -o qwen2.5-1.5b-instruct-q4_k_m.gguf \
  https://huggingface.co/Qwen/Qwen2.5-1.5B-Instruct-GGUF/resolve/main/qwen2.5-1.5b-instruct-q4_k_m.gguf

# Verify size (~1.12 GB)
ls -lh qwen2.5-1.5b-instruct-q4_k_m.gguf

# Record the hash so we can pin it later
sha256sum qwen2.5-1.5b-instruct-q4_k_m.gguf \
  | tee qwen2.5-1.5b-instruct-q4_k_m.gguf.sha256
```

**Hash pin:** on first download, commit the produced `.sha256` file to
the HELIX deployment repo (not this runbook) so subsequent installs
can verify integrity. Do **not** hardcode a hash here — the
HuggingFace revision may change under us.

> HuggingFace model card:
> <https://huggingface.co/Qwen/Qwen2.5-1.5B-Instruct-GGUF>

### 3.1 Backup model — Gemma-3-1B-it Q4_K_M

Only if Qwen2.5 fails the memory or latency budget on the target
Jetson. Gemma carries a custom Google license with prohibited-use
policy; fine for demo, flag for legal review before any commercial
ship.

```bash
curl -L -o gemma-3-1b-it-q4_k_m.gguf \
  https://huggingface.co/unsloth/gemma-3-1b-it-GGUF/resolve/main/gemma-3-1b-it-q4_k_m.gguf

sha256sum gemma-3-1b-it-q4_k_m.gguf \
  | tee gemma-3-1b-it-q4_k_m.gguf.sha256
```

> HuggingFace model card:
> <https://huggingface.co/unsloth/gemma-3-1b-it-GGUF>

---

## 4. Launch `llama-server`

### 4.1 Manual launch (first-time smoke test)

```bash
/opt/helix/llama.cpp/build/bin/llama-server \
  --model /opt/helix/models/qwen2.5-1.5b-instruct-q4_k_m.gguf \
  --alias qwen2.5-1.5b-instruct \
  --host 127.0.0.1 \
  --port 8080 \
  --ctx-size 1024 \
  --parallel 1 \
  --n-gpu-layers -1 \
  --threads 4 \
  --no-warmup \
  --metrics \
  --log-disable
```

Flag rationale:

| Flag | Why |
|---|---|
| `--ctx-size 1024` | Survey §6 cap: keeps KV cache under budget. |
| `--parallel 1` | Single request in flight (2 GB RAM cap). |
| `--n-gpu-layers -1` | Full GPU offload — Orin NX has enough VRAM for 1.5B Q4. |
| `--threads 4` | Prompt processing CPU threads. Orin NX has 8 A78AE cores; leave headroom for ROS. |
| `--host 127.0.0.1` | Loopback only. Do not expose 8080 outside the robot. |
| `--alias ...` | Model alias returned in OpenAI-compat responses; the ROS client's `llama_model` param must match. |

Expose `response_format: json_schema` is on by default in modern
builds — no extra flag needed.

### 4.2 Health check

From another terminal on the Jetson:

```bash
# Liveness
curl -s http://127.0.0.1:8080/health | jq .
# Expect: {"status":"ok"}  (or "loading model" during warm-up)

# End-to-end advisory call matching the HELIX Healer schema
curl -s http://127.0.0.1:8080/v1/chat/completions \
  -H 'Content-Type: application/json' \
  -d '{
    "model": "qwen2.5-1.5b-instruct",
    "messages": [
      {"role": "system", "content": "Answer with JSON matching the schema."},
      {"role": "user",   "content": "fault: lidar rate dropped to 3 Hz"}
    ],
    "response_format": {
      "type": "json_schema",
      "json_schema": {
        "name": "helix_healer",
        "strict": true,
        "schema": {
          "type": "object",
          "additionalProperties": false,
          "properties": {
            "action":     {"type":"string","enum":["STOP_AND_HOLD","RESUME","LOG_ONLY"]},
            "confidence": {"type":"number","minimum":0,"maximum":1},
            "reasoning":  {"type":"string","maxLength":240}
          },
          "required":["action","confidence","reasoning"]
        }
      }
    },
    "max_tokens": 192,
    "temperature": 0.2
  }' | jq -r .choices[0].message.content
```

Expect a single JSON object whose `action` is one of the three
allowlist values. Anything else is a server / model / flag problem.

---

## 5. systemd unit — auto-start on boot

Drop this file at `/etc/systemd/system/helix-llama-server.service`:

```ini
[Unit]
Description=HELIX llama-server sidecar (Qwen2.5-1.5B-Instruct Q4_K_M)
After=network-online.target nvpmodel.service
Wants=network-online.target

[Service]
Type=simple
User=helix
Group=helix
WorkingDirectory=/opt/helix
ExecStart=/opt/helix/llama.cpp/build/bin/llama-server \
  --model /opt/helix/models/qwen2.5-1.5b-instruct-q4_k_m.gguf \
  --alias qwen2.5-1.5b-instruct \
  --host 127.0.0.1 \
  --port 8080 \
  --ctx-size 1024 \
  --parallel 1 \
  --n-gpu-layers -1 \
  --threads 4 \
  --no-warmup \
  --metrics
Restart=on-failure
RestartSec=5
# Hard cap RSS a little above measured peak so we OOM before the system does.
MemoryMax=1800M
# Optional: keep llama-server out of the OOM killer's top N candidates.
OOMScoreAdjust=-100

[Install]
WantedBy=multi-user.target
```

Install and enable:

```bash
# Create the service user if it does not exist (sandbox only, no login)
sudo useradd --system --home /opt/helix --shell /usr/sbin/nologin helix || true
sudo chown -R helix:helix /opt/helix

sudo install -m 0644 helix-llama-server.service \
  /etc/systemd/system/helix-llama-server.service
sudo systemctl daemon-reload
sudo systemctl enable --now helix-llama-server.service

# Verify
systemctl status helix-llama-server.service --no-pager
journalctl -u helix-llama-server.service -n 50 --no-pager
```

On boot the model takes ~10–15 s to warm up. During that window the
`helix_explanation` node's LLM calls will time out and the
deterministic template path will still run — that's by design.

---

## 6. Integration with HELIX

### 6.1 ROS parameters

Set these on the `helix_llm_explainer` node (see `helix_explanation`
package):

```yaml
helix_llm_explainer:
  ros__parameters:
    llama_server_url: "http://localhost:8080"
    llama_model: "qwen2.5-1.5b-instruct"   # must match --alias
    llm_timeout_s: 6.0
    llm_enabled: true                       # opt-in once server is healthy
    llm_role: "healer"                      # healer | flagger | predictor
```

`llm_enabled` ships as `false`. Flip it to `true` only after:

1. `systemctl is-active helix-llama-server.service` returns `active`.
2. The curl smoke test in §4.2 returns a valid schema-conformant JSON.
3. `ps -o pid,rss,cmd -C llama-server` shows RSS < 1.8 GB.

### 6.2 Diagnostics topic

The node publishes per-request LLM diagnostics on
`/helix/llm_diagnostics` as `std_msgs/String` (JSON payload with
`status`, `latency_ms`, `data`). Use `ros2 topic echo
/helix/llm_diagnostics` during bring-up to watch latency and schema
validity in real time.

---

## 7. Swapping in Gemma-3-1B as backup

1. Stop the service:

    ```bash
    sudo systemctl stop helix-llama-server.service
    ```

2. Edit the unit file and change:
    - `--model .../gemma-3-1b-it-q4_k_m.gguf`
    - `--alias gemma-3-1b-it`

3. Reload and restart:

    ```bash
    sudo systemctl daemon-reload
    sudo systemctl start helix-llama-server.service
    ```

4. Update the `llama_model` ROS param to `"gemma-3-1b-it"` and
   redeploy the `helix_explanation` node.

The client code does not care which model is loaded as long as
`response_format: json_schema` is honored, which Gemma-3-1B handles
correctly in current llama.cpp builds.

---

## 8. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `curl /health` returns `{"status":"loading model"}` for >30 s | Model file corrupt or I/O slow | Re-download, re-verify sha256 |
| First-token >1 s | Super Mode not active | `sudo nvpmodel -m 0 && sudo jetson_clocks` |
| OOM kill on llama-server | ctx too big, or parallel > 1 | Drop `--ctx-size` to 768 |
| Schema-invalid JSON in /helix/llm_diagnostics | Model released before json_schema support hit llama.cpp | Upgrade llama.cpp tag; verify with the §4.2 curl |
| `address already in use` on port 8080 | Old instance still running | `sudo systemctl stop helix-llama-server.service && pkill -f llama-server` |
| CUDA build fails with SM mismatch | Wrong `CMAKE_CUDA_ARCHITECTURES` | Orin NX = 87. AGX Orin = 87. Xavier NX = 72. |
| High first-token variance | Thermal throttling | `tegrastats`; verify the Jetson fan profile |

Always start debugging with:

```bash
journalctl -u helix-llama-server.service -f
tegrastats
ros2 topic echo /helix/llm_diagnostics
```

---

## 9. References

- llama.cpp source: <https://github.com/ggml-org/llama.cpp>
- llama-server README: <https://github.com/ggml-org/llama.cpp/tree/master/tools/server>
- JetPack 6.2 Super Mode announcement:
  <https://www.edge-ai-vision.com/2025/01/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/>
- Jetson AI Lab llama.cpp tutorial: <https://www.jetson-ai-lab.com/tutorials/>
- Qwen2.5-1.5B-Instruct GGUF: <https://huggingface.co/Qwen/Qwen2.5-1.5B-Instruct-GGUF>
- Gemma-3-1B-it GGUF: <https://huggingface.co/unsloth/gemma-3-1b-it-GGUF>
- HELIX local-LLM survey: `notes/local_llm_survey_2026-04-18.md`
