# RoamerX Dev Agent

Independent MQTT bridge between the cloud platform and the local Codex CLI.
It deliberately runs separately from `roamerx-edge-agent` so development jobs
cannot block navigation telemetry or command handling.

## Run locally

```bash
python3 run_dev_agent.py \
  --edge-config /home/robot/edge_agent/config.yaml \
  --config config.example.yaml
```

The runtime service uses the MQTT credentials already present in the deployed
edge configuration; no secret is copied into this repository.

## NX-local speech recognition

`roamerx-local-asr.service` runs FunASR SenseVoice with FSMN VAD on the Orin
GPU, bound only to `127.0.0.1:18080`. The dev agent sends each detected speech
segment there first and marks a successful result as `nx-sensevoice` before it
is forwarded to the cloud task broker. Cloud Whisper remains an automatic
availability fallback only when the local ASR endpoint is unavailable.

## Conversation continuity

The first task creates a Codex thread. Its thread ID is persisted in
`/home/robot/.local/state/roamerx-dev-agent/conversation.json`; every later
task resumes that same thread, including after an agent or machine restart.
Each request also names its target workspace and asks Codex to load the
workspace's applicable `AGENTS.md` before acting.
