# RoamerX Dev Agent

Independent MQTT bridge between the cloud platform and the local Codex CLI.
It deliberately runs separately from `roamerx-edge-agent` so development jobs
cannot block navigation telemetry or command handling.

## Run locally

```bash
python3 run_dev_agent.py \
  --edge-config /home/dogrobot/runtime/nx-edge/conf/edge-agent.yaml \
  --config config.example.yaml
```

The runtime service uses the MQTT credentials already present in the deployed
edge configuration; no secret is copied into this repository.

## NX-local speech recognition

`roamerx-local-asr.service` runs FunASR SenseVoice with FSMN VAD on the Orin
GPU, bound only to `127.0.0.1:18080`. The dev agent sends each detected speech
segment there first and marks a successful result as `nx-sensevoice`. A
successful local wake command is routed directly to Codex and is not sent to
the cloud task broker again; all other segments remain available to the cloud
ASR fallback when needed.

## 小太阳语音唤醒

本地转写以“小太阳”开头且后面带操作命令时，Dev Agent 会去掉唤醒名，将其作为 `voice` 会话中的普通 Codex 任务自动提交到 `voice.wake_workspace`。因此共享 Codex Home 中所有匹配的已安装 Skill 都可以处理“小太阳，<操作命令>”。转写不以唤醒名开头、或只有唤醒名时不会执行任务。

唤醒名只负责路由，不是实体机器人操作的安全确认。基础遥控仍要求在同一条命令内明确确认现场通道和行进路线安全；停止、阻尼等已定义安全动作保持原有直达规则。每次接受的唤醒命令都会发布到 `robots/<robot_id>/dev/voice/wake`，供页面或监控系统显示。

## Conversation continuity

The first task creates a Codex thread. Its thread ID is persisted in
`/home/dogrobot/.local/state/roamerx-dev-agent/conversation.json`; every later
task resumes that same thread, including after an agent or machine restart.
Each request also names its target workspace and asks Codex to load the
workspace's applicable `AGENTS.md` before acting.
