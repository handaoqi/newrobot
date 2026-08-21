# Codex Main Session Backup

The active project conversation is stored as NX runtime data so that repository
instructions and the long-running engineering context can be recovered together.

## Current Main Session

- Session ID: `019f2bd3-33ba-7220-935f-1089bceece23`
- Runtime location: `data/codex/sessions/2026/07/04/`
- Codex compatibility path: `/home/robot/.codex/sessions/2026/07/04/`

The compatibility path is a symbolic link to the runtime file. Codex therefore
continues appending to the runtime copy while the session is active.

## Archive And Restore

Run these commands as the same Linux user that runs Codex:

```bash
runtime/nx-edge/scripts/archive-codex-session.sh "$CODEX_THREAD_ID"
runtime/nx-edge/scripts/restore-codex-session.sh \
  019f2bd3-33ba-7220-935f-1089bceece23
codex resume 019f2bd3-33ba-7220-935f-1089bceece23
```

Set `CODEX_HOME=/home/robot/.codex` when restoring from another Linux user.
Raw session JSONL can contain prompts, command output, device addresses and
credentials. It is ignored by Git and must only be copied through the encrypted
runtime-data backup process.
