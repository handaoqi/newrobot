const TERMINAL_AUDIO_STATES = new Set(['finished', 'failed', 'expired', 'superseded'])

export function audioPreviewResultLabel(command) {
  if (command?.status !== 'finished') {
    throw new Error(
      command?.error_message
      || (command?.status === 'superseded' ? '试播已被新的播报替换' : '试播失败'),
    )
  }
  const mode = command.response_payload?.playback_mode
  if (mode === 'single_nx') return 'NX 单音响试播完成（3588不可用）'
  if (mode === 'single_3588') return '3588 单音响试播完成（NX不可用）'
  return '双音响试播完成'
}

export async function waitForAudioPreview({
  initialCommand,
  robotId,
  fetchCommand,
  timeoutMs = 180_000,
  intervalMs = 1000,
  sleep = (milliseconds) => new Promise((resolve) => window.setTimeout(resolve, milliseconds)),
  now = () => Date.now(),
}) {
  let command = initialCommand
  const deadline = now() + timeoutMs
  while (command?.id && !TERMINAL_AUDIO_STATES.has(command.status) && now() < deadline) {
    await sleep(intervalMs)
    command = await fetchCommand(robotId, command.id)
  }
  if (!TERMINAL_AUDIO_STATES.has(command?.status)) {
    throw new Error(`试播已下发，但${Math.round(timeoutMs / 1000)}秒内未收到播放结果`)
  }
  return command
}
