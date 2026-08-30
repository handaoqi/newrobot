const TERMINAL_TASK_STATES = new Set([
  'completed',
  'failed',
  'cancelled',
  'timed_out',
  'rejected',
])

function timestamp(value) {
  const parsed = new Date(value || '').getTime()
  return Number.isFinite(parsed) ? parsed : 0
}

function rosbagCandidate(status, occurredAt, order) {
  if (
    !status
    || typeof status !== 'object'
    || Array.isArray(status)
    || !Object.keys(status).length
  ) return null
  return { status, occurredAt: timestamp(occurredAt), order }
}

export function resolveTaskRosbagStatus(execution, nowMs = Date.now()) {
  if (!execution) return null
  const candidates = []
  let order = 0
  for (const event of execution.events || []) {
    const candidate = rosbagCandidate(
      event?.payload?.rosbag,
      event.received_at || event.occurred_at,
      order++,
    )
    if (candidate) candidates.push(candidate)
  }
  for (const command of [...(execution.commands || [])].reverse()) {
    const candidate = rosbagCandidate(
      command?.result_payload?.rosbag,
      command.finished_at || command.started_at || command.issued_at,
      order++,
    )
    if (candidate) candidates.push(candidate)
  }
  if (!candidates.length) return null
  candidates.sort((left, right) => left.occurredAt - right.occurredAt || left.order - right.order)
  const status = { ...candidates[candidates.length - 1].status }
  if (!TERMINAL_TASK_STATES.has(execution.state) || !status.running) return status

  const stoppedAtMs = timestamp(execution.finished_at) || nowMs
  const startedAtMs = Number(status.started_at_unix || 0) * 1000
  return {
    ...status,
    running: false,
    duration_seconds: Number(status.duration_seconds || (
      startedAtMs > 0 ? Math.max(0, Math.floor((stoppedAtMs - startedAtMs) / 1000)) : 0
    )),
  }
}
