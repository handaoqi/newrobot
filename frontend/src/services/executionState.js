export const TERMINAL_STATES = new Set(['completed', 'failed', 'cancelled', 'timed_out', 'rejected'])

export function executionActions(state) {
  return {
    pause: state === 'running',
    resume: state === 'paused',
    cancel: ['running', 'paused', 'pausing', 'resuming', 'interrupted'].includes(state),
  }
}

export function powerLabel(status) {
  if (!status || !status.power_available) return '未知'
  return `${status.battery_percent ?? '--'}%`
}
