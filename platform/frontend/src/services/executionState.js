export const TERMINAL_STATES = new Set(['completed', 'failed', 'cancelled', 'timed_out', 'rejected'])
export const ACTIVE_STATES = new Set([
  'created',
  'dispatching',
  'accepted',
  'running',
  'pausing',
  'paused',
  'resuming',
  'cancelling',
  'interrupted',
])

export function isExecutionActive(state) {
  return ACTIVE_STATES.has(state)
}

export const FORCE_EXIT_CONFIRM_TEXT = [
  '强制退出会停止当前导航，并清理该机器人的全部未结束任务。',
  '退出后请等待约 40 秒：任务应变为已取消，定位丢失点不再显示“正在停止并重定位”。',
  '确认自动恢复已停后，再到路径规划设初始定位。不要立即点继续或下发初始位。',
].join('')

export const FORCE_EXIT_WAIT_HINT = [
  '任务已强制退出。请等待约 40 秒，确认任务已取消、定位丢失点不再显示“正在停止并重定位”，再去路径规划设初始定位。',
  '不要点继续，也不要在自动恢复还在跑时下发初始位。',
].join('')

export function executionActions(state, options = {}) {
  const executing = state === 'running' || state === 'resuming'
  const pauseable = ['created', 'dispatching', 'accepted', 'running', 'resuming'].includes(state)
  const paused = state === 'pausing' || state === 'paused'
  const interrupted = state === 'interrupted'
  const exiting = state === 'cancelling'
  const ended = TERMINAL_STATES.has(state)
  const localizationPaused = Boolean(options.localizationPaused) && (paused || interrupted)
  const control = pauseable
    ? { enabled: true, action: 'pause', label: '暂停' }
    : localizationPaused
      ? { enabled: false, action: null, label: '请先强制退出' }
    : paused || interrupted
      ? { enabled: true, action: 'resume', label: '继续' }
      : ended
        ? { enabled: false, action: null, label: '任务结束' }
        : exiting
          ? { enabled: false, action: null, label: '退出中' }
          : { enabled: false, action: null, label: state ? '等待执行' : '加载中' }
  return {
    control,
    statusLabel: executing
      ? '任务执行中'
      : paused
        ? '任务暂停中'
        : interrupted
          ? '任务中断'
          : exiting
            ? '任务退出中'
        : ended
          ? '任务结束'
          : '任务准备中',
    // 强制退出是恢复入口：即使页面已显示终态，也允许重复下发，
    // 以清理机器人端可能残留的任务上下文。
    forceExit: true,
  }
}

export function powerLabel(status) {
  if (!status || !status.power_available) return '未知'
  return `${status.battery_percent ?? '--'}%`
}
