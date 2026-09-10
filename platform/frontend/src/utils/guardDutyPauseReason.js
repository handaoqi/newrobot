const REASON_LABELS = {
  LOCALIZATION_LOST: '定位丢失，导航已停车，正在自动重定位',
  ABSOLUTE_LOCALIZATION_REQUIRED: '已到达点位，正在等待绝对定位校正',
  ARRIVAL_CONFIRMATION_UNSTABLE: '到点位姿未稳定，等待重新确认',
  RECOVERY_BUDGET_EXHAUSTED: '自动恢复次数已耗尽，设备进入安全保持',
  EDGE_SYNC_PAUSED: '云边状态已对账，设备端任务处于暂停状态',
}

function eventVersion(event) {
  const value = Number(event?.state_version)
  return Number.isFinite(value) ? value : -1
}

export function guardDutyPauseReason(execution) {
  if (!['pausing', 'paused', 'interrupted'].includes(String(execution?.state || ''))) return ''
  const events = Array.isArray(execution?.events) ? execution.events : []
  const event = events
    .filter((item) => (
      ['paused', 'pausing', 'interrupted'].includes(String(item?.state || ''))
      || ['task.paused', 'task.pausing', 'task.safe_hold', 'task.sync_state_reconciled'].includes(item?.event_type)
    ))
    .sort((left, right) => eventVersion(right) - eventVersion(left))[0]
  const code = String(event?.reason_code || execution?.failure_code || '')
  const message = String(
    event?.reason_message
      || event?.payload?.reason_message
      || execution?.failure_message
      || '',
  ).trim()
  return message || REASON_LABELS[code] || (execution?.state === 'interrupted'
    ? '设备重启或连接中断，等待状态对账'
    : '设备已安全停车，等待继续或恢复条件满足')
}
