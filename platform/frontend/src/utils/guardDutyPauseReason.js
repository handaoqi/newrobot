const REASON_LABELS = {
  ROBOT_BUSY: '机器狗仍有未结束的运动任务，请先恢复、结束或强制退出原任务',
  COMMAND_TIMED_OUT: '中心等待设备确认超时，任务状态需要与 Edge 重新对账',
  MQTT_PUBLISH_FAILED: '任务指令发送失败，请检查消息服务和设备连接',
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

const PAUSE_STATES = ['pausing', 'paused', 'interrupted']
const TERMINAL_REASON_STATES = ['rejected', 'failed', 'timed_out']

function reasonLabel(state) {
  if (state === 'rejected') return '拒绝原因'
  if (state === 'failed') return '失败原因'
  if (state === 'timed_out') return '超时原因'
  if (state === 'interrupted') return '中断原因'
  return '暂停原因'
}

function newestReasonEvent(execution, state) {
  const events = Array.isArray(execution?.events) ? execution.events : []
  return events
    .filter((item) => {
      if (PAUSE_STATES.includes(state)) {
        return PAUSE_STATES.includes(String(item?.state || ''))
          || ['task.paused', 'task.pausing', 'task.safe_hold', 'task.sync_state_reconciled'].includes(item?.event_type)
      }
      return String(item?.state || '') === state
        || ['command.ack', 'command.result'].includes(item?.event_type)
    })
    .sort((left, right) => eventVersion(right) - eventVersion(left))[0]
}

function commandReason(execution, state) {
  const commands = Array.isArray(execution?.commands) ? execution.commands : []
  const command = commands.find((item) => (
    (state === 'rejected' && item?.status === 'rejected')
    || (state === 'timed_out' && ['timed_out', 'expired'].includes(item?.status))
    || (state === 'failed' && item?.status === 'failed')
  ))
  return {
    code: String(command?.ack_reason_code || command?.error_code || ''),
    message: String(command?.ack_reason_message || command?.error_message || '').trim(),
  }
}

export function guardDutyExecutionReason(execution) {
  const state = String(execution?.state || '')
  if (![...PAUSE_STATES, ...TERMINAL_REASON_STATES].includes(state)) return null
  const event = newestReasonEvent(execution, state)
  const command = commandReason(execution, state)
  const code = String(event?.reason_code || execution?.failure_code || command.code || '')
  const message = String(
    event?.reason_message
      || event?.payload?.reason_message
      || execution?.failure_message
      || command.message
      || '',
  ).trim()
  const mapped = REASON_LABELS[code]
  let text = message || mapped
  if (TERMINAL_REASON_STATES.includes(state) && mapped) text = mapped
  if (!text) {
    if (state === 'rejected') text = '设备拒绝了任务，请检查是否存在未结束的运动任务'
    else if (state === 'failed') text = '任务执行失败，请查看执行记录和设备日志'
    else if (state === 'timed_out') text = '任务等待超时，请检查设备连接和云边任务状态'
    else if (state === 'interrupted') text = '设备重启或连接中断，等待状态对账'
    else text = '设备已安全停车，等待继续或恢复条件满足'
  }
  return { label: reasonLabel(state), text, code }
}

export function guardDutyPauseReason(execution) {
  if (!PAUSE_STATES.includes(String(execution?.state || ''))) return ''
  return guardDutyExecutionReason(execution)?.text || ''
}
