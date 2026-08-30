const LOW_BATTERY_EVENT_TYPES = new Set([
  'low_battery_alert',
  'low_battery_return_charge',
])

const LOW_BATTERY_SOURCE_CODES = new Set([
  'LOW_BATTERY_ALERT',
  'LOW_BATTERY_RETURN_CHARGE',
])

export const LOW_BATTERY_STOP_PERCENT = 20

export function isLowBatteryStopAlert(event) {
  const eventType = String(event?.event_type || '').trim().toLowerCase()
  const sourceCode = String(event?.source_code || event?.source?.code || '').trim().toUpperCase()
  const objectClass = String(
    event?.object_class
      || event?.raw_detection?.class
      || event?.detection?.class
      || '',
  ).trim().toLowerCase()
  return LOW_BATTERY_EVENT_TYPES.has(eventType)
    || LOW_BATTERY_SOURCE_CODES.has(sourceCode)
    || objectClass === 'low_battery'
}

export function isLowBatteryTaskError(error) {
  const code = String(error?.payload?.code || error?.payload?.error_code || '').toUpperCase()
  const message = String(error?.message || error?.payload?.detail || '')
  return code.includes('LOW_BATTERY') || message.includes('电量不足') || message.includes('低电量')
}

export function isLowBatteryBlocked(batteryPercent) {
  const percent = Number(batteryPercent)
  return Number.isFinite(percent) && percent < LOW_BATTERY_STOP_PERCENT
}

export function lowBatteryGuardMessage(batteryPercent) {
  const percent = Number(batteryPercent)
  const batteryText = Number.isFinite(percent) ? `当前电量 ${Math.round(percent)}%，` : ''
  return `低电量停车告警：${batteryText}当前任务及循环巡检已停止，请及时人工处理或手动回充。`
}
