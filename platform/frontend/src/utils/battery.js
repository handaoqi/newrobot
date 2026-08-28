function normalizeBatteryPercent(value) {
  if (value === null || value === undefined || value === '') return null
  const percent = Number(value)
  if (!Number.isFinite(percent)) return null
  return Math.round(Math.min(100, Math.max(0, percent)))
}

export function resolveBatteryPercent(status, robot) {
  const candidates = [
    status?.battery_percent,
    status?.power?.percent,
    robot?.battery_level,
  ]

  for (const candidate of candidates) {
    const percent = normalizeBatteryPercent(candidate)
    if (percent !== null) return percent
  }
  return null
}
