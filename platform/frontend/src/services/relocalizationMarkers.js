function finitePose(value) {
  if (!value || typeof value !== 'object') return null
  const x = Number(value.x)
  const y = Number(value.y)
  const yaw = Number(value.yaw || 0)
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null
  return { x, y, yaw: Number.isFinite(yaw) ? yaw : 0 }
}

export function relocalizationMarkerStorageKey(robotId) {
  return `roamerx.relocalizationMarkers.v2.${robotId || 'unknown'}`
}

export function createRelocalizationMarker(pose, {
  source = '',
  commandType = '',
  commandId = '',
  occurredAt = new Date().toISOString(),
  verification = '',
  candidateNumber = null,
  candidateLabel = '',
} = {}) {
  const normalized = finitePose(pose)
  if (!normalized) return null
  const normalizedCandidateNumber = Number(candidateNumber)
  return {
    id: `${Date.parse(occurredAt) || Date.now()}-${Math.round(normalized.x * 1000)}-${Math.round(normalized.y * 1000)}`,
    ...normalized,
    source: String(source || ''),
    commandType: String(commandType || ''),
    commandId: String(commandId || ''),
    occurredAt,
    verification: String(verification || ''),
    candidateNumber: Number.isInteger(normalizedCandidateNumber) && normalizedCandidateNumber > 0
      ? normalizedCandidateNumber
      : null,
    candidateLabel: String(candidateLabel || ''),
  }
}

export function appendRelocalizationMarker(markers, pose, meta = {}) {
  const marker = createRelocalizationMarker(pose, meta)
  if (!marker) return Array.isArray(markers) ? markers : []
  const current = Array.isArray(markers) ? markers : []
  if (marker.commandId && current.some(item => item.commandId === marker.commandId)) {
    return current
  }
  const markerTime = Date.parse(marker.occurredAt)
  const duplicate = current.some((item) => {
    const itemTime = Date.parse(item.occurredAt || '')
    const closeInSpace = Math.hypot(item.x - marker.x, item.y - marker.y) < 0.5
    const closeInTime = Number.isFinite(markerTime)
      && Number.isFinite(itemTime)
      && Math.abs(markerTime - itemTime) < 15_000
    return closeInSpace && closeInTime
  })
  if (duplicate) return current
  return [...current, {
    ...marker,
    sequence: current.length + 1,
  }].slice(-30)
}

export function readStoredRelocalizationMarkers(robotId) {
  if (typeof sessionStorage === 'undefined' || !robotId) return []
  try {
    const raw = sessionStorage.getItem(relocalizationMarkerStorageKey(robotId))
    const parsed = raw ? JSON.parse(raw) : []
    return Array.isArray(parsed) ? parsed : []
  } catch {
    return []
  }
}

export function writeStoredRelocalizationMarkers(robotId, markers) {
  if (typeof sessionStorage === 'undefined' || !robotId) return
  sessionStorage.setItem(relocalizationMarkerStorageKey(robotId), JSON.stringify(markers || []))
}

export function relocalizationMarkerTitle(marker) {
  const source = marker?.source || marker?.commandType || '重定位'
  const time = marker?.occurredAt ? new Date(marker.occurredAt).toLocaleString('zh-CN', { hour12: false }) : '—'
  const candidate = Number.isInteger(Number(marker?.candidateNumber)) && Number(marker.candidateNumber) > 0
    ? ` · 最优候选 #${Number(marker.candidateNumber)}${marker?.candidateLabel ? `（${marker.candidateLabel}）` : ''}`
    : ''
  const verification = marker?.verification ? ` · ${marker.verification}` : ''
  return `${source}${candidate} · x ${Number(marker.x).toFixed(3)} / y ${Number(marker.y).toFixed(3)}${verification} · ${time}`
}
