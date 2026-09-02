export function expectedLegacyMapVersion(mapId) {
  return mapId === null || mapId === undefined || mapId === ''
    ? ''
    : `legacy-mapdata-${mapId}`
}

export function navigationMapIdentity(payload = {}) {
  const safe = payload && typeof payload === 'object' ? payload : {}
  const status = safe.status || {}
  const currentMap = status.current_map || {}
  return {
    mapId: String(currentMap.map_id || status.map_id || safe.current_map_id || ''),
    mapVersion: String(currentMap.map_version || status.map_version || safe.current_map_version || ''),
  }
}

export function navigationStatusFresh(payload = {}, {
  nowMs = Date.now(),
  maxAgeMs = 10_000,
} = {}) {
  const status = payload?.status || {}
  const quality = status.localization_quality || {}
  if (quality.localization_fresh === false) return false
  const sampledAt = quality.localization_sampled_at
    || quality.sampled_at
    || status.sampled_at
    || status.received_at
  if (!sampledAt) return false
  const sampledAtMs = new Date(sampledAt).getTime()
  const ageMs = nowMs - sampledAtMs
  return Number.isFinite(sampledAtMs)
    && ageMs >= -maxAgeMs
    && ageMs <= maxAgeMs
}

export function shouldFallbackToGlobalRelocalization(errorCode) {
  return new Set([
    'RELOCALIZATION_SEED_UNAVAILABLE',
    'ACTIVE_RELOCALIZATION_FAILED',
  ]).has(String(errorCode || ''))
}

export function navigationReadyForMap(payload, mapId, mapVersion = expectedLegacyMapVersion(mapId)) {
  const status = payload?.status || {}
  const identity = navigationMapIdentity(payload)
  const localizationStatus = status.localization_status || payload?.localization_status
  const navReady = status.nav_ready ?? payload?.nav_ready
  return payload?.connection_status === 'online'
    && identity.mapId === String(mapId || '')
    && identity.mapVersion === String(mapVersion || '')
    && localizationStatus === 'normal'
    && Boolean(navReady)
    && navigationStatusFresh(payload)
}
