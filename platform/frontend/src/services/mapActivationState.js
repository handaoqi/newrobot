export function expectedLegacyMapVersion(mapId) {
  return mapId === null || mapId === undefined || mapId === ''
    ? ''
    : `legacy-mapdata-${mapId}`
}

export function navigationMapIdentity(payload = {}) {
  const status = payload.status || {}
  const currentMap = status.current_map || {}
  return {
    mapId: String(currentMap.map_id || status.map_id || payload.current_map_id || ''),
    mapVersion: String(currentMap.map_version || status.map_version || payload.current_map_version || ''),
  }
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
}
