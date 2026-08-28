function finitePose(pose) {
  return Boolean(pose)
    && Number.isFinite(Number(pose.x))
    && Number.isFinite(Number(pose.y))
}

function sameMap(left, right) {
  return !left || !right || String(left) === String(right)
}

function fallbackTrustedPose(event, trajectory) {
  const reported = event?.payload?.last_trusted_pose
  if (finitePose(reported)) return reported
  const lossTime = new Date(event?.occurred_at || '').getTime()
  if (!Number.isFinite(lossTime)) return null
  return [...(trajectory || [])].reverse().find((point) => {
    const sampleTime = new Date(point.sampled_at || point.received_at || '').getTime()
    return point.localization_status === 'normal'
      && Number.isFinite(sampleTime)
      && sampleTime <= lossTime
      && finitePose(point)
  }) || null
}

function recoveryState(events, lossEvent) {
  const later = (events || []).filter((event) => (
    Number(event.state_version || 0) > Number(lossEvent.state_version || 0)
  ))
  if (later.some((event) => event.event_type === 'task.resuming' && event.reason_code === 'LOCALIZATION_RECOVERED')) {
    return 'recovered'
  }
  if (later.some((event) => ['task.failed', 'task.cancelled'].includes(event.event_type))) return 'ended'
  return 'recovering'
}

export function buildLocalizationLossMarkers(execution, trajectory = [], displayedMapId = null) {
  const events = execution?.events || []
  return events
    .filter((event) => event.reason_code === 'LOCALIZATION_LOST' && event.event_type === 'task.pausing')
    .filter((event) => sameMap(event.payload?.map_id || execution?.map_data, displayedMapId))
    .map((event, index) => {
      const trusted = fallbackTrustedPose(event, trajectory)
      if (!finitePose(trusted)) return null
      return {
        ...trusted,
        eventId: event.id,
        sequence: index + 1,
        occurredAt: event.occurred_at,
        mapId: event.payload?.map_id || execution?.map_data || null,
        mapVersion: event.payload?.map_version || null,
        rawPose: finitePose(event.payload?.raw_pose) ? event.payload.raw_pose : null,
        quality: event.payload?.localization_quality || null,
        decision: event.payload?.localization_decision || null,
        waypoint: event.payload?.current_waypoint || null,
        waypointIndex: event.payload?.current_waypoint_index,
        recoveryState: recoveryState(events, event),
      }
    })
    .filter(Boolean)
}

export function currentRobotMapPose(statusEnvelope, displayedMapId, lossMarkers = []) {
  const status = statusEnvelope?.status || statusEnvelope
  const statusMapId = status?.map_id || statusEnvelope?.current_map_id
  if (finitePose(status) && sameMap(statusMapId, displayedMapId)) {
    return {
      x: Number(status.x),
      y: Number(status.y),
      yaw: Number(status.yaw || 0),
      sampledAt: status.sampled_at || status.received_at || null,
      localizationStatus: status.localization_status || 'unknown',
      trusted: status.localization_status === 'normal',
      source: 'telemetry',
    }
  }
  const fallback = lossMarkers[lossMarkers.length - 1]
  if (!fallback) return null
  return {
    x: Number(fallback.x),
    y: Number(fallback.y),
    yaw: Number(fallback.yaw || 0),
    sampledAt: fallback.occurredAt,
    localizationStatus: 'lost',
    trusted: false,
    source: 'last_trusted',
  }
}

export function localizationRecoveryLabel(state) {
  return {
    recovered: '已恢复并续航',
    recovering: '正在停止并重定位',
    ended: '任务已结束',
  }[state] || '状态未知'
}

export function isLocalizationLossPause(execution) {
  if (!['pausing', 'paused'].includes(execution?.state)) return false
  const events = [...(execution?.events || [])].reverse()
  const latestPause = events.find((event) => ['task.pausing', 'task.paused'].includes(event.event_type))
  return ['LOCALIZATION_LOST', 'ABSOLUTE_LOCALIZATION_REQUIRED'].includes(latestPause?.reason_code)
}

export function localizationRecoveryStillRunning(execution) {
  const markers = buildLocalizationLossMarkers(execution)
  return markers.some((point) => point.recoveryState === 'recovering')
}
