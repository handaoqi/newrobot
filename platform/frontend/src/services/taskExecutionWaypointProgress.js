import {
  GUARD_WAYPOINT_STATE,
  activeGuardDutyTarget,
  guardDutyWaypointStates,
} from '../utils/guardDutyWaypointState.js'

function milestoneEvents(execution) {
  return (execution?.events || [])
    .filter((event) => ['task.target_dispatched', 'task.waypoint_reached'].includes(event.event_type))
    .sort((left, right) => Number(left.state_version || 0) - Number(right.state_version || 0))
}

const ACTIVE_EXECUTION_STATES = new Set([
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

const TERMINAL_EVENT_PRESENTATION = {
  'task.completed': { type: 'complete', title: '预演完成' },
  'task.cancelled': { type: 'stop', title: '预演已取消' },
  'task.failed': { type: 'error', title: '预演失败' },
  'task.timed_out': { type: 'error', title: '预演超时' },
  'task.rejected': { type: 'error', title: '预演被拒绝' },
}

function eventTimestamp(event) {
  const value = Date.parse(event?.occurred_at || event?.received_at || '')
  return Number.isFinite(value) ? value : 0
}

function eventOrder(event) {
  const value = Number(event?.state_version)
  return Number.isFinite(value) ? value : Number.MAX_SAFE_INTEGER
}

function eventKey(event) {
  return String(
    event?.id
    || event?.message_id
    || `${event?.event_type || 'task.event'}:${event?.state_version ?? ''}:${event?.occurred_at || event?.received_at || ''}`,
  )
}

function eventArrivalKey(event) {
  const payload = event?.payload || {}
  const waypoint = payload.waypoint || {}
  const identity = payload.execution_waypoint_index ?? waypoint.waypoint_id ?? waypoint.map_point_number
  return `${payload.round_number ?? ''}:${identity ?? ''}`
}

function coordinateText(point) {
  const x = Number(point?.x)
  const y = Number(point?.y)
  if (!Number.isFinite(x) || !Number.isFinite(y)) return ''
  return `x ${x.toFixed(2)} / y ${y.toFixed(2)}`
}

function eventWaypointLabel(event, execution) {
  const payload = event?.payload || {}
  const point = payload.waypoint || {}
  const mapPointNumber = point.map_point_number
  if (mapPointNumber !== null && mapPointNumber !== undefined && mapPointNumber !== '') {
    return `${mapPointNumber}号点`
  }
  const executionIndex = Number(payload.execution_waypoint_index)
  if (Number.isInteger(executionIndex) && executionIndex >= 0) {
    return formatWaypointLabel(execution?.route_snapshot?.waypoints || [], executionIndex)
  }
  return '未知航点'
}

function eventDetail(event) {
  const payload = event?.payload || {}
  const target = coordinateText(payload.waypoint)
  const robot = coordinateText(payload.robot_pose)
  const reason = event?.reason_message || payload.reason_message || event?.reason_code || payload.reason_code || ''
  const arrival = []
  if (payload.arrival_mode === 'lightweight') arrival.push('轻量到达（跳过定位校正）')
  if (payload.arrival_mode === 'full_correction') arrival.push('完整定位校正')
  if (Number.isFinite(Number(payload.distance_m))) arrival.push(`偏差 ${Number(payload.distance_m).toFixed(2)}m`)
  if (Number.isFinite(Number(payload.acceptance_tolerance_m))) {
    arrival.push(`验收半径 ${Number(payload.acceptance_tolerance_m).toFixed(2)}m`)
  }
  if (Number.isFinite(Number(payload.reapproach_attempts))) {
    arrival.push(`追加靠近 ${Number(payload.reapproach_attempts)} 次`)
  }
  if (payload.coarse_completed === true) arrival.push('两次靠近后按 0.50m 粗范围完成')
  return [
    target ? `目标 ${target}` : '',
    robot ? `机器狗 ${robot}` : '',
    ...arrival,
    reason,
  ].filter(Boolean).join(' · ')
}

function timelinePresentation(event, execution) {
  const eventType = String(event?.event_type || '')
  const waypointLabel = eventWaypointLabel(event, execution)
  if (eventType === 'task.created') return { type: 'created', title: '预演任务已创建' }
  if (eventType === 'task.accepted') return { type: 'accepted', title: '机器狗已接受任务' }
  if (eventType === 'task.started') return { type: 'start', title: '预演开始' }
  if (eventType === 'task.target_dispatched') {
    return { type: 'target', title: `${waypointLabel}目标已下发`, pointName: waypointLabel }
  }
  if (eventType === 'task.waypoint_reached') {
    return { type: 'arrival', title: `${waypointLabel}已到达`, pointName: waypointLabel }
  }
  if (eventType === 'task.arrival_confirmed') {
    const suffix = event?.payload?.coarse_completed === true ? '（粗范围完成）' : ''
    return { type: 'arrival', title: `${waypointLabel}验收完成${suffix}`, pointName: waypointLabel }
  }
  if (eventType === 'task.pausing') return { type: 'pause', title: '正在暂停预演' }
  if (eventType === 'task.paused') return { type: 'pause', title: '预演已暂停' }
  if (eventType === 'task.resuming') return { type: 'resume', title: '正在恢复预演' }
  if (eventType === 'task.resumed') return { type: 'resume', title: '预演已恢复' }
  if (eventType === 'task.cancelling') return { type: 'stop', title: '正在取消预演' }
  if (eventType === 'task.interrupted') return { type: 'pause', title: '预演已中断' }
  return TERMINAL_EVENT_PRESENTATION[eventType] || null
}

export function taskExecutionIsActive(execution) {
  return ACTIVE_EXECUTION_STATES.has(String(execution?.state || ''))
}

/**
 * Convert durable task events into the route-planner drill timeline shape.
 * The conversion deliberately uses the same waypoint milestone source as the
 * guard-duty screen so reverse and repeated routes keep their physical labels.
 */
export function buildTaskExecutionTimeline(execution, {
  requestedAt = null,
  requestError = '',
} = {}) {
  const requestedTimestamp = Number(requestedAt) || 0
  const rawEvents = [...(execution?.events || [])]
    .sort((left, right) => (
      eventOrder(left) - eventOrder(right)
      || eventTimestamp(left) - eventTimestamp(right)
      || eventKey(left).localeCompare(eventKey(right))
    ))
  const seen = new Set()
  const events = rawEvents.filter((event) => {
    const key = eventKey(event)
    if (seen.has(key)) return false
    seen.add(key)
    return true
  })
  const firstEventTimestamp = events.map(eventTimestamp).find(Boolean) || 0
  const executionTimestamp = Date.parse(execution?.created_at || execution?.started_at || '')
  const startedAt = requestedTimestamp
    || (Number.isFinite(executionTimestamp) ? executionTimestamp : 0)
    || firstEventTimestamp
    || Date.now()
  const timeline = []
  const confirmedArrivalKeys = new Set(
    events
      .filter(event => event.event_type === 'task.arrival_confirmed')
      .map(eventArrivalKey),
  )

  if (requestedTimestamp) {
    timeline.push({
      id: 'preview-requested',
      type: requestError ? 'error' : 'dispatch',
      title: requestError ? '预演下发失败' : '路线下发中',
      detail: requestError || execution?.route_name || '',
      occurredAt: requestedTimestamp,
      elapsedSeconds: 0,
    })
  }

  events.forEach((event) => {
    if (
      event.event_type === 'task.waypoint_reached'
      && confirmedArrivalKeys.has(eventArrivalKey(event))
    ) return
    const presentation = timelinePresentation(event, execution)
    if (!presentation) return
    const occurredAt = eventTimestamp(event) || startedAt
    timeline.push({
      id: `execution-${eventKey(event)}`,
      ...presentation,
      detail: eventDetail(event),
      occurredAt,
      elapsedSeconds: Math.max(0, (occurredAt - startedAt) / 1000),
      stateVersion: event.state_version,
    })
  })

  return timeline
}

function mapPointNumberForIndex(waypoints, index) {
  const point = waypoints?.[index]
  if (!point) return index + 1
  if (point.map_point_number !== null && point.map_point_number !== undefined && point.map_point_number !== '') {
    return point.map_point_number
  }
  if (Number.isFinite(Number(point.sequence))) return Number(point.sequence) + 1
  return index + 1
}

/**
 * Resolve the live waypoint cursor for task execution UI.
 *
 * Prefer milestone events (same source as 巡检值守) so reverse loop routes
 * show map point numbers instead of the raw array index + 1.
 */
export function resolveTaskExecutionWaypointProgress(execution) {
  const waypoints = execution?.route_snapshot?.waypoints || []
  const milestones = milestoneEvents(execution)
  const states = guardDutyWaypointStates(waypoints, milestones)
  const activeTarget = activeGuardDutyTarget(milestones, waypoints, states)

  let currentIndex = Number(execution?.current_waypoint_index)
  if (!Number.isInteger(currentIndex) || currentIndex < 0) currentIndex = 0

  if (activeTarget) {
    const fromPayload = Number(activeTarget.payload?.execution_waypoint_index)
    if (Number.isInteger(fromPayload) && fromPayload >= 0 && fromPayload < waypoints.length) {
      currentIndex = fromPayload
    } else {
      const targetStateIndex = states.findIndex((state) => state === GUARD_WAYPOINT_STATE.target)
      if (targetStateIndex >= 0) currentIndex = targetStateIndex
    }
  } else if (waypoints.length) {
    currentIndex = Math.min(currentIndex, waypoints.length - 1)
  }

  const currentMapPointNumber = activeTarget?.payload?.waypoint?.map_point_number
    ?? mapPointNumberForIndex(waypoints, currentIndex)

  return {
    waypoints,
    milestones,
    states,
    activeTarget,
    currentIndex,
    currentMapPointNumber,
    totalWaypoints: Number(execution?.total_waypoints) || waypoints.length,
  }
}

export function taskExecutionWaypointClass({
  index,
  states = [],
  currentIndex = 0,
  failedIndexes = [],
  isActive = false,
}) {
  if (failedIndexes.includes(index)) return 'failed'
  const state = states[index]
  if (state === GUARD_WAYPOINT_STATE.reached) return 'done'
  if (state === GUARD_WAYPOINT_STATE.target && isActive) return 'current'
  if (!states.length || states.every((item) => item === GUARD_WAYPOINT_STATE.idle)) {
    if (index < currentIndex) return 'done'
    if (index === currentIndex && isActive) return 'current'
  }
  return ''
}

export function formatWaypointLabel(waypoints, index) {
  return `${mapPointNumberForIndex(waypoints, index)}号点`
}
