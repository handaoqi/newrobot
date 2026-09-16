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
  'task.completed': { type: 'complete', title: '任务执行完成' },
  'task.cancelled': { type: 'stop', title: '任务执行已取消' },
  'task.failed': { type: 'error', title: '任务执行失败' },
  'task.timed_out': { type: 'error', title: '任务执行超时' },
  'task.rejected': { type: 'error', title: '任务执行被拒绝' },
}

const TERMINAL_STATE_PRESENTATION = {
  completed: TERMINAL_EVENT_PRESENTATION['task.completed'],
  cancelled: TERMINAL_EVENT_PRESENTATION['task.cancelled'],
  failed: TERMINAL_EVENT_PRESENTATION['task.failed'],
  timed_out: TERMINAL_EVENT_PRESENTATION['task.timed_out'],
  rejected: TERMINAL_EVENT_PRESENTATION['task.rejected'],
}

const NAVIGATION_STAGE_PRESENTATION = {
  target_dispatch: '目标下发',
  path_planning: '全局规划与路径平滑',
  path_tracking: '路径跟踪',
  stop_confirmation: '停车与零速确认',
  localization_correction: '静止定位校正',
  fine_approach: '校正后细靠近',
  arrival_heading: '独立最终转向',
  micro_adjustment: '保持航向微调',
  arrival_acceptance: 'XY / 航向联合验收',
  waypoint_actions: '航点动作',
  waypoint_postprocess: '播报、驻留与后处理',
  departure_heading: '对准下个航点',
}

const NAVIGATION_STAGE_STATUS_LABELS = {
  active: '进行中',
  completed: '完成',
  failed: '失败',
  skipped: '跳过',
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

function hasFiniteNumber(value) {
  return value !== null && value !== undefined && value !== '' && Number.isFinite(Number(value))
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

function coarseArrivalCompletionText(payload) {
  if (payload?.coarse_completed !== true) return ''
  if (payload.arrival_mode === 'full_correction') return '一次细靠近后按 0.50m 完成'
  return '旧策略：按 0.50m 粗范围完成'
}

function coarseArrivalTitleSuffix(payload) {
  if (payload?.coarse_completed !== true) return ''
  if (payload.arrival_mode === 'full_correction') return '（0.50m 降级完成）'
  return '（粗范围完成）'
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
  if (Number.isFinite(Number(payload.stop_confirmation_seconds))) {
    arrival.push(`连续零速 ${Number(payload.stop_confirmation_seconds).toFixed(1)}s`)
  }
  if (Number.isFinite(Number(payload.elapsed_seconds))) {
    arrival.push(`本阶段 ${Number(payload.elapsed_seconds).toFixed(1)}s`)
  }
  const stageMetrics = payload.stage_metrics && typeof payload.stage_metrics === 'object'
    ? payload.stage_metrics
    : {}
  if (Number.isFinite(Number(stageMetrics.distance_m))) {
    arrival.push(`偏差 ${Number(stageMetrics.distance_m).toFixed(2)}m`)
  }
  if (Number.isFinite(Number(stageMetrics.acceptance_tolerance_m))) {
    arrival.push(`验收半径 ${Number(stageMetrics.acceptance_tolerance_m).toFixed(2)}m`)
  }
  if (Number.isFinite(Number(stageMetrics.reapproach_attempts))) {
    arrival.push(`追加靠近 ${Number(stageMetrics.reapproach_attempts)} 次`)
  }
  if (Number.isFinite(Number(stageMetrics.distance_remaining_m))) {
    arrival.push(`剩余 ${Number(stageMetrics.distance_remaining_m).toFixed(2)}m`)
  }
  if (Number.isFinite(Number(stageMetrics.next_map_point_number))) {
    arrival.push(`对准 ${Number(stageMetrics.next_map_point_number)} 号点`)
  }
  if (Number.isFinite(Number(stageMetrics.heading_error_deg))) {
    arrival.push(`航向误差 ${Number(stageMetrics.heading_error_deg).toFixed(1)}度`)
  }
  if (Number.isFinite(Number(stageMetrics.action_count))) {
    arrival.push(`动作 ${Number(stageMetrics.action_count)} 项`)
  }
  if (Number.isFinite(Number(stageMetrics.dwell_seconds)) && Number(stageMetrics.dwell_seconds) > 0) {
    arrival.push(`驻留 ${Number(stageMetrics.dwell_seconds).toFixed(1)}s`)
  }
  if (stageMetrics.localization_mode) arrival.push(`校正 ${String(stageMetrics.localization_mode).toUpperCase()}`)
  if (stageMetrics.speech_mode) arrival.push(`播报 ${stageMetrics.speech_mode}`)
  const coarseText = coarseArrivalCompletionText(payload)
  if (coarseText) arrival.push(coarseText)
  return [
    target ? `目标 ${target}` : '',
    robot ? `机器狗 ${robot}` : '',
    ...arrival,
    reason,
  ].filter(Boolean).join(' · ')
}

function executionFailureDetail(execution) {
  const failedCommand = [...(execution?.commands || [])]
    .sort((left, right) => Date.parse(right?.finished_at || right?.issued_at || '') - Date.parse(left?.finished_at || left?.issued_at || ''))
    .find(command => ['failed', 'timed_out', 'rejected', 'expired'].includes(String(command?.status || '')))
  const code = execution?.failure_code || failedCommand?.error_code || failedCommand?.ack_reason_code || ''
  const message = execution?.failure_message || failedCommand?.error_message || failedCommand?.ack_reason_message || ''
  return [code, message].filter(Boolean).join(' · ')
}

function systemLogDetail(log) {
  const data = log?.data || {}
  const startup = data.startup_progress && typeof data.startup_progress === 'object'
    ? data.startup_progress
    : null
  if (startup) {
    const actions = Array.isArray(startup.actions) ? startup.actions : []
    const completed = actions.filter(item => item?.status === 'completed').map(item => item.label).filter(Boolean)
    const active = actions.find(item => item?.status === 'in_progress')
    const failed = actions.find(item => item?.status === 'failed')
    return [
      completed.length ? `已完成：${completed.join('、')}` : '',
      failed?.label ? `失败：${failed.label}` : (active?.label ? `当前：${active.label}` : ''),
      startup.current_action && startup.current_action !== active?.label ? startup.current_action : '',
      startup.next_action ? `下一步：${startup.next_action}` : '',
    ].filter(Boolean).join(' · ')
  }
  const navigation = data.navigation_progress && typeof data.navigation_progress === 'object'
    ? data.navigation_progress
    : null
  if (navigation) {
    const progress = navigation.progress || {}
    const modules = navigation.modules || {}
    const strategy = navigation.strategy || {}
    const globalPlannerLabels = { theta_star: 'ThetaStar', navfn: 'NavFn', smac_hybrid: 'Smac Hybrid' }
    const localControllerLabels = { mppi: 'MPPI（FollowPath）', rpp: 'RPP', ilqr: 'iLQR' }
    const smootherLabels = { savitzky_golay: 'Savitzky-Golay', simple_smoother: 'Simple Smoother', passthrough_smoother: '直通平滑器' }
    const goalCheckerLabels = { general_goal_checker: '通用 GoalChecker', precision_goal_checker: '精确 GoalChecker' }
    const localizationLabels = { ndt: 'NDT', ukf: 'UKF', rtk: 'RTK' }
    const moduleNames = [
      globalPlannerLabels[modules.global_planner] || modules.global_planner,
      localControllerLabels[modules.local_controller] || modules.local_controller,
      smootherLabels[modules.smoother] || modules.smoother,
      goalCheckerLabels[modules.goal_checker] || modules.goal_checker,
    ].filter(Boolean)
    const speedLabels = { micro: '微速', low: '低速', medium: '中速', high: '高速' }
    const arrivalPolicyLabels = {
      pass_through: '通过不停留',
      stop_and_confirm: '停车校正确认',
      precision: '精确到点',
      dock: '停靠确认',
    }
    const strategyParts = [
      speedLabels[strategy.speed_level] || strategy.speed_level,
      strategy.speed_profile === 'final' ? '终点靠近' : '巡航',
      hasFiniteNumber(strategy.configured_linear_limit_mps)
        ? `线速度上限 ${Number(strategy.configured_linear_limit_mps).toFixed(2)}m/s`
        : '',
      strategy.detour_enabled === true ? '绕行开启' : strategy.detour_enabled === false ? '绕行关闭' : '',
      strategy.collision_slowdown_enabled === true ? '碰撞减速开启' : strategy.collision_slowdown_enabled === false ? '碰撞减速关闭' : '',
      strategy.collision_stop_enabled === true ? '硬急停开启' : '',
      strategy.arrival_policy ? `到点 ${arrivalPolicyLabels[strategy.arrival_policy] || strategy.arrival_policy}` : '',
      hasFiniteNumber(strategy.xy_goal_tolerance_m)
        ? `XY ${Number(strategy.xy_goal_tolerance_m).toFixed(2)}m`
        : '',
    ].filter(Boolean)
    return [
      hasFiniteNumber(progress.completed_waypoints) && hasFiniteNumber(progress.total_waypoints)
        ? `进度 ${Number(progress.completed_waypoints)}/${Number(progress.total_waypoints)}`
        : '',
      hasFiniteNumber(progress.distance_remaining_m)
        ? `距航点 ${Number(progress.distance_remaining_m).toFixed(2)}m`
        : '',
      moduleNames.length ? `执行模块：${moduleNames.join(' → ')}` : '',
      modules.localization_mode
        ? `定位策略：${localizationLabels[modules.localization_mode] || modules.localization_mode}`
        : '',
      strategyParts.length ? `控制策略：${strategyParts.join('、')}` : '',
    ].filter(Boolean).join(' · ')
  }
  const reason = data.reason_message || data.error_message || data.reason_code || data.error_code || ''
  const parts = [log?.event_code || '', reason]
  if (log?.waypoint_index !== null && log?.waypoint_index !== undefined) {
    parts.push(`航点序号 ${Number(log.waypoint_index) + 1}`)
  }
  if (Number(log?.repeat_count || 1) > 1) parts.push(`重复 ${Number(log.repeat_count)} 次`)
  return [...new Set(parts.filter(Boolean))].join(' · ')
}

function systemLogPresentation(log) {
  const level = String(log?.level || 'INFO').toUpperCase()
  return {
    type: level === 'ERROR' ? 'error' : level === 'WARNING' ? 'pause' : 'diagnostic',
    title: log?.message || log?.event_code || '任务诊断事件',
  }
}

function loopEventPresentation(event) {
  const state = String(event?.state || '')
  return {
    type: state === 'failed' ? 'error' : ['observing', 'recovering', 'paused'].includes(state) ? 'pause' : 'diagnostic',
    title: event?.reason_message || `循环任务：${event?.event_type || state || '状态更新'}`,
  }
}

function timelinePresentation(event, execution) {
  const eventType = String(event?.event_type || '')
  const waypointLabel = eventWaypointLabel(event, execution)
  if (eventType === 'task.created') return { type: 'created', title: '任务执行已创建' }
  if (eventType === 'task.accepted') return { type: 'accepted', title: '机器狗已接受任务' }
  if (eventType === 'task.started') return { type: 'start', title: '任务执行开始' }
  if (eventType === 'task.navigation_stage') {
    const payload = event?.payload || {}
    const stage = String(payload.navigation_stage || '')
    const status = String(payload.stage_status || 'active')
    const label = NAVIGATION_STAGE_PRESENTATION[stage] || stage || '导航阶段'
    const statusLabel = NAVIGATION_STAGE_STATUS_LABELS[status] || status
    return {
      type: status === 'failed' ? 'error' : (status === 'active' ? 'diagnostic' : 'arrival'),
      title: `${waypointLabel} · ${label}：${statusLabel}`,
      pointName: waypointLabel,
    }
  }
  if (eventType === 'task.target_dispatched') {
    return { type: 'target', title: `${waypointLabel}目标已下发`, pointName: waypointLabel }
  }
  if (eventType === 'task.waypoint_reached') {
    return { type: 'arrival', title: `${waypointLabel}已到达`, pointName: waypointLabel }
  }
  if (eventType === 'task.arrival_confirmed') {
    const suffix = coarseArrivalTitleSuffix(event?.payload)
    return { type: 'arrival', title: `${waypointLabel}验收完成${suffix}`, pointName: waypointLabel }
  }
  if (eventType === 'task.arrival_degraded_accepted') {
    return { type: 'arrival', title: `${waypointLabel}一次细靠近后按 0.50m 放行`, pointName: waypointLabel }
  }
  if (eventType === 'task.arrival_nav2_stopping') {
    return { type: 'pause', title: `${waypointLabel}等待 Nav2 停止`, pointName: waypointLabel }
  }
  if (eventType === 'task.arrival_zero_confirming') {
    return { type: 'pause', title: `${waypointLabel}零速确认中`, pointName: waypointLabel }
  }
  if (eventType === 'task.arrival_zero_confirmed') {
    return { type: 'arrival', title: `${waypointLabel}零速已确认`, pointName: waypointLabel }
  }
  if (eventType === 'task.arrival_zero_timeout') {
    return { type: 'error', title: `${waypointLabel}零速确认超时`, pointName: waypointLabel }
  }
  if (eventType === 'task.arrival_correcting') {
    return { type: 'pause', title: `${waypointLabel}静止定位校正`, pointName: waypointLabel }
  }
  if (eventType === 'task.pausing') return { type: 'pause', title: '正在暂停任务' }
  if (eventType === 'task.paused') return { type: 'pause', title: '任务已暂停' }
  if (eventType === 'task.resuming') return { type: 'resume', title: '正在恢复任务' }
  if (eventType === 'task.resumed') return { type: 'resume', title: '任务已恢复' }
  if (eventType === 'task.cancelling') return { type: 'stop', title: '正在取消任务' }
  if (eventType === 'task.interrupted') return { type: 'pause', title: '任务已中断' }
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

  const eventTypes = new Set(events.map(event => String(event?.event_type || '')))
  ;(execution?.system_logs || []).forEach((log) => {
    // State events already carry a richer, protocol-level representation.
    if (eventTypes.has(String(log?.event_code || ''))) return
    const occurredAt = Date.parse(log?.occurred_at || log?.received_at || '')
    const timestamp = Number.isFinite(occurredAt) ? occurredAt : startedAt
    timeline.push({
      id: `system-log-${log?.id || `${log?.event_code || 'event'}-${timestamp}`}`,
      ...systemLogPresentation(log),
      detail: systemLogDetail(log),
      occurredAt: timestamp,
      elapsedSeconds: Math.max(0, (timestamp - startedAt) / 1000),
    })
  })

  ;(execution?.loop_events || []).forEach((event) => {
    const occurredAt = Date.parse(event?.occurred_at || '')
    const timestamp = Number.isFinite(occurredAt) ? occurredAt : startedAt
    const detail = [
      event?.event_type || '',
      event?.reason_code || '',
      Number(event?.recovery_attempt || 0) > 0 ? `自愈第 ${Number(event.recovery_attempt)} 次` : '',
    ].filter(Boolean).join(' · ')
    timeline.push({
      id: `loop-event-${event?.id || `${event?.event_type || 'event'}-${timestamp}`}`,
      ...loopEventPresentation(event),
      detail,
      occurredAt: timestamp,
      elapsedSeconds: Math.max(0, (timestamp - startedAt) / 1000),
    })
  })

  const terminalPresentation = TERMINAL_STATE_PRESENTATION[String(execution?.state || '')]
  const hasTerminalEvent = events.some(event => TERMINAL_EVENT_PRESENTATION[event?.event_type])
  if (terminalPresentation && !hasTerminalEvent) {
    const finishedAt = Date.parse(execution?.finished_at || execution?.updated_at || '')
    const occurredAt = Number.isFinite(finishedAt) ? finishedAt : startedAt
    timeline.push({
      id: `execution-terminal-${execution?.id || execution?.state}`,
      ...terminalPresentation,
      detail: executionFailureDetail(execution),
      occurredAt,
      elapsedSeconds: Math.max(0, (occurredAt - startedAt) / 1000),
    })
  }

  timeline.forEach((item, index) => { item.timelineOrder = index })
  timeline.sort((left, right) => (
    Number(left.occurredAt || 0) - Number(right.occurredAt || 0)
    || left.timelineOrder - right.timelineOrder
  ))
  timeline.forEach((item) => { delete item.timelineOrder })

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
