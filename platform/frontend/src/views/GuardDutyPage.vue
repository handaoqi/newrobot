<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useRouter } from 'vue-router'

import AppToast from '../components/AppToast.vue'
import LiveVideoPlayer from '../components/LiveVideoPlayer.vue'
import RobotDogIcon from '../components/RobotDogIcon.vue'
import { useToast } from '../composables/useToast'
import {
  API_BASE,
  createPatrolLoopSession,
  executePatrolTask,
  fetchMapDetail,
  fetchOverview,
  fetchPatrolTasks,
  fetchPatrolLoopSession,
  fetchPatrolLoopSessions,
  fetchRobotDetail,
  fetchRobotNavigationStatus,
  fetchRobots,
  fetchRouteDetail,
  fetchTaskExecution,
  fetchTaskTrajectory,
  sendRecordedAudioCommand,
  sendRobotNavigationCommand,
  sendPatrolLoopSessionAction,
  sendTaskExecutionAction,
  sendTextToSpeechCommand,
} from '../services/api'
import { executionActions, isExecutionActive } from '../services/executionState'
import {
  buildLocalizationLossMarkers,
  currentRobotMapPose,
  localizationRecoveryLabel,
} from '../services/taskMapState'
import {
  guardDutyTaskOptions,
  initialGuardDutyExecution,
} from '../utils/guardDutyTaskSelection'
import {
  acquireGuardDutyLoopLease,
  releaseGuardDutyLoopLease,
  renewGuardDutyLoopLease,
} from '../utils/guardDutyLoopLease'
import {
  clearGuardDutyLoopExecution,
  guardDutyLoopCleanupExecutionId,
} from '../utils/guardDutyLoopStop'
import { resolveBatteryPercent } from '../utils/battery'
import {
  isLowBatteryBlocked,
  isLowBatteryStopAlert,
  isLowBatteryTaskError,
  lowBatteryGuardMessage,
} from '../utils/guardDutyLowBattery'
import { activateAndRelocalizeMap, activateRouteMap, waitForRobotCommand } from '../services/mapActivationFlow'
import {
  DEFAULT_LOOP_REST_SECONDS,
  ensureGuardDutyLoopNavigationReady,
  guardDutyLoopRepairFailureMessage,
  loopRestMilliseconds,
  restoreLoopRestSeconds,
  waitForGuardDutyLoopRepair,
} from '../services/guardDutyLoopNavRepair'
import { expectedLegacyMapVersion, navigationReadyForMap, navigationUnreadinessReason } from '../services/mapActivationState'
import { initializeProgressiveLocalization } from '../services/progressiveLocalization'
import {
  LOCALIZATION_ATTEMPT_COMMAND_TYPES,
  beginStoredAttemptSession,
  readStoredAttemptSession,
  updateStoredAttemptSession,
} from '../services/localizationAttemptSession'
import {
  activeGuardDutyTarget,
  guardDutyExecutionWaypointPlan,
  guardDutyRouteState,
  guardDutyWaypointStates,
} from '../utils/guardDutyWaypointState'
import {
  calculateTrajectoryDistance,
  displayedGuardDutyDistance,
  guardDutyTrajectoryCaptureState,
} from '../utils/guardDutyDistance'
import {
  guardDutySpeedLabel,
  guardDutySpeedReading,
  guardDutySpeedTitle,
  isLatestTrajectoryResponse,
} from '../utils/guardDutySpeed'
import { guardDutyExecutionReason } from '../utils/guardDutyPauseReason'

const overview = ref(null)
const robots = ref([])
const tasks = ref([])
const selectedRobot = ref(null)
const selectedTaskId = ref('')
const execution = ref(null)
const loading = ref(false)
const dataLoading = ref(true)
const busy = ref(false)
const localizationBusy = ref(false)
const navigationStatus = ref(null)
const localizationInitState = ref('uninitialized')
const localizationInitMessage = ref('地图未初始化')
const mapData = ref(null)
const routeData = ref(null)
const trajectory = ref([])
const trajectoryExecutionId = ref('')
const mapImageRef = ref(null)
const imageReadyTick = ref(0)
const loopDurationMinutes = ref(60)
const loopRestSeconds = ref(DEFAULT_LOOP_REST_SECONDS)
const loopActive = ref(false)
const loopState = ref('idle')
const loopStartedAt = ref(0)
const loopEndsAt = ref(0)
const loopStoppedAt = ref(0)
const loopRestUntil = ref(0)
const loopRounds = ref(0)
const loopSessionId = ref('')
const loopAccumulatedDistance = ref(0)
const loopCountedExecutionIds = ref([])
const loopCurrentExecutionId = ref('')
const loopMessage = ref('未启动循环巡检')
const serverLoopSession = ref(null)
const nowMs = ref(Date.now())
const liveSpeechOpen = ref(false)
const liveSpeechText = ref('')
const liveSpeechSending = ref(false)
const liveRecording = ref(false)
const liveRecordingSeconds = ref(0)
const streamUnavailable = ref(false)
const { toastMessage, toastVariant, visible, showToast } = useToast()
const router = useRouter()

let alertEventSource = null
let refreshTimer = null
let executionTimer = null
let loopTimer = null
let loopLeaseTimer = null
let streamRetryTimer = null
let loopCycleBusy = false
let loopLeaseOwned = false
let loopNavRepairBusy = false
let loopNavRepairToken = 0
const loopOwnerId = globalThis.crypto?.randomUUID?.()
  || `guard-${Date.now()}-${Math.random().toString(16).slice(2)}`
let localizationRunId = 0
let liveMediaRecorder = null
let liveRecordingStream = null
let liveRecordingTimer = null
let liveRecordingChunks = []
let trajectoryRequestGeneration = 0

const failedCommandStatuses = new Set(['rejected', 'failed', 'cancelled', 'timed_out', 'expired'])
const activeCommandStatuses = new Set(['created', 'published', 'accepted', 'executing'])

const latestRobot = computed(() => selectedRobot.value || overview.value?.latest_robot || null)
const playUrls = computed(() => latestRobot.value?.play_urls || {})
const playUrlKey = computed(() => `${latestRobot.value?.id || ''}\n${playUrls.value.flv || ''}\n${playUrls.value.hls || ''}`)
const hasStream = computed(() => !streamUnavailable.value && Boolean(playUrls.value.flv || playUrls.value.hls))
const taskOptions = computed(() => guardDutyTaskOptions(tasks.value, latestRobot.value?.id))
const presetTask = computed(() => {
  return taskOptions.value.find((task) => String(task.id) === String(selectedTaskId.value))
    || taskOptions.value[0]
    || null
})
const latestAlert = computed(() => latestRobot.value?.recent_events?.[0] || overview.value?.live_event || null)
const alerts = computed(() => latestRobot.value?.recent_events || [])
const pendingEventCount = computed(() => Number(overview.value?.summary?.pending_event_count || 0))
const batteryPercent = computed(() => resolveBatteryPercent(null, latestRobot.value))
const lowBatteryBlocked = computed(() => isLowBatteryBlocked(batteryPercent.value))
const actions = computed(() => executionActions(execution.value?.state))
const isRunning = computed(() => isExecutionActive(execution.value?.state))
const loopContinuationState = computed(() => String(serverLoopSession.value?.state || ''))
const manualTakeoverActive = computed(() => latestRobot.value?.control_mode === 'manual_takeover')
const taskControl = computed(() => {
  if (loopActive.value && loopSessionId.value) {
    if (loopContinuationState.value === 'recovering') {
      return { label: '自愈中...', action: '', enabled: false, hint: '正在执行恢复，请勿重复提交' }
    }
    if (['paused', 'observing'].includes(loopContinuationState.value)) {
      if (manualTakeoverActive.value) {
        return { label: '继续', action: '', enabled: false, hint: '先退出人工接管，再恢复自主控制' }
      }
      return { label: '继续', action: 'continue', enabled: true, hint: '将重新执行安全观察后恢复当前保存的航点和阶段' }
    }
    return { label: '暂停', action: 'pause', enabled: loopContinuationState.value !== 'stopping', hint: '暂停当前循环任务' }
  }
  return actions.value.control
})
const routeWaypoints = computed(() => execution.value?.route_snapshot?.waypoints || routeData.value?.waypoints || [])
const displayRouteWaypoints = computed(() => {
  const routeMapId = execution.value?.map_data || routeData.value?.map_data || presetTask.value?.map_id
  if (routeMapId && mapData.value?.id && String(routeMapId) !== String(mapData.value.id)) return []
  return routeWaypoints.value
})
const waypointMilestones = computed(() => (execution.value?.events || [])
  .filter((event) => ['task.target_dispatched', 'task.waypoint_reached'].includes(event.event_type))
  .sort((left, right) => Number(left.state_version || 0) - Number(right.state_version || 0)))
const executionWaypointOrder = computed(() => {
  const started = (execution.value?.events || []).find((event) => event.event_type === 'task.started')
  const order = started?.payload?.execution_waypoint_order
    || waypointMilestones.value[0]?.payload?.execution_waypoint_order
  if (Array.isArray(order) && order.length) return order
  return routeWaypoints.value.map((point, index) => point.map_point_number
    ?? (Number.isFinite(Number(point.sequence)) ? Number(point.sequence) + 1 : index + 1))
})
const executionWaypointPlan = computed(() => guardDutyExecutionWaypointPlan(
  displayRouteWaypoints.value,
  executionWaypointOrder.value,
))
const currentExecutionRound = computed(() => {
  if (!execution.value?.id) return 0
  return Math.max(1, Number(execution.value.round_number || 1))
})
const executionReason = computed(() => guardDutyExecutionReason(execution.value))
const waypointStates = computed(() => guardDutyWaypointStates(
  displayRouteWaypoints.value,
  waypointMilestones.value,
))
const routeStatusClass = computed(() => `is-${guardDutyRouteState(execution.value?.state)}`)
const activeTargetMilestone = computed(() => activeGuardDutyTarget(
  waypointMilestones.value,
  displayRouteWaypoints.value,
  waypointStates.value,
))
const localizationLossMarkers = computed(() => buildLocalizationLossMarkers(
  execution.value,
  trajectory.value,
  mapData.value?.id,
))
const currentExecutionDistance = computed(() => calculateTrajectoryDistance(trajectory.value))
const currentMovementSpeed = computed(() => guardDutySpeedReading({
  executionState: execution.value?.state,
  navigationStatus: navigationStatus.value,
  trajectory: trajectory.value,
  nowMs: nowMs.value,
}))
const currentMovementSpeedLabel = computed(() => guardDutySpeedLabel(currentMovementSpeed.value))
const currentMovementSpeedTitle = computed(() => guardDutySpeedTitle(currentMovementSpeed.value))
const displayedTotalDistance = computed(() => {
  return displayedGuardDutyDistance({
    currentDistance: currentExecutionDistance.value,
    currentExecutionId: execution.value?.id,
    executionLoopSessionId: execution.value?.loop_session_id,
    loopStartedAt: loopStartedAt.value,
    loopSessionId: loopSessionId.value,
    loopActive: loopActive.value,
    loopAccumulatedDistance: loopAccumulatedDistance.value,
    loopCountedExecutionIds: loopCountedExecutionIds.value,
    loopCurrentExecutionId: loopCurrentExecutionId.value,
    serverLoopSessionId: serverLoopSession.value?.id,
    serverTotalDistance: serverLoopSession.value?.total_distance_m,
  })
})
const elapsedMilliseconds = computed(() => {
  if (loopStartedAt.value) {
    const end = loopActive.value ? nowMs.value : (loopStoppedAt.value || nowMs.value)
    return Math.max(0, end - loopStartedAt.value)
  }
  const started = new Date(execution.value?.started_at || execution.value?.created_at || '').getTime()
  if (!Number.isFinite(started)) return 0
  const finished = new Date(execution.value?.finished_at || '').getTime()
  return Math.max(0, (Number.isFinite(finished) ? finished : nowMs.value) - started)
})
const loopRemainingMilliseconds = computed(() => loopActive.value ? Math.max(0, loopEndsAt.value - nowMs.value) : 0)
const restRemainingMilliseconds = computed(() => loopState.value === 'resting' ? Math.max(0, loopRestUntil.value - nowMs.value) : 0)
const guardRuntimeStatus = computed(() => {
  if (loopActive.value && ['observing', 'recovering', 'paused', 'stopping'].includes(loopState.value)) return loopMessage.value
  if (loopActive.value && loopState.value === 'resting') return `轮次休息中（${formatDuration(restRemainingMilliseconds.value)}）`
  if (loopActive.value && loopState.value === 'starting') return '正在启动下一轮'
  if (loopActive.value && loopState.value === 'finishing') return '循环到时，本轮结束后停止'
  if (loopActive.value && ['pausing', 'paused', 'interrupted'].includes(execution.value?.state)) {
    return `第 ${currentExecutionRound.value} 轮已暂停`
  }
  if (loopActive.value) return `循环巡检中 · 第 ${loopRounds.value} 轮`
  return taskStateText.value
})
const taskStateText = computed(() => {
  const labels = {
    created: '任务准备中',
    dispatching: '任务下发中',
    accepted: '机器狗准备中',
    running: '任务执行中',
    pausing: '任务暂停中',
    paused: '任务暂停中',
    resuming: '任务执行中',
    cancelling: '任务退出中',
    interrupted: '任务已中断',
    completed: '巡检已完成',
    failed: '巡检失败',
    cancelled: '任务结束',
    timed_out: '巡检已超时',
    rejected: '任务被拒绝',
  }
  return labels[execution.value?.state] || (presetTask.value ? '待执行' : '未配置任务')
})
const localizationInitLabel = computed(() => ({
  uninitialized: '地图未初始化',
  initializing: '初始化中',
  success: '初始化成功',
  failed: '初始化失败',
  not_ready: '定位/导航未就绪',
}[localizationInitState.value] || '地图未初始化'))

function sleep(ms) {
  return new Promise((resolve) => window.setTimeout(resolve, ms))
}

function createLoopSessionId() {
  if (window.crypto?.randomUUID) return window.crypto.randomUUID()
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (token) => {
    const value = Math.floor(Math.random() * 16)
    return (token === 'x' ? value : ((value & 0x3) | 0x8)).toString(16)
  })
}

function navigationReady(payload = navigationStatus.value) {
  const mapId = presetTask.value?.map_id || routeData.value?.map_data
  if (!payload || !mapId) return false
  return navigationReadyForMap(payload, mapId, expectedLegacyMapVersion(mapId))
}

function syncLocalizationState(payload = navigationStatus.value) {
  if (localizationBusy.value) return
  const command = payload?.command
  if (['nav.restart', 'nav.initial_pose'].includes(command?.command_type) && activeCommandStatuses.has(command?.status)) {
    localizationInitState.value = 'initializing'
    localizationInitMessage.value = '正在重启导航/定位栈'
    return
  }
  if (navigationReady(payload)) {
    localizationInitState.value = 'success'
    localizationInitMessage.value = '当前地图定位正常，导航栈已就绪'
    return
  }
  if (['nav.restart', 'nav.initial_pose'].includes(command?.command_type) && failedCommandStatuses.has(command?.status)) {
    localizationInitState.value = 'failed'
    localizationInitMessage.value = command.error_message || command.error_code || '最近一次定位初始化失败'
    return
  }
  const mapId = presetTask.value?.map_id || routeData.value?.map_data
  const reason = navigationUnreadinessReason(payload, mapId, expectedLegacyMapVersion(mapId))
  const uninitialized = !reason || reason === '尚未获取定位状态' || reason === '尚未加载任务地图'
  localizationInitState.value = uninitialized ? 'uninitialized' : 'not_ready'
  localizationInitMessage.value = reason || '地图未初始化'
}

function formatTime(value) {
  if (!value) return '--'
  const date = new Date(value)
  return Number.isNaN(date.getTime()) ? value : date.toLocaleString('zh-CN', { hour: '2-digit', minute: '2-digit', second: '2-digit' })
}

function formatExecutionTime(value) {
  if (!value) return '未执行'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return value
  const pad = number => String(number).padStart(2, '0')
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())} ${date.getFullYear()}/${pad(date.getMonth() + 1)}/${pad(date.getDate())}`
}

function executionStateLabel(state) {
  return ({
    created: '准备中',
    dispatching: '下发中',
    accepted: '已接收',
    running: '执行中',
    pausing: '暂停中',
    paused: '已暂停',
    resuming: '恢复中',
    cancelling: '退出中',
    interrupted: '已中断',
    completed: '已完成',
    failed: '已失败',
    cancelled: '已结束',
    timed_out: '已超时',
    rejected: '已拒绝',
  })[state] || '未执行'
}

function taskOptionLabel(task) {
  const latest = task.latest_execution
  if (!latest) return `${task.name} · 未执行`
  const date = new Date(latest.created_at || '')
  const timeLabel = Number.isNaN(date.getTime())
    ? ''
    : date.toLocaleString('zh-CN', { month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit' })
  return `${task.name} · ${executionStateLabel(latest.state)}${timeLabel ? ` · ${timeLabel}` : ''}`
}

function formatPose(pose) {
  if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return '位置未上报'
  const yaw = Number.isFinite(Number(pose.yaw)) ? ` / yaw ${Number(pose.yaw).toFixed(2)}` : ''
  return `X ${Number(pose.x).toFixed(2)} / Y ${Number(pose.y).toFixed(2)}${yaw}`
}

function formatDuration(milliseconds) {
  const totalSeconds = Math.max(0, Math.floor(Number(milliseconds || 0) / 1000))
  const hours = Math.floor(totalSeconds / 3600)
  const minutes = Math.floor((totalSeconds % 3600) / 60)
  const seconds = totalSeconds % 60
  return [hours, minutes, seconds].map((value) => String(value).padStart(2, '0')).join(':')
}

function fullUrl(relativeUrl) {
  if (!relativeUrl) return ''
  if (relativeUrl.startsWith('http')) return relativeUrl
  return `${API_BASE.replace('/api', '')}${relativeUrl}`
}

function refreshImageGeometry() {
  imageReadyTick.value += 1
}

function mapGeometry() {
  imageReadyTick.value
  const map = mapData.value
  const image = mapImageRef.value
  if (!map || !image) return null
  const mapWidth = Number(map.width || image.naturalWidth || 0)
  const mapHeight = Number(map.height || image.naturalHeight || 0)
  const resolution = Number(map.resolution || 0.05)
  const origin = Array.isArray(map.origin) && map.origin.length >= 2
    ? [Number(map.origin[0]), Number(map.origin[1]), Number(map.origin[2] || 0)]
    : [0, 0, 0]
  const rect = image.getBoundingClientRect()
  if (!mapWidth || !mapHeight || !resolution || !rect.width || !rect.height) return null
  return { mapWidth, mapHeight, resolution, origin, rect }
}

function displayPosition(point) {
  const geometry = mapGeometry()
  if (!geometry || point?.x === null || point?.x === undefined || point?.y === null || point?.y === undefined) return null
  const imageX = (Number(point.x) - geometry.origin[0]) / geometry.resolution
  const imageY = geometry.mapHeight - ((Number(point.y) - geometry.origin[1]) / geometry.resolution)
  if (!Number.isFinite(imageX) || !Number.isFinite(imageY)) return null
  return {
    left: `${imageX * (geometry.rect.width / geometry.mapWidth)}px`,
    top: `${imageY * (geometry.rect.height / geometry.mapHeight)}px`,
  }
}

function polylinePoints(points) {
  return points
    .map((point) => displayPosition(point))
    .filter(Boolean)
    .map((position) => `${Number.parseFloat(position.left)},${Number.parseFloat(position.top)}`)
    .join(' ')
}

function robotPoint() {
  return currentRobotMapPose(navigationStatus.value, mapData.value?.id, localizationLossMarkers.value)
}

function robotHeadingStyle() {
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - Number(robotPoint()?.yaw || 0)}rad)` }
}

function robotMarkerTitle() {
  const point = robotPoint()
  if (!point) return '暂无定位'
  return `${point.trusted ? '机器狗当前定位' : '机器狗最新上报位置（定位不可信）'}\nx=${Number(point.x).toFixed(2)}, y=${Number(point.y).toFixed(2)}, yaw=${Number(point.yaw || 0).toFixed(3)}`
}

function lossHeadingStyle(point) {
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - Number(point?.yaw || 0)}rad)` }
}

function lossMarkerTitle(point) {
  const score = Number(point.quality?.matching_error)
  const target = point.waypoint?.map_point_number || (Number.isFinite(Number(point.waypointIndex)) ? Number(point.waypointIndex) + 1 : '—')
  return `第 ${point.sequence} 次定位丢失\n最后可信位置 x=${Number(point.x).toFixed(2)}, y=${Number(point.y).toFixed(2)}\n目标 ${target}号点 · NDT ${Number.isFinite(score) ? score.toFixed(3) : '—'}\n${localizationRecoveryLabel(point.recoveryState)}\n${formatTime(point.occurredAt)}`
}

function waypointClass(index) {
  const state = waypointStates.value[index]
  if (state === 'target') return 'is-target'
  if (state === 'reached') return 'is-reached'
  return 'is-idle'
}

function eventImage(event) {
  return event?.annotated_snapshot_url || event?.snapshot_url || ''
}

function stopForLowBattery({ notify = true } = {}) {
  loopActive.value = false
  loopState.value = 'stopped'
  loopStoppedAt.value = Date.now()
  loopRestUntil.value = 0
  loopCurrentExecutionId.value = ''
  loopMessage.value = lowBatteryGuardMessage(batteryPercent.value)
  persistLoopState()
  releaseLoopOwnership()
  if (notify) showToast(loopMessage.value, { variant: 'alert', duration: 8000 })
}

const SERVER_LOOP_ACTIVE_STATES = new Set([
  'starting', 'running', 'resting', 'observing', 'recovering', 'paused', 'stopping',
])

function recoveryActionLabel(session) {
  const action = String(session?.metadata?.recovery_in_progress_action || '')
  if (action === 'nav2_reapproach') return '到点重接近恢复'
  if (action === 'precision_localization_recovery') return '精准定位恢复'
  if (action === 'localization_recovery_in_progress') return '定位恢复'
  if (action === 'resume_pending_waypoint') return '到点处理恢复'
  return '恢复动作'
}

function serverLoopMessage(session) {
  if (!session) return '未启动循环巡检'
  if (session.state === 'observing') {
    const started = new Date(session.observation_started_at || '').getTime()
    const blocker = session.metadata?.observation_blocker
    const blockerText = blocker?.message || blocker?.code || ''
    if (!Number.isFinite(started)) {
      return `等待安全条件 · ${blockerText || session.recovery_reason_message || '机器人保持停车'}`
    }
    const remaining = Math.max(0, 5 - Math.floor((Date.now() - started) / 1000))
    return `异常观察中 ${remaining} 秒 · ${blockerText || session.recovery_reason_message || '等待安全条件稳定'}`
  }
  if (session.state === 'recovering') {
    const started = new Date(session.metadata?.recovery_in_progress_started_at || '').getTime()
    const timeout = Number(session.metadata?.recovery_in_progress_timeout_seconds)
    if (Number.isFinite(started) && Number.isFinite(timeout) && timeout > 0) {
      const elapsed = Math.max(0, Math.floor((Date.now() - started) / 1000))
      return `${recoveryActionLabel(session)}进行中 · 第 ${session.recovery_attempt}/${session.recovery_max_attempts} 次（${elapsed}/${timeout} 秒）`
    }
    if (session.metadata?.recovery_in_progress_started_at) {
      return `${recoveryActionLabel(session)}进行中 · 第 ${session.recovery_attempt}/${session.recovery_max_attempts} 次`
    }
    return `正在自愈 ${session.recovery_attempt}/${session.recovery_max_attempts} · ${session.recovery_reason_message || '恢复导航'}`
  }
  if (session.state === 'stopping' && session.metadata?.stop_scope === 'round') {
    return '自愈耗尽，正在确认本轮机器人停车'
  }
  if (session.state === 'paused') return '循环已人工暂停，等待明确继续'
  if (session.state === 'resting') return `第 ${session.current_round} 轮完成，等待下一轮`
  if (session.state === 'low_battery_stopped') return lowBatteryGuardMessage(batteryPercent.value)
  if (session.state === 'completed') return '循环巡检已按设定时长完成'
  if (session.state === 'cancelled') return '循环已停止'
  if (session.state === 'failed') return session.last_error || '循环执行失败'
  if (session.state === 'starting') return '正在启动下一轮'
  return `循环巡检中 · 第 ${session.current_round} 轮`
}

function syncServerLoopSession(session) {
  if (!session?.id) return
  serverLoopSession.value = session
  loopSessionId.value = String(session.id)
  loopState.value = String(session.state || 'idle')
  loopActive.value = SERVER_LOOP_ACTIVE_STATES.has(loopState.value)
  loopStartedAt.value = new Date(session.started_at || '').getTime() || 0
  loopEndsAt.value = new Date(session.ends_at || '').getTime() || 0
  loopStoppedAt.value = new Date(session.finished_at || '').getTime() || 0
  loopRestUntil.value = session.state === 'resting' ? (new Date(session.next_action_at || '').getTime() || 0) : 0
  loopRounds.value = Number(session.current_round || 0)
  loopCurrentExecutionId.value = String(session.current_execution || '')
  loopMessage.value = serverLoopMessage(session)
  if (session.current_execution_detail) {
    execution.value = session.current_execution_detail
    selectedTaskId.value = String(session.task || selectedTaskId.value)
    startExecutionPolling()
  }
  persistLoopState()
}

async function restoreServerLoopSession(robotId) {
  if (!robotId) return null
  const active = await fetchPatrolLoopSessions({ robotId, active: true })
  if (active?.length) {
    syncServerLoopSession(active[0])
    return active[0]
  }
  if (loopSessionId.value && (loopActive.value || !serverLoopSession.value)) {
    try {
      const existing = await fetchPatrolLoopSession(loopSessionId.value)
      syncServerLoopSession(existing)
      return existing
    } catch {}
  }
  return null
}

function reconcileLowBatteryState() {
  const recentLowBattery = (latestRobot.value?.recent_events || []).find(isLowBatteryStopAlert)
  const detectedAt = new Date(recentLowBattery?.detected_at || '').getTime()
  const happenedDuringLoop = loopActive.value
    && Number.isFinite(detectedAt)
    && detectedAt >= loopStartedAt.value
  if (lowBatteryBlocked.value || happenedDuringLoop) stopForLowBattery({ notify: false })
}

function addRealtimeEvent(event) {
  if (!event?.id) return
  const robot = latestRobot.value
  if (!robot || (event.robot_code && event.robot_code !== robot.code)) return
  const isLowBattery = isLowBatteryStopAlert(event)
  if (isLowBattery) stopForLowBattery({ notify: false })
  const current = robot.recent_events || []
  if (current.some((item) => item.id === event.id)) return
  robot.recent_events = [event, ...current].slice(0, 5)
  if (event.status === 'pending' && overview.value?.summary) {
    overview.value.summary.pending_event_count = pendingEventCount.value + 1
  }
  showToast(isLowBattery ? loopMessage.value : '收到新的现场报警', {
    variant: 'alert',
    duration: isLowBattery ? 8000 : 5200,
  })
  if (isLowBattery) {
    void refreshExecution()
    if (loopSessionId.value) {
      void fetchPatrolLoopSession(loopSessionId.value).then(syncServerLoopSession).catch(() => {})
    }
  }
}

function openPendingEvents() {
  router.push({ name: 'events', query: { status: 'pending' } })
}

function openAlertStream() {
  const token = localStorage.getItem('inspection_token')
  if (!token || typeof EventSource === 'undefined') return
  alertEventSource?.close()
  alertEventSource = new EventSource(`${API_BASE}/events/stream/?token=${encodeURIComponent(token)}`)
  alertEventSource.addEventListener('inspection_event_created', (message) => {
    try {
      addRealtimeEvent(JSON.parse(message.data || '{}').event)
    } catch {}
  })
}

async function chooseRobot(robotId) {
  if (!robotId) return
  selectedRobot.value = await fetchRobotDetail(robotId)
  streamUnavailable.value = false
  await refreshLocalizationStatus()
}

async function load() {
  // The overview endpoint also loads the large event summary and can take
  // several seconds. It must not delay task/trajectory restoration, otherwise
  // the page shows its default 0 m state while the real execution is already
  // available from the lighter endpoints.
  const overviewPromise = fetchOverview()
    .then((result) => {
      overview.value = result
      return result
    })
    .catch((error) => {
      console.warn('值守概览刷新失败:', error)
      return null
    })
  const [robotResult, taskResult] = await Promise.all([
    fetchRobots(),
    fetchPatrolTasks(),
  ])
  robots.value = robotResult
  tasks.value = taskResult
  const robotId = robotResult[0]?.id
  if (robotId && selectedRobot.value?.id !== robotId) await chooseRobot(robotId)
  const availableTasks = guardDutyTaskOptions(taskResult, robotId)
  if (!availableTasks.some((task) => String(task.id) === String(selectedTaskId.value))) {
    selectedTaskId.value = String(availableTasks[0]?.id || '')
  }
  restoreLoopState(robotId)
  await restoreServerLoopSession(robotId)
  reconcileLowBatteryState()
  await restoreExecution(taskResult, robotId)
  await refreshExecutionVisual()
  void overviewPromise
}

function startExecutionPolling() {
  if (executionTimer) window.clearInterval(executionTimer)
  executionTimer = isRunning.value ? window.setInterval(refreshExecution, 2000) : null
}

async function restoreExecution(taskList, robotId) {
  if (loopActive.value && loopCurrentExecutionId.value) {
    try {
      execution.value = await fetchTaskExecution(loopCurrentExecutionId.value)
      selectedTaskId.value = String(execution.value.task || selectedTaskId.value)
      startExecutionPolling()
      return
    } catch {}
  }
  const availableTasks = guardDutyTaskOptions(taskList, robotId)
  const summary = initialGuardDutyExecution(availableTasks)
  if (!summary?.id) {
    execution.value = null
    startExecutionPolling()
    return
  }
  execution.value = await fetchTaskExecution(summary.id)
  selectedTaskId.value = String(execution.value.task || selectedTaskId.value)
  startExecutionPolling()
}

async function changeSelectedTask() {
  if (busy.value || loopActive.value || isRunning.value) return
  const task = presetTask.value
  if (!task) return
  busy.value = true
  if (executionTimer) window.clearInterval(executionTimer)
  executionTimer = null
  execution.value = null
  trajectory.value = []
  trajectoryExecutionId.value = ''
  routeData.value = null
  mapData.value = null
  localizationInitState.value = 'uninitialized'
  localizationInitMessage.value = '所选任务地图尚未初始化'
  try {
    if (task.latest_execution?.id) {
      execution.value = await fetchTaskExecution(task.latest_execution.id)
    }
    await refreshExecutionVisual()
  } catch (error) {
    showToast(error.message || '任务记录加载失败', { variant: 'alert' })
  } finally {
    busy.value = false
  }
}

async function refreshRobot() {
  if (!latestRobot.value?.id) return
  try {
    selectedRobot.value = await fetchRobotDetail(latestRobot.value.id)
    reconcileLowBatteryState()
  } catch {}
}

async function refreshLocalizationStatus({ sync = true } = {}) {
  if (!latestRobot.value?.id) return null
  try {
    const robotId = latestRobot.value.id
    let result = await fetchRobotNavigationStatus(robotId, { summary: true })
    const command = result?.command
    const storedAttempt = readStoredAttemptSession(robotId)
    const localizationCommand = LOCALIZATION_ATTEMPT_COMMAND_TYPES.has(command?.command_type)
    const commandIsActive = activeCommandStatuses.has(command?.status)
    const storedHasTerminalSnapshot = String(storedAttempt?.commandId || '') === String(command?.id || '')
      && Boolean(storedAttempt?.commandFinishedAt)
    // Manual initialization and loop repair already stream every command
    // snapshot. For task self-healing or commands started elsewhere, fetch
    // the detailed payload only while it can add fresh attempt information.
    if (
      localizationCommand
      && !localizationBusy.value
      && !loopNavRepairBusy
      && (commandIsActive || !storedHasTerminalSnapshot)
    ) {
      try {
        const detailed = await fetchRobotNavigationStatus(robotId)
        if (detailed?.localization_command) {
          updateStoredAttemptSession(robotId, detailed.localization_command, {
            phase: 'localization',
            showCandidates: true,
          })
        }
        result = detailed
      } catch {
        // Keep the compact status usable; the next poll retries attempt sync.
      }
    }
    navigationStatus.value = result
    if (sync) syncLocalizationState(result)
    return result
  } catch (error) {
    if (sync && !localizationBusy.value) {
      localizationInitState.value = 'failed'
      localizationInitMessage.value = error.message || '定位状态获取失败'
    }
    return null
  }
}

async function refreshGuardState() {
  const [, overviewResult] = await Promise.all([
    refreshRobot(),
    fetchOverview().catch(() => null),
    refreshLocalizationStatus(),
    refreshExecutionVisual(),
  ])
  if (overviewResult) overview.value = overviewResult
}

async function refreshExecutionVisual() {
  const executionId = execution.value?.id
  const requestGeneration = ++trajectoryRequestGeneration
  try {
    if (executionId) {
      const track = await fetchTaskTrajectory(executionId)
      if (!isLatestTrajectoryResponse({
        requestGeneration,
        latestGeneration: trajectoryRequestGeneration,
        requestedExecutionId: executionId,
        currentExecutionId: execution.value?.id,
      })) return
      trajectory.value = track.points || []
      trajectoryExecutionId.value = String(executionId)
    }
    const routeId = execution.value?.route || presetTask.value?.route
    if (routeId && String(routeData.value?.id || '') !== String(routeId)) {
      routeData.value = await fetchRouteDetail(routeId)
    }
    const mapId = (isRunning.value ? execution.value?.map_data : null)
      || routeData.value?.map_data
      || presetTask.value?.map_id
      || navigationStatus.value?.status?.map_id
      || navigationStatus.value?.current_map_id
    if (mapId && String(mapData.value?.id || '') !== String(mapId)) {
      mapData.value = await fetchMapDetail(mapId)
      await nextTick()
      refreshImageGeometry()
    }
  } catch (error) {
    console.warn('值守地图/轨迹刷新失败:', error)
  }
}

async function initializeLocalization() {
  const robot = latestRobot.value
  if (!robot?.id || localizationBusy.value || busy.value || isRunning.value) return
  if ((navigationStatus.value?.connection_status || robot.status) !== 'online') {
    localizationInitState.value = 'failed'
    localizationInitMessage.value = '机器人未在线，无法初始化定位'
    showToast(localizationInitMessage.value, { variant: 'alert' })
    return
  }

  const runId = ++localizationRunId
  localizationBusy.value = true
  localizationInitState.value = 'initializing'
  localizationInitMessage.value = '正在下发所选任务的路线地图'
  try {
    const mapId = presetTask.value?.map_id || routeData.value?.map_data
    if (!mapId) throw new Error('没有可初始化的地图，请先为值守任务配置路线地图')
    beginStoredAttemptSession(robot.id, { phase: 'transfer', commandType: 'map.activate' })
    if (String(mapData.value?.id || '') !== String(mapId)) {
      mapData.value = await fetchMapDetail(mapId)
    }
    const mapVersion = expectedLegacyMapVersion(mapId)
    const initialization = await initializeProgressiveLocalization({
      mapId,
      robotId: robot.id,
      mapVersion,
      sceneScope: routeData.value?.scene_scope || mapData.value?.scene_scope || 'indoor',
      coordinateMode: mapData.value?.coordinate_mode || '',
      waypoints: routeData.value?.waypoints || [],
      onProgress: message => { localizationInitMessage.value = message },
      onCommand: event => updateStoredAttemptSession(robot.id, event.command, event),
      dependencies: {
        activateRouteMap,
        sendRobotNavigationCommand,
        waitForRobotCommand,
      },
    })
    navigationStatus.value = initialization.activation.navigationStatus
    const command = initialization.command

    for (let attempt = 0; attempt < 25 && runId === localizationRunId; attempt += 1) {
      await sleep(3000)
      const latest = await refreshLocalizationStatus({ sync: false })
      if (!latest) continue
      const latestCommand = latest.command
      const isCurrentCommand = String(latestCommand?.id || '') === String(command.id || '')
      if (isCurrentCommand && failedCommandStatuses.has(latestCommand.status)) {
        throw new Error(latestCommand.error_message || latestCommand.error_code || '渐进定位初始化失败')
      }
      if (isCurrentCommand && latestCommand.status === 'succeeded') {
        localizationInitMessage.value = initialization.selectedSource === 'rtk_fixed'
          ? 'RTK固定解与本地NDT验证已完成，正在等待定位和导航栈同步'
          : '原点航向/1米范围、航点及全局匹配已完成，正在等待定位收敛'
        if (navigationReadyForMap(latest, mapId, mapVersion)) {
          localizationInitState.value = 'success'
          localizationInitMessage.value = '已重新初始化到最优定位点，导航栈已就绪'
          showToast('定位初始化成功')
          return
        }
      }
    }
    if (runId !== localizationRunId) return
    throw new Error('初始化超时，未检测到定位收敛或导航栈就绪')
  } catch (error) {
    if (runId !== localizationRunId) return
    if (error?.command) {
      updateStoredAttemptSession(robot.id, error.command, {
        phase: error.command.command_type === 'map.activate' ? 'transfer' : 'localization',
        showCandidates: error.command.command_type !== 'map.activate',
      })
    }
    localizationInitState.value = 'failed'
    localizationInitMessage.value = error.message || '定位初始化失败'
    showToast(localizationInitMessage.value, { variant: 'alert' })
  } finally {
    if (runId === localizationRunId) localizationBusy.value = false
  }
}

async function refreshExecution() {
  if (!execution.value?.id) return
  try {
    execution.value = await fetchTaskExecution(execution.value.id)
    await Promise.all([
      refreshExecutionVisual(),
      refreshLocalizationStatus(),
    ])
    if (!isRunning.value) {
      window.clearInterval(executionTimer)
      executionTimer = null
    }
  } catch {}
}

function loopStorageKey(robotId = latestRobot.value?.id) {
  return robotId ? `guard-duty-loop:${robotId}` : ''
}

function loopLeaseStorageKey(robotId = latestRobot.value?.id) {
  return robotId ? `guard-duty-loop-owner:${robotId}` : ''
}

function acquireLoopOwnership({ notify = false } = {}) {
  const key = loopLeaseStorageKey()
  if (!key) return false
  loopLeaseOwned = acquireGuardDutyLoopLease(localStorage, key, loopOwnerId)
  if (!loopLeaseOwned && notify) {
    showToast('循环巡检正在另一个页面运行，请在原页面操作', { variant: 'alert' })
  }
  return loopLeaseOwned
}

function renewLoopOwnership() {
  if (!loopLeaseOwned) return
  const key = loopLeaseStorageKey()
  loopLeaseOwned = Boolean(key) && renewGuardDutyLoopLease(localStorage, key, loopOwnerId)
}

function ensureLoopOwnership() {
  const key = loopLeaseStorageKey()
  if (!key) return false
  if (loopLeaseOwned && renewGuardDutyLoopLease(localStorage, key, loopOwnerId)) return true
  loopLeaseOwned = false
  return acquireLoopOwnership()
}

function releaseLoopOwnership() {
  const key = loopLeaseStorageKey()
  if (key && loopLeaseOwned) releaseGuardDutyLoopLease(localStorage, key, loopOwnerId)
  loopLeaseOwned = false
}

function handleLoopStorageChange(event) {
  if (event.storageArea !== localStorage) return
  if (event.key === loopStorageKey() && !loopLeaseOwned) restoreLoopState(latestRobot.value?.id)
}

function persistLoopState() {
  const key = loopStorageKey()
  if (!key) return
  localStorage.setItem(key, JSON.stringify({
    durationMinutes: Number(loopDurationMinutes.value),
    restSeconds: Number(loopRestSeconds.value),
    active: loopActive.value,
    state: loopState.value,
    startedAt: loopStartedAt.value,
    endsAt: loopEndsAt.value,
    stoppedAt: loopStoppedAt.value,
    restUntil: loopRestUntil.value,
    rounds: loopRounds.value,
    sessionId: loopSessionId.value,
    accumulatedDistance: loopAccumulatedDistance.value,
    countedExecutionIds: loopCountedExecutionIds.value,
    currentExecutionId: loopCurrentExecutionId.value,
    message: loopMessage.value,
  }))
}

function restoreLoopState(robotId) {
  const key = loopStorageKey(robotId)
  if (!key) return
  try {
    const saved = JSON.parse(localStorage.getItem(key) || '{}')
    loopDurationMinutes.value = Number(saved.durationMinutes) > 0 ? Number(saved.durationMinutes) : 60
    // The old restMinutes value is intentionally not reused: after the unit
    // change, existing browsers must receive the new 10-second default too.
    loopRestSeconds.value = restoreLoopRestSeconds(saved)
    loopStartedAt.value = Number(saved.startedAt || 0)
    loopEndsAt.value = Number(saved.endsAt || 0)
    loopStoppedAt.value = Number(saved.stoppedAt || 0)
    loopRestUntil.value = Number(saved.restUntil || 0)
    loopRounds.value = Number(saved.rounds || 0)
    loopSessionId.value = String(saved.sessionId || '')
    loopAccumulatedDistance.value = Number(saved.accumulatedDistance || 0)
    loopCountedExecutionIds.value = Array.isArray(saved.countedExecutionIds) ? saved.countedExecutionIds.map(String) : []
    loopCurrentExecutionId.value = String(saved.currentExecutionId || '')
    if (saved.active && loopEndsAt.value > Date.now()) {
      loopActive.value = true
      loopState.value = saved.state || 'running'
      loopMessage.value = saved.message || '循环巡检已恢复'
    } else {
      loopActive.value = false
      loopCurrentExecutionId.value = ''
      loopState.value = saved.active ? 'completed' : (saved.state || 'idle')
      loopStoppedAt.value = loopStoppedAt.value || (loopEndsAt.value ? Math.min(Date.now(), loopEndsAt.value) : 0)
      loopMessage.value = saved.active ? '循环时长已结束' : (saved.message || '未启动循环巡检')
    }
  } catch {
    localStorage.removeItem(key)
  }
}

function finishLoop(message, { notify = true } = {}) {
  loopNavRepairToken += 1
  loopNavRepairBusy = false
  loopActive.value = false
  loopState.value = 'completed'
  loopStoppedAt.value = Date.now()
  loopRestUntil.value = 0
  loopMessage.value = message
  persistLoopState()
  releaseLoopOwnership()
  if (notify) showToast(message)
}

async function stopLoop({ notify = true, clearExecution = true } = {}) {
  if (!loopActive.value) return null
  if (loopSessionId.value) {
    busy.value = true
    try {
      const session = await sendPatrolLoopSessionAction(loopSessionId.value, 'stop')
      syncServerLoopSession(session)
      if (notify) showToast(loopMessage.value, { variant: 'alert' })
      return session
    } catch (error) {
      if (notify) showToast(error.message || '停止中心循环失败', { variant: 'alert' })
      return null
    } finally {
      busy.value = false
    }
  }
  loopNavRepairToken += 1
  loopNavRepairBusy = false
  const executionId = guardDutyLoopCleanupExecutionId(loopCurrentExecutionId.value, execution.value)
  const shouldClearExecution = clearExecution && Boolean(executionId)
  loopActive.value = false
  loopState.value = 'stopped'
  loopStoppedAt.value = Date.now()
  loopRestUntil.value = 0
  loopMessage.value = shouldClearExecution ? '正在停止循环并清理当前任务' : '循环已停止'
  persistLoopState()
  if (!shouldClearExecution) {
    releaseLoopOwnership()
    if (notify) showToast(loopMessage.value)
    return null
  }

  busy.value = true
  try {
    execution.value = await clearGuardDutyLoopExecution(sendTaskExecutionAction, executionId)
    loopCurrentExecutionId.value = ''
    loopMessage.value = '循环和当前任务已停止，任务状态已清理'
    persistLoopState()
    startExecutionPolling()
    if (notify) showToast(loopMessage.value, { variant: 'alert' })
    return execution.value
  } catch (error) {
    loopMessage.value = `循环已停止，但任务状态清理失败：${error.message || '请点击强制退出重试'}`
    persistLoopState()
    if (notify) showToast(loopMessage.value, { variant: 'alert' })
    return null
  } finally {
    busy.value = false
    releaseLoopOwnership()
  }
}

async function launchTask({ fromLoop = false } = {}) {
  if (!presetTask.value || busy.value || isRunning.value || (!fromLoop && loopActive.value)) return null
  if (lowBatteryBlocked.value) {
    showToast(lowBatteryGuardMessage(batteryPercent.value), { variant: 'alert' })
    return null
  }
  if (fromLoop && !navigationReady()) {
    try {
      const readiness = await ensureLoopNavigationReady((message) => {
        loopMessage.value = message
      })
      if (!readiness.ok) {
        showToast('定位或导航栈未就绪，本轮稍后重试', { variant: 'alert' })
        return null
      }
    } catch (error) {
      showToast(guardDutyLoopRepairFailureMessage(error), { variant: 'alert' })
      return null
    }
  }
  busy.value = true
  try {
    const requestedRound = fromLoop ? loopRounds.value + 1 : 1
    execution.value = await executePatrolTask(presetTask.value.id, {
      loopExecution: fromLoop,
      loopSessionId: fromLoop ? loopSessionId.value : null,
      roundNumber: requestedRound,
    })
    trajectoryRequestGeneration += 1
    trajectory.value = []
    trajectoryExecutionId.value = String(execution.value.id)
    if (fromLoop) {
      loopRounds.value = Number(execution.value.round_number || requestedRound)
      loopSessionId.value = String(execution.value.loop_session_id || loopSessionId.value)
      loopCurrentExecutionId.value = String(execution.value.id)
      loopState.value = 'running'
      loopMessage.value = `第 ${loopRounds.value} 轮巡检执行中`
      persistLoopState()
    } else {
      loopStartedAt.value = 0
      loopEndsAt.value = 0
      loopStoppedAt.value = 0
      loopRounds.value = 0
      loopSessionId.value = ''
      loopAccumulatedDistance.value = 0
      loopCountedExecutionIds.value = []
      loopCurrentExecutionId.value = ''
      loopState.value = 'idle'
      loopMessage.value = '当前为单次巡检'
      persistLoopState()
    }
    showToast(fromLoop ? `循环巡检第 ${loopRounds.value} 轮已开始` : `已开始执行：${presetTask.value.name}`)
    startExecutionPolling()
    await refreshExecutionVisual()
    return execution.value
  } catch (error) {
    if (isLowBatteryTaskError(error)) stopForLowBattery()
    else showToast(error.message || '任务启动失败', { variant: 'alert' })
    return null
  } finally {
    busy.value = false
  }
}

async function startTask() {
  await launchTask()
}

async function captureLoopDistance() {
  const executionId = String(loopCurrentExecutionId.value || '')
  if (!executionId || loopCountedExecutionIds.value.includes(executionId)) return true
  // The task can become terminal before the last trajectory batches arrive at
  // the cloud. Always refresh here so the first capture cannot freeze 0 m.
  await refreshExecutionVisual()
  const distance = calculateTrajectoryDistance(trajectory.value)
  const captureState = guardDutyTrajectoryCaptureState({
    execution: execution.value,
    points: trajectory.value,
    distance,
  })
  if (!captureState.ready) {
    const retrySeconds = Math.max(1, Math.ceil(captureState.retryAfterMilliseconds / 1000))
    loopMessage.value = `本轮轨迹数据同步中，最多等待 ${retrySeconds} 秒`
    persistLoopState()
    return false
  }
  loopAccumulatedDistance.value += distance
  loopCountedExecutionIds.value = [...loopCountedExecutionIds.value, executionId].slice(-100)
  persistLoopState()
  return true
}

function scheduleNextLoopRound(message = '', { shortRetry = false } = {}) {
  const restMilliseconds = loopRestMilliseconds(loopRestSeconds.value, { shortRetry })
  loopState.value = 'resting'
  loopRestUntil.value = Date.now() + restMilliseconds
  const defaultMessage = shortRetry
    ? `启动或导航维护失败，${Math.round(restMilliseconds / 1000)} 秒后重试`
    : `第 ${loopRounds.value} 轮完成，休息 ${loopRestSeconds.value} 秒`
  loopMessage.value = message || defaultMessage
  persistLoopState()
  // Use the rest window to bring Nav2 / localization back instead of idle waiting.
  void startLoopRestNavigationRepair()
}

function loopTargetMapId() {
  return presetTask.value?.map_id || routeData.value?.map_data || mapData.value?.id || null
}

async function ensureLoopNavigationReady(onProgress = () => {}) {
  const robot = latestRobot.value
  const mapId = loopTargetMapId()
  if (!robot?.id || !mapId) {
    throw new Error('缺少机器人或任务地图，无法维护导航栈')
  }
  const mapVersion = expectedLegacyMapVersion(mapId)
  const result = await ensureGuardDutyLoopNavigationReady({
    fetchStatus: () => fetchRobotNavigationStatus(robot.id, { summary: true }),
    isReady: (status) => navigationReadyForMap(status, mapId, mapVersion),
    repair: async ({ onProgress: repairProgress }) => {
      beginStoredAttemptSession(robot.id, { phase: 'transfer', commandType: 'map.activate' })
      try {
        const outcome = await activateAndRelocalizeMap({
          mapId,
          robotId: robot.id,
          mapVersion,
          waypoints: routeWaypoints.value,
          onProgress: repairProgress,
          onCommand: event => updateStoredAttemptSession(robot.id, event.command, event),
        })
        return outcome.navigationStatus
      } catch (error) {
        if (error?.command) {
          updateStoredAttemptSession(robot.id, error.command, {
            phase: error.command.command_type === 'map.activate' ? 'transfer' : 'localization',
            showCandidates: error.command.command_type !== 'map.activate',
          })
        }
        throw error
      }
    },
    onProgress,
  })
  if (result.status) {
    navigationStatus.value = result.status
    syncLocalizationState(result.status)
  }
  return result
}

async function startLoopRestNavigationRepair() {
  if (!loopActive.value || loopNavRepairBusy) return
  const token = ++loopNavRepairToken
  loopNavRepairBusy = true
  try {
    const result = await ensureLoopNavigationReady((message) => {
      if (token !== loopNavRepairToken || !loopActive.value) return
      if (loopState.value === 'resting' || loopState.value === 'starting') {
        loopMessage.value = message
      }
    })
    if (token !== loopNavRepairToken || !loopActive.value) return
    if (result.ok && loopState.value === 'resting') {
      loopMessage.value = `轮次休息中，导航/定位已就绪（剩余 ${formatDuration(restRemainingMilliseconds.value)}）`
      persistLoopState()
    } else if (!result.ok && loopState.value === 'resting') {
      loopMessage.value = '轮次休息维护未完成，将在休息结束后再次修复'
      persistLoopState()
    }
  } catch (error) {
    if (token !== loopNavRepairToken || !loopActive.value) return
    if (loopState.value === 'resting') {
      loopMessage.value = guardDutyLoopRepairFailureMessage(error, { duringRest: true })
      persistLoopState()
    }
  } finally {
    if (token === loopNavRepairToken) loopNavRepairBusy = false
  }
}

async function runLoopCycle() {
  nowMs.value = Date.now()
  if (loopSessionId.value && (loopActive.value || !serverLoopSession.value)) {
    if (loopCycleBusy) return
    loopCycleBusy = true
    try {
      syncServerLoopSession(await fetchPatrolLoopSession(loopSessionId.value))
    } catch (error) {
      loopMessage.value = `中心循环状态同步失败：${error.message || '稍后重试'}`
    } finally {
      loopCycleBusy = false
    }
    return
  }
  if (!loopActive.value || loopCycleBusy) return
  if (lowBatteryBlocked.value) {
    stopForLowBattery()
    return
  }
  if (!ensureLoopOwnership()) return
  loopCycleBusy = true
  try {
    if (nowMs.value >= loopEndsAt.value) {
      if (isRunning.value && String(execution.value?.id || '') === String(loopCurrentExecutionId.value || '')) {
        loopState.value = 'finishing'
        loopMessage.value = '循环时长已到，本轮结束后停止'
        persistLoopState()
        return
      }
      if (!await captureLoopDistance()) return
      finishLoop('循环巡检已按设定时长完成')
      return
    }

    if (loopState.value === 'resting') {
      if (nowMs.value < loopRestUntil.value || busy.value) return
      loopState.value = 'starting'
      loopMessage.value = '休息结束，正在确认导航/定位就绪'
      persistLoopState()
      if (loopNavRepairBusy) {
        let repairGate
        try {
          repairGate = await waitForGuardDutyLoopRepair({
            isBusy: () => loopNavRepairBusy && loopActive.value,
            fetchStatus: () => fetchRobotNavigationStatus(latestRobot.value.id, { summary: true }),
            isReady: (status) => navigationReadyForMap(
              status,
              loopTargetMapId(),
              expectedLegacyMapVersion(loopTargetMapId()),
            ),
            onProgress: (message) => {
              if (loopActive.value) loopMessage.value = message
            },
          })
        } catch (error) {
          if (loopActive.value) {
            scheduleNextLoopRound(guardDutyLoopRepairFailureMessage(error), { shortRetry: true })
          }
          return
        }
        if (!loopActive.value) return
        if (repairGate.status) {
          navigationStatus.value = repairGate.status
          syncLocalizationState(repairGate.status)
        }
        if (!repairGate.ok) {
          scheduleNextLoopRound('导航维护仍在执行，短间隔后重新检查', { shortRetry: true })
          return
        }
        if (repairGate.supersede) {
          // The device is already ready; detach the stale page-side repair
          // promise so it cannot keep the loop in `starting`.
          loopNavRepairToken += 1
          loopNavRepairBusy = false
        }
      }
      try {
        const readiness = await ensureLoopNavigationReady((message) => {
          if (loopActive.value && (loopState.value === 'starting' || loopState.value === 'resting')) {
            loopMessage.value = message
          }
        })
        if (!readiness.ok) {
          if (loopActive.value) {
            scheduleNextLoopRound('导航/定位仍未就绪，短间隔后重试修复', { shortRetry: true })
          }
          return
        }
      } catch (error) {
        if (loopActive.value) {
          scheduleNextLoopRound(guardDutyLoopRepairFailureMessage(error), { shortRetry: true })
        }
        return
      }
      const started = await launchTask({ fromLoop: true })
      if (!started && loopActive.value) {
        scheduleNextLoopRound('下一轮启动失败，短间隔后重试', { shortRetry: true })
      }
      return
    }

    if (isRunning.value && String(execution.value?.id || '') === String(loopCurrentExecutionId.value || '')) {
      if (loopState.value !== 'finishing') loopState.value = 'running'
      return
    }

    if (loopCurrentExecutionId.value && String(execution.value?.id || '') === String(loopCurrentExecutionId.value)) {
      if (!await captureLoopDistance()) return
      scheduleNextLoopRound(execution.value?.state === 'completed'
        ? `第 ${loopRounds.value} 轮完成，进入休息并维护导航栈`
        : `第 ${loopRounds.value} 轮已结束，休息中维护导航栈后继续`)
      return
    }

    const started = await launchTask({ fromLoop: true })
    if (!started && loopActive.value) {
      scheduleNextLoopRound('任务启动失败，短间隔后重试', { shortRetry: true })
    }
  } finally {
    loopCycleBusy = false
  }
}

async function toggleLoop() {
  if (loopActive.value) {
    await stopLoop()
    return
  }
  if (!presetTask.value || isRunning.value || busy.value || localizationBusy.value) return
  if (lowBatteryBlocked.value) {
    showToast(lowBatteryGuardMessage(batteryPercent.value), { variant: 'alert' })
    return
  }
  if (!navigationReady()) {
    showToast('请先完成地图定位初始化', { variant: 'alert' })
    return
  }
  const duration = Number(loopDurationMinutes.value)
  const rest = Number(loopRestSeconds.value)
  if (!Number.isFinite(duration) || duration <= 0 || !Number.isFinite(rest) || rest < 0) {
    showToast('请填写正确的循环时长和休息时间', { variant: 'alert' })
    return
  }
  busy.value = true
  try {
    const session = await createPatrolLoopSession({
      taskId: presetTask.value.id,
      durationSeconds: Math.round(duration * 60),
      restSeconds: Math.round(rest),
      sessionId: createLoopSessionId(),
    })
    loopAccumulatedDistance.value = 0
    loopCountedExecutionIds.value = []
    syncServerLoopSession(session)
    showToast('中心循环巡检已启动')
    await refreshExecutionVisual()
  } catch (error) {
    if (isLowBatteryTaskError(error)) stopForLowBattery()
    else showToast(error.message || '中心循环启动失败', { variant: 'alert' })
  } finally {
    busy.value = false
  }
}

async function controlTask() {
  const action = taskControl.value.action
  if ((!execution.value?.id && !loopSessionId.value) || busy.value || !taskControl.value.enabled || !action) return
  busy.value = true
  try {
    if (loopActive.value && loopSessionId.value) {
      syncServerLoopSession(await sendPatrolLoopSessionAction(loopSessionId.value, action))
      showToast(action === 'pause' ? '循环已人工暂停' : '已请求继续，正在进行安全观察')
    } else {
      execution.value = await sendTaskExecutionAction(execution.value.id, action)
      showToast(action === 'pause' ? '任务暂停中' : '任务继续执行中')
    }
    startExecutionPolling()
  } catch (error) {
    showToast(error.message || (action === 'pause' ? '暂停任务失败' : '继续任务失败'), { variant: 'alert' })
  } finally {
    busy.value = false
  }
}

async function forceExitTask() {
  if (!execution.value?.id || busy.value) return
  if (!window.confirm('强制退出会停止当前导航，并清理该机器人的全部未结束任务。确认继续？')) return
  const serverLoopWasActive = Boolean(loopActive.value && loopSessionId.value)
  await stopLoop({ notify: false, clearExecution: false })
  if (serverLoopWasActive) {
    showToast('循环和当前任务已强制退出', { variant: 'alert' })
    return
  }
  busy.value = true
  try {
    execution.value = await sendTaskExecutionAction(execution.value.id, 'force-exit')
    showToast('任务已强制退出', { variant: 'alert' })
    startExecutionPolling()
  } catch (error) {
    showToast(error.message || '强制退出失败', { variant: 'alert' })
  } finally {
    busy.value = false
  }
}

function liveRecordingDurationLabel() {
  const minutes = String(Math.floor(liveRecordingSeconds.value / 60)).padStart(2, '0')
  const seconds = String(liveRecordingSeconds.value % 60).padStart(2, '0')
  return `${minutes}:${seconds}`
}

function pickLiveRecordingMimeType() {
  const candidates = ['audio/webm;codecs=opus', 'audio/webm', 'audio/ogg;codecs=opus', 'audio/ogg']
  return candidates.find((type) => window.MediaRecorder?.isTypeSupported(type)) || ''
}

function cleanupLiveRecorder() {
  if (liveRecordingTimer) window.clearInterval(liveRecordingTimer)
  liveRecordingTimer = null
  if (liveRecordingStream) liveRecordingStream.getTracks().forEach((track) => track.stop())
  liveRecordingStream = null
  liveMediaRecorder = null
  liveRecording.value = false
}

async function uploadLiveRecording(blob) {
  const robot = latestRobot.value
  if (!robot?.id || !blob?.size) {
    showToast('没有录到有效声音', { variant: 'alert' })
    return
  }
  const extension = blob.type.includes('ogg') ? 'ogg' : 'webm'
  const file = new File([blob], `guard-live-speech.${extension}`, { type: blob.type || 'audio/webm' })
  liveSpeechSending.value = true
  try {
    const result = await sendRecordedAudioCommand(robot.id, file, {
      title: `值守实时喊话 ${new Date().toLocaleTimeString('zh-CN', { hour12: false })}`,
      playNow: true,
    })
    const recording = result?.recording || {}
    const transcript = String(recording.transcript || '').trim()
    if (transcript) {
      liveSpeechText.value = transcript
      showToast('实时录音已播放，ASR文字已填入文字播报栏')
    } else if (recording.asr_status === 'failed') {
      showToast(`录音已下发，但ASR失败：${recording.asr_error || '未识别到文字'}`, { variant: 'alert' })
    } else {
      showToast('实时录音已下发，但未识别到文字', { variant: 'alert' })
    }
  } catch (error) {
    showToast(error.message || '实时录音下发失败', { variant: 'alert' })
  } finally {
    liveSpeechSending.value = false
  }
}

async function startLiveRecording() {
  if (liveRecording.value || liveSpeechSending.value) return
  if (!navigator.mediaDevices?.getUserMedia || !window.MediaRecorder) {
    showToast('当前浏览器不支持麦克风录音', { variant: 'alert' })
    return
  }
  if (!window.isSecureContext) {
    showToast('网页麦克风需要使用 HTTPS 地址打开本页面', { variant: 'alert' })
    return
  }
  try {
    liveRecordingStream = await navigator.mediaDevices.getUserMedia({
      audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: true },
    })
    const mimeType = pickLiveRecordingMimeType()
    liveMediaRecorder = new MediaRecorder(liveRecordingStream, mimeType ? { mimeType } : undefined)
    liveRecordingChunks = []
    liveMediaRecorder.ondataavailable = (event) => {
      if (event.data?.size > 0) liveRecordingChunks.push(event.data)
    }
    liveMediaRecorder.onstop = async () => {
      const blobType = liveMediaRecorder?.mimeType || mimeType || 'audio/webm'
      const blob = new Blob(liveRecordingChunks, { type: blobType })
      cleanupLiveRecorder()
      await uploadLiveRecording(blob)
    }
    liveMediaRecorder.start(250)
    liveRecording.value = true
    liveRecordingSeconds.value = 0
    liveRecordingTimer = window.setInterval(() => { liveRecordingSeconds.value += 1 }, 1000)
    showToast('网页麦克风已接通，结束后将立即播放')
  } catch (error) {
    cleanupLiveRecorder()
    showToast(error?.name === 'NotAllowedError' ? '麦克风权限被拒绝' : '无法接通网页麦克风', { variant: 'alert' })
  }
}

function stopLiveRecordingAndPlay() {
  if (!liveRecording.value || !liveMediaRecorder || liveMediaRecorder.state === 'inactive') return
  liveMediaRecorder.stop()
}

async function sendLiveTextSpeech() {
  const robot = latestRobot.value
  const text = liveSpeechText.value.trim()
  if (!robot?.id || !text || liveSpeechSending.value || liveRecording.value) return
  liveSpeechSending.value = true
  try {
    await sendTextToSpeechCommand(robot.id, text, '保安值守实时喊话')
    showToast('实时文字喊话已下发')
  } catch (error) {
    showToast(error.message || '实时喊话下发失败', { variant: 'alert' })
  } finally {
    liveSpeechSending.value = false
  }
}

function toggleLiveSpeechPanel() {
  if (liveRecording.value || liveSpeechSending.value) return
  liveSpeechOpen.value = !liveSpeechOpen.value
}

function showVideoNotice({ message, variant }) {
  showToast(message, variant ? { variant } : undefined)
}

function handleStreamError() {
  streamUnavailable.value = true
  if (streamRetryTimer) return
  streamRetryTimer = window.setTimeout(() => {
    streamRetryTimer = null
    streamUnavailable.value = false
  }, 8000)
}

onMounted(async () => {
  let loaded = false
  try {
    await load()
    loaded = true
    openAlertStream()
  } catch (error) {
    showToast(error.message || '加载值守页面失败', { variant: 'alert' })
  } finally {
    dataLoading.value = false
  }
  if (loaded) {
    refreshTimer = window.setInterval(refreshGuardState, 5000)
    loopTimer = window.setInterval(runLoopCycle, 1000)
    loopLeaseTimer = window.setInterval(renewLoopOwnership, 2000)
    if (loopActive.value) acquireLoopOwnership()
    window.addEventListener('storage', handleLoopStorageChange)
    window.addEventListener('resize', refreshImageGeometry)
  }
})

onBeforeUnmount(() => {
  trajectoryRequestGeneration += 1
  localizationRunId += 1
  window.clearInterval(refreshTimer)
  window.clearInterval(executionTimer)
  window.clearInterval(loopTimer)
  window.clearInterval(loopLeaseTimer)
  if (streamRetryTimer) window.clearTimeout(streamRetryTimer)
  window.removeEventListener('storage', handleLoopStorageChange)
  window.removeEventListener('resize', refreshImageGeometry)
  releaseLoopOwnership()
  if (liveMediaRecorder && liveMediaRecorder.state !== 'inactive') {
    liveMediaRecorder.onstop = null
    liveMediaRecorder.stop()
  }
  cleanupLiveRecorder()
  alertEventSource?.close()
})

watch(playUrlKey, () => {
  streamUnavailable.value = false
})
</script>

<template>
  <section class="guard-page">
    <div v-if="loading" class="guard-loading">正在加载值守画面...</div>
    <template v-else>
      <header class="guard-header">
        <div>
          <span class="guard-eyebrow">值守模式</span>
          <h1>机器狗巡检值守</h1>
        </div>
        <div class="guard-status" :class="latestRobot?.status === 'online' ? 'is-online' : 'is-offline'">
          <span class="guard-status-dot"></span>
          {{ latestRobot?.status_label || '设备状态未知' }}
        </div>
      </header>

      <main class="guard-grid">
        <section class="guard-video-panel">
          <div class="guard-video-stage">
            <LiveVideoPlayer
              :play-urls="playUrls"
              :robot-id="latestRobot?.id"
              :available="hasStream"
              :loading="dataLoading"
              object-fit="contain"
              @notice="showVideoNotice"
              @stream-error="handleStreamError"
            >
              <template #empty>
                <div class="guard-video-empty">
                  <RobotDogIcon :size="64" label="机器狗示意图标" />
                  <strong>视频暂不可用</strong>
                  <span>{{ latestRobot?.stream_id || '机器人未上报视频流' }}</span>
                </div>
              </template>
              <template #overlay>
                <div class="guard-video-label">
                  <strong>{{ latestRobot?.name || latestRobot?.code || '机器狗' }}</strong>
                  <span>{{ latestRobot?.location || '位置未知' }}</span>
                </div>
              </template>
            </LiveVideoPlayer>
          </div>

          <div class="guard-task-bar">
            <label class="guard-task-selector">
              <span>当前任务</span>
              <select
                v-model="selectedTaskId"
                :disabled="busy || loopActive || isRunning || !taskOptions.length"
                @change="changeSelectedTask"
              >
                <option v-if="!taskOptions.length" value="">管理员尚未配置巡检任务</option>
                <option v-for="task in taskOptions" :key="task.id" :value="String(task.id)">
                  {{ taskOptionLabel(task) }}
                </option>
              </select>
            </label>
            <div class="guard-task-state">
              <span>执行状态</span>
              <strong>{{ taskStateText }}</strong>
              <small :title="formatExecutionTime(execution?.created_at || presetTask?.latest_execution?.created_at)">
                {{ formatExecutionTime(execution?.created_at || presetTask?.latest_execution?.created_at) }}
              </small>
              <small class="guard-task-recording" :class="{ enabled: presetTask?.effective_record_rosbag }">
                导航调试包：{{ presetTask?.effective_record_rosbag ? (loopActive ? '循环单包录制中' : '已开启') : '未开启' }}
              </small>
              <small v-if="executionReason" class="guard-task-pause-reason" :title="executionReason.text">
                {{ executionReason.label }}：{{ executionReason.text }}
              </small>
            </div>
            <div class="guard-task-actions">
              <button class="guard-primary" :disabled="busy || localizationBusy || loopActive || !presetTask || isRunning" @click="startTask">
                {{ busy ? '处理中...' : '开始巡检' }}
              </button>
              <button
                class="guard-secondary"
                :disabled="busy || localizationBusy || !taskControl.enabled"
                :title="taskControl.hint"
                @click="controlTask"
              >{{ taskControl.label }}</button>
              <button class="guard-danger" :disabled="busy || localizationBusy || !execution?.id || !actions.forceExit" @click="forceExitTask">强制退出</button>
            </div>
          </div>

          <div class="guard-localization-bar">
            <div class="guard-localization-copy">
              <span>初始化状态</span>
              <strong :class="`is-${localizationInitState}`">{{ localizationInitLabel }}</strong>
              <small>{{ localizationInitMessage }}</small>
            </div>
            <button
              class="guard-initialize"
              :disabled="busy || localizationBusy || loopActive || isRunning || !latestRobot || latestRobot.status !== 'online'"
              @click="initializeLocalization"
            >
              {{ localizationBusy ? '初始化中...' : '初始化定位' }}
            </button>
          </div>

          <section class="guard-loop-panel guard-loop-inline">
            <div class="guard-loop-heading">
              <div>
                <span class="guard-eyebrow">任务循环</span>
                <h2>实时信息</h2>
              </div>
              <span class="guard-loop-light" :class="{ active: loopActive }"></span>
            </div>
            <div class="guard-loop-controls">
              <div class="guard-loop-settings">
                <label>
                  <span>循环时长（分钟）</span>
                  <input v-model.number="loopDurationMinutes" type="number" min="1" step="1" :disabled="loopActive" @change="persistLoopState" />
                </label>
                <label>
                  <span>每轮休息（秒）</span>
                  <input v-model.number="loopRestSeconds" type="number" min="0" step="1" :disabled="loopActive" @change="persistLoopState" />
                </label>
              </div>
              <div class="guard-countdown-clock">
                <span>倒计时</span>
                <strong>{{ loopActive ? formatDuration(loopRemainingMilliseconds) : '00:00:00' }}</strong>
              </div>
              <button
                class="guard-loop-toggle"
                :class="{ 'is-active': loopActive }"
                :disabled="busy || localizationBusy || (!loopActive && (!presetTask || isRunning || !navigationReady()))"
                @click="toggleLoop"
              >
                {{ loopActive ? '停止循环' : '循环执行' }}
              </button>
            </div>
            <div class="guard-runtime-grid">
              <div><span>累计计时</span><strong>{{ formatDuration(elapsedMilliseconds) }}</strong></div>
              <div><span>行走距离</span><strong>{{ displayedTotalDistance.toFixed(1) }} m</strong></div>
              <div>
                <span>当前速度</span>
                <strong :class="{ 'is-data-stale': !currentMovementSpeed.available }" :title="currentMovementSpeedTitle">
                  {{ currentMovementSpeedLabel }}
                </strong>
              </div>
              <div><span>执行轮次</span><strong>{{ loopRounds }} 轮</strong></div>
              <div class="guard-runtime-status"><span>当前状态</span><strong>{{ guardRuntimeStatus }}</strong></div>
            </div>
            <p class="guard-loop-message">{{ loopMessage }}</p>
            <p v-if="executionReason" class="guard-pause-reason">
              {{ executionReason.label }}：{{ executionReason.text }}
            </p>
          </section>
        </section>

        <aside class="guard-side">
          <section class="guard-alert-panel" :class="{ 'has-alert': latestAlert }">
            <div class="guard-panel-title">
              <div>
                <span class="guard-eyebrow">实时事件</span>
                <h2>报警信息</h2>
              </div>
              <div class="guard-alert-actions">
                <button :class="{ active: liveSpeechOpen }" @click="toggleLiveSpeechPanel">实时喊话</button>
                <button
                  class="guard-pending-events"
                  :title="`有 ${pendingEventCount} 条待处理事件，点击进入事件中心`"
                  :aria-label="`有 ${pendingEventCount} 条待处理事件，点击进入事件中心`"
                  @click="openPendingEvents"
                >
                  {{ pendingEventCount }}条
                </button>
              </div>
            </div>
            <div v-if="liveSpeechOpen" class="guard-live-speech">
              <label>
                <span>文字实时播报</span>
                <textarea v-model="liveSpeechText" maxlength="500" placeholder="输入文字后立即在机器狗音响播放"></textarea>
              </label>
              <button class="guard-live-text-send" :disabled="!liveSpeechText.trim() || liveSpeechSending || liveRecording" @click="sendLiveTextSpeech">
                {{ liveSpeechSending && !liveRecording ? '下发中...' : '播放文字' }}
              </button>
              <div class="guard-live-microphone">
                <div>
                  <span>网页麦克风</span>
                  <strong>{{ liveRecording ? `录音中 ${liveRecordingDurationLabel()}` : liveSpeechSending ? '正在发送录音' : '等待接通' }}</strong>
                </div>
                <button v-if="!liveRecording" :disabled="liveSpeechSending" @click="startLiveRecording">开始录音</button>
                <button v-else class="is-recording" @click="stopLiveRecordingAndPlay">结束并播放</button>
              </div>
              <small>录音时会暂时关闭现场收音，避免声音回授；发送完成后自动恢复。</small>
            </div>
            <article v-if="latestAlert" class="guard-alert-main">
              <div v-if="eventImage(latestAlert)" class="guard-alert-image" :style="{ backgroundImage: `url(${eventImage(latestAlert)})` }"></div>
              <div>
                <strong>{{ latestAlert.title || latestAlert.event_type || '现场异常' }}</strong>
                <span>{{ latestAlert.location || latestRobot?.location || '位置未知' }}</span>
                <small>{{ formatTime(latestAlert.detected_at) }}</small>
              </div>
            </article>
            <p v-else class="guard-empty">当前没有新的报警</p>
            <div class="guard-alert-list">
              <div v-for="event in alerts.slice(1, 4)" :key="event.id">
                <span>{{ event.title || event.event_type || '现场异常' }}</span>
                <small>{{ formatTime(event.detected_at) }}</small>
              </div>
            </div>
          </section>

          <section class="guard-map-panel">
            <div class="guard-map-head">
              <div>
                <span class="guard-eyebrow">当前地图</span>
                <h2>{{ mapData?.name || execution?.map_name || '地图预览' }}</h2>
              </div>
              <small>{{ trajectory.length }} 个轨迹点 · {{ robotPoint() ? `x ${Number(robotPoint().x).toFixed(2)} / y ${Number(robotPoint().y).toFixed(2)}` : '暂无定位' }}</small>
            </div>
            <div class="guard-map-stage">
              <div v-if="!mapData?.thumbnail_url" class="guard-map-empty">当前地图暂无缩略图</div>
              <div v-else class="guard-map-layer">
                <img ref="mapImageRef" :src="fullUrl(mapData.thumbnail_url)" :alt="mapData.name" @load="refreshImageGeometry" />
                <svg class="guard-map-lines">
                  <polyline v-if="displayRouteWaypoints.length > 1" :points="polylinePoints(displayRouteWaypoints)" fill="none" stroke="#2563eb" stroke-width="3" />
                  <polyline v-if="trajectory.length > 1" :points="polylinePoints(trajectory)" fill="none" stroke="#10b981" stroke-width="2" stroke-dasharray="6 4" />
                </svg>
                <div class="guard-map-markers">
                  <span
                    v-for="(point, index) in displayRouteWaypoints"
                    :key="point.waypoint_id || index"
                    class="guard-map-waypoint"
                    :class="waypointClass(index)"
                    :style="displayPosition(point)"
                  >{{ point.map_point_number ?? point.sequence + 1 }}</span>
                  <div
                    v-for="point in localizationLossMarkers"
                    :key="`guard-loss-${point.eventId}`"
                    class="guard-localization-loss"
                    :style="displayPosition(point)"
                    :title="lossMarkerTitle(point)"
                  >
                    <i :style="lossHeadingStyle(point)"></i><small>{{ point.sequence }}</small>
                  </div>
                  <div v-if="robotPoint() && displayPosition(robotPoint())" class="guard-map-robot" :class="{ untrusted: !robotPoint()?.trusted }" :style="displayPosition(robotPoint())" :title="robotMarkerTitle()">
                    <RobotDogIcon :size="26" />
                    <i :style="robotHeadingStyle()"></i>
                    <small>{{ robotPoint()?.trusted ? '机器狗' : '定位不可信' }}</small>
                  </div>
                </div>
              </div>
            </div>
            <div class="guard-map-legend">
              <span class="route">规划路线</span>
              <span class="track">实际轨迹</span>
              <span class="robot"><RobotDogIcon :size="16" />机器狗</span>
              <span class="loss">定位丢失点 {{ localizationLossMarkers.length }}</span>
            </div>
            <section class="guard-route-log">
              <div class="guard-route-summary" :class="routeStatusClass">
                <div><span>本次轮次</span><strong>第 {{ currentExecutionRound || 1 }} 轮</strong></div>
                <div><span>计划路线</span><strong>{{ execution?.route_name || routeData?.name || '--' }}</strong></div>
              </div>
              <div class="guard-route-order" aria-label="本轮计划航点顺序">
                <span
                  v-for="(item, index) in executionWaypointPlan"
                  :key="`${item.mapPointNumber}-${index}`"
                  :class="waypointClass(item.waypointIndex)"
                >{{ item.mapPointNumber }}</span>
                <small v-if="!executionWaypointPlan.length">暂无航点</small>
              </div>
              <div v-if="activeTargetMilestone" class="guard-current-target">
                <span>当前下发目标</span>
                <strong>{{ activeTargetMilestone.payload?.waypoint?.map_point_number }}号点</strong>
                <small>{{ formatTime(activeTargetMilestone.occurred_at) }} · {{ formatPose(activeTargetMilestone.payload?.waypoint) }}</small>
              </div>
              <div class="guard-waypoint-log">
                <article v-for="event in waypointMilestones" :key="event.id">
                  <i :class="event.event_type === 'task.waypoint_reached' ? 'is-reached' : 'is-target'"></i>
                  <div>
                    <strong>{{ event.payload?.waypoint?.map_point_number }}号点 · {{ event.event_type === 'task.waypoint_reached' ? '已到达' : '目标已下发' }}</strong>
                    <span>{{ formatTime(event.occurred_at) }}</span>
                    <small>目标 {{ formatPose(event.payload?.waypoint) }}</small>
                    <small>机器人 {{ formatPose(event.payload?.robot_pose) }}</small>
                  </div>
                </article>
                <p v-if="!waypointMilestones.length">本次执行暂未产生航点记录</p>
              </div>
            </section>
          </section>
        </aside>
      </main>
    </template>
    <AppToast :show="visible" :message="toastMessage" :variant="toastVariant" />
  </section>
</template>

<style scoped>
.guard-page { min-height: calc(100vh - 120px); padding: 24px; background: #eef2f5; color: #142330; }
.guard-header { display: flex; align-items: center; justify-content: space-between; gap: 24px; margin: 0 auto 18px; width: 100%; max-width: none; }
.guard-header h1 { margin: 4px 0 0; font-size: clamp(24px, 3vw, 40px); letter-spacing: 0; }
.guard-eyebrow { color: #667785; font-size: 12px; font-weight: 800; letter-spacing: .08em; text-transform: uppercase; }
.guard-status { display: flex; align-items: center; gap: 10px; padding: 12px 16px; border: 1px solid #d6dee3; background: #fff; font-weight: 800; }
.guard-status-dot { width: 10px; height: 10px; border-radius: 50%; background: #9aa8b2; }
.guard-status.is-online .guard-status-dot { background: #1b9b65; }
.guard-grid { display: grid; grid-template-columns: minmax(0, 1fr) minmax(240px, 272px); gap: 18px; width: 100%; max-width: none; margin: 0 auto; }
.guard-video-panel, .guard-alert-panel { border: 1px solid #d6dee3; background: #fff; }
.guard-video-stage { position: relative; min-height: 520px; background: #152633; overflow: hidden; }
.guard-video, .guard-video-empty { display: block; width: 100%; height: 100%; min-height: 520px; object-fit: contain; }
.guard-video-empty { display: grid; place-content: center; gap: 8px; color: #d7e0e6; text-align: center; }
.guard-video-empty > .robot-dog-icon { justify-self: center; margin-bottom: 6px; }
.guard-video-empty span { color: #9fb0ba; font-size: 13px; }
.guard-video-label { position: absolute; left: 18px; top: 18px; display: grid; gap: 4px; padding: 10px 12px; color: #fff; background: rgba(12, 25, 34, .78); }
.guard-video-label span { color: #c5d1d8; font-size: 13px; }
.guard-listen-toggle { position: absolute; top: 18px; right: 18px; min-height: 42px; padding: 0 16px; border: 1px solid rgba(255, 255, 255, .4); color: #fff; background: rgba(10, 29, 41, .82); font: inherit; font-weight: 800; cursor: pointer; }
.guard-listen-toggle.active { border-color: #52d99c; background: rgba(16, 110, 73, .9); }
.guard-playback-controls { position: absolute; top: 18px; left: 50%; z-index: 2; display: flex; align-items: center; justify-content: center; gap: 7px; padding: 9px; background: rgba(10, 29, 41, .82); transform: translateX(-50%); }
.guard-playback-controls button { min-height: 32px; padding: 0 10px; border: 1px solid rgba(255, 255, 255, .36); color: #fff; background: rgba(27, 62, 81, .9); font: inherit; font-size: 12px; font-weight: 800; cursor: pointer; }
.guard-playback-controls button:hover { background: rgba(42, 99, 128, .96); }
.guard-playback-controls button.active { border-color: #52d99c; background: rgba(16, 110, 73, .92); }
.guard-playback-controls button:disabled { cursor: not-allowed; opacity: .48; }
.guard-playback-controls small { position: absolute; top: calc(100% + 5px); left: 50%; width: max-content; max-width: 260px; padding: 4px 7px; color: #dbe8ee; background: rgba(10, 29, 41, .72); font-size: 11px; transform: translateX(-50%); }
.guard-task-bar { display: grid; grid-template-columns: minmax(220px, 1.5fr) minmax(140px, .8fr) minmax(240px, .65fr); align-items: center; gap: 12px; padding: 16px; }
.guard-task-bar > div { display: grid; gap: 5px; min-width: 0; }
.guard-task-bar span { color: #70808c; font-size: 12px; }
.guard-task-bar strong { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.guard-task-state small { overflow: hidden; color: #70808c; font-size: 11px; text-overflow: ellipsis; white-space: nowrap; }
.guard-task-state .guard-task-recording.enabled { color: #14734c; font-weight: 800; }
.guard-task-state .guard-task-pause-reason { color: #c94b32; font-weight: 700; }
.guard-task-actions { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; min-width: 0; }
.guard-task-actions > button { width: 100%; min-width: 0; padding-inline: 10px; font-size: 11px; white-space: nowrap; }
.guard-task-selector { display: grid; gap: 5px; min-width: 0; }
.guard-task-selector select { width: 100%; min-width: 0; height: 38px; padding: 0 32px 0 10px; border: 1px solid #c8d3d9; color: #1c303c; background: #fff; font: inherit; font-weight: 800; text-overflow: ellipsis; }
.guard-task-selector select:disabled { cursor: not-allowed; opacity: .65; }
.guard-localization-bar { display: flex; align-items: center; justify-content: space-between; gap: 20px; padding: 14px 16px; border-top: 1px solid #e2e8ec; background: #f7f9fa; }
.guard-localization-copy { display: grid; grid-template-columns: auto auto; align-items: center; gap: 4px 12px; min-width: 0; }
.guard-localization-copy > span { color: #70808c; font-size: 12px; }
.guard-localization-copy > strong { font-size: 15px; }
.guard-localization-copy > strong.is-success { color: #14734c; }
.guard-localization-copy > strong.is-failed { color: #b8322c; }
.guard-localization-copy > strong.is-initializing,
.guard-localization-copy > strong.is-not_ready { color: #b0640e; }
.guard-localization-copy > small { grid-column: 1 / -1; overflow: hidden; color: #71818c; text-overflow: ellipsis; white-space: nowrap; }
.guard-primary, .guard-secondary, .guard-danger, .guard-initialize { min-height: 52px; padding: 0 22px; border: 0; font: inherit; font-weight: 800; cursor: pointer; }
.guard-primary { color: #fff; background: #19724d; }
.guard-secondary { color: #263944; background: #dfe7eb; }
.guard-danger { color: #fff; background: #b8322c; }
.guard-initialize { min-width: 150px; color: #fff; background: #146fb3; }
.guard-primary:disabled, .guard-secondary:disabled, .guard-danger:disabled, .guard-initialize:disabled { cursor: not-allowed; opacity: .45; }
.guard-side { display: grid; min-width: 0; align-content: start; gap: 18px; }
.guard-side > * { min-width: 0; max-width: 100%; }
.guard-alert-panel, .guard-loop-panel, .guard-map-panel { border: 1px solid #d6dee3; background: #fff; }
.guard-alert-panel { min-height: 360px; padding: 18px; }
.guard-panel-title { display: flex; flex-wrap: wrap; justify-content: space-between; align-items: start; gap: 10px; padding-bottom: 14px; border-bottom: 1px solid #e4eaed; }
.guard-panel-title h2 { margin: 5px 0 0; font-size: 24px; letter-spacing: 0; }
.guard-alert-actions { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; }
.guard-alert-actions > button { min-height: 38px; padding: 0 12px; border: 1px solid #19724d; color: #176343; background: #edf8f3; font: inherit; font-size: 12px; font-weight: 900; cursor: pointer; }
.guard-alert-actions > button.active { color: #fff; background: #19724d; }
.guard-alert-actions > .guard-pending-events { display: grid; place-items: center; min-width: 44px; height: 38px; box-sizing: border-box; padding: 0 7px; border: 0; color: #fff; background: #ba302b; }
.guard-alert-actions > .guard-pending-events:hover { background: #98241f; }
.guard-live-speech { display: grid; gap: 10px; margin-top: 14px; padding: 12px; border: 1px solid #cbd8de; background: #f4f8fa; }
.guard-live-speech label { display: grid; gap: 6px; color: #687a86; font-size: 12px; font-weight: 800; }
.guard-live-speech textarea { width: 100%; min-height: 74px; box-sizing: border-box; resize: vertical; padding: 9px 10px; border: 1px solid #c7d2d8; color: #172b37; background: #fff; font: inherit; line-height: 1.45; }
.guard-live-text-send, .guard-live-microphone button { min-height: 40px; border: 0; color: #fff; background: #19724d; font: inherit; font-weight: 900; cursor: pointer; }
.guard-live-text-send:disabled, .guard-live-microphone button:disabled { cursor: not-allowed; opacity: .45; }
.guard-live-microphone { display: grid; grid-template-columns: 1fr 112px; align-items: center; gap: 10px; padding-top: 10px; border-top: 1px solid #d6e0e5; }
.guard-live-microphone > div { display: grid; gap: 3px; }
.guard-live-microphone span { color: #687a86; font-size: 11px; }
.guard-live-microphone strong { font-size: 13px; }
.guard-live-microphone button.is-recording { background: #b8322c; }
.guard-live-speech > small { color: #71818c; font-size: 11px; line-height: 1.45; }
.guard-alert-main { display: grid; grid-template-columns: 88px 1fr; gap: 12px; align-items: center; padding: 16px 0; }
.guard-alert-image { width: 88px; height: 68px; background-position: center; background-size: cover; }
.guard-alert-main div:last-child { display: grid; gap: 5px; }
.guard-alert-main span, .guard-alert-main small { color: #6d7e89; font-size: 13px; }
.guard-alert-list { display: grid; gap: 10px; border-top: 1px solid #e4eaed; padding-top: 12px; }
.guard-alert-list div { display: flex; justify-content: space-between; gap: 10px; font-size: 13px; }
.guard-alert-list small { color: #788893; white-space: nowrap; }
.guard-empty { color: #71828d; }
.guard-loop-panel { display: grid; gap: 14px; padding: 18px; }
.guard-loop-inline { grid-template-columns: 150px minmax(0, 1fr); align-items: end; border-right: 0; border-bottom: 0; border-left: 0; }
.guard-loop-heading { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
.guard-loop-heading h2 { margin: 5px 0 0; font-size: 22px; }
.guard-loop-controls { display: grid; grid-template-columns: minmax(250px, 360px) minmax(108px, 124px) minmax(96px, 110px); align-items: end; justify-content: end; gap: 10px; }
.guard-loop-inline .guard-loop-settings { max-width: 360px; }
.guard-loop-inline .guard-loop-toggle { height: 42px; min-height: 42px; }
.guard-countdown-clock { display: grid; gap: 4px; }
.guard-countdown-clock span { color: #687a86; font-size: 12px; font-weight: 700; }
.guard-countdown-clock strong { display: flex; align-items: center; justify-content: center; height: 42px; box-sizing: border-box; border: 1px solid #bfccd3; color: #102c3d; background: #eaf0f3; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size: 21px; letter-spacing: .05em; font-variant-numeric: tabular-nums; }
.guard-loop-inline .guard-runtime-grid { grid-column: 1 / -1; grid-template-columns: repeat(4, minmax(0, 1fr)) minmax(240px, 2fr); }
.guard-loop-inline .guard-runtime-grid > div { border-right: 1px solid #e5eaed; border-bottom: 0; }
.guard-loop-inline .guard-runtime-grid > div:last-child { border-right: 0; }
.guard-loop-inline .guard-runtime-status strong { white-space: normal; }
.guard-loop-inline .guard-loop-message,
.guard-loop-inline .guard-pause-reason { grid-column: 1 / -1; }
.guard-loop-light { width: 14px; height: 14px; border-radius: 50%; background: #a9b5bd; box-shadow: 0 0 0 5px rgba(169, 181, 189, .18); }
.guard-loop-light.active { background: #19a568; box-shadow: 0 0 0 5px rgba(25, 165, 104, .18); }
.guard-loop-settings { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.guard-loop-settings label { display: grid; gap: 6px; color: #687a86; font-size: 12px; font-weight: 700; }
.guard-loop-settings input { width: 100%; min-width: 0; height: 42px; box-sizing: border-box; padding: 0 10px; border: 1px solid #cad4da; background: #fff; color: #172b37; font: inherit; font-size: 15px; font-weight: 800; }
.guard-loop-settings input:disabled { background: #edf1f3; color: #75858f; }
.guard-loop-toggle { width: 100%; min-width: 0; min-height: 42px; padding: 0 10px; border: 0; color: #fff; background: #136fac; font: inherit; font-size: 14px; font-weight: 900; cursor: pointer; }
.guard-loop-toggle.is-active { background: #a9402d; }
.guard-loop-toggle:disabled { cursor: not-allowed; opacity: .45; }
.guard-runtime-grid { display: grid; grid-template-columns: 1fr 1fr; border: 1px solid #dce4e8; }
.guard-runtime-grid > div { display: grid; gap: 4px; min-width: 0; padding: 10px; border-bottom: 1px solid #e5eaed; }
.guard-runtime-grid > div:nth-child(odd) { border-right: 1px solid #e5eaed; }
.guard-runtime-grid > div:nth-last-child(-n + 2) { border-bottom: 0; }
.guard-runtime-grid span { color: #72838e; font-size: 11px; }
.guard-runtime-grid strong { overflow: hidden; font-size: 14px; text-overflow: ellipsis; white-space: nowrap; }
.guard-runtime-grid strong.is-data-stale { color: #c57a12; }
.guard-loop-message { margin: 0; color: #657681; font-size: 12px; line-height: 1.5; }
.guard-pause-reason { margin: 0; color: #c94b32; font-size: 12px; font-weight: 700; line-height: 1.5; }
.guard-map-panel { --guard-waypoint-idle: #2563eb; --guard-waypoint-target: #e79a18; --guard-waypoint-reached: #159a63; padding: 16px; }
.guard-map-head { display: flex; align-items: end; justify-content: space-between; gap: 12px; margin-bottom: 12px; }
.guard-map-head h2 { margin: 4px 0 0; font-size: 20px; }
.guard-map-head small { color: #71818c; white-space: nowrap; }
.guard-map-stage { position: relative; min-height: 230px; overflow: hidden; border: 1px solid #dce4e8; background: #e9eef1; }
.guard-map-empty { display: grid; place-items: center; min-height: 230px; color: #71818c; font-size: 13px; }
.guard-map-layer { position: relative; width: 100%; background: #fff; }
.guard-map-layer img { display: block; width: 100%; height: auto; user-select: none; }
.guard-map-lines, .guard-map-markers { position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; }
.guard-map-waypoint { position: absolute; display: grid; place-items: center; width: 20px; height: 20px; border: 2px solid #fff; border-radius: 50%; color: #fff; background: var(--guard-waypoint-idle); box-shadow: 0 2px 6px rgba(15, 35, 48, .38); font-size: 10px; font-weight: 900; transform: translate(-50%, -50%); }
.guard-map-waypoint.is-target { background: var(--guard-waypoint-target); }
.guard-map-waypoint.is-reached { background: var(--guard-waypoint-reached); }
.guard-map-robot { position: absolute; z-index: 5; width: 28px; height: 28px; transform: translate(-50%, -50%); }
.guard-map-robot::before { content: ''; position: absolute; inset: 3px; border: 3px solid #fff; border-radius: 50%; background: #ec4a3f; box-shadow: 0 2px 8px rgba(236, 74, 63, .5); }
.guard-map-robot > .robot-dog-icon { position: absolute; inset: 1px; z-index: 6; display: block; }
.guard-map-robot i { position: absolute; left: 50%; top: 50%; z-index: 6; width: 0; height: 0; border-right: 5px solid transparent; border-bottom: 14px solid #8f2019; border-left: 5px solid transparent; transform-origin: 50% 70%; }
.guard-map-robot > small { position: absolute; top: 28px; left: 50%; min-width: max-content; padding: 2px 5px; color: #fff; background: #8f2019; font-size: 9px; transform: translateX(-50%); }
.guard-map-robot.untrusted::before { border-style: dashed; background: #f59e0b; box-shadow: 0 0 0 4px rgba(220, 38, 38, .28); }
.guard-map-robot.untrusted i { border-bottom-color: #b45309; }
.guard-map-robot.untrusted > small { background: #b45309; }
.guard-localization-loss { position: absolute; z-index: 7; width: 28px; height: 28px; transform: translate(-50%, -50%); }
.guard-localization-loss::before { content: ''; position: absolute; inset: 6px; border: 3px solid #fff; border-radius: 50%; background: #dc2626; box-shadow: 0 0 0 3px rgba(220, 38, 38, .28); }
.guard-localization-loss i { position: absolute; left: 50%; top: 50%; z-index: 2; width: 0; height: 0; border-right: 5px solid transparent; border-bottom: 15px solid #7f1d1d; border-left: 5px solid transparent; transform-origin: 50% 100%; }
.guard-localization-loss small { position: absolute; top: 26px; left: 50%; min-width: 16px; padding: 1px 3px; color: #fff; background: #991b1b; font-size: 9px; text-align: center; transform: translateX(-50%); }
.guard-map-legend { display: flex; flex-wrap: wrap; gap: 14px; padding-top: 10px; color: #657681; font-size: 11px; }
.guard-map-legend .robot { display: inline-flex; align-items: center; gap: 4px; }
.guard-map-legend .robot .robot-dog-icon {
  box-sizing: content-box;
  flex: 0 0 auto;
  padding: 2px;
  border-radius: 50%;
  background: #b42318;
}
.guard-map-legend span::before { content: ''; display: inline-block; width: 14px; height: 3px; margin-right: 5px; vertical-align: middle; background: #2563eb; }
.guard-map-legend .track::before { background: #10b981; }
.guard-map-legend .robot::before { width: 0; height: 0; margin-right: 0; background: transparent; }
.guard-map-legend .loss::before { width: 8px; height: 8px; border-radius: 50%; background: #dc2626; }
.guard-route-log { display: grid; gap: 10px; margin-top: 12px; padding-top: 12px; border-top: 1px solid #dde5e9; }
.guard-route-summary { display: grid; grid-template-columns: 92px minmax(0, 1fr); gap: 8px; }
.guard-route-summary > div { display: grid; gap: 2px; min-width: 0; padding: 8px 10px; color: #fff; background: var(--guard-waypoint-idle); transition: background-color .2s ease; }
.guard-route-summary.is-reached > div { background: var(--guard-waypoint-reached); }
.guard-route-summary span, .guard-current-target > span { color: #71818c; font-size: 10px; }
.guard-route-summary > div > span { color: rgba(255, 255, 255, .82); }
.guard-route-summary strong { overflow: hidden; color: #fff; font-size: 12px; text-overflow: ellipsis; white-space: nowrap; }
.guard-route-order { display: flex; flex-wrap: wrap; align-items: center; gap: 5px; }
.guard-route-order span { display: grid; place-items: center; width: 24px; height: 24px; color: #fff; background: var(--guard-waypoint-idle); font-size: 11px; font-weight: 900; }
.guard-route-order span.is-target { background: var(--guard-waypoint-target); }
.guard-route-order span.is-reached { background: var(--guard-waypoint-reached); }
.guard-route-order span:not(:last-child)::after { content: ''; }
.guard-route-order small { color: #71818c; font-size: 11px; }
.guard-current-target { display: grid; grid-template-columns: 1fr auto; gap: 3px 8px; padding: 9px 10px; border-left: 3px solid #e79a18; background: #fff7e7; }
.guard-current-target strong { color: #9a5b00; font-size: 12px; }
.guard-current-target small { grid-column: 1 / -1; color: #6f7d86; font-size: 10px; }
.guard-waypoint-log { display: grid; max-height: 260px; overflow-y: auto; border-top: 1px solid #e1e7ea; }
.guard-waypoint-log article { display: grid; grid-template-columns: 10px minmax(0, 1fr); gap: 8px; padding: 9px 2px; border-bottom: 1px solid #edf1f3; }
.guard-waypoint-log article > i { width: 8px; height: 8px; margin-top: 4px; border-radius: 50%; background: var(--guard-waypoint-target); }
.guard-waypoint-log article > i.is-reached { background: var(--guard-waypoint-reached); }
.guard-waypoint-log article > div { display: grid; gap: 2px; min-width: 0; }
.guard-waypoint-log strong { font-size: 11px; }
.guard-waypoint-log span, .guard-waypoint-log small, .guard-waypoint-log p { color: #71818c; font-size: 10px; line-height: 1.4; }
.guard-waypoint-log p { margin: 8px 0 0; }
.guard-loading { display: grid; place-items: center; min-height: 50vh; color: #657681; }

/* Guard duty uses a legacy light palette, so explicitly map its surfaces to the
   application theme when the global dark theme is active. */
:global([data-theme="dark"] .guard-page) { background: transparent; color: var(--text); }
:global([data-theme="dark"] .guard-page .guard-status),
:global([data-theme="dark"] .guard-page .guard-video-panel),
:global([data-theme="dark"] .guard-page .guard-alert-panel),
:global([data-theme="dark"] .guard-page .guard-loop-panel),
:global([data-theme="dark"] .guard-page .guard-map-panel) {
  border-color: var(--line);
  background: var(--panel);
  color: var(--text);
}
:global([data-theme="dark"] .guard-page .guard-localization-bar),
:global([data-theme="dark"] .guard-page .guard-live-speech),
:global([data-theme="dark"] .guard-page .guard-map-stage),
:global([data-theme="dark"] .guard-page .guard-countdown-clock strong),
:global([data-theme="dark"] .guard-page .guard-map-layer) {
  border-color: var(--line);
  background: var(--panel-soft);
  color: var(--text);
}
:global([data-theme="dark"] .guard-page .guard-live-speech textarea),
:global([data-theme="dark"] .guard-page .guard-loop-settings input),
:global([data-theme="dark"] .guard-page .guard-task-selector select) {
  border-color: var(--line);
  background: var(--input-bg);
  color: var(--text);
}
:global([data-theme="dark"] .guard-page .guard-loop-settings input:disabled) {
  background: rgba(255, 255, 255, .02);
  color: var(--muted);
}
:global([data-theme="dark"] .guard-page .guard-secondary) {
  color: var(--text);
  background: var(--ghost-bg);
}
:global([data-theme="dark"] .guard-page .guard-alert-actions > button) {
  border-color: var(--green);
  color: var(--green);
  background: rgba(31, 191, 120, .1);
}
:global([data-theme="dark"] .guard-page .guard-alert-actions > button.active) {
  color: #071d16;
  background: var(--green);
}
:global([data-theme="dark"] .guard-page .guard-panel-title),
:global([data-theme="dark"] .guard-page .guard-alert-list),
:global([data-theme="dark"] .guard-page .guard-live-microphone),
:global([data-theme="dark"] .guard-page .guard-route-log),
:global([data-theme="dark"] .guard-page .guard-waypoint-log),
:global([data-theme="dark"] .guard-page .guard-runtime-grid),
:global([data-theme="dark"] .guard-page .guard-runtime-grid > div),
:global([data-theme="dark"] .guard-page .guard-loop-inline .guard-runtime-grid > div),
:global([data-theme="dark"] .guard-page .guard-waypoint-log article) {
  border-color: var(--line);
}
:global([data-theme="dark"] .guard-page .guard-runtime-grid) { background: rgba(255, 255, 255, .02); }
:global([data-theme="dark"] .guard-page .guard-runtime-grid strong.is-data-stale) { color: #ffd58a; }
:global([data-theme="dark"] .guard-page .guard-current-target) {
  border-left-color: var(--warning);
  background: rgba(255, 196, 92, .12);
}
:global([data-theme="dark"] .guard-page .guard-current-target strong) { color: #ffd58a; }
:global([data-theme="dark"] .guard-page .guard-countdown-clock strong) { color: var(--text); }
:global([data-theme="dark"] .guard-page .guard-eyebrow),
:global([data-theme="dark"] .guard-page .guard-task-bar span),
:global([data-theme="dark"] .guard-page .guard-localization-copy > span),
:global([data-theme="dark"] .guard-page .guard-localization-copy > small),
:global([data-theme="dark"] .guard-page .guard-live-speech label),
:global([data-theme="dark"] .guard-page .guard-live-microphone span),
:global([data-theme="dark"] .guard-page .guard-live-speech > small),
:global([data-theme="dark"] .guard-page .guard-alert-main span),
:global([data-theme="dark"] .guard-page .guard-alert-main small),
:global([data-theme="dark"] .guard-page .guard-alert-list small),
:global([data-theme="dark"] .guard-page .guard-empty),
:global([data-theme="dark"] .guard-page .guard-countdown-clock span),
:global([data-theme="dark"] .guard-page .guard-loop-settings label),
:global([data-theme="dark"] .guard-page .guard-runtime-grid span),
:global([data-theme="dark"] .guard-page .guard-loop-message),
:global([data-theme="dark"] .guard-page .guard-map-head small),
:global([data-theme="dark"] .guard-page .guard-map-empty),
:global([data-theme="dark"] .guard-page .guard-map-legend),
:global([data-theme="dark"] .guard-page .guard-route-summary span),
:global([data-theme="dark"] .guard-page .guard-route-order small),
:global([data-theme="dark"] .guard-page .guard-current-target > span),
:global([data-theme="dark"] .guard-page .guard-current-target small),
:global([data-theme="dark"] .guard-page .guard-waypoint-log span),
:global([data-theme="dark"] .guard-page .guard-waypoint-log small),
:global([data-theme="dark"] .guard-page .guard-waypoint-log p),
:global([data-theme="dark"] .guard-page .guard-loading) {
  color: var(--muted);
}
:global([data-theme="dark"] .guard-page .guard-route-summary > div > span) {
  color: rgba(255, 255, 255, .82);
}
@media (min-width: 981px) {
  .guard-video-label {
    gap: 2px;
    padding: 8px 10px;
    font-size: 12px;
  }
  .guard-video-label span {
    font-size: 10px;
  }
  .guard-playback-controls {
    gap: 5px;
    padding: 7px;
  }
  .guard-playback-controls button {
    padding-inline: 8px;
    font-size: 10px;
    white-space: nowrap;
  }
  .guard-task-bar {
    grid-template-columns: minmax(180px, 1.2fr) minmax(100px, .5fr) minmax(240px, .65fr);
    gap: 10px;
    padding: 12px;
  }
  .guard-task-bar > div,
  .guard-task-selector {
    gap: 3px;
  }
  .guard-task-bar span {
    font-size: 10px;
    white-space: nowrap;
  }
  .guard-task-bar strong {
    font-size: 12px;
  }
  .guard-task-actions {
    grid-template-columns: repeat(3, minmax(72px, 1fr));
    gap: 8px;
  }
  .guard-task-selector select {
    height: 34px;
    padding-left: 8px;
    font-size: 11px;
    white-space: nowrap;
  }
  .guard-primary,
  .guard-secondary,
  .guard-danger,
  .guard-initialize {
    min-height: 40px;
    padding-inline: 14px;
    font-size: 11px;
    white-space: nowrap;
  }
  .guard-localization-bar {
    gap: 12px;
    padding: 10px 12px;
  }
  .guard-localization-copy {
    gap: 3px 8px;
  }
  .guard-localization-copy > span,
  .guard-localization-copy > small {
    font-size: 10px;
  }
  .guard-localization-copy > strong {
    font-size: 12px;
    white-space: nowrap;
  }
  .guard-initialize {
    min-width: 120px;
  }
  .guard-loop-panel {
    gap: 10px;
    padding: 12px;
  }
  .guard-loop-inline {
    grid-template-columns: 110px minmax(0, 1fr);
  }
  .guard-loop-heading {
    gap: 8px;
  }
  .guard-loop-heading .guard-eyebrow {
    font-size: 10px;
    letter-spacing: .05em;
    white-space: nowrap;
  }
  .guard-loop-heading h2 {
    margin-top: 3px;
    font-size: 17px;
    white-space: nowrap;
  }
  .guard-loop-light {
    width: 11px;
    height: 11px;
  }
  .guard-loop-controls { grid-template-columns: minmax(220px, 300px) minmax(96px, 110px) minmax(92px, 104px); gap: 8px; }
  .guard-loop-settings {
    gap: 8px;
  }
  .guard-loop-settings label,
  .guard-countdown-clock span {
    gap: 4px;
    font-size: 10px;
    white-space: nowrap;
  }
  .guard-loop-settings input {
    height: 36px;
    padding-inline: 8px;
    font-size: 12px;
  }
  .guard-countdown-clock {
    gap: 3px;
  }
  .guard-countdown-clock strong {
    height: 36px;
    font-size: 16px;
    white-space: nowrap;
  }
  .guard-loop-inline .guard-loop-toggle {
    height: 36px;
    min-height: 36px;
    font-size: 11px;
    white-space: nowrap;
  }
  .guard-loop-inline .guard-runtime-grid {
    grid-template-columns: repeat(4, minmax(0, 1fr)) minmax(180px, 1.7fr);
  }
  .guard-runtime-grid > div {
    gap: 3px;
    padding: 8px;
  }
  .guard-runtime-grid span {
    font-size: 9px;
    white-space: nowrap;
  }
  .guard-runtime-grid strong,
  .guard-loop-inline .guard-runtime-status strong {
    font-size: 11px;
    white-space: nowrap;
  }
  .guard-loop-inline .guard-loop-message {
    overflow: hidden;
    font-size: 10px;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .guard-loop-inline .guard-pause-reason { font-size: 10px; }
  .guard-side {
    gap: 14px;
    font-size: 11px;
  }
  .guard-alert-panel {
    min-height: 320px;
    padding: 14px;
  }
  .guard-map-panel {
    padding: 12px;
  }
  .guard-panel-title {
    flex-wrap: nowrap;
    gap: 8px;
  }
  .guard-panel-title > div:first-child,
  .guard-map-head > div {
    min-width: 0;
  }
  .guard-side .guard-eyebrow {
    font-size: 10px;
    letter-spacing: .05em;
    white-space: nowrap;
  }
  .guard-panel-title h2 {
    margin-top: 3px;
    font-size: 18px;
    white-space: nowrap;
  }
  .guard-alert-actions {
    flex: 0 0 auto;
    flex-wrap: nowrap;
    gap: 6px;
  }
  .guard-alert-actions > button {
    min-height: 32px;
    padding: 0 8px;
    font-size: 10px;
    white-space: nowrap;
  }
  .guard-alert-actions > .guard-pending-events {
    min-width: 44px;
    height: 32px;
    padding-inline: 5px;
  }
  .guard-live-speech {
    gap: 8px;
    margin-top: 10px;
    padding: 10px;
  }
  .guard-live-speech label,
  .guard-live-speech textarea,
  .guard-live-speech > small {
    font-size: 10px;
  }
  .guard-live-microphone {
    grid-template-columns: minmax(0, 1fr) 88px;
    gap: 7px;
  }
  .guard-live-microphone span,
  .guard-live-microphone strong,
  .guard-live-microphone button,
  .guard-live-text-send {
    font-size: 10px;
  }
  .guard-live-microphone strong,
  .guard-live-microphone button {
    white-space: nowrap;
  }
  .guard-alert-main {
    grid-template-columns: 64px minmax(0, 1fr);
    gap: 9px;
    padding: 12px 0;
  }
  .guard-alert-image {
    width: 64px;
    height: 52px;
  }
  .guard-alert-main div:last-child,
  .guard-alert-list div {
    min-width: 0;
  }
  .guard-alert-main strong,
  .guard-alert-main span,
  .guard-alert-main small,
  .guard-alert-list div {
    overflow: hidden;
    font-size: 10px;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .guard-alert-list {
    gap: 7px;
    padding-top: 9px;
  }
  .guard-map-head {
    gap: 7px;
    margin-bottom: 9px;
  }
  .guard-map-head h2 {
    overflow: hidden;
    margin-top: 2px;
    font-size: 15px;
    text-overflow: ellipsis;
    white-space: nowrap;
  }
  .guard-map-head small {
    min-width: 0;
    overflow: hidden;
    font-size: 9px;
    text-overflow: ellipsis;
  }
  .guard-map-empty {
    font-size: 10px;
  }
  .guard-map-legend {
    gap: 5px 8px;
    padding-top: 8px;
    font-size: 9px;
  }
  .guard-route-log {
    gap: 8px;
    margin-top: 9px;
    padding-top: 9px;
  }
  .guard-route-summary {
    grid-template-columns: 76px minmax(0, 1fr);
    gap: 6px;
  }
  .guard-route-summary span,
  .guard-route-summary strong,
  .guard-current-target > span,
  .guard-current-target strong,
  .guard-current-target small,
  .guard-route-order small,
  .guard-waypoint-log strong,
  .guard-waypoint-log span,
  .guard-waypoint-log small,
  .guard-waypoint-log p {
    font-size: 9px;
  }
}
@media (max-width: 980px) {
  .guard-grid { grid-template-columns: 1fr; }
  .guard-side { grid-template-columns: 1fr 220px; align-items: start; }
  .guard-video-stage, .guard-video, .guard-video-empty { min-height: 56vw; }
  .guard-task-bar { grid-template-columns: 1fr; }
  .guard-task-selector, .guard-task-state, .guard-task-actions { grid-column: 1 / -1; }
  .guard-task-actions { grid-template-columns: repeat(3, minmax(0, 1fr)); }
  .guard-localization-bar { align-items: stretch; flex-direction: column; }
  .guard-initialize { width: 100%; }
  .guard-loop-inline { grid-template-columns: 1fr; }
  .guard-loop-inline .guard-runtime-grid,
  .guard-loop-inline .guard-loop-message,
  .guard-loop-inline .guard-pause-reason { grid-column: 1; }
}
@media (min-width: 641px) and (max-width: 1024px) and (orientation: portrait) {
  .guard-page { min-width: 0; padding: 18px; }
  .guard-grid, .guard-side { grid-template-columns: minmax(0, 1fr); }
  .guard-video-stage, .guard-video, .guard-video-empty {
    min-height: 0;
    aspect-ratio: 16 / 9;
  }
  .guard-task-bar { grid-template-columns: minmax(0, 1fr); }
  .guard-task-selector, .guard-task-state, .guard-task-actions { grid-column: 1; }
  .guard-task-actions { grid-template-columns: repeat(3, minmax(0, 1fr)); }
  .guard-primary, .guard-secondary, .guard-danger, .guard-initialize { width: 100%; min-width: 0; }
  .guard-loop-controls { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .guard-loop-inline .guard-loop-toggle { height: auto; min-height: 44px; }
  .guard-loop-inline .guard-loop-settings { max-width: none; }
  .guard-loop-inline .guard-runtime-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .guard-loop-inline .guard-runtime-grid > div { border-bottom: 1px solid #e5eaed; }
  .guard-loop-inline .guard-runtime-grid > div:nth-child(even) { border-right: 0; }
  .guard-loop-inline .guard-runtime-status { grid-column: 1 / -1; border-bottom: 0; }
  .guard-alert-actions > button,
  .guard-live-text-send,
  .guard-live-microphone button,
  .guard-playback-controls button { min-height: 44px; }
  .guard-map-stage, .guard-map-empty { min-height: 300px; }
}
@media (min-width: 1200px) and (max-width: 2048px) and (min-height: 900px) and (max-height: 1280px) and (orientation: landscape) {
  .guard-loop-inline .guard-loop-toggle,
  .guard-alert-actions > button,
  .guard-live-text-send,
  .guard-live-microphone button,
  .guard-playback-controls button,
  .guard-primary,
  .guard-secondary,
  .guard-danger,
  .guard-initialize {
    min-height: 44px;
  }
}
@media (max-width: 640px) {
  .guard-page { padding: 14px; }
  .guard-header { align-items: start; flex-direction: column; }
  .guard-grid, .guard-side { grid-template-columns: 1fr; }
  .guard-task-bar { grid-template-columns: 1fr; }
  .guard-task-selector, .guard-task-state, .guard-task-actions { grid-column: 1; }
  .guard-task-actions { grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 4px; }
  .guard-task-actions > button { padding-inline: 6px; font-size: 10px; }
  .guard-primary, .guard-secondary, .guard-danger { width: 100%; }
  .guard-loop-controls { grid-template-columns: minmax(0, 1fr) minmax(0, 1fr); }
  .guard-loop-inline .guard-loop-settings { grid-column: 1 / -1; max-width: none; }
  .guard-loop-inline .guard-runtime-grid { grid-template-columns: 1fr 1fr; }
  .guard-loop-inline .guard-runtime-grid > div { border-right: 0; border-bottom: 1px solid #e5eaed; }
  .guard-loop-inline .guard-runtime-grid > div:nth-child(odd) { border-right: 1px solid #e5eaed; }
  .guard-loop-inline .guard-runtime-status { grid-column: 1 / -1; border-right: 0; border-bottom: 0; }
  .guard-playback-controls { top: 70px; white-space: nowrap; }
}
</style>
