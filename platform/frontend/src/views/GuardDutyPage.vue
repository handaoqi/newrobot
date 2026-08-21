<script setup>
import Hls from 'hls.js'
import mpegts from 'mpegts.js'
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useRouter } from 'vue-router'

import AppToast from '../components/AppToast.vue'
import LiveVideoPlayer from '../components/LiveVideoPlayer.vue'
import { useToast } from '../composables/useToast'
import {
  API_BASE,
  executePatrolTask,
  fetchMapDetail,
  fetchOverview,
  fetchPatrolTasks,
  fetchRobotDetail,
  fetchRobotNavigationStatus,
  fetchRobots,
  fetchRouteDetail,
  fetchTaskExecution,
  fetchTaskTrajectory,
  sendRecordedAudioCommand,
  sendRobotNavigationCommand,
  setRobotStreamAudioCapture,
  sendTaskExecutionAction,
  sendTextToSpeechCommand,
} from '../services/api'
import { executionActions, isExecutionActive } from '../services/executionState'

const overview = ref(null)
const robots = ref([])
const tasks = ref([])
const selectedRobot = ref(null)
const execution = ref(null)
const loading = ref(true)
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
const loopRestMinutes = ref(1)
const loopActive = ref(false)
const loopState = ref('idle')
const loopStartedAt = ref(0)
const loopEndsAt = ref(0)
const loopStoppedAt = ref(0)
const loopRestUntil = ref(0)
const loopRounds = ref(0)
const loopAccumulatedDistance = ref(0)
const loopCountedExecutionIds = ref([])
const loopCurrentExecutionId = ref('')
const loopMessage = ref('未启动循环巡检')
const nowMs = ref(Date.now())
const liveAudioEnabled = ref(false)
const browserAudioMuted = ref(true)
const browserAudioVolume = ref(1)
const playbackMode = ref('live')
const historyPlaybackPaused = ref(false)
const historyOffsetSeconds = ref(30)
const liveSpeechOpen = ref(false)
const liveSpeechText = ref('')
const liveSpeechSending = ref(false)
const liveRecording = ref(false)
const liveRecordingSeconds = ref(0)
const streamUnavailable = ref(false)
const videoRef = ref(null)
const videoStageRef = ref(null)
const { toastMessage, toastVariant, visible, showToast } = useToast()
const router = useRouter()

let flvPlayer = null
let hlsPlayer = null
let liveGuardTimer = null
let alertEventSource = null
let refreshTimer = null
let executionTimer = null
let loopTimer = null
let loopCycleBusy = false
let localizationRunId = 0
let liveMediaRecorder = null
let liveRecordingStream = null
let liveRecordingTimer = null
let liveRecordingChunks = []
let historySeekTimer = null
let playerResetInProgress = false
let historyManifestUrl = ''
let applyingBrowserAudio = false

const failedCommandStatuses = new Set(['rejected', 'failed', 'cancelled', 'timed_out', 'expired'])
const activeCommandStatuses = new Set(['created', 'published', 'accepted', 'executing'])
const HISTORY_BUFFER_SECONDS = 30 * 60

const latestRobot = computed(() => selectedRobot.value || overview.value?.latest_robot || null)
const playUrls = computed(() => latestRobot.value?.play_urls || {})
const playUrlKey = computed(() => `${playUrls.value.flv || ''}\n${playUrls.value.hls || ''}`)
const hasStream = computed(() => !streamUnavailable.value && Boolean(playUrls.value.flv || playUrls.value.hls))
const isHistoryPlayback = computed(() => playbackMode.value === 'history')
const isFrozenPlayback = computed(() => playbackMode.value === 'paused' || isHistoryPlayback.value)
const historyPlaybackStatus = computed(() => {
  if (isHistoryPlayback.value) return historyPlaybackPaused.value ? '历史回放已暂停' : '历史回放 · 00:00 起播'
  if (playbackMode.value === 'paused') return historyPlaybackPaused.value ? '直播已暂停，可拖动播放条' : '暂停片段播放中，可拖动播放条'
  return '实时直播'
})
const robotTasks = computed(() => {
  if (!latestRobot.value?.id) return tasks.value
  return tasks.value.filter((task) => String(task.robot) === String(latestRobot.value.id))
})
const presetTask = computed(() => {
  const enabledTasks = robotTasks.value.filter((task) => task.enabled)
  const status = navigationStatus.value?.status || {}
  const activeMapId = status.map_id || navigationStatus.value?.current_map_id || latestRobot.value?.current_map_id
  if (activeMapId) {
    return enabledTasks.find((task) => String(task.map_id || '') === String(activeMapId)) || null
  }
  return enabledTasks[0] || null
})
const latestAlert = computed(() => latestRobot.value?.recent_events?.[0] || overview.value?.live_event || null)
const alerts = computed(() => latestRobot.value?.recent_events || [])
const pendingEventCount = computed(() => Number(overview.value?.summary?.pending_event_count || 0))
const actions = computed(() => executionActions(execution.value?.state))
const isRunning = computed(() => isExecutionActive(execution.value?.state))
const executionTaskName = computed(() => execution.value?.task_name || presetTask.value?.name || '管理员尚未配置巡检任务')
const routeWaypoints = computed(() => execution.value?.route_snapshot?.waypoints || routeData.value?.waypoints || [])
const displayRouteWaypoints = computed(() => {
  const routeMapId = execution.value?.map_data || routeData.value?.map_data || presetTask.value?.map_id
  if (routeMapId && mapData.value?.id && String(routeMapId) !== String(mapData.value.id)) return []
  return routeWaypoints.value
})
const currentWaypointIndex = computed(() => Number(execution.value?.current_waypoint_index || 0))
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
const currentExecutionRound = computed(() => {
  if (!execution.value?.id) return 0
  return String(execution.value.id) === String(loopCurrentExecutionId.value || '') ? Math.max(1, loopRounds.value) : 1
})
const latestTargetMilestone = computed(() => [...waypointMilestones.value]
  .reverse()
  .find((event) => event.event_type === 'task.target_dispatched') || null)
const currentExecutionDistance = computed(() => calculateTrajectoryDistance(trajectory.value))
const currentMovementSpeed = computed(() => calculateCurrentSpeed(trajectory.value))
const displayedTotalDistance = computed(() => {
  const counted = loopCountedExecutionIds.value.includes(String(execution.value?.id || ''))
  if (!loopStartedAt.value) return currentExecutionDistance.value
  const currentBelongsToLoop = String(execution.value?.id || '') === String(loopCurrentExecutionId.value || '')
  return loopAccumulatedDistance.value + (currentBelongsToLoop && !counted ? currentExecutionDistance.value : 0)
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
  if (loopActive.value && loopState.value === 'resting') return `轮次休息中（${formatDuration(restRemainingMilliseconds.value)}）`
  if (loopActive.value && loopState.value === 'starting') return '正在启动下一轮'
  if (loopActive.value && loopState.value === 'finishing') return '循环到时，本轮结束后停止'
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
}[localizationInitState.value] || '地图未初始化'))

function sleep(ms) {
  return new Promise((resolve) => window.setTimeout(resolve, ms))
}

function navigationReady(payload = navigationStatus.value) {
  const status = payload?.status || {}
  const mapId = status.map_id || payload?.current_map_id
  const localizationStatus = status.localization_status || payload?.localization_status
  const navReady = status.nav_ready ?? payload?.nav_ready
  return Boolean(mapId) && payload?.connection_status === 'online' && localizationStatus === 'normal' && Boolean(navReady)
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
  localizationInitState.value = 'uninitialized'
  localizationInitMessage.value = '地图未初始化'
}

function formatTime(value) {
  if (!value) return '--'
  const date = new Date(value)
  return Number.isNaN(date.getTime()) ? value : date.toLocaleString('zh-CN', { hour: '2-digit', minute: '2-digit', second: '2-digit' })
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

function calculateTrajectoryDistance(points) {
  let distance = 0
  for (let index = 1; index < points.length; index += 1) {
    const previous = points[index - 1]
    const current = points[index]
    if (previous.map_id && current.map_id && String(previous.map_id) !== String(current.map_id)) continue
    const segment = Math.hypot(Number(current.x) - Number(previous.x), Number(current.y) - Number(previous.y))
    if (Number.isFinite(segment) && segment >= 0 && segment <= 10) distance += segment
  }
  return distance
}

function calculateCurrentSpeed(points) {
  if (!['running', 'resuming'].includes(execution.value?.state) || points.length < 2) return 0
  const current = points[points.length - 1]
  const currentTime = new Date(current.sampled_at || current.received_at || '').getTime()
  if (!Number.isFinite(currentTime) || nowMs.value - currentTime > 5000) return 0
  for (let index = points.length - 2; index >= 0; index -= 1) {
    const previous = points[index]
    const previousTime = new Date(previous.sampled_at || previous.received_at || '').getTime()
    const elapsedSeconds = (currentTime - previousTime) / 1000
    if (!Number.isFinite(elapsedSeconds) || elapsedSeconds < 0.2) continue
    if (previous.map_id && current.map_id && String(previous.map_id) !== String(current.map_id)) return 0
    const distance = Math.hypot(Number(current.x) - Number(previous.x), Number(current.y) - Number(previous.y))
    if (!Number.isFinite(distance) || distance > 10) return 0
    const speed = distance / elapsedSeconds
    return speed < 0.01 ? 0 : Math.min(speed, 5)
  }
  return 0
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
  const status = navigationStatus.value?.status
  const mapId = status?.map_id || navigationStatus.value?.current_map_id
  if (!status || status.localization_status !== 'normal' || String(mapId || '') !== String(mapData.value?.id || '')) return null
  if (status.x === null || status.x === undefined || status.y === null || status.y === undefined) return null
  return { x: Number(status.x), y: Number(status.y), yaw: Number(status.yaw || 0) }
}

function robotHeadingStyle() {
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - Number(robotPoint()?.yaw || 0)}rad)` }
}

function waypointClass(index) {
  if (!execution.value) return ''
  if (index < Number(execution.value.completed_waypoints || 0)) return 'done'
  if (index === currentWaypointIndex.value && isRunning.value) return 'current'
  return ''
}

function eventImage(event) {
  return event?.annotated_snapshot_url || event?.snapshot_url || ''
}

function addRealtimeEvent(event) {
  if (!event?.id) return
  const robot = latestRobot.value
  if (!robot || (event.robot_code && event.robot_code !== robot.code)) return
  const current = robot.recent_events || []
  if (current.some((item) => item.id === event.id)) return
  robot.recent_events = [event, ...current].slice(0, 5)
  if (event.status === 'pending' && overview.value?.summary) {
    overview.value.summary.pending_event_count = pendingEventCount.value + 1
  }
  showToast('收到新的现场报警', { variant: 'alert', duration: 5200 })
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
  const [overviewResult, robotResult, taskResult] = await Promise.all([
    fetchOverview(),
    fetchRobots(),
    fetchPatrolTasks(),
  ])
  overview.value = overviewResult
  robots.value = robotResult
  tasks.value = taskResult
  const robotId = overviewResult.latest_robot?.id || robotResult[0]?.id
  if (robotId && selectedRobot.value?.id !== robotId) await chooseRobot(robotId)
  restoreLoopState(robotId)
  await restoreExecution(taskResult, robotId)
  await refreshExecutionVisual()
}

function startExecutionPolling() {
  if (executionTimer) window.clearInterval(executionTimer)
  executionTimer = isRunning.value ? window.setInterval(refreshExecution, 2000) : null
}

async function restoreExecution(taskList, robotId) {
  if (loopCurrentExecutionId.value) {
    try {
      execution.value = await fetchTaskExecution(loopCurrentExecutionId.value)
      startExecutionPolling()
      return
    } catch {}
  }
  const candidates = taskList
    .filter((task) => !robotId || String(task.robot) === String(robotId))
    .map((task) => task.latest_execution)
    .filter(Boolean)
  const summary = candidates.find((item) => isExecutionActive(item.state))
    || presetTask.value?.latest_execution
    || candidates.sort((a, b) => new Date(b.created_at || 0) - new Date(a.created_at || 0))[0]
  if (!summary?.id) return
  execution.value = await fetchTaskExecution(summary.id)
  startExecutionPolling()
}

async function refreshRobot() {
  if (!latestRobot.value?.id) return
  try {
    selectedRobot.value = await fetchRobotDetail(latestRobot.value.id)
  } catch {}
}

async function refreshLocalizationStatus({ sync = true } = {}) {
  if (!latestRobot.value?.id) return null
  try {
    const result = await fetchRobotNavigationStatus(latestRobot.value.id)
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
  try {
    if (executionId) {
      const track = await fetchTaskTrajectory(executionId)
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
  localizationInitMessage.value = '正在重启导航/定位栈'
  try {
    const current = navigationStatus.value || await refreshLocalizationStatus({ sync: false })
    const status = current?.status || {}
    const mapId = presetTask.value?.map_id || status.map_id || current?.current_map_id
    const mapVersion = status.map_version || current?.current_map_version || ''
    if (!mapId) throw new Error('没有可初始化的地图，请先为值守任务配置路线地图')

    const command = await sendRobotNavigationCommand(robot.id, 'restart', {
      map_id: mapId,
      map_version: mapVersion,
    })

    for (let attempt = 0; attempt < 25 && runId === localizationRunId; attempt += 1) {
      await sleep(3000)
      const latest = await refreshLocalizationStatus({ sync: false })
      if (!latest) continue
      const latestCommand = latest.command
      const isCurrentCommand = String(latestCommand?.id || '') === String(command.id || '')
      if (isCurrentCommand && failedCommandStatuses.has(latestCommand.status)) {
        throw new Error(latestCommand.error_message || latestCommand.error_code || '导航/定位栈重启失败')
      }
      if (isCurrentCommand && latestCommand.status === 'succeeded') {
        localizationInitMessage.value = '导航栈已重启，正在等待定位收敛'
        if (navigationReady(latest)) {
          localizationInitState.value = 'success'
          localizationInitMessage.value = '当前地图定位正常，导航栈已就绪'
          showToast('定位初始化成功')
          return
        }
      }
    }
    if (runId !== localizationRunId) return
    throw new Error('初始化超时，未检测到定位收敛或导航栈就绪')
  } catch (error) {
    if (runId !== localizationRunId) return
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
    await refreshExecutionVisual()
    if (!isRunning.value) {
      window.clearInterval(executionTimer)
      executionTimer = null
    }
  } catch {}
}

function loopStorageKey(robotId = latestRobot.value?.id) {
  return robotId ? `guard-duty-loop:${robotId}` : ''
}

function persistLoopState() {
  const key = loopStorageKey()
  if (!key) return
  localStorage.setItem(key, JSON.stringify({
    durationMinutes: Number(loopDurationMinutes.value),
    restMinutes: Number(loopRestMinutes.value),
    active: loopActive.value,
    state: loopState.value,
    startedAt: loopStartedAt.value,
    endsAt: loopEndsAt.value,
    stoppedAt: loopStoppedAt.value,
    restUntil: loopRestUntil.value,
    rounds: loopRounds.value,
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
    loopRestMinutes.value = Number(saved.restMinutes) >= 0 ? Number(saved.restMinutes) : 1
    loopStartedAt.value = Number(saved.startedAt || 0)
    loopEndsAt.value = Number(saved.endsAt || 0)
    loopStoppedAt.value = Number(saved.stoppedAt || 0)
    loopRestUntil.value = Number(saved.restUntil || 0)
    loopRounds.value = Number(saved.rounds || 0)
    loopAccumulatedDistance.value = Number(saved.accumulatedDistance || 0)
    loopCountedExecutionIds.value = Array.isArray(saved.countedExecutionIds) ? saved.countedExecutionIds.map(String) : []
    loopCurrentExecutionId.value = String(saved.currentExecutionId || '')
    if (saved.active && loopEndsAt.value > Date.now()) {
      loopActive.value = true
      loopState.value = saved.state || 'running'
      loopMessage.value = saved.message || '循环巡检已恢复'
    } else {
      loopActive.value = false
      loopState.value = saved.active ? 'completed' : (saved.state || 'idle')
      loopStoppedAt.value = loopStoppedAt.value || (loopEndsAt.value ? Math.min(Date.now(), loopEndsAt.value) : 0)
      loopMessage.value = saved.active ? '循环时长已结束' : (saved.message || '未启动循环巡检')
    }
  } catch {
    localStorage.removeItem(key)
  }
}

function finishLoop(message, { notify = true } = {}) {
  loopActive.value = false
  loopState.value = 'completed'
  loopStoppedAt.value = Date.now()
  loopRestUntil.value = 0
  loopMessage.value = message
  persistLoopState()
  if (notify) showToast(message)
}

function stopLoop({ notify = true } = {}) {
  if (!loopActive.value) return
  loopActive.value = false
  loopState.value = 'stopped'
  loopStoppedAt.value = Date.now()
  loopRestUntil.value = 0
  loopMessage.value = isRunning.value ? '循环已停止，当前轮次继续执行' : '循环已停止'
  persistLoopState()
  if (notify) showToast(loopMessage.value)
}

async function launchTask({ fromLoop = false } = {}) {
  if (!presetTask.value || busy.value || isRunning.value || (!fromLoop && loopActive.value)) return null
  if (fromLoop && !navigationReady()) {
    showToast('定位或导航栈未就绪，本轮稍后重试', { variant: 'alert' })
    return null
  }
  busy.value = true
  try {
    execution.value = await executePatrolTask(presetTask.value.id, { loopExecution: fromLoop })
    trajectory.value = []
    trajectoryExecutionId.value = String(execution.value.id)
    if (fromLoop) {
      loopRounds.value += 1
      loopCurrentExecutionId.value = String(execution.value.id)
      loopState.value = 'running'
      loopMessage.value = `第 ${loopRounds.value} 轮巡检执行中`
      persistLoopState()
    } else {
      loopStartedAt.value = 0
      loopEndsAt.value = 0
      loopStoppedAt.value = 0
      loopRounds.value = 0
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
    showToast(error.message || '任务启动失败', { variant: 'alert' })
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
  if (!executionId || loopCountedExecutionIds.value.includes(executionId)) return
  if (String(trajectoryExecutionId.value) !== executionId) await refreshExecutionVisual()
  loopAccumulatedDistance.value += calculateTrajectoryDistance(trajectory.value)
  loopCountedExecutionIds.value = [...loopCountedExecutionIds.value, executionId].slice(-100)
  persistLoopState()
}

function scheduleNextLoopRound(message = '') {
  const restMilliseconds = Math.max(0, Number(loopRestMinutes.value || 0) * 60 * 1000)
  loopState.value = 'resting'
  loopRestUntil.value = Date.now() + restMilliseconds
  loopMessage.value = message || `第 ${loopRounds.value} 轮完成，休息 ${loopRestMinutes.value} 分钟`
  persistLoopState()
}

async function runLoopCycle() {
  nowMs.value = Date.now()
  if (!loopActive.value || loopCycleBusy) return
  loopCycleBusy = true
  try {
    if (nowMs.value >= loopEndsAt.value) {
      if (isRunning.value && String(execution.value?.id || '') === String(loopCurrentExecutionId.value || '')) {
        loopState.value = 'finishing'
        loopMessage.value = '循环时长已到，本轮结束后停止'
        persistLoopState()
        return
      }
      await captureLoopDistance()
      finishLoop('循环巡检已按设定时长完成')
      return
    }

    if (loopState.value === 'resting') {
      if (nowMs.value < loopRestUntil.value || busy.value) return
      loopState.value = 'starting'
      loopMessage.value = '正在启动下一轮巡检'
      persistLoopState()
      const started = await launchTask({ fromLoop: true })
      if (!started && loopActive.value) scheduleNextLoopRound('下一轮启动失败，休息后自动重试')
      return
    }

    if (isRunning.value && String(execution.value?.id || '') === String(loopCurrentExecutionId.value || '')) {
      if (loopState.value !== 'finishing') loopState.value = 'running'
      return
    }

    if (loopCurrentExecutionId.value && String(execution.value?.id || '') === String(loopCurrentExecutionId.value)) {
      await captureLoopDistance()
      scheduleNextLoopRound(execution.value?.state === 'completed'
        ? `第 ${loopRounds.value} 轮完成，进入休息`
        : `第 ${loopRounds.value} 轮已结束，休息后继续下一轮`)
      return
    }

    const started = await launchTask({ fromLoop: true })
    if (!started && loopActive.value) scheduleNextLoopRound('任务启动失败，休息后自动重试')
  } finally {
    loopCycleBusy = false
  }
}

async function toggleLoop() {
  if (loopActive.value) {
    stopLoop()
    return
  }
  if (!presetTask.value || isRunning.value || busy.value || localizationBusy.value) return
  if (!navigationReady()) {
    showToast('请先完成地图定位初始化', { variant: 'alert' })
    return
  }
  const duration = Number(loopDurationMinutes.value)
  const rest = Number(loopRestMinutes.value)
  if (!Number.isFinite(duration) || duration <= 0 || !Number.isFinite(rest) || rest < 0) {
    showToast('请填写正确的循环时长和休息时间', { variant: 'alert' })
    return
  }
  const startedAt = Date.now()
  loopActive.value = true
  loopState.value = 'starting'
  loopStartedAt.value = startedAt
  loopEndsAt.value = startedAt + duration * 60 * 1000
  loopStoppedAt.value = 0
  loopRestUntil.value = 0
  loopRounds.value = 0
  loopAccumulatedDistance.value = 0
  loopCountedExecutionIds.value = []
  loopCurrentExecutionId.value = ''
  loopMessage.value = '正在启动第 1 轮巡检'
  persistLoopState()
  const started = await launchTask({ fromLoop: true })
  if (!started && loopActive.value) scheduleNextLoopRound('第 1 轮启动失败，休息后自动重试')
}

async function controlTask() {
  const action = actions.value.control.action
  if (!execution.value?.id || busy.value || !action) return
  busy.value = true
  try {
    execution.value = await sendTaskExecutionAction(execution.value.id, action)
    showToast(action === 'pause' ? '任务暂停中' : '任务继续执行中')
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
  stopLoop({ notify: false })
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

async function setLiveAudio(enabled, { notify = true } = {}) {
  const robot = latestRobot.value
  if (!robot?.id) {
    if (notify) showToast('当前没有可控制的机器人音频采集', { variant: 'alert' })
    return false
  }
  try {
    await setRobotStreamAudioCapture(robot.id, enabled)
    liveAudioEnabled.value = enabled
    if (notify) showToast(enabled ? '已请求开启 NX 现场音频采集' : '已请求关闭 NX 现场音频采集')
    return true
  } catch (error) {
    if (notify) showToast(error.message || 'NX 现场音频采集控制失败', { variant: 'alert' })
    return false
  }
}

async function toggleLiveAudio() {
  await setLiveAudio(!liveAudioEnabled.value)
}

function applyBrowserAudio(element, { muted = browserAudioMuted.value, volume = browserAudioVolume.value } = {}) {
  applyingBrowserAudio = true
  element.muted = muted
  element.volume = volume
  applyingBrowserAudio = false
}

function handleBrowserAudioChange(event) {
  if (applyingBrowserAudio) return
  const element = event.currentTarget
  browserAudioMuted.value = element.muted
  browserAudioVolume.value = element.volume
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

function releaseHistoryManifest() {
  if (!historyManifestUrl) return
  URL.revokeObjectURL(historyManifestUrl)
  historyManifestUrl = ''
}

function destroyPlayers() {
  if (historySeekTimer) {
    window.clearTimeout(historySeekTimer)
    historySeekTimer = null
  }
  stopLiveGuard()
  const element = videoRef.value
  const replacingAttachedPlayer = Boolean(element && (flvPlayer || hlsPlayer || element.currentSrc))
  if (replacingAttachedPlayer) playerResetInProgress = true
  flvPlayer?.destroy()
  hlsPlayer?.destroy()
  flvPlayer = null
  hlsPlayer = null
  releaseHistoryManifest()
  if (element) {
    element.removeAttribute('src')
    element.load()
  }
  if (replacingAttachedPlayer) {
    window.setTimeout(() => { playerResetInProgress = false }, 250)
  }
}

function markStreamUnavailable() {
  streamUnavailable.value = true
  destroyPlayers()
}

function seekLatestFrame() {
  const element = videoRef.value
  if (!element) return
  const ranges = element.buffered
  if (ranges?.length) {
    const liveEnd = ranges.end(ranges.length - 1)
    if (Number.isFinite(liveEnd) && liveEnd - element.currentTime > 0.8) {
      element.currentTime = Math.max(0, liveEnd - 0.12)
    }
  } else if (Number.isFinite(element.duration) && element.duration > 0 && element.duration - element.currentTime > 0.8) {
    element.currentTime = Math.max(0, element.duration - 0.12)
  }
}

function keepLivePlaying() {
  if (playbackMode.value !== 'live') return
  const element = videoRef.value
  if (!element) return
  // Do not force mute here: this page exposes an operator-controlled live-audio toggle.
  seekLatestFrame()
  if (element.paused) element.play().catch(() => {})
}

function startLiveGuard() {
  if (playbackMode.value !== 'live') return
  stopLiveGuard()
  keepLivePlaying()
  liveGuardTimer = window.setInterval(keepLivePlaying, 800)
}

function stopLiveGuard() {
  if (liveGuardTimer) {
    window.clearInterval(liveGuardTimer)
    liveGuardTimer = null
  }
}

function playbackRange(element) {
  const ranges = element?.seekable?.length ? element.seekable : element?.buffered
  if (!ranges?.length) return null
  const start = ranges.start(0)
  const end = ranges.end(ranges.length - 1)
  return Number.isFinite(start) && Number.isFinite(end) && end > start ? { start, end } : null
}

async function freezeHistoryManifest(hlsUrl) {
  const response = await fetch(hlsUrl, { cache: 'no-store' })
  if (!response.ok) throw new Error(`历史播放清单读取失败（${response.status}）`)
  const playlist = await response.text()
  const lines = playlist
    .split(/\r?\n/)
    .map((line) => (line && !line.startsWith('#') ? new URL(line, hlsUrl).href : line))
    .filter((line) => line !== '')
  if (!lines.includes('#EXT-X-ENDLIST')) lines.push('#EXT-X-ENDLIST')

  const blob = new Blob([`${lines.join('\n')}\n`], { type: 'application/vnd.apple.mpegurl' })
  historyManifestUrl = URL.createObjectURL(blob)
  return historyManifestUrl
}

function applyHistoryPosition(element, attempts = 0) {
  if (!isFrozenPlayback.value || !element) return
  const range = playbackRange(element)
  if (!range) {
    if (attempts < 12) {
      historySeekTimer = window.setTimeout(() => applyHistoryPosition(element, attempts + 1), 250)
    }
    return
  }

  const target = historyOffsetSeconds.value >= HISTORY_BUFFER_SECONDS
    ? range.start
    : Math.max(range.start, range.end - historyOffsetSeconds.value)
  historyOffsetSeconds.value = Math.max(0, Math.round(range.end - target))
  // Hls.js does not otherwise fetch an older segment while it is following
  // the live edge.  Starting at the requested media time keeps audio/video
  // aligned in the retained HLS window.
  hlsPlayer?.startLoad?.(target)
  element.currentTime = target

  if (playbackMode.value === 'paused' || historyPlaybackPaused.value) {
    const pauseAfterFrame = () => element.pause()
    element.addEventListener('canplay', pauseAfterFrame, { once: true })
    element.play().then(() => applyBrowserAudio(element)).catch(() => {})
  } else {
    element.play().then(() => applyBrowserAudio(element)).catch(() => {})
  }
}

async function startHistoryPlayback(offsetSeconds = 30, { paused = false } = {}) {
  if (!playUrls.value.hls) {
    showToast('历史回放需要 HLS 视频流', { variant: 'alert' })
    return
  }
  historyOffsetSeconds.value = Math.min(HISTORY_BUFFER_SECONDS, Math.max(0, Number(offsetSeconds) || 0))
  historyPlaybackPaused.value = paused
  playbackMode.value = 'history'
  await setupPlayer()
}

async function pauseLivePlayback({ notify = true } = {}) {
  if (!playUrls.value.hls) {
    if (notify) showToast('暂停需要 HLS 视频流', { variant: 'alert' })
    return
  }
  if (playbackMode.value === 'paused') return
  stopLiveGuard()
  historyOffsetSeconds.value = 0
  historyPlaybackPaused.value = true
  playbackMode.value = 'paused'
  await setupPlayer()
  if (notify) showToast('直播已暂停，片段已冻结，可拖动播放条')
}

async function replayHistoryFromBeginning() {
  await startHistoryPlayback(HISTORY_BUFFER_SECONDS)
  showToast('正在从历史 00:00 开始回放')
}

async function returnToLive() {
  if (playbackMode.value === 'live') return
  playbackMode.value = 'live'
  historyPlaybackPaused.value = false
  await setupPlayer()
  showToast('已返回实时画面')
}

function handleVideoPause() {
  if (playerResetInProgress) return
  if (isFrozenPlayback.value) {
    historyPlaybackPaused.value = true
    return
  }
  if (playbackMode.value === 'live') {
    void pauseLivePlayback({ notify: false })
  }
}

function handleVideoPlay() {
  if (isFrozenPlayback.value) {
    historyPlaybackPaused.value = false
    return
  }
  if (playbackMode.value === 'live') {
    startLiveGuard()
  }
}

async function setupPlayer() {
  await nextTick()
  destroyPlayers()
  const element = videoRef.value
  if (!element || !hasStream.value) return
  // Start muted for autoplay, then restore the browser player's own audio
  // state. NX capture is controlled only by the field-audio button.
  applyBrowserAudio(element, { muted: true, volume: browserAudioVolume.value })
  const { flv, hls } = playUrls.value
  try {
    // Keep low-latency FLV for real-time duty.  A paused/history session uses
    // the retained HLS playlist so it never gets pulled back to the live edge.
    if (!isFrozenPlayback.value && flv && mpegts.getFeatureList().mseLivePlayback) {
      flvPlayer = mpegts.createPlayer({ type: 'flv', isLive: true, url: flv }, {
        enableStashBuffer: false,
        lazyLoad: false,
        liveSync: true,
        liveSyncMaxLatency: 1.0,
        liveSyncTargetLatency: 0.35,
        liveSyncPlaybackRate: 1.75,
        liveBufferLatencyChasing: true,
        liveBufferLatencyMaxLatency: 3.0,
        liveBufferLatencyMinRemain: 0.35,
      })
      flvPlayer.on(mpegts.Events.ERROR, markStreamUnavailable)
      flvPlayer.attachMediaElement(element)
      flvPlayer.load()
      await element.play()
      applyBrowserAudio(element)
      startLiveGuard()
      return
    }
    if (hls && Hls.isSupported()) {
      const frozen = isFrozenPlayback.value
      const hlsSource = frozen ? await freezeHistoryManifest(hls) : hls
      hlsPlayer = new Hls({
        lowLatencyMode: !frozen,
        startPosition: frozen ? 0 : -1,
        liveSyncDurationCount: 1,
        liveMaxLatencyDurationCount: 2,
        maxLiveSyncPlaybackRate: 1.75,
        backBufferLength: frozen ? HISTORY_BUFFER_SECONDS : 15,
      })
      hlsPlayer.loadSource(hlsSource)
      hlsPlayer.attachMedia(element)
      hlsPlayer.on(Hls.Events.ERROR, (_event, data) => data?.fatal && markStreamUnavailable())
      hlsPlayer.on(Hls.Events.MANIFEST_PARSED, async () => {
        if (isFrozenPlayback.value) {
          applyHistoryPosition(element)
          return
        }
        try {
          await element.play()
          applyBrowserAudio(element)
          startLiveGuard()
        } catch {
          markStreamUnavailable()
        }
      })
      return
    }
    if (hls && element.canPlayType('application/vnd.apple.mpegurl')) {
      element.src = isFrozenPlayback.value ? await freezeHistoryManifest(hls) : hls
      if (isFrozenPlayback.value) {
        element.addEventListener('loadedmetadata', () => applyHistoryPosition(element), { once: true })
      } else {
        await element.play()
        applyBrowserAudio(element)
        startLiveGuard()
      }
      return
    }
  } catch {
    markStreamUnavailable()
  }
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
    loading.value = false
  }
  if (loaded) {
    refreshTimer = window.setInterval(refreshGuardState, 5000)
    loopTimer = window.setInterval(runLoopCycle, 1000)
    window.addEventListener('resize', refreshImageGeometry)
  }
})

onBeforeUnmount(() => {
  localizationRunId += 1
  window.clearInterval(refreshTimer)
  window.clearInterval(executionTimer)
  window.clearInterval(loopTimer)
  window.removeEventListener('resize', refreshImageGeometry)
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
          <div ref="videoStageRef" class="guard-video-stage">
            <LiveVideoPlayer
              :play-urls="playUrls"
              :robot-id="latestRobot?.id"
              :available="hasStream"
              object-fit="contain"
              @notice="showVideoNotice"
              @stream-error="streamUnavailable = true"
            >
              <template #empty>
                <div class="guard-video-empty">
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
            <div>
              <span>当前任务</span>
              <strong>{{ executionTaskName }}</strong>
            </div>
            <div>
              <span>执行状态</span>
              <strong>{{ taskStateText }}</strong>
            </div>
            <button class="guard-primary" :disabled="busy || localizationBusy || loopActive || !presetTask || isRunning" @click="startTask">
              {{ busy ? '处理中...' : '开始巡检' }}
            </button>
            <button class="guard-secondary" :disabled="busy || localizationBusy || !actions.control.enabled" @click="controlTask">
              {{ actions.control.label }}
            </button>
            <button class="guard-danger" :disabled="busy || localizationBusy || !execution?.id || !actions.forceExit" @click="forceExitTask">强制退出</button>
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
                  <span>每轮休息（分钟）</span>
                  <input v-model.number="loopRestMinutes" type="number" min="0" step="1" :disabled="loopActive" @change="persistLoopState" />
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
              <div><span>当前速度</span><strong>{{ currentMovementSpeed.toFixed(2) }} m/s</strong></div>
              <div><span>执行轮次</span><strong>{{ loopRounds }} 轮</strong></div>
              <div class="guard-runtime-status"><span>当前状态</span><strong>{{ guardRuntimeStatus }}</strong></div>
            </div>
            <p class="guard-loop-message">{{ loopMessage }}</p>
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
              <small>{{ trajectory.length }} 个轨迹点</small>
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
                  <div v-if="robotPoint() && displayPosition(robotPoint())" class="guard-map-robot" :style="displayPosition(robotPoint())">
                    <i :style="robotHeadingStyle()"></i>
                  </div>
                </div>
              </div>
            </div>
            <div class="guard-map-legend"><span class="route">规划路线</span><span class="track">实际轨迹</span><span class="robot">机器狗</span></div>
            <section class="guard-route-log">
              <div class="guard-route-summary">
                <div><span>本次轮次</span><strong>第 {{ currentExecutionRound || 1 }} 轮</strong></div>
                <div><span>计划路线</span><strong>{{ execution?.route_name || routeData?.name || '--' }}</strong></div>
              </div>
              <div class="guard-route-order" aria-label="本轮计划航点顺序">
                <span v-for="(number, index) in executionWaypointOrder" :key="`${number}-${index}`">{{ number }}</span>
                <small v-if="!executionWaypointOrder.length">暂无航点</small>
              </div>
              <div v-if="latestTargetMilestone" class="guard-current-target">
                <span>当前下发目标</span>
                <strong>{{ latestTargetMilestone.payload?.waypoint?.map_point_number }}号点</strong>
                <small>{{ formatTime(latestTargetMilestone.occurred_at) }} · {{ formatPose(latestTargetMilestone.payload?.waypoint) }}</small>
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
.guard-header { display: flex; align-items: center; justify-content: space-between; gap: 24px; margin: 0 auto 18px; max-width: 1440px; }
.guard-header h1 { margin: 4px 0 0; font-size: clamp(24px, 3vw, 40px); letter-spacing: 0; }
.guard-eyebrow { color: #667785; font-size: 12px; font-weight: 800; letter-spacing: .08em; text-transform: uppercase; }
.guard-status { display: flex; align-items: center; gap: 10px; padding: 12px 16px; border: 1px solid #d6dee3; background: #fff; font-weight: 800; }
.guard-status-dot { width: 10px; height: 10px; border-radius: 50%; background: #9aa8b2; }
.guard-status.is-online .guard-status-dot { background: #1b9b65; }
.guard-grid { display: grid; grid-template-columns: minmax(0, 1fr) 360px; gap: 18px; max-width: 1440px; margin: 0 auto; }
.guard-video-panel, .guard-alert-panel { border: 1px solid #d6dee3; background: #fff; }
.guard-video-stage { position: relative; min-height: 520px; background: #152633; overflow: hidden; }
.guard-video, .guard-video-empty { display: block; width: 100%; height: 100%; min-height: 520px; object-fit: contain; }
.guard-video-empty { display: grid; place-content: center; gap: 8px; color: #d7e0e6; text-align: center; }
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
.guard-task-bar { display: grid; grid-template-columns: minmax(220px, 1.5fr) minmax(140px, .8fr) repeat(3, auto); align-items: center; gap: 14px; padding: 16px; }
.guard-task-bar > div { display: grid; gap: 5px; min-width: 0; }
.guard-task-bar span { color: #70808c; font-size: 12px; }
.guard-task-bar strong { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.guard-localization-bar { display: flex; align-items: center; justify-content: space-between; gap: 20px; padding: 14px 16px; border-top: 1px solid #e2e8ec; background: #f7f9fa; }
.guard-localization-copy { display: grid; grid-template-columns: auto auto; align-items: center; gap: 4px 12px; min-width: 0; }
.guard-localization-copy > span { color: #70808c; font-size: 12px; }
.guard-localization-copy > strong { font-size: 15px; }
.guard-localization-copy > strong.is-success { color: #14734c; }
.guard-localization-copy > strong.is-failed { color: #b8322c; }
.guard-localization-copy > strong.is-initializing { color: #b0640e; }
.guard-localization-copy > small { grid-column: 1 / -1; overflow: hidden; color: #71818c; text-overflow: ellipsis; white-space: nowrap; }
.guard-primary, .guard-secondary, .guard-danger, .guard-initialize { min-height: 52px; padding: 0 22px; border: 0; font: inherit; font-weight: 800; cursor: pointer; }
.guard-primary { color: #fff; background: #19724d; }
.guard-secondary { color: #263944; background: #dfe7eb; }
.guard-danger { color: #fff; background: #b8322c; }
.guard-initialize { min-width: 150px; color: #fff; background: #146fb3; }
.guard-primary:disabled, .guard-secondary:disabled, .guard-danger:disabled, .guard-initialize:disabled { cursor: not-allowed; opacity: .45; }
.guard-side { display: grid; align-content: start; gap: 18px; }
.guard-alert-panel, .guard-loop-panel, .guard-map-panel { border: 1px solid #d6dee3; background: #fff; }
.guard-alert-panel { min-height: 360px; padding: 18px; }
.guard-panel-title { display: flex; justify-content: space-between; align-items: start; padding-bottom: 14px; border-bottom: 1px solid #e4eaed; }
.guard-panel-title h2 { margin: 5px 0 0; font-size: 24px; letter-spacing: 0; }
.guard-alert-actions { display: flex; align-items: center; gap: 8px; }
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
.guard-loop-controls { display: grid; grid-template-columns: minmax(300px, 1fr) 160px 150px; align-items: end; gap: 14px; }
.guard-loop-inline .guard-loop-settings { max-width: 460px; }
.guard-loop-inline .guard-loop-toggle { height: 42px; min-height: 42px; }
.guard-countdown-clock { display: grid; gap: 4px; }
.guard-countdown-clock span { color: #687a86; font-size: 12px; font-weight: 700; }
.guard-countdown-clock strong { display: flex; align-items: center; justify-content: center; height: 42px; box-sizing: border-box; border: 1px solid #bfccd3; color: #102c3d; background: #eaf0f3; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size: 21px; letter-spacing: .05em; font-variant-numeric: tabular-nums; }
.guard-loop-inline .guard-runtime-grid { grid-column: 1 / -1; grid-template-columns: repeat(4, minmax(0, 1fr)) minmax(240px, 2fr); }
.guard-loop-inline .guard-runtime-grid > div { border-right: 1px solid #e5eaed; border-bottom: 0; }
.guard-loop-inline .guard-runtime-grid > div:last-child { border-right: 0; }
.guard-loop-inline .guard-runtime-status strong { white-space: normal; }
.guard-loop-inline .guard-loop-message { grid-column: 1 / -1; }
.guard-loop-light { width: 14px; height: 14px; border-radius: 50%; background: #a9b5bd; box-shadow: 0 0 0 5px rgba(169, 181, 189, .18); }
.guard-loop-light.active { background: #19a568; box-shadow: 0 0 0 5px rgba(25, 165, 104, .18); }
.guard-loop-settings { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
.guard-loop-settings label { display: grid; gap: 6px; color: #687a86; font-size: 12px; font-weight: 700; }
.guard-loop-settings input { width: 100%; min-width: 0; height: 42px; box-sizing: border-box; padding: 0 10px; border: 1px solid #cad4da; background: #fff; color: #172b37; font: inherit; font-size: 15px; font-weight: 800; }
.guard-loop-settings input:disabled { background: #edf1f3; color: #75858f; }
.guard-loop-toggle { min-height: 48px; border: 0; color: #fff; background: #136fac; font: inherit; font-weight: 900; cursor: pointer; }
.guard-loop-toggle.is-active { background: #a9402d; }
.guard-loop-toggle:disabled { cursor: not-allowed; opacity: .45; }
.guard-runtime-grid { display: grid; grid-template-columns: 1fr 1fr; border: 1px solid #dce4e8; }
.guard-runtime-grid > div { display: grid; gap: 4px; min-width: 0; padding: 10px; border-bottom: 1px solid #e5eaed; }
.guard-runtime-grid > div:nth-child(odd) { border-right: 1px solid #e5eaed; }
.guard-runtime-grid > div:nth-last-child(-n + 2) { border-bottom: 0; }
.guard-runtime-grid span { color: #72838e; font-size: 11px; }
.guard-runtime-grid strong { overflow: hidden; font-size: 14px; text-overflow: ellipsis; white-space: nowrap; }
.guard-loop-message { margin: 0; color: #657681; font-size: 12px; line-height: 1.5; }
.guard-map-panel { padding: 16px; }
.guard-map-head { display: flex; align-items: end; justify-content: space-between; gap: 12px; margin-bottom: 12px; }
.guard-map-head h2 { margin: 4px 0 0; font-size: 20px; }
.guard-map-head small { color: #71818c; white-space: nowrap; }
.guard-map-stage { position: relative; min-height: 230px; overflow: hidden; border: 1px solid #dce4e8; background: #e9eef1; }
.guard-map-empty { display: grid; place-items: center; min-height: 230px; color: #71818c; font-size: 13px; }
.guard-map-layer { position: relative; width: 100%; background: #fff; }
.guard-map-layer img { display: block; width: 100%; height: auto; user-select: none; }
.guard-map-lines, .guard-map-markers { position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; }
.guard-map-waypoint { position: absolute; display: grid; place-items: center; width: 20px; height: 20px; border: 2px solid #fff; border-radius: 50%; color: #fff; background: #2563eb; box-shadow: 0 2px 6px rgba(15, 35, 48, .38); font-size: 10px; font-weight: 900; transform: translate(-50%, -50%); }
.guard-map-waypoint.done { background: #10b981; }
.guard-map-waypoint.current { background: #f59e0b; }
.guard-map-robot { position: absolute; z-index: 5; width: 28px; height: 28px; transform: translate(-50%, -50%); }
.guard-map-robot::before { content: ''; position: absolute; inset: 3px; border: 3px solid #fff; border-radius: 50%; background: #ec4a3f; box-shadow: 0 2px 8px rgba(236, 74, 63, .5); }
.guard-map-robot i { position: absolute; left: 50%; top: 50%; z-index: 6; width: 0; height: 0; border-right: 5px solid transparent; border-bottom: 14px solid #8f2019; border-left: 5px solid transparent; transform-origin: 50% 70%; }
.guard-map-legend { display: flex; flex-wrap: wrap; gap: 14px; padding-top: 10px; color: #657681; font-size: 11px; }
.guard-map-legend span::before { content: ''; display: inline-block; width: 14px; height: 3px; margin-right: 5px; vertical-align: middle; background: #2563eb; }
.guard-map-legend .track::before { background: #10b981; }
.guard-map-legend .robot::before { width: 8px; height: 8px; border-radius: 50%; background: #ec4a3f; }
.guard-route-log { display: grid; gap: 10px; margin-top: 12px; padding-top: 12px; border-top: 1px solid #dde5e9; }
.guard-route-summary { display: grid; grid-template-columns: 92px minmax(0, 1fr); gap: 8px; }
.guard-route-summary > div { display: grid; gap: 2px; min-width: 0; }
.guard-route-summary span, .guard-current-target > span { color: #71818c; font-size: 10px; }
.guard-route-summary strong { overflow: hidden; font-size: 12px; text-overflow: ellipsis; white-space: nowrap; }
.guard-route-order { display: flex; flex-wrap: wrap; align-items: center; gap: 5px; }
.guard-route-order span { display: grid; place-items: center; width: 24px; height: 24px; color: #fff; background: #2563eb; font-size: 11px; font-weight: 900; }
.guard-route-order span:not(:last-child)::after { content: ''; }
.guard-route-order small { color: #71818c; font-size: 11px; }
.guard-current-target { display: grid; grid-template-columns: 1fr auto; gap: 3px 8px; padding: 9px 10px; border-left: 3px solid #e79a18; background: #fff7e7; }
.guard-current-target strong { color: #9a5b00; font-size: 12px; }
.guard-current-target small { grid-column: 1 / -1; color: #6f7d86; font-size: 10px; }
.guard-waypoint-log { display: grid; max-height: 260px; overflow-y: auto; border-top: 1px solid #e1e7ea; }
.guard-waypoint-log article { display: grid; grid-template-columns: 10px minmax(0, 1fr); gap: 8px; padding: 9px 2px; border-bottom: 1px solid #edf1f3; }
.guard-waypoint-log article > i { width: 8px; height: 8px; margin-top: 4px; border-radius: 50%; background: #e79a18; }
.guard-waypoint-log article > i.is-reached { background: #159a63; }
.guard-waypoint-log article > div { display: grid; gap: 2px; min-width: 0; }
.guard-waypoint-log strong { font-size: 11px; }
.guard-waypoint-log span, .guard-waypoint-log small, .guard-waypoint-log p { color: #71818c; font-size: 10px; line-height: 1.4; }
.guard-waypoint-log p { margin: 8px 0 0; }
.guard-loading { display: grid; place-items: center; min-height: 50vh; color: #657681; }
@media (max-width: 980px) {
  .guard-grid { grid-template-columns: 1fr; }
  .guard-side { grid-template-columns: 1fr 220px; align-items: start; }
  .guard-video-stage, .guard-video, .guard-video-empty { min-height: 56vw; }
  .guard-task-bar { grid-template-columns: 1fr 1fr; }
  .guard-task-bar > div:first-child { grid-column: 1 / -1; }
  .guard-localization-bar { align-items: stretch; flex-direction: column; }
  .guard-initialize { width: 100%; }
  .guard-loop-inline { grid-template-columns: 1fr; }
  .guard-loop-inline .guard-runtime-grid, .guard-loop-inline .guard-loop-message { grid-column: 1; }
}
@media (max-width: 640px) {
  .guard-page { padding: 14px; }
  .guard-header { align-items: start; flex-direction: column; }
  .guard-grid, .guard-side { grid-template-columns: 1fr; }
  .guard-task-bar { grid-template-columns: 1fr 1fr; }
  .guard-task-bar > div:first-child { grid-column: 1 / -1; }
  .guard-primary, .guard-secondary, .guard-danger { width: 100%; }
  .guard-loop-controls { grid-template-columns: 1fr 1fr; }
  .guard-loop-inline .guard-loop-settings { grid-column: 1 / -1; max-width: none; }
  .guard-loop-inline .guard-runtime-grid { grid-template-columns: 1fr 1fr; }
  .guard-loop-inline .guard-runtime-grid > div { border-right: 0; border-bottom: 1px solid #e5eaed; }
  .guard-loop-inline .guard-runtime-grid > div:nth-child(odd) { border-right: 1px solid #e5eaed; }
  .guard-loop-inline .guard-runtime-status { grid-column: 1 / -1; border-right: 0; border-bottom: 0; }
  .guard-playback-controls { top: 70px; white-space: nowrap; }
}
</style>
