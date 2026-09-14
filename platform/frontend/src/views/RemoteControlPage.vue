<script setup>
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { createAsyncPoller, useAsyncPoller } from '../composables/useAsyncPoller'
import { API_BASE } from '../services/api/client.js'

import AppToast from '../components/AppToast.vue'
import LiveVideoPlayer from '../components/LiveVideoPlayer.vue'
import { useToast } from '../composables/useToast'
import {
  fetchRobotDetail,
  fetchRobotCommand,
  fetchRobotPersonDetections,
  fetchRobots,
  fetchRobotStatus,
  setRobotPersonDetection,
  sendRobotCommand,
} from '../services/api'

const robots = ref([])
const selectedRobot = ref(null)
const liveStatus = ref(null)
const robotListLoading = ref(true)
const robotListError = ref('')
const switchingRobot = ref(false)
const commandSending = ref(false)
const streamUnavailable = ref(false)
const activeHoldAction = ref('')
const speedMode = ref('medium')
const commandFeedback = ref('')
const personDetectionState = ref({ detections: [] })
const selectedPersonTrackId = ref('')
const followActive = ref(false)
const followStatus = ref('等待选择人员')
const personDetectionChanging = ref(false)

const statusPoller = useAsyncPoller((signal) => refreshStatus({ signal }), { intervalMs: 1_000 })
const personDetectionPoller = useAsyncPoller((signal) => refreshPersonDetections({ signal }), { intervalMs: 350 })
const followPoller = createAsyncPoller(() => followControlTick(), { intervalMs: 400 })
let followCommandInFlight = false
let targetLostSince = 0
let holdTimer = null
let holdAction = null
let holdPointerId = null
let holdTarget = null
let holdInFlight = false
let holdPromise = null
let statusRefreshing = false
let robotLoadController = null
let robotListLoadVersion = 0
let alertEventSource = null

const HOLD_REPEAT_MS = 150
const SPEED_MODES = [
  { id: 'micro', action: 'speed_micro', label: '微速', scale: 1.75 },
  { id: 'low', action: 'speed_slow', label: '低速', scale: 2.5 },
  { id: 'medium', action: 'speed_normal', label: '中速', scale: 3.5 },
  { id: 'high', action: 'speed_fast', label: '高速', scale: 5.0 },
]
const KEY_ACTIONS = {
  w: 'move_forward',
  ArrowUp: 'move_forward',
  s: 'move_backward',
  ArrowDown: 'move_backward',
  a: 'move_left',
  ArrowLeft: 'move_left',
  d: 'move_right',
  ArrowRight: 'move_right',
  q: 'turn_left',
  e: 'turn_right',
}
const { toastMessage, toastVariant, visible, showToast } = useToast()

const selectedRobotId = computed(() => selectedRobot.value?.id || '')
const livePlayUrls = computed(() => selectedRobot.value?.play_urls || {})
const liveSourceKey = computed(() => `${selectedRobot.value?.id || ''}\n${livePlayUrls.value.flv || ''}\n${livePlayUrls.value.hls || ''}`)
const hasLiveStream = computed(() => !streamUnavailable.value && Boolean(livePlayUrls.value.flv || livePlayUrls.value.hls))
const status = computed(() => liveStatus.value?.status || {})
const localizationQuality = computed(() => status.value?.localization_quality || {})
const localizationDecision = computed(() => localizationQuality.value?.decision || {})
const navigationStatus = computed(() => status.value?.navigation || {})
const canControl = computed(() => !followActive.value && selectedRobot.value?.id && !commandSending.value)
const motionControlState = computed(() => {
  const service = liveStatus.value?.status?.power_mode?.services?.controller_motion
  if (!service?.available) return '状态未知'
  return service.active ? '运控已启动' : '运控已停止'
})
const personDetections = computed(() => (personDetectionState.value?.detections || []).filter(
  (item) => String(item.label || '').toLowerCase() === 'person',
))
const selectedPerson = computed(() => personDetections.value.find((item) => item.track_id === selectedPersonTrackId.value) || null)
const personDetectionEnabled = computed(() => Boolean(personDetectionState.value?.enabled))
const recentAlerts = computed(() => selectedRobot.value?.recent_events || [])
const todayAlertCount = computed(() => Number(selectedRobot.value?.today_alerts || 0))

function showVideoNotice({ message, variant }) {
  showToast(message, variant ? { variant } : undefined)
}

function fallbackToSnapshot() {
  streamUnavailable.value = true
}

function applyRealtimeAlert(event, { created = false } = {}) {
  const robot = selectedRobot.value
  if (!event?.id || !robot || (event.robot_code && event.robot_code !== robot.code)) return
  const items = robot.recent_events || []
  const index = items.findIndex((item) => item.id === event.id)
  if (index >= 0) items[index] = event
  else if (created) {
    robot.recent_events = [event, ...items].slice(0, 5)
    robot.today_alerts = todayAlertCount.value + 1
    showToast('收到新的现场告警', { variant: 'alert', duration: 5200 })
  }
}

function openAlertStream() {
  const token = localStorage.getItem('inspection_token')
  if (!token || typeof EventSource === 'undefined') return
  alertEventSource?.close()
  alertEventSource = new EventSource(`${API_BASE}/events/stream/?token=${encodeURIComponent(token)}`)
  alertEventSource.addEventListener('inspection_event_created', (message) => {
    try { applyRealtimeAlert(JSON.parse(message.data || '{}').event, { created: true }) } catch {}
  })
  alertEventSource.addEventListener('inspection_event_updated', (message) => {
    try { applyRealtimeAlert(JSON.parse(message.data || '{}').event) } catch {}
  })
}

const motionActions = computed(() => [
  { action: 'move_forward', label: '前进', arrow: '↑', className: 'up', payload: { vx: roundSpeed(0.60) } },
  { action: 'move_left', label: '左移', arrow: '←', className: 'left', payload: { vy: roundSpeed(0.45) } },
  { action: 'move_right', label: '右移', arrow: '→', className: 'right', payload: { vy: -roundSpeed(0.45) } },
  { action: 'move_backward', label: '后退', arrow: '↓', className: 'down', payload: { vx: -roundSpeed(0.60) } },
])
const turnActions = computed(() => [
  { action: 'turn_left', label: '左转', arrow: '↶', className: 'left', payload: { yaw_rate: roundSpeed(1.05) } },
  { action: 'turn_right', label: '右转', arrow: '↷', className: 'right', payload: { yaw_rate: -roundSpeed(1.05) } },
])
const actionByName = computed(() => {
  const actions = [...motionActions.value, ...turnActions.value]
  return Object.fromEntries(actions.map((item) => [item.action, item]))
})

const selectedSpeedMode = computed(() => SPEED_MODES.find((item) => item.id === speedMode.value) || SPEED_MODES[2])
const speedModeLabel = computed(() => selectedSpeedMode.value.label)
const speedScale = computed(() => selectedSpeedMode.value.scale)

function roundSpeed(value) {
  return Number((value * speedScale.value).toFixed(3))
}

async function setSpeedMode(mode) {
  const selected = SPEED_MODES.find((item) => item.id === mode)
  if (!selected || !selectedRobot.value?.id || commandSending.value) return
  commandSending.value = true
  try {
    const command = await dispatchRobotAction(selected.action, {}, 'remote_control_speed_mode')
    await waitForRobotCommand(command, `${selected.label}档`)
    speedMode.value = selected.id
    showToast(`${selected.label}档已确认`)
  } catch (error) {
    commandFeedback.value = error.message || `${selected.label}档切换失败`
    showToast(commandFeedback.value)
  } finally {
    commandSending.value = false
  }
}

const expectedSpeedText = computed(() => {
  return `前后 ${roundSpeed(0.60).toFixed(1)} m/s，横移 ${roundSpeed(0.45).toFixed(1)} m/s，转向 ${roundSpeed(1.05).toFixed(1)} rad/s`
})
const taskId = computed(() => status.value?.task_execution_id || liveStatus.value?.task_execution_id || '')
const localizationSourceLabels = {
  ndt_imu: 'NDT + IMU',
  rtk_imu: 'RTK + IMU',
  imu_odom_bridge: 'IMU + 里程计兜底',
  unavailable: '定位不可用',
}
const localizationModeLabel = computed(() => {
  const mode = String(localizationDecision.value?.preferred_source || '').toLowerCase()
  return mode === 'rtk' ? 'RTK优先' : mode === 'ndt' ? 'NDT优先' : '未收到任务策略'
})
const activeLocalizationLabel = computed(() => (
  localizationSourceLabels[localizationDecision.value?.active_source] || '未确定'
))
const rtkQualityLabel = computed(() => localizationDecision.value?.rtk_quality || '未知')

function formatNumber(value, digits = 2, suffix = '') {
  if (value === undefined || value === null || value === '' || Number.isNaN(Number(value))) return '--'
  return `${Number(value).toFixed(digits)}${suffix}`
}

function statusText(value) {
  if (!value) return 'unknown'
  const labels = {
    normal: '正常',
    lost: '丢失',
    initializing: '初始化',
    relocalizing: '重定位',
    relocalized: '重定位成功',
    online: '在线',
    offline: '离线',
  }
  return labels[value] || value
}

function formatSampleTime(value) {
  if (!value) return '--'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return value
  return date.toLocaleTimeString('zh-CN', { hour: '2-digit', minute: '2-digit', second: '2-digit' })
}

function qualityValue(key) {
  const value = localizationQuality.value?.[key]
  if (value === undefined || value === null || value === '') return '--'
  return typeof value === 'number' ? value.toFixed(key.includes('score') || key.includes('error') ? 3 : 2) : value
}

async function dispatchRobotAction(action, payload = {}, source = 'remote_control') {
  const robot = selectedRobot.value
  if (!robot?.id) return false
  return sendRobotCommand(robot.id, {
    action,
    payload: {
      source,
      ...payload,
    },
  })
}

async function waitForRobotCommand(command, label, timeoutMs = 12000) {
  const robotId = selectedRobot.value?.id
  if (!robotId || !command?.id) return command
  const terminalStates = new Set(['succeeded', 'failed', 'rejected', 'cancelled', 'timed_out', 'expired'])
  let result = command
  const deadline = Date.now() + timeoutMs
  while (!terminalStates.has(result?.status) && Date.now() < deadline) {
    commandFeedback.value = `${label} · ${result?.status || '下发中'}`
    await new Promise((resolve) => window.setTimeout(resolve, 300))
    result = await fetchRobotCommand(robotId, command.id)
  }
  if (result?.status !== 'succeeded') {
    throw new Error(result?.error_message || result?.ack_reason_message || `${label}未获设备确认`)
  }
  commandFeedback.value = `${label} · 设备已确认`
  return result
}

async function sendStop(source = 'remote_control_stop') {
  if (!selectedRobot.value?.id) return
  try {
    await dispatchRobotAction('move_stop', {}, source)
  } catch (error) {
    showToast(error.message || '停止指令失败')
  }
}

async function refreshPersonDetections({ signal } = {}) {
  if (!selectedRobot.value?.id) return
  try {
    personDetectionState.value = await fetchRobotPersonDetections(selectedRobot.value.id, { signal })
  } catch {
    personDetectionState.value = { detections: [] }
  }
}

async function togglePersonDetection() {
  if (!selectedRobot.value?.id || personDetectionChanging.value || followActive.value) return
  personDetectionChanging.value = true
  const enabled = !personDetectionEnabled.value
  try {
    await setRobotPersonDetection(selectedRobot.value.id, enabled)
    selectedPersonTrackId.value = ''
    personDetectionState.value = { ...personDetectionState.value, enabled, detections: [] }
    followStatus.value = enabled ? '人员识别已开启，等待首个有效检测框' : '人员识别已关闭'
    showToast(enabled ? '人员跟踪识别已开启' : '人员跟踪识别已关闭')
  } catch (error) {
    showToast(error.message || '人员识别状态切换失败')
  } finally {
    personDetectionChanging.value = false
  }
}

function detectionStyle(item) {
  const frameWidth = personDetectionState.value?.frame_width || 1
  const frameHeight = personDetectionState.value?.frame_height || 1
  const box = item.bbox || {}
  return {
    left: `${(Number(box.x || 0) / frameWidth) * 100}%`,
    top: `${(Number(box.y || 0) / frameHeight) * 100}%`,
    width: `${(Number(box.width || 0) / frameWidth) * 100}%`,
    height: `${(Number(box.height || 0) / frameHeight) * 100}%`,
  }
}

function selectPerson(item) {
  if (followActive.value) return
  selectedPersonTrackId.value = item.track_id
  followStatus.value = `已选择 ${item.track_id}，可以开始跟随`
}

function calculateFollowVelocity(person) {
  const frameWidth = Number(personDetectionState.value?.frame_width || 1)
  const frameHeight = Number(personDetectionState.value?.frame_height || 1)
  const box = person.bbox || {}
  const centerError = (Number(box.x || 0) + Number(box.width || 0) / 2) / frameWidth - 0.5
  const heightRatio = Number(box.height || 0) / frameHeight
  let vx = 0
  if (heightRatio < 0.40) vx = Math.min(0.16, (0.40 - heightRatio) * 0.7)
  else if (heightRatio > 0.62) vx = Math.max(-0.08, (0.62 - heightRatio) * 0.5)
  const yawRate = Math.abs(centerError) < 0.07 ? 0 : Math.max(-0.28, Math.min(0.28, -centerError * 0.75))
  if (Math.abs(centerError) > 0.32) vx = 0
  return { vx: Number(vx.toFixed(3)), vy: 0, yaw_rate: Number(yawRate.toFixed(3)) }
}

async function followControlTick() {
  if (!followActive.value || followCommandInFlight) return
  const person = selectedPerson.value
  if (!person || personDetectionState.value?.stale) {
    if (!targetLostSince) targetLostSince = Date.now()
    followStatus.value = '目标暂时丢失，已停车等待恢复'
    followCommandInFlight = true
    try { await sendStop('person_follow_target_lost') } finally { followCommandInFlight = false }
    if (Date.now() - targetLostSince >= 2000) await stopFollowing('目标丢失超过 2 秒，跟随已停止')
    return
  }
  targetLostSince = 0
  const velocity = calculateFollowVelocity(person)
  followStatus.value = `跟随中 · 前进 ${velocity.vx.toFixed(2)} m/s · 转向 ${velocity.yaw_rate.toFixed(2)} rad/s`
  followCommandInFlight = true
  try {
    await dispatchRobotAction('move_velocity', velocity, 'person_follow')
  } catch (error) {
    await stopFollowing(error.message || '跟随指令失败')
  } finally {
    followCommandInFlight = false
  }
}

async function toggleFollowing() {
  if (followActive.value) {
    await stopFollowing('跟随已停止')
    return
  }
  await startFollowing()
}

async function startFollowing() {
  if (!selectedPersonTrackId.value || followActive.value || commandSending.value) return
  commandSending.value = true
  try {
    const command = await dispatchRobotAction('stand_up', {}, 'person_follow_enter')
    await waitForRobotCommand(command, '起立')
    followActive.value = true
    targetLostSince = 0
    followStatus.value = '跟随已启动'
    await followControlTick()
    followPoller.start()
    showToast('人员跟随已启动，请保持现场通道畅通')
  } catch (error) {
    followStatus.value = error.message || '启动跟随失败'
    showToast(followStatus.value)
  } finally {
    commandSending.value = false
  }
}

async function stopFollowing(message = '跟随已停止') {
  const wasActive = followActive.value
  followActive.value = false
  followPoller.stop()
  targetLostSince = 0
  followStatus.value = message
  if (wasActive) await sendStop('person_follow_stop')
}

async function sendDiscreteAction(action, label) {
  if (!selectedRobot.value?.id || commandSending.value) return
  commandSending.value = true
  try {
    const command = await dispatchRobotAction(action, {}, 'remote_control_action')
    await waitForRobotCommand(command, label)
    showToast(`${label}已确认`)
  } catch (error) {
    commandFeedback.value = error.message || `${label}失败`
    showToast(commandFeedback.value)
  } finally {
    commandSending.value = false
  }
}

async function setMotionControl(action, label) {
  if (!selectedRobot.value?.id || commandSending.value) return
  commandSending.value = true
  try {
    const command = await dispatchRobotAction(action, {}, 'remote_motion_runtime')
    await waitForRobotCommand(command, label, 30000)
    await refreshStatus()
    showToast(`${label}已确认`)
  } catch (error) {
    commandFeedback.value = error.message || `${label}失败`
    showToast(commandFeedback.value)
  } finally {
    commandSending.value = false
  }
}

async function emergencyStop() {
  if (!selectedRobot.value?.id || commandSending.value) return
  commandSending.value = true
  try {
    await stopFollowing('阻尼已触发')
    await stopHoldAction()
    const command = await dispatchRobotAction('passive', { note: 'Enter damping/passive mode.' }, 'remote_control_damping')
    await waitForRobotCommand(command, '阻尼')
    showToast('设备已进入阻尼')
  } catch (error) {
    commandFeedback.value = error.message || '阻尼失败'
    showToast(commandFeedback.value)
  } finally {
    commandSending.value = false
  }
}

async function sendHeldAction() {
  if (!holdAction || holdInFlight) return
  holdInFlight = true
  try {
    holdPromise = dispatchRobotAction(holdAction.action, holdAction.payload || {}, 'remote_control_hold')
    await holdPromise
  } catch (error) {
    await stopHoldAction()
    showToast(error.message || '运动指令失败')
  } finally {
    holdPromise = null
    holdInFlight = false
  }
}

function startHoldAction(item, event = null) {
  if (!canControl.value || holdAction) return
  holdAction = item
  holdPointerId = event?.pointerId ?? null
  holdTarget = event?.currentTarget ?? null
  activeHoldAction.value = item.action
  try {
    holdTarget?.setPointerCapture?.(holdPointerId)
  } catch {}
  sendHeldAction()
  holdTimer = window.setInterval(sendHeldAction, HOLD_REPEAT_MS)
}

async function stopHoldAction(event = null) {
  if (!holdAction) return
  if (event?.pointerId != null && holdPointerId != null && event.pointerId !== holdPointerId) return

  const previousTarget = holdTarget
  const previousPointerId = holdPointerId
  const pendingHoldPromise = holdPromise
  window.clearInterval(holdTimer)
  holdTimer = null
  holdAction = null
  holdPointerId = null
  holdTarget = null
  activeHoldAction.value = ''
  try {
    previousTarget?.releasePointerCapture?.(previousPointerId)
  } catch {}
  await pendingHoldPromise?.catch(() => {})
  await sendStop('remote_control_hold_release')
}

async function chooseRobot(robotId) {
  if (!robotId || switchingRobot.value || selectedRobot.value?.id === robotId) return
  switchingRobot.value = true
  robotLoadController?.abort()
  robotLoadController = new AbortController()
  const { signal } = robotLoadController
  try {
    await stopHoldAction()
    await stopFollowing('已切换设备')
    if (personDetectionEnabled.value && selectedRobot.value?.id) {
      await setRobotPersonDetection(selectedRobot.value.id, false).catch(() => {})
    }
    const listRobot = robots.value.find((robot) => String(robot.id) === String(robotId))
    selectedRobot.value = listRobot || { id: robotId }
    liveStatus.value = null
    personDetectionState.value = { detections: [] }
    streamUnavailable.value = false
    const [detailResult, statusResult, detectionResult] = await Promise.allSettled([
      fetchRobotDetail(robotId, { signal }),
      fetchRobotStatus(robotId, { signal }),
      fetchRobotPersonDetections(robotId, { signal }),
    ])
    if (detailResult.status === 'fulfilled') selectedRobot.value = detailResult.value
    if (statusResult.status === 'fulfilled') liveStatus.value = statusResult.value
    if (detectionResult.status === 'fulfilled') personDetectionState.value = detectionResult.value
    const firstError = [detailResult, statusResult, detectionResult]
      .find((result) => result.status === 'rejected' && result.reason?.name !== 'AbortError')
    if (firstError) throw firstError.reason
  } catch (error) {
    if (error?.name !== 'AbortError') showToast(error.message || '切换设备失败')
  } finally {
    if (robotLoadController?.signal === signal) {
      switchingRobot.value = false
      robotLoadController = null
    }
  }
}

async function refreshStatus({ signal } = {}) {
  if (!selectedRobot.value?.id || statusRefreshing) return
  statusRefreshing = true
  try {
    liveStatus.value = await fetchRobotStatus(selectedRobot.value.id, { signal })
  } catch {} finally {
    statusRefreshing = false
  }
}

async function loadRobotList({ force = false } = {}) {
  const version = ++robotListLoadVersion
  robotListLoading.value = true
  robotListError.value = ''
  try {
    const result = await fetchRobots({ force })
    if (version !== robotListLoadVersion) return
    robots.value = Array.isArray(result) ? result : []
    if (robots.value[0]?.id) void chooseRobot(robots.value[0].id)
  } catch (error) {
    if (version !== robotListLoadVersion) return
    robotListError.value = error?.message || '设备列表加载失败'
  } finally {
    if (version === robotListLoadVersion) robotListLoading.value = false
  }
}

function handleKeyDown(event) {
  if (event.repeat || event.target?.tagName === 'INPUT') return
  const action = KEY_ACTIONS[event.key]
  if (!action) return
  event.preventDefault()
  const item = actionByName.value[action]
  if (item) startHoldAction(item)
}

function handleKeyUp(event) {
  if (!KEY_ACTIONS[event.key]) return
  event.preventDefault()
  stopHoldAction()
}

function handleVisibilityChange() {
  if (document.hidden) stopHoldAction()
}

function preventRemoteGesture(event) {
  if (event.target?.closest?.('.remote-control-page')) {
    event.preventDefault()
  }
}

onMounted(() => {
  window.addEventListener('keydown', handleKeyDown)
  window.addEventListener('keyup', handleKeyUp)
  window.addEventListener('blur', stopHoldAction)
  document.addEventListener('visibilitychange', handleVisibilityChange)
  document.addEventListener('contextmenu', preventRemoteGesture, { capture: true })
  document.addEventListener('selectstart', preventRemoteGesture, { capture: true })
  document.addEventListener('dragstart', preventRemoteGesture, { capture: true })
  openAlertStream()
  // Render the control shell immediately. Device discovery is independent of
  // the page layout, and must not leave the whole remote-control page blank
  // while the cloud request is waiting on a slow network.
  void loadRobotList()
})

onBeforeUnmount(async () => {
  window.removeEventListener('keydown', handleKeyDown)
  window.removeEventListener('keyup', handleKeyUp)
  window.removeEventListener('blur', stopHoldAction)
  document.removeEventListener('visibilitychange', handleVisibilityChange)
  document.removeEventListener('contextmenu', preventRemoteGesture, { capture: true })
  document.removeEventListener('selectstart', preventRemoteGesture, { capture: true })
  document.removeEventListener('dragstart', preventRemoteGesture, { capture: true })
  statusPoller.stop()
  personDetectionPoller.stop()
  followPoller.stop()
  robotListLoadVersion += 1
  robotLoadController?.abort()
  robotLoadController = null
  alertEventSource?.close()
  alertEventSource = null
  await stopFollowing('页面关闭，跟随已停止')
  await stopHoldAction()
})

watch(liveSourceKey, () => {
  streamUnavailable.value = false
})
</script>

<template>
  <section class="remote-control-page">
    <div class="remote-main">
      <section class="panel remote-video-panel">
        <div class="panel-head">
          <div>
            <h3>远程视频操控</h3>
            <p>按住方向键持续移动，松开立即停止；键盘支持 W/A/S/D 和 Q/E。</p>
          </div>
          <span class="remote-state ok">
            遥控协议在线
          </span>
        </div>

        <div class="remote-video-stage">
          <LiveVideoPlayer
            :play-urls="livePlayUrls"
            :robot-id="selectedRobot?.id"
            :available="hasLiveStream"
            @notice="showVideoNotice"
            @stream-error="fallbackToSnapshot"
          >
            <template #empty>
              <div class="remote-video-source remote-no-signal">
                <strong>无视频流</strong>
                <span>{{ selectedRobot?.stream_id || '当前设备未上报 FLV/HLS 播放地址' }}</span>
              </div>
            </template>
            <template #overlay>
              <button
                v-for="person in personDetections"
                :key="person.track_id"
                type="button"
                class="person-detection-box"
                :class="{ selected: selectedPersonTrackId === person.track_id, following: followActive && selectedPersonTrackId === person.track_id }"
                :style="detectionStyle(person)"
                :disabled="followActive"
                @click="selectPerson(person)"
              >
                <span>{{ person.track_id }} · {{ Math.round(person.confidence * 100) }}%</span>
              </button>
              <div class="remote-video-overlay">
                <span>{{ selectedRobot?.code || '--' }}</span>
                <strong>{{ selectedRobot?.location || '未知位置' }}</strong>
              </div>
            </template>
          </LiveVideoPlayer>
        </div>
        <div class="person-follow-toolbar">
          <div>
            <strong>人员跟随</strong>
            <span>{{ personDetectionEnabled ? (personDetectionState?.available ? `识别到 ${personDetections.length} 人` : (personDetectionState?.stale ? '人员识别数据延迟' : '等待人员识别首帧')) : '人员模型未启用' }}</span>
            <small>{{ followStatus }}</small>
          </div>
          <button class="follow-start-btn" type="button" :disabled="personDetectionChanging || followActive" @click="togglePersonDetection">
            {{ personDetectionEnabled ? '关闭识别' : '开启跟踪识别' }}
          </button>
          <button class="follow-start-btn" type="button" :disabled="personDetectionChanging || commandSending || (!selectedPerson && !followActive)" @click="toggleFollowing">
            {{ followActive ? '停止跟随' : '开始跟随' }}
          </button>
        </div>
        <section class="remote-alert-panel" aria-label="实时告警">
          <div><strong>实时告警</strong><span>今日 {{ todayAlertCount }} 条</span></div>
          <article v-if="recentAlerts[0]" class="remote-alert-main">
            <img v-if="recentAlerts[0].annotated_snapshot_url || recentAlerts[0].snapshot_url" :src="recentAlerts[0].annotated_snapshot_url || recentAlerts[0].snapshot_url" :alt="recentAlerts[0].title" />
            <span><strong>{{ recentAlerts[0].title || recentAlerts[0].event_type }}</strong><small>{{ recentAlerts[0].detected_at }}</small></span>
          </article>
          <p v-else>当前没有新的现场告警</p>
          <small v-for="event in recentAlerts.slice(1, 4)" :key="event.id">{{ event.title || event.event_type }}</small>
        </section>
      </section>

      <section class="panel remote-console">
        <div class="remote-pad-wrap">
          <div class="remote-pad">
            <button
              v-for="item in motionActions"
              :key="item.action"
              type="button"
              :class="['remote-pad-btn', item.className, { active: activeHoldAction === item.action }]"
              :disabled="!canControl"
              :aria-label="item.label"
              @pointerdown.prevent="startHoldAction(item, $event)"
              @pointerup.prevent="stopHoldAction($event)"
              @pointercancel.prevent="stopHoldAction($event)"
              @lostpointercapture="stopHoldAction($event)"
              @contextmenu.prevent
            >
              <span class="control-glyph" aria-hidden="true"></span>
            </button>
          </div>
        </div>

        <div class="remote-turn">
            <button
              v-for="item in turnActions"
              :key="item.action"
              type="button"
              :class="['remote-turn-btn', item.className, { active: activeHoldAction === item.action }]"
              :disabled="!canControl"
              @pointerdown.prevent="startHoldAction(item, $event)"
              @pointerup.prevent="stopHoldAction($event)"
              @pointercancel.prevent="stopHoldAction($event)"
              @lostpointercapture="stopHoldAction($event)"
              @contextmenu.prevent
            >
              <span class="turn-glyph" aria-hidden="true"></span>
              <span class="turn-label">{{ item.label }}</span>
            </button>
        </div>

        <div class="remote-controls">
          <div class="speed-mode-control">
            <span>速度档位</span>
            <strong>{{ speedModeLabel }}</strong>
            <div class="speed-mode-buttons" role="group" aria-label="速度档位">
              <button
                v-for="mode in SPEED_MODES"
                :key="mode.id"
                type="button"
                :class="['speed-mode-button', { active: speedMode === mode.id }]"
                :disabled="commandSending || !selectedRobot"
                @click="setSpeedMode(mode.id)"
              >
                {{ mode.label }}
              </button>
            </div>
            <small class="speed-estimate">预计速度：{{ expectedSpeedText }}</small>
          </div>
          <div class="remote-actions remote-special-actions">
            <button class="danger-btn" type="button" :disabled="commandSending || !selectedRobot" @click="emergencyStop">阻尼</button>
            <button class="ghost-btn" type="button" :disabled="commandSending || !selectedRobot" @click="sendDiscreteAction('two_leg_stand', '双腿站立')">
              双腿站立
            </button>
            <button class="ghost-btn" type="button" :disabled="commandSending || !selectedRobot" @click="sendDiscreteAction('shake_hand', '打招呼')">
              打招呼
            </button>
          </div>
          <div class="remote-actions">
            <button class="ghost-btn" type="button" :disabled="commandSending || !selectedRobot" @click="setMotionControl('motion_start', '启动运控')">启动运控</button>
            <button class="danger-btn" type="button" :disabled="commandSending || !selectedRobot" @click="setMotionControl('motion_stop', '停止运控')">停止运控</button>
            <span class="remote-feedback">{{ motionControlState }}</span>
          </div>
          <div class="remote-actions">
            <button class="ghost-btn" type="button" :disabled="commandSending || !selectedRobot" @click="sendDiscreteAction('stand_up', '起立')">
              起立
            </button>
            <button class="danger-btn" type="button" :disabled="commandSending || !selectedRobot" @click="sendDiscreteAction('lie_down', '匍匐')">
              匍匐
            </button>
          </div>
          <small v-if="commandFeedback" class="remote-feedback">{{ commandFeedback }}</small>
        </div>
      </section>
    </div>

    <aside class="remote-side">
      <section class="panel">
        <div class="panel-head">
          <div>
            <h3>设备选择</h3>
            <p>{{ switchingRobot ? '正在切换设备...' : '切换后可直接发送遥控器指令' }}</p>
          </div>
        </div>
        <div v-if="robotListLoading" class="remote-list-state">正在加载设备列表…</div>
        <div v-else-if="robotListError" class="remote-list-state remote-list-error">
          <span>{{ robotListError }}</span>
          <button type="button" class="ghost-btn" @click="loadRobotList({ force: true })">重试</button>
        </div>
        <div v-else-if="!robots.length" class="remote-list-state">暂无可用机器人</div>
        <div v-else class="robot-list compact">
          <button
            v-for="robot in robots"
            :key="robot.id"
            type="button"
            class="robot-item"
            :class="{ selected: selectedRobotId === robot.id }"
            :disabled="switchingRobot"
            @click="chooseRobot(robot.id)"
          >
            <div class="robot-main">
              <span class="robot-code">{{ robot.code }}</span>
              <strong>{{ robot.name }}</strong>
              <span>{{ robot.location }}</span>
            </div>
            <div class="robot-side">
              <span :class="['robot-status', robot.status]">{{ robot.status_label }}</span>
              <span class="robot-battery">{{ robot.battery_level }}%</span>
            </div>
          </button>
        </div>
      </section>

      <section class="panel remote-status-panel">
        <div class="panel-head">
          <div>
            <h3>实时状态</h3>
            <p>2 秒刷新一次机器人上报状态</p>
          </div>
        </div>
        <div class="remote-status-grid">
          <div>
            <span>连接</span>
            <strong>{{ statusText(liveStatus?.connection_status) }}</strong>
          </div>
          <div>
            <span>定位</span>
            <strong>{{ statusText(liveStatus?.localization_status || status.localization_status) }}</strong>
          </div>
          <div>
            <span>导航</span>
            <strong>{{ liveStatus?.nav_ready || status.nav_ready ? 'ready' : 'not ready' }}</strong>
          </div>
          <div>
            <span>当前任务</span>
            <strong :title="taskId || '无执行中任务'">{{ taskId ? `${taskId.slice(0, 8)}…` : '无执行中任务' }}</strong>
          </div>
          <div>
            <span>任务定位策略</span>
            <strong>{{ localizationModeLabel }}</strong>
          </div>
          <div>
            <span>当前定位源</span>
            <strong>{{ activeLocalizationLabel }}</strong>
          </div>
          <div>
            <span>控制</span>
            <strong>{{ statusText(status.control_mode || selectedRobot?.control_mode) }}</strong>
          </div>
          <div>
            <span>电量</span>
            <strong>{{ status.battery_percent ?? selectedRobot?.battery_level ?? '--' }}%</strong>
          </div>
          <div>
            <span>NDT分数</span>
            <strong>{{ qualityValue('matching_error') }}</strong>
          </div>
          <div>
            <span>内点率</span>
            <strong>{{ qualityValue('inlier_fraction') }}</strong>
          </div>
          <div>
            <span>RTK质量 / XY</span>
            <strong>{{ rtkQualityLabel }} · {{ formatNumber(localizationDecision.rtk_x) }}, {{ formatNumber(localizationDecision.rtk_y) }}</strong>
          </div>
          <div>
            <span>RTK航向 / 稳定</span>
            <strong>{{ formatNumber(localizationDecision.rtk_yaw, 3, ' rad') }} · {{ localizationDecision.absolute_stable ? '已稳定' : '未稳定' }}</strong>
          </div>
          <div>
            <span>前方障碍</span>
            <strong>{{ formatNumber(navigationStatus.front_obstacle_distance_m, 2, ' m') }}</strong>
          </div>
          <div>
            <span>兜底状态</span>
            <strong>{{ localizationDecision.bridge_rejection_reason || (localizationDecision.active_source === 'imu_odom_bridge' ? `${formatNumber(localizationDecision.bridge_distance_m)} m` : '未启用') }}</strong>
          </div>
          <div>
            <span>速度</span>
            <strong>{{ status.speed_mps ?? '--' }} m/s</strong>
          </div>
          <div>
            <span>采样</span>
            <strong>{{ formatSampleTime(status.sampled_at) }}</strong>
          </div>
        </div>
      </section>
    </aside>

    <AppToast :show="visible" :message="toastMessage" :variant="toastVariant" />
  </section>
</template>

<style scoped>
.remote-control-page {
  display: grid;
  grid-template-columns: minmax(0, 1fr) minmax(224px, 272px);
  gap: 16px;
  align-items: start;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
  touch-action: manipulation;
}

.remote-main,
.remote-side {
  display: grid;
  min-width: 0;
  align-content: start;
  gap: 16px;
}

.remote-video-stage {
  position: relative;
  min-height: 0;
  aspect-ratio: 16 / 9;
  border: 1px solid rgba(120, 194, 255, 0.22);
  border-radius: 18px;
  overflow: hidden;
  background: #06111f;
}

.person-detection-box {
  position: absolute;
  z-index: 3;
  border: 3px solid #42e5ff;
  border-radius: 8px;
  padding: 0;
  color: white;
  background: rgba(20, 190, 235, 0.08);
  cursor: crosshair;
  box-shadow: 0 0 0 1px rgba(3, 14, 25, 0.8), 0 0 18px rgba(66, 229, 255, 0.28);
}

.person-detection-box span {
  position: absolute;
  left: -3px;
  top: -29px;
  padding: 4px 7px;
  border-radius: 6px 6px 6px 0;
  font-size: 11px;
  font-weight: 900;
  white-space: nowrap;
  background: #0787aa;
}

.person-detection-box.selected,
.person-detection-box.following {
  border-color: #ffcc36;
  background: rgba(255, 204, 54, 0.14);
  box-shadow: 0 0 0 1px rgba(3, 14, 25, 0.8), 0 0 24px rgba(255, 204, 54, 0.55);
}

.person-detection-box.selected span,
.person-detection-box.following span {
  color: #182332;
  background: #ffcc36;
}

.person-follow-toolbar {
  display: grid;
  grid-template-columns: minmax(0, 1fr) auto auto;
  gap: 12px;
  align-items: center;
  margin-top: 14px;
  padding: 14px;
  border: 1px solid rgba(66, 229, 255, 0.3);
  border-radius: 14px;
  background: rgba(66, 229, 255, 0.07);
}

.person-follow-toolbar > div {
  display: grid;
  gap: 4px;
}

.person-follow-toolbar span,
.person-follow-toolbar small {
  color: var(--muted);
}

.follow-start-btn {
  min-height: 44px;
  border: 1px solid rgba(255, 204, 54, 0.65);
  border-radius: 999px;
  padding: 0 20px;
  color: #17202c;
  font-weight: 900;
  background: #ffcc36;
}

.remote-video-source {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  object-fit: cover;
}

.remote-no-signal {
  display: grid;
  place-content: center;
  gap: 10px;
  text-align: center;
  color: rgba(233, 247, 255, 0.8);
  background:
    linear-gradient(135deg, rgba(7, 19, 36, 0.92), rgba(20, 43, 70, 0.82)),
    repeating-linear-gradient(0deg, rgba(255, 255, 255, 0.05) 0 1px, transparent 1px 5px);
}

.remote-no-signal strong {
  color: #f4fbff;
  font-size: 28px;
}

.remote-video-overlay {
  position: absolute;
  left: 18px;
  top: 18px;
  display: grid;
  gap: 5px;
  max-width: min(420px, calc(100% - 36px));
  padding: 12px 14px;
  border: 1px solid rgba(146, 197, 255, 0.18);
  border-radius: 12px;
  color: #f4fbff;
  background: rgba(4, 13, 24, 0.48);
  backdrop-filter: blur(12px);
}

.remote-video-overlay span,
.remote-video-overlay strong {
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.remote-state {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  min-width: 82px;
  border-radius: 999px;
  padding: 8px 12px;
  font-size: 12px;
  font-weight: 900;
  background: rgba(131, 146, 170, 0.14);
}

.remote-state.ok {
  color: #075f43;
  background: rgba(82, 223, 154, 0.18);
}

.remote-console {
  display: grid;
  grid-template-columns: minmax(244px, 0.9fr) minmax(120px, 140px) minmax(250px, 1fr);
  gap: 16px;
  align-items: center;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
  touch-action: none;
}

.remote-pad-wrap {
  display: flex;
  justify-content: center;
  align-items: center;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
  touch-action: none;
}

.remote-pad {
  display: grid;
  grid-template-columns: repeat(3, 64px);
  grid-template-rows: repeat(3, 64px);
  gap: 10px;
  padding: 14px;
  border: 1px solid var(--line);
  border-radius: 18px;
  background: var(--panel-soft);
  touch-action: none;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
}

.remote-pad-btn,
.remote-turn-btn,
.remote-stop {
  border: 1px solid var(--line);
  color: var(--text);
  background: var(--table-bg);
  font-weight: 900;
  touch-action: none;
  -webkit-tap-highlight-color: transparent;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
}

.remote-pad-btn {
  display: grid;
  place-items: center;
  width: 64px;
  height: 64px;
  border-radius: 50%;
}

.remote-pad-btn.up { grid-area: 1 / 2; }
.remote-pad-btn.left { grid-area: 2 / 1; }
.remote-pad-btn.right { grid-area: 2 / 3; }
.remote-pad-btn.down { grid-area: 3 / 2; }

.control-glyph {
  display: block;
  width: 0;
  height: 0;
  pointer-events: none;
}

.remote-pad-btn.up .control-glyph {
  border-left: 13px solid transparent;
  border-right: 13px solid transparent;
  border-bottom: 22px solid currentColor;
}

.remote-pad-btn.down .control-glyph {
  border-left: 13px solid transparent;
  border-right: 13px solid transparent;
  border-top: 22px solid currentColor;
}

.remote-pad-btn.left .control-glyph {
  border-top: 13px solid transparent;
  border-bottom: 13px solid transparent;
  border-right: 22px solid currentColor;
}

.remote-pad-btn.right .control-glyph {
  border-top: 13px solid transparent;
  border-bottom: 13px solid transparent;
  border-left: 22px solid currentColor;
}

.remote-pad-btn:hover,
.remote-pad-btn.active,
.remote-turn-btn:hover,
.remote-turn-btn.active {
  border-color: rgba(67, 213, 255, 0.45);
  background: rgba(67, 213, 255, 0.14);
}

.remote-pad-btn:disabled,
.remote-turn-btn:disabled {
  cursor: not-allowed;
  opacity: 0.42;
}

.remote-turn {
  display: grid;
  grid-template-columns: 1fr;
  gap: 12px;
  min-width: 0;
}

.remote-turn-btn {
  min-height: 64px;
  border-radius: 999px;
  padding: 8px 10px;
  display: inline-flex;
  place-items: center;
  justify-content: center;
  flex-direction: column;
  gap: 10px;
}

.turn-glyph {
  position: relative;
  display: block;
  width: 28px;
  height: 28px;
  border: 4px solid currentColor;
  border-right-color: transparent;
  border-radius: 50%;
  pointer-events: none;
}

.turn-glyph::after {
  content: "";
  position: absolute;
  top: -7px;
  left: 12px;
  width: 0;
  height: 0;
  border-left: 7px solid transparent;
  border-right: 7px solid transparent;
  border-bottom: 10px solid currentColor;
  transform: rotate(34deg);
}

.remote-turn-btn.right .turn-glyph {
  transform: scaleX(-1);
}

.remote-turn-btn.left {
  color: #46d7ff;
}

.remote-turn-btn.right {
  color: #ffcc36;
}

.turn-label {
  font-size: 13px;
  font-weight: 900;
  white-space: nowrap;
}

.sr-only {
  position: absolute;
  width: 1px;
  height: 1px;
  padding: 0;
  margin: -1px;
  overflow: hidden;
  clip: rect(0, 0, 0, 0);
  white-space: nowrap;
  border: 0;
}

.remote-controls {
  display: grid;
  gap: 16px;
}

.speed-mode-control {
  display: grid;
  grid-template-columns: 1fr auto;
  gap: 8px 12px;
  color: var(--muted);
  font-size: 13px;
}

.speed-mode-control strong {
  color: var(--text);
}

.speed-estimate {
  grid-column: 1 / -1;
  color: var(--muted);
  line-height: 1.4;
}

.speed-mode-buttons {
  grid-column: 1 / -1;
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 8px;
}

.speed-mode-button {
  min-height: 36px;
  border: 1px solid var(--border);
  border-radius: 6px;
  background: var(--panel);
  color: var(--muted);
  font: inherit;
}

.speed-mode-button.active {
  border-color: var(--accent);
  background: color-mix(in srgb, var(--accent) 14%, var(--panel));
  color: var(--text);
}

.speed-mode-button:disabled {
  cursor: not-allowed;
  opacity: 0.55;
}

.remote-actions {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 10px;
}

.remote-actions button,
.remote-stop {
  min-height: 44px;
}

.remote-stop {
  border-radius: 999px;
  background: rgba(67, 213, 255, 0.12);
}

.remote-feedback {
  color: var(--muted);
  line-height: 1.5;
}

.remote-status-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px;
}

.remote-status-grid div {
  display: grid;
  gap: 6px;
  min-width: 0;
  padding: 13px;
  border-radius: 12px;
  background: var(--panel-soft);
}

.remote-status-grid span {
  color: var(--muted);
  font-size: 12px;
}

.remote-status-grid strong {
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.remote-list-state {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  min-height: 72px;
  padding: 14px;
  border-radius: 12px;
  color: var(--muted);
  background: var(--panel-soft);
  font-size: 13px;
}

.remote-list-error {
  color: var(--danger, #d14b4b);
}

.robot-list.compact {
  display: grid;
  gap: 12px;
  max-height: none;
  overflow: visible;
}

@media (min-width: 1181px) {
  .remote-video-stage {
    height: min(52dvh, 620px);
    aspect-ratio: auto;
  }

  .remote-side {
    gap: 13px;
    font-size: 12px;
  }

  .remote-side .panel {
    padding: 18px;
  }

  .remote-side .panel-head {
    gap: 8px;
  }

  .remote-side .panel-head h3 {
    font-size: 17px;
  }

  .remote-side .panel-head p {
    font-size: 11px;
    line-height: 1.4;
  }

  .remote-side .robot-list.compact {
    gap: 10px;
  }

  .remote-side .robot-item {
    display: grid;
    grid-template-columns: minmax(0, 1fr) auto;
    gap: 8px;
    padding: 12px;
    font-size: 11px;
  }

  .remote-side .robot-main,
  .remote-side .robot-side {
    min-width: 0;
  }

  .remote-side .remote-status-grid {
    gap: 9px;
  }

  .remote-side .remote-status-grid div {
    gap: 4px;
    padding: 10px;
  }

  .remote-side .remote-status-grid span {
    font-size: 10px;
  }

  .remote-side .remote-status-grid strong {
    font-size: 11px;
  }

  .remote-side .remote-list-state {
    min-height: 58px;
    gap: 8px;
    padding: 11px;
    font-size: 11px;
  }
}

.remote-alert-panel { display: grid; gap: 8px; margin-top: 14px; padding-top: 14px; border-top: 1px solid var(--line); }
.remote-alert-panel > div { display: flex; justify-content: space-between; gap: 10px; }
.remote-alert-panel > div span, .remote-alert-panel > p, .remote-alert-panel > small, .remote-alert-main small { color: var(--muted); font-size: 12px; }
.remote-alert-panel > p { margin: 0; }
.remote-alert-main { display: grid; grid-template-columns: 70px minmax(0, 1fr); align-items: center; gap: 9px; }
.remote-alert-main img { width: 70px; height: 50px; object-fit: cover; border-radius: 5px; background: #172b37; }
.remote-alert-main span { display: grid; gap: 3px; min-width: 0; }
.remote-alert-main strong, .remote-alert-panel > small { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }

@media (max-width: 1180px) {
  .remote-control-page,
  .remote-console {
    grid-template-columns: 1fr;
  }

  .remote-turn {
    width: min(180px, 100%);
    justify-self: center;
  }
}

@media (min-width: 641px) and (max-width: 1024px) and (orientation: portrait) {
  .remote-control-page,
  .remote-main,
  .remote-side,
  .remote-console {
    min-width: 0;
  }

  .remote-main,
  .remote-side { gap: 18px; }

  .person-follow-toolbar { grid-template-columns: minmax(0, 1fr); }

  .person-follow-toolbar button,
  .speed-mode-button,
  .remote-actions button,
  .remote-stop { min-height: 44px; }

  .remote-status-grid div { min-width: 0; }
}

@media (min-width: 1200px) and (max-width: 2048px) and (min-height: 900px) and (max-height: 1280px) and (orientation: landscape) {
  .speed-mode-button {
    min-height: 44px;
  }
}

@media (max-width: 720px) {
  .remote-video-stage {
    min-height: 0;
  }

  .person-follow-toolbar { grid-template-columns: 1fr; }

  .remote-pad-wrap {
    display: grid;
    grid-template-columns: 1fr;
  }

  .remote-pad {
    grid-template-columns: repeat(3, 62px);
    grid-template-rows: repeat(3, 62px);
    justify-content: center;
  }

  .remote-pad-btn {
    width: 62px;
    height: 62px;
  }

  .remote-actions,
  .remote-status-grid {
    grid-template-columns: 1fr;
  }
}
</style>
