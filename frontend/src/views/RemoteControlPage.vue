<script setup>
import Hls from 'hls.js'
import mpegts from 'mpegts.js'
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'

import AppToast from '../components/AppToast.vue'
import { useToast } from '../composables/useToast'
import { fetchRobotDetail, fetchRobots, fetchRobotStatus, sendRobotCommand } from '../services/api'

const robots = ref([])
const selectedRobot = ref(null)
const liveStatus = ref(null)
const loading = ref(true)
const switchingRobot = ref(false)
const commandSending = ref(false)
const takeoverActive = ref(false)
const streamUnavailable = ref(false)
const activeHoldAction = ref('')
const speedScale = ref(0.7)
const commandFeedback = ref('')
const videoRef = ref(null)

let flvPlayer = null
let hlsPlayer = null
let statusTimer = null
let holdTimer = null
let holdAction = null
let holdPointerId = null
let holdTarget = null
let holdInFlight = false
let holdPromise = null

const HOLD_REPEAT_MS = 300
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
const hasLiveStream = computed(() => !streamUnavailable.value && Boolean(livePlayUrls.value.flv || livePlayUrls.value.hls))
const status = computed(() => liveStatus.value?.status || {})
const localizationQuality = computed(() => status.value?.localization_quality || {})
const canControl = computed(() => takeoverActive.value && selectedRobot.value?.id && !commandSending.value)

const motionActions = computed(() => [
  { action: 'move_forward', label: '前进', arrow: '↑', className: 'up', payload: { vx: roundSpeed(0.35) } },
  { action: 'move_left', label: '左移', arrow: '←', className: 'left', payload: { vy: roundSpeed(0.25) } },
  { action: 'move_right', label: '右移', arrow: '→', className: 'right', payload: { vy: -roundSpeed(0.25) } },
  { action: 'move_backward', label: '后退', arrow: '↓', className: 'down', payload: { vx: -roundSpeed(0.30) } },
])
const turnActions = computed(() => [
  { action: 'turn_left', label: '左转', arrow: '↶', className: 'left', payload: { yaw_rate: roundSpeed(0.45) } },
  { action: 'turn_right', label: '右转', arrow: '↷', className: 'right', payload: { yaw_rate: -roundSpeed(0.45) } },
])
const actionByName = computed(() => {
  const actions = [...motionActions.value, ...turnActions.value]
  return Object.fromEntries(actions.map((item) => [item.action, item]))
})

function roundSpeed(value) {
  return Number((value * speedScale.value).toFixed(3))
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

async function enterTakeover() {
  if (!selectedRobot.value?.id || commandSending.value) return
  commandSending.value = true
  try {
    const command = await dispatchRobotAction('takeover_enter', { note: 'Enter platform remote control page.' }, 'remote_control_enter')
    takeoverActive.value = true
    commandFeedback.value = `接管指令已下发 · ${command?.status || 'created'}`
    showToast('接管指令已下发')
  } catch (error) {
    showToast(error.message || '接管失败')
  } finally {
    commandSending.value = false
  }
}

async function exitTakeover() {
  if (!selectedRobot.value?.id || commandSending.value) return
  commandSending.value = true
  try {
    await stopHoldAction()
    const command = await dispatchRobotAction('takeover_exit', { passive: true, note: 'Exit platform remote control page.' }, 'remote_control_exit')
    takeoverActive.value = false
    commandFeedback.value = `释放指令已下发 · ${command?.status || 'created'}`
    showToast('释放指令已下发')
  } catch (error) {
    showToast(error.message || '释放接管失败')
  } finally {
    commandSending.value = false
  }
}

async function sendStop(source = 'remote_control_stop') {
  if (!selectedRobot.value?.id) return
  try {
    await dispatchRobotAction('move_stop', {}, source)
  } catch (error) {
    showToast(error.message || '停止指令失败')
  }
}

async function sendDiscreteAction(action, label) {
  if (!selectedRobot.value?.id || commandSending.value) return
  commandSending.value = true
  try {
    const command = await dispatchRobotAction(action, {}, 'remote_control_action')
    commandFeedback.value = `${label}指令已下发 · ${command?.status || 'created'}`
    if (action === 'stand_up') takeoverActive.value = true
    if (action === 'lie_down') takeoverActive.value = false
    showToast(`${label}指令已下发`)
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
    await stopHoldAction()
    await dispatchRobotAction('passive', { note: 'Remote page emergency stop.' }, 'remote_control_emergency_stop')
    showToast('已下发软急停')
  } catch (error) {
    showToast(error.message || '急停失败')
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
  try {
    await stopHoldAction()
    if (takeoverActive.value) {
      await dispatchRobotAction('takeover_exit', { passive: true }, 'remote_control_switch_robot')
      takeoverActive.value = false
    }
    selectedRobot.value = await fetchRobotDetail(robotId)
    streamUnavailable.value = false
    await refreshStatus()
    if (!loading.value) setupLivePlayer()
  } catch (error) {
    showToast(error.message || '切换设备失败')
  } finally {
    switchingRobot.value = false
  }
}

async function refreshStatus() {
  if (!selectedRobot.value?.id) return
  try {
    liveStatus.value = await fetchRobotStatus(selectedRobot.value.id)
    const mode = liveStatus.value?.status?.control_mode
    if (mode === 'manual_takeover') takeoverActive.value = true
    if (mode === 'autonomous' || mode === 'emergency_stop') takeoverActive.value = false
  } catch {}
}

function fallbackToSnapshot() {
  streamUnavailable.value = true
  destroyVideoPlayers()
}

async function canReachStream(url) {
  if (!url) return false
  const controller = new AbortController()
  const timeout = window.setTimeout(() => controller.abort(), 1800)
  try {
    await fetch(url, { method: 'GET', mode: 'no-cors', cache: 'no-store', signal: controller.signal })
    return true
  } catch {
    return false
  } finally {
    window.clearTimeout(timeout)
  }
}

function destroyVideoPlayers() {
  if (flvPlayer) {
    flvPlayer.destroy()
    flvPlayer = null
  }
  if (hlsPlayer) {
    hlsPlayer.destroy()
    hlsPlayer = null
  }
  if (videoRef.value) {
    videoRef.value.removeAttribute('src')
    videoRef.value.load()
  }
}

async function setupLivePlayer() {
  await nextTick()
  destroyVideoPlayers()
  const element = videoRef.value
  if (!element || !hasLiveStream.value) return
  const { flv, hls } = livePlayUrls.value
  const playableFlv = flv && mpegts.getFeatureList().mseLivePlayback && (await canReachStream(flv))
  const playableHls = hls && (await canReachStream(hls))
  if (!playableFlv && !playableHls) {
    fallbackToSnapshot()
    return
  }

  element.addEventListener('error', fallbackToSnapshot, { once: true })
  if (playableFlv) {
    flvPlayer = mpegts.createPlayer({ type: 'flv', isLive: true, url: flv })
    flvPlayer.on(mpegts.Events.ERROR, fallbackToSnapshot)
    flvPlayer.attachMediaElement(element)
    flvPlayer.load()
    flvPlayer.play().catch(fallbackToSnapshot)
    return
  }

  if (playableHls && Hls.isSupported()) {
    hlsPlayer = new Hls({ lowLatencyMode: true })
    hlsPlayer.loadSource(hls)
    hlsPlayer.attachMedia(element)
    hlsPlayer.on(Hls.Events.ERROR, (_event, data) => {
      if (data?.fatal) fallbackToSnapshot()
    })
    hlsPlayer.on(Hls.Events.MANIFEST_PARSED, () => element.play().catch(fallbackToSnapshot))
    return
  }

  if (playableHls) {
    element.src = hls
    element.play().catch(fallbackToSnapshot)
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

onMounted(async () => {
  window.addEventListener('keydown', handleKeyDown)
  window.addEventListener('keyup', handleKeyUp)
  window.addEventListener('blur', stopHoldAction)
  document.addEventListener('visibilitychange', handleVisibilityChange)
  document.addEventListener('contextmenu', preventRemoteGesture, { capture: true })
  document.addEventListener('selectstart', preventRemoteGesture, { capture: true })
  document.addEventListener('dragstart', preventRemoteGesture, { capture: true })
  try {
    robots.value = await fetchRobots()
    if (robots.value[0]?.id) await chooseRobot(robots.value[0].id)
    statusTimer = window.setInterval(refreshStatus, 2000)
  } finally {
    loading.value = false
  }
  setupLivePlayer()
})

onBeforeUnmount(async () => {
  window.removeEventListener('keydown', handleKeyDown)
  window.removeEventListener('keyup', handleKeyUp)
  window.removeEventListener('blur', stopHoldAction)
  document.removeEventListener('visibilitychange', handleVisibilityChange)
  document.removeEventListener('contextmenu', preventRemoteGesture, { capture: true })
  document.removeEventListener('selectstart', preventRemoteGesture, { capture: true })
  document.removeEventListener('dragstart', preventRemoteGesture, { capture: true })
  window.clearInterval(statusTimer)
  if (takeoverActive.value) {
    await stopHoldAction()
    await dispatchRobotAction('takeover_exit', { passive: true }, 'remote_control_unmount').catch(() => {})
  }
  destroyVideoPlayers()
})

watch(livePlayUrls, () => {
  streamUnavailable.value = false
  setupLivePlayer()
})
</script>

<template>
  <section v-if="!loading" class="remote-control-page">
    <div class="remote-main">
      <section class="panel remote-video-panel">
        <div class="panel-head">
          <div>
            <h3>远程视频操控</h3>
            <p>按住方向键持续移动，松开立即停止；键盘支持 W/A/S/D 和 Q/E。</p>
          </div>
          <span :class="['remote-state', takeoverActive ? 'ok' : 'idle']">
            {{ takeoverActive ? '接管中' : '未接管' }}
          </span>
        </div>

        <div class="remote-video-stage">
          <video
            v-if="hasLiveStream"
            ref="videoRef"
            class="remote-video-source"
            muted
            playsinline
            autoplay
          ></video>
          <div v-else class="remote-video-source remote-no-signal">
            <strong>无视频流</strong>
            <span>{{ selectedRobot?.stream_id || '当前设备未上报 FLV/HLS 播放地址' }}</span>
          </div>
          <div class="remote-video-overlay">
            <span>{{ selectedRobot?.code || '--' }}</span>
            <strong>{{ selectedRobot?.location || '未知位置' }}</strong>
          </div>
        </div>
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
          <div class="remote-turn">
            <button
              v-for="item in turnActions"
              :key="item.action"
              type="button"
              :class="['remote-turn-btn', { active: activeHoldAction === item.action }]"
              :disabled="!canControl"
              @pointerdown.prevent="startHoldAction(item, $event)"
              @pointerup.prevent="stopHoldAction($event)"
              @pointercancel.prevent="stopHoldAction($event)"
              @lostpointercapture="stopHoldAction($event)"
              @contextmenu.prevent
            >
              <span class="turn-glyph" aria-hidden="true"></span>
              <span class="sr-only">{{ item.label }}</span>
            </button>
          </div>
        </div>

        <div class="remote-controls">
          <label>
            <span>速度比例</span>
            <strong>{{ Math.round(speedScale * 100) }}%</strong>
            <input v-model.number="speedScale" type="range" min="0.3" max="1" step="0.1" :disabled="takeoverActive" />
          </label>
          <div class="remote-actions">
            <button class="takeover-btn" type="button" :disabled="commandSending || !selectedRobot" @click="enterTakeover">
              {{ commandSending && !takeoverActive ? '接管中...' : '接管' }}
            </button>
            <button class="ghost-btn" type="button" :disabled="commandSending || !takeoverActive" @click="exitTakeover">释放</button>
            <button class="danger-btn" type="button" :disabled="commandSending || !selectedRobot" @click="emergencyStop">软急停</button>
          </div>
          <div class="remote-actions">
            <button class="ghost-btn" type="button" :disabled="commandSending || !selectedRobot" @click="sendDiscreteAction('stand_up', '起立')">
              起立
            </button>
            <button class="ghost-btn" type="button" :disabled="commandSending || !selectedRobot" @click="sendDiscreteAction('lie_down', '趴下')">
              趴下
            </button>
            <button class="ghost-btn" type="button" :disabled="commandSending || !selectedRobot" @click="refreshStatus">
              刷新状态
            </button>
          </div>
          <button class="remote-stop" type="button" :disabled="!selectedRobot" @click="sendStop()">
            停止移动
          </button>
          <small v-if="commandFeedback" class="remote-feedback">{{ commandFeedback }}</small>
        </div>
      </section>
    </div>

    <aside class="remote-side">
      <section class="panel">
        <div class="panel-head">
          <div>
            <h3>设备选择</h3>
            <p>{{ switchingRobot ? '正在切换设备...' : '切换设备前会释放当前接管' }}</p>
          </div>
        </div>
        <div class="robot-list compact">
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
  grid-template-columns: minmax(0, 1fr) minmax(320px, 380px);
  gap: 22px;
  align-items: start;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
  touch-action: manipulation;
}

.remote-main,
.remote-side {
  display: grid;
  gap: 22px;
}

.remote-video-stage {
  position: relative;
  min-height: 520px;
  border: 1px solid rgba(120, 194, 255, 0.22);
  border-radius: 18px;
  overflow: hidden;
  background: #06111f;
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
  grid-template-columns: minmax(360px, 1fr) minmax(300px, 360px);
  gap: 24px;
  align-items: center;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
  touch-action: none;
}

.remote-pad-wrap {
  display: flex;
  gap: 18px;
  align-items: center;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  user-select: none;
  touch-action: none;
}

.remote-pad {
  display: grid;
  grid-template-columns: repeat(3, 72px);
  grid-template-rows: repeat(3, 72px);
  gap: 12px;
  padding: 18px;
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
  width: 72px;
  height: 72px;
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
  gap: 12px;
  min-width: 130px;
}

.remote-turn-btn {
  min-height: 54px;
  border-radius: 999px;
  padding: 0 16px;
  display: grid;
  place-items: center;
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

.remote-controls label {
  display: grid;
  grid-template-columns: 1fr auto;
  gap: 8px 12px;
  color: var(--muted);
  font-size: 13px;
}

.remote-controls label strong {
  color: var(--text);
}

.remote-controls input {
  grid-column: 1 / -1;
  width: 100%;
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

.robot-list.compact {
  display: grid;
  gap: 12px;
  max-height: 440px;
  overflow: auto;
}

@media (max-width: 1180px) {
  .remote-control-page,
  .remote-console {
    grid-template-columns: 1fr;
  }
}

@media (max-width: 720px) {
  .remote-video-stage {
    min-height: 360px;
  }

  .remote-pad-wrap {
    display: grid;
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
