<script setup>
import Hls from 'hls.js'
import mpegts from 'mpegts.js'
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'

import AppToast from '../components/AppToast.vue'
import { useToast } from '../composables/useToast'
import {
  API_BASE,
  executePatrolTask,
  fetchOverview,
  fetchPatrolTasks,
  fetchRobotDetail,
  fetchRobots,
  fetchTaskExecution,
  sendRobotCommand,
  sendTaskExecutionAction,
} from '../services/api'

const overview = ref(null)
const robots = ref([])
const tasks = ref([])
const selectedRobot = ref(null)
const execution = ref(null)
const loading = ref(true)
const busy = ref(false)
const streamUnavailable = ref(false)
const videoRef = ref(null)
const videoStageRef = ref(null)
const { toastMessage, toastVariant, visible, showToast } = useToast()

let flvPlayer = null
let hlsPlayer = null
let alertEventSource = null
let refreshTimer = null
let executionTimer = null

const latestRobot = computed(() => selectedRobot.value || overview.value?.latest_robot || null)
const playUrls = computed(() => latestRobot.value?.play_urls || {})
const hasStream = computed(() => !streamUnavailable.value && Boolean(playUrls.value.flv || playUrls.value.hls))
const presetTask = computed(() => tasks.value.find((task) => task.enabled) || tasks.value[0] || null)
const latestAlert = computed(() => latestRobot.value?.recent_events?.[0] || overview.value?.live_event || null)
const alerts = computed(() => latestRobot.value?.recent_events || [])
const isRunning = computed(() => ['queued', 'starting', 'running', 'paused'].includes(execution.value?.state))
const taskStateText = computed(() => {
  const labels = {
    queued: '等待执行',
    starting: '正在启动',
    running: '巡检执行中',
    paused: '巡检已暂停',
    completed: '巡检已完成',
    failed: '巡检失败',
    cancelled: '巡检已停止',
  }
  return labels[execution.value?.state] || (presetTask.value ? '待执行' : '未配置任务')
})

function formatTime(value) {
  if (!value) return '--'
  const date = new Date(value)
  return Number.isNaN(date.getTime()) ? value : date.toLocaleString('zh-CN', { hour: '2-digit', minute: '2-digit', second: '2-digit' })
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
  showToast('收到新的现场报警', { variant: 'alert', duration: 5200 })
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
}

async function refreshRobot() {
  if (!latestRobot.value?.id) return
  try {
    selectedRobot.value = await fetchRobotDetail(latestRobot.value.id)
  } catch {}
}

async function refreshExecution() {
  if (!execution.value?.id) return
  try {
    execution.value = await fetchTaskExecution(execution.value.id)
    if (!isRunning.value) {
      window.clearInterval(executionTimer)
      executionTimer = null
    }
  } catch {}
}

async function startTask() {
  if (!presetTask.value || busy.value || isRunning.value) return
  busy.value = true
  try {
    execution.value = await executePatrolTask(presetTask.value.id)
    showToast(`已开始执行：${presetTask.value.name}`)
    executionTimer = window.setInterval(refreshExecution, 2500)
  } catch (error) {
    showToast(error.message || '任务启动失败', { variant: 'alert' })
  } finally {
    busy.value = false
  }
}

async function stopTask() {
  if (!execution.value?.id || busy.value) return
  if (!window.confirm('确定停止当前巡检任务吗？')) return
  busy.value = true
  try {
    execution.value = await sendTaskExecutionAction(execution.value.id, 'cancel')
    showToast('已下发停止任务指令')
  } catch (error) {
    showToast(error.message || '停止任务失败', { variant: 'alert' })
  } finally {
    busy.value = false
  }
}

async function emergencyStop() {
  if (!latestRobot.value?.id || busy.value) return
  if (!window.confirm('确定立即让机器狗停止运动吗？')) return
  busy.value = true
  try {
    await sendRobotCommand(latestRobot.value.id, {
      action: 'passive',
      payload: { source: 'guard_duty_emergency_stop', note: 'Guard duty soft emergency stop.' },
    })
    showToast('已下发软急停指令', { variant: 'alert' })
  } catch (error) {
    showToast(error.message || '急停指令下发失败', { variant: 'alert' })
  } finally {
    busy.value = false
  }
}

function destroyPlayers() {
  flvPlayer?.destroy()
  hlsPlayer?.destroy()
  flvPlayer = null
  hlsPlayer = null
  if (videoRef.value) {
    videoRef.value.removeAttribute('src')
    videoRef.value.load()
  }
}

function markStreamUnavailable() {
  streamUnavailable.value = true
  destroyPlayers()
}

async function setupPlayer() {
  await nextTick()
  destroyPlayers()
  const element = videoRef.value
  if (!element || !hasStream.value) return
  const { flv, hls } = playUrls.value
  try {
    if (flv && mpegts.getFeatureList().mseLivePlayback) {
      flvPlayer = mpegts.createPlayer({ type: 'flv', isLive: true, url: flv })
      flvPlayer.on(mpegts.Events.ERROR, markStreamUnavailable)
      flvPlayer.attachMediaElement(element)
      flvPlayer.load()
      await element.play()
      return
    }
    if (hls && Hls.isSupported()) {
      hlsPlayer = new Hls({ lowLatencyMode: true })
      hlsPlayer.loadSource(hls)
      hlsPlayer.attachMedia(element)
      hlsPlayer.on(Hls.Events.ERROR, (_event, data) => data?.fatal && markStreamUnavailable())
      hlsPlayer.on(Hls.Events.MANIFEST_PARSED, () => element.play().catch(markStreamUnavailable))
      return
    }
    if (hls) {
      element.src = hls
      await element.play()
    }
  } catch {
    markStreamUnavailable()
  }
}

onMounted(async () => {
  try {
    await load()
    openAlertStream()
    await setupPlayer()
    refreshTimer = window.setInterval(refreshRobot, 5000)
  } catch (error) {
    showToast(error.message || '加载值守页面失败', { variant: 'alert' })
  } finally {
    loading.value = false
  }
})

onBeforeUnmount(() => {
  window.clearInterval(refreshTimer)
  window.clearInterval(executionTimer)
  alertEventSource?.close()
  destroyPlayers()
})

watch(playUrls, () => {
  streamUnavailable.value = false
  setupPlayer()
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
            <video v-if="hasStream" ref="videoRef" class="guard-video" muted playsinline autoplay controls></video>
            <div v-else class="guard-video-empty">
              <strong>视频暂不可用</strong>
              <span>{{ latestRobot?.stream_id || '机器人未上报视频流' }}</span>
            </div>
            <div class="guard-video-label">
              <strong>{{ latestRobot?.name || latestRobot?.code || '机器狗' }}</strong>
              <span>{{ latestRobot?.location || '位置未知' }}</span>
            </div>
          </div>

          <div class="guard-task-bar">
            <div>
              <span>当前任务</span>
              <strong>{{ presetTask?.name || '管理员尚未配置巡检任务' }}</strong>
            </div>
            <div>
              <span>执行状态</span>
              <strong>{{ taskStateText }}</strong>
            </div>
            <button class="guard-primary" :disabled="busy || !presetTask || isRunning" @click="startTask">
              {{ busy ? '处理中...' : '开始巡检' }}
            </button>
            <button class="guard-secondary" :disabled="busy || !isRunning" @click="stopTask">停止巡检</button>
          </div>
        </section>

        <aside class="guard-side">
          <section class="guard-alert-panel" :class="{ 'has-alert': latestAlert }">
            <div class="guard-panel-title">
              <div>
                <span class="guard-eyebrow">实时事件</span>
                <h2>报警信息</h2>
              </div>
              <strong>{{ alerts.length }}</strong>
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

          <button class="guard-stop" :disabled="busy || !latestRobot" @click="emergencyStop">软急停</button>
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
.guard-task-bar { display: grid; grid-template-columns: 1.5fr 1fr auto auto; align-items: center; gap: 14px; padding: 16px; }
.guard-task-bar > div { display: grid; gap: 5px; min-width: 0; }
.guard-task-bar span { color: #70808c; font-size: 12px; }
.guard-task-bar strong { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.guard-primary, .guard-secondary, .guard-stop { min-height: 52px; padding: 0 22px; border: 0; font: inherit; font-weight: 800; cursor: pointer; }
.guard-primary { color: #fff; background: #19724d; }
.guard-secondary { color: #263944; background: #dfe7eb; }
.guard-stop { width: 100%; color: #fff; background: #ba302b; font-size: 20px; }
.guard-primary:disabled, .guard-secondary:disabled, .guard-stop:disabled { cursor: not-allowed; opacity: .45; }
.guard-side { display: grid; align-content: start; gap: 18px; }
.guard-alert-panel { min-height: 360px; padding: 18px; }
.guard-panel-title { display: flex; justify-content: space-between; align-items: start; padding-bottom: 14px; border-bottom: 1px solid #e4eaed; }
.guard-panel-title h2 { margin: 5px 0 0; font-size: 24px; letter-spacing: 0; }
.guard-panel-title > strong { display: grid; place-items: center; min-width: 38px; height: 38px; color: #fff; background: #ba302b; }
.guard-alert-main { display: grid; grid-template-columns: 88px 1fr; gap: 12px; align-items: center; padding: 16px 0; }
.guard-alert-image { width: 88px; height: 68px; background-position: center; background-size: cover; }
.guard-alert-main div:last-child { display: grid; gap: 5px; }
.guard-alert-main span, .guard-alert-main small { color: #6d7e89; font-size: 13px; }
.guard-alert-list { display: grid; gap: 10px; border-top: 1px solid #e4eaed; padding-top: 12px; }
.guard-alert-list div { display: flex; justify-content: space-between; gap: 10px; font-size: 13px; }
.guard-alert-list small { color: #788893; white-space: nowrap; }
.guard-empty { color: #71828d; }
.guard-loading { display: grid; place-items: center; min-height: 50vh; color: #657681; }
@media (max-width: 980px) {
  .guard-grid { grid-template-columns: 1fr; }
  .guard-side { grid-template-columns: 1fr 220px; align-items: start; }
  .guard-video-stage, .guard-video, .guard-video-empty { min-height: 56vw; }
}
@media (max-width: 640px) {
  .guard-page { padding: 14px; }
  .guard-header { align-items: start; flex-direction: column; }
  .guard-grid, .guard-side { grid-template-columns: 1fr; }
  .guard-task-bar { grid-template-columns: 1fr 1fr; }
  .guard-task-bar > div:first-child { grid-column: 1 / -1; }
  .guard-primary, .guard-secondary { width: 100%; }
}
</style>
