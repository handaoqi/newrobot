<script setup>
import Hls from 'hls.js'
import mpegts from 'mpegts.js'
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'

import AppToast from '../components/AppToast.vue'
import { useToast } from '../composables/useToast'
import {
  API_BASE,
  createSpeechTemplate,
  deleteSpeechTemplate,
  fetchOverview,
  fetchRobotDetail,
  fetchRobots,
  fetchSpeechTemplates,
  sendRecordedAudioCommand,
  sendRobotCommand,
  sendTextToSpeechCommand,
  synthesizeSpeech,
  updateSpeechTemplate,
} from '../services/api'

const overview = ref(null)
const robots = ref([])
const selectedRobot = ref(null)
const loading = ref(true)
const loadError = ref('')
const switchingRobot = ref(false)
const commandSending = ref(false)
const takeoverActive = ref(false)
const speakerText = ref('您好，这里禁止自行车长时间停放，请尽快驶离指定区域，感谢配合。')
const speechTemplates = ref([])
const selectedSpeakerTemplateId = ref(null)
const templateName = ref('驶离提醒')
const templateSaving = ref(false)
const audioCommandSending = ref(false)
const recording = ref(false)
const recordedBlob = ref(null)
const recordedUrl = ref('')
const recordingSeconds = ref(0)
const videoRef = ref(null)
const videoStageRef = ref(null)
const streamUnavailable = ref(false)
let flvPlayer = null
let hlsPlayer = null
let liveGuardTimer = null
let alertEventSource = null
let holdTimer = null
let holdAction = null
let holdPointerId = null
let holdTarget = null
let holdInFlight = false
let holdPromise = null
let takeoverExitInFlight = false
let mediaRecorder = null
let recordingTimer = null
let recordingStream = null
let recordedChunks = []
let previewPlayer = null
const realtimeEventIds = new Set()
const HOLD_REPEAT_MS = 300
const AUDIO_COMMAND_COOLDOWN_MS = 3000
const { toastMessage, toastVariant, visible, showToast } = useToast()

const eventImages = ['/images/event-1.jpg', '/images/event-2.jpg', '/images/event-3.jpg']
const latestRobot = computed(() => selectedRobot.value || overview.value?.latest_robot || null)
const liveEvent = computed(() => latestRobot.value?.recent_events?.[0] || overview.value?.live_event || null)
const livePlayUrls = computed(() => latestRobot.value?.play_urls || {})
const hasLiveStream = computed(() => !streamUnavailable.value && Boolean(livePlayUrls.value.flv || livePlayUrls.value.hls))
const activeHoldAction = ref('')
const motionActions = [
  { action: 'move_forward', label: '前进', arrow: '↑', position: 'up', payload: { vx: 0.35 }, hold: true },
  { action: 'move_left', label: '左移', arrow: '←', position: 'left', payload: { vy: 0.25 }, hold: true },
  { action: 'move_right', label: '右移', arrow: '→', position: 'right', payload: { vy: -0.25 }, hold: true },
  { action: 'move_backward', label: '后退', arrow: '↓', position: 'down', payload: { vx: -0.35 }, hold: true },
]
const turnActions = [
  { action: 'turn_left', label: '左转', arrow: '↶', position: 'left', payload: { yaw_rate: 0.45 }, hold: true },
  { action: 'turn_right', label: '右转', arrow: '↷', position: 'right', payload: { yaw_rate: -0.45 }, hold: true },
]
const skillActions = [
  { action: 'stand_up', label: '站立' },
  { action: 'move_stop', label: '停止', primary: true },
  { action: 'lie_down', label: '趴下' },
  { action: 'shake_hand', label: '握手' },
]
const allControlActions = [...motionActions, ...turnActions, ...skillActions]

function eventThumbStyle(index) {
  const event = latestRobot.value?.recent_events?.[index]
  const image = event?.annotated_snapshot_url || event?.snapshot_url || eventImages[index % eventImages.length]
  return {
    backgroundImage: `linear-gradient(rgba(6, 16, 28, 0.08), rgba(6, 16, 28, 0.18)), url(${image})`,
  }
}

function formatEventTime(value) {
  if (!value) return '--'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return value
  return date.toLocaleString('zh-CN', {
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
  })
}

function setSpeakerTemplate(template) {
  speakerText.value = template.text
  templateName.value = template.name
  selectedSpeakerTemplateId.value = template.id
  showToast('已切换喊话模板')
}

async function loadSpeechTemplates() {
  speechTemplates.value = await fetchSpeechTemplates()
  const selected = speechTemplates.value.find((item) => item.id === selectedSpeakerTemplateId.value) ||
    speechTemplates.value.find((item) => item.name === templateName.value) ||
    speechTemplates.value[0]
  if (selected && selectedSpeakerTemplateId.value === null) setSpeakerTemplate(selected)
}

async function beginSpeak() {
  const robot = latestRobot.value
  if (!robot?.id || audioCommandSending.value) return
  const text = speakerText.value.trim()
  if (!text) {
    showToast('请输入需要播报的文字')
    return
  }

  const commandStartedAt = Date.now()
  audioCommandSending.value = true
  try {
    await sendTextToSpeechCommand(robot.id, text, templateName.value.trim() || '实时文字喊话')
    showToast('文字已生成语音并下发，等待机器狗播放')
  } catch (error) {
    showToast(error.message || '文字转语音下发失败')
  } finally {
    const cooldownRemaining = AUDIO_COMMAND_COOLDOWN_MS - (Date.now() - commandStartedAt)
    if (cooldownRemaining > 0) {
      await new Promise((resolve) => window.setTimeout(resolve, cooldownRemaining))
    }
    audioCommandSending.value = false
  }
}

async function previewVoice() {
  const text = speakerText.value.trim()
  if (!text || audioCommandSending.value) {
    if (!text) showToast('请输入需要预览的文字')
    return
  }
  audioCommandSending.value = true
  try {
    const result = await synthesizeSpeech(text)
    if (previewPlayer) previewPlayer.pause()
    previewPlayer = new Audio(result.audio_url)
    await previewPlayer.play()
    showToast('正在预览当前文字生成的语音')
  } catch (error) {
    showToast(error.message || '语音预览失败')
  } finally {
    audioCommandSending.value = false
  }
}

async function saveSpeechTemplate() {
  const name = templateName.value.trim()
  const text = speakerText.value.trim()
  if (!name || !text || templateSaving.value) {
    showToast(!name ? '请输入文案名称' : '请输入播报文字')
    return
  }
  templateSaving.value = true
  try {
    const saved = selectedSpeakerTemplateId.value
      ? await updateSpeechTemplate(selectedSpeakerTemplateId.value, { name, text })
      : await createSpeechTemplate({ name, text })
    selectedSpeakerTemplateId.value = saved.id
    await loadSpeechTemplates()
    showToast('喊话文案已保存')
  } catch (error) {
    showToast(error.message || '保存文案失败')
  } finally {
    templateSaving.value = false
  }
}

function startNewSpeechTemplate() {
  selectedSpeakerTemplateId.value = null
  templateName.value = ''
  speakerText.value = ''
}

async function removeSpeechTemplate() {
  if (!selectedSpeakerTemplateId.value || templateSaving.value) return
  templateSaving.value = true
  try {
    await deleteSpeechTemplate(selectedSpeakerTemplateId.value)
    selectedSpeakerTemplateId.value = null
    templateName.value = ''
    speakerText.value = ''
    await loadSpeechTemplates()
    showToast('喊话文案已删除')
  } catch (error) {
    showToast(error.message || '删除文案失败')
  } finally {
    templateSaving.value = false
  }
}

function pickRecordingMimeType() {
  const candidates = ['audio/webm;codecs=opus', 'audio/webm', 'audio/ogg;codecs=opus', 'audio/ogg']
  return candidates.find((type) => window.MediaRecorder?.isTypeSupported(type)) || ''
}

function resetRecording() {
  if (recordedUrl.value) URL.revokeObjectURL(recordedUrl.value)
  recordedBlob.value = null
  recordedUrl.value = ''
  recordingSeconds.value = 0
}

function cleanupRecorder() {
  if (recordingTimer) {
    window.clearInterval(recordingTimer)
    recordingTimer = null
  }
  if (recordingStream) {
    recordingStream.getTracks().forEach((track) => track.stop())
    recordingStream = null
  }
  mediaRecorder = null
  recording.value = false
}

async function startRecording() {
  if (recording.value || audioCommandSending.value) return
  if (!navigator.mediaDevices?.getUserMedia || !window.MediaRecorder) {
    showToast('当前浏览器不支持录音')
    return
  }
  if (!window.isSecureContext) {
    showToast('浏览器录音需要通过 HTTPS 或 localhost 访问')
    return
  }
  try {
    resetRecording()
    recordingStream = await navigator.mediaDevices.getUserMedia({
      audio: { echoCancellation: true, noiseSuppression: true, autoGainControl: true },
    })
    const mimeType = pickRecordingMimeType()
    mediaRecorder = new MediaRecorder(recordingStream, mimeType ? { mimeType } : undefined)
    recordedChunks = []
    mediaRecorder.addEventListener('dataavailable', (event) => {
      if (event.data?.size > 0) recordedChunks.push(event.data)
    })
    mediaRecorder.addEventListener('stop', () => {
      const blobType = mediaRecorder?.mimeType || mimeType || 'audio/webm'
      recordedBlob.value = new Blob(recordedChunks, { type: blobType })
      recordedUrl.value = URL.createObjectURL(recordedBlob.value)
      cleanupRecorder()
    })
    mediaRecorder.start(250)
    recording.value = true
    recordingSeconds.value = 0
    recordingTimer = window.setInterval(() => {
      recordingSeconds.value += 1
    }, 1000)
    showToast('录音已开始')
  } catch (error) {
    cleanupRecorder()
    showToast(error?.name === 'NotAllowedError' ? '麦克风权限被拒绝' : '无法启动录音')
  }
}

function stopRecording() {
  if (recording.value && mediaRecorder) mediaRecorder.stop()
}

function recordingDurationLabel() {
  const minutes = String(Math.floor(recordingSeconds.value / 60)).padStart(2, '0')
  const seconds = String(recordingSeconds.value % 60).padStart(2, '0')
  return `${minutes}:${seconds}`
}

async function playRecordedAudio() {
  const robot = latestRobot.value
  if (!robot?.id || !recordedBlob.value || audioCommandSending.value) return
  const extension = recordedBlob.value.type.includes('ogg') ? 'ogg' : 'webm'
  const file = new File([recordedBlob.value], `dashboard-recording.${extension}`, {
    type: recordedBlob.value.type || 'audio/webm',
  })
  const commandStartedAt = Date.now()
  audioCommandSending.value = true
  try {
    await sendRecordedAudioCommand(robot.id, file, '现场录音')
    showToast('已上传录音并下发播放指令')
  } catch (error) {
    showToast(error.message || '录音播放指令下发失败')
  } finally {
    const cooldownRemaining = AUDIO_COMMAND_COOLDOWN_MS - (Date.now() - commandStartedAt)
    if (cooldownRemaining > 0) {
      await new Promise((resolve) => window.setTimeout(resolve, cooldownRemaining))
    }
    audioCommandSending.value = false
  }
}

async function emergencyStop() {
  const robot = latestRobot.value
  if (!robot?.id || commandSending.value) return
  if (!takeoverActive.value) {
    showToast('手柄模式下未初始化远程控制，请先接管后再执行远程急停')
    return
  }
  commandSending.value = true
  try {
    await sendRobotCommand(robot.id, {
      action: 'passive',
      payload: {
        source: 'emergency_stop',
        note: 'Remote takeover emergency stop.',
      },
    })
    showToast('已下发急停指令')
  } catch (error) {
    showToast(error.message || '控制指令下发失败')
  } finally {
    commandSending.value = false
  }
}

async function dispatchRobotAction(action, payload = {}, source = 'dashboard_control') {
  const robot = latestRobot.value
  if (!robot?.id) return false
  await sendRobotCommand(robot.id, {
    action,
    payload: {
      source,
      ...payload,
    },
  })
  return true
}

async function sendControlAction(action, payload = {}) {
  if (commandSending.value || activeHoldAction.value) return
  commandSending.value = true
  try {
    await dispatchRobotAction(action, payload, takeoverActive.value ? 'manual_takeover' : 'dashboard_control')
    const label = allControlActions.find((item) => item.action === action)?.label || action
    showToast(`已下发指令：${label}`)
  } catch (error) {
    showToast(error.message || '控制指令下发失败')
  } finally {
    commandSending.value = false
  }
}

async function sendHeldAction() {
  if (!holdAction || holdInFlight) return
  holdInFlight = true
  try {
    holdPromise = dispatchRobotAction(holdAction.action, holdAction.payload || {}, 'manual_takeover_hold')
    await holdPromise
  } catch (error) {
    stopHoldAction()
    showToast(error.message || '运动控制指令下发失败')
  } finally {
    holdPromise = null
    holdInFlight = false
  }
}

function startHoldAction(item, event) {
  if (!takeoverActive.value || commandSending.value) return
  if (holdAction?.action === item.action) return
  if (holdAction) return

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

async function stopHoldAction(event) {
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

  try {
    await pendingHoldPromise?.catch(() => {})
    await dispatchRobotAction('move_stop', {}, 'manual_takeover_hold_release')
  } catch (error) {
    showToast(error.message || '停止指令下发失败')
  }
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
  const element = videoRef.value
  if (!element) return
  element.muted = true
  element.controls = false
  seekLatestFrame()
  if (element.paused) {
    element.play().catch(() => {})
  }
}

function startLiveGuard() {
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

function setupAlertStream() {
  closeAlertStream()
  const token = localStorage.getItem('inspection_token')
  if (!token || typeof EventSource === 'undefined') return

  const url = `${API_BASE}/events/stream/?token=${encodeURIComponent(token)}`
  alertEventSource = new EventSource(url)
  alertEventSource.addEventListener('inspection_event_created', (message) => {
    let payload = {}
    try {
      payload = JSON.parse(message.data || '{}')
    } catch {}
    addRealtimeEvent(payload.event)
    showToast('新告警：自行车违停', {
      variant: 'alert',
      duration: 5200,
    })
  })
}

function addRealtimeEvent(event) {
  if (!event?.id) return
  if (realtimeEventIds.has(event.id)) return
  const alreadyVisible = [selectedRobot.value, overview.value?.latest_robot].some((robot) =>
    robot?.recent_events?.some((item) => item.id === event.id),
  )
  if (alreadyVisible) {
    realtimeEventIds.add(event.id)
    return
  }
  realtimeEventIds.add(event.id)

  const updateRobotEvents = (robot) => {
    if (!robot?.recent_events) return
    if (robot.id && event.robot_code && robot.code !== event.robot_code) return
    robot.recent_events = [event, ...robot.recent_events].slice(0, 5)
  }

  updateRobotEvents(selectedRobot.value)
  updateRobotEvents(overview.value?.latest_robot)

  if (overview.value?.summary) {
    overview.value.summary.today_alert_count = (overview.value.summary.today_alert_count || 0) + 1
  }
  if (overview.value?.header) {
    overview.value.header.today_alerts = (overview.value.header.today_alerts || 0) + 1
  }
  if (selectedRobot.value && selectedRobot.value.code === event.robot_code) {
    selectedRobot.value.today_alerts = (selectedRobot.value.today_alerts || 0) + 1
  }
}

function closeAlertStream() {
  if (alertEventSource) {
    alertEventSource.close()
    alertEventSource = null
  }
}

async function enterTakeover() {
  const robot = latestRobot.value
  if (!robot?.id || commandSending.value) return
  if (!hasLiveStream.value) {
    showToast('当前设备暂无可用视频流')
    return
  }
  commandSending.value = true
  try {
    await sendRobotCommand(robot.id, {
      action: 'takeover_enter',
      payload: {
        source: 'manual_takeover_enter',
        note: 'Enter remote takeover mode before showing fullscreen controls.',
      },
    })
    takeoverActive.value = true
    await nextTick()
    startLiveGuard()
    try {
      await videoStageRef.value?.requestFullscreen?.()
    } catch {}
    showToast('已切换至远程接管模式')
  } catch (error) {
    showToast(error.message || '接管指令下发失败')
  } finally {
    commandSending.value = false
  }
}

async function exitTakeover(options = {}) {
  if (takeoverExitInFlight) return
  takeoverExitInFlight = true
  try {
    await stopHoldAction()
    const robot = latestRobot.value
    if (robot?.id) {
      await sendRobotCommand(robot.id, {
        action: 'takeover_exit',
        payload: {
          source: options.source || 'manual_takeover_exit',
          passive: true,
          note: 'Exit remote takeover mode and release SDK control.',
        },
      })
      showToast('已切回手柄模式')
    }
  } catch (error) {
    showToast(error.message || '退出接管失败')
  } finally {
    takeoverActive.value = false
    stopLiveGuard()
    takeoverExitInFlight = false
  }
  if (!options.skipFullscreen && document.fullscreenElement) {
    document.exitFullscreen?.().catch(() => {})
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
  takeoverActive.value = false
  void stopHoldAction()
  stopLiveGuard()
  if (document.fullscreenElement === videoStageRef.value) {
    document.exitFullscreen?.().catch(() => {})
  }
}

async function chooseRobot(robotId, announce = true) {
  if (!robotId || switchingRobot.value || selectedRobot.value?.id === robotId) return
  switchingRobot.value = true
  try {
    selectedRobot.value = await fetchRobotDetail(robotId)
    streamUnavailable.value = false
    if (announce) showToast(`已切换至 ${selectedRobot.value.name}`)
  } finally {
    switchingRobot.value = false
  }
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
    await fetch(url, {
      method: 'GET',
      mode: 'no-cors',
      cache: 'no-store',
      signal: controller.signal,
    })
    return true
  } catch {
    return false
  } finally {
    window.clearTimeout(timeout)
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

  try {
    element.addEventListener('error', fallbackToSnapshot, { once: true })
  } catch {}

  if (playableFlv) {
    flvPlayer = mpegts.createPlayer({
      type: 'flv',
      isLive: true,
      url: flv,
    }, {
      enableStashBuffer: false,
      liveBufferLatencyChasing: true,
    })
    flvPlayer.on(mpegts.Events.ERROR, fallbackToSnapshot)
    flvPlayer.attachMediaElement(element)
    flvPlayer.load()
    flvPlayer.play().catch(fallbackToSnapshot)
    startLiveGuard()
    return
  }

  if (playableHls && Hls.isSupported()) {
    hlsPlayer = new Hls({
      lowLatencyMode: true,
      liveSyncDurationCount: 1,
      liveMaxLatencyDurationCount: 2,
      maxLiveSyncPlaybackRate: 1.5,
    })
    hlsPlayer.loadSource(hls)
    hlsPlayer.attachMedia(element)
    hlsPlayer.on(Hls.Events.ERROR, (_event, data) => {
      if (data?.fatal) fallbackToSnapshot()
    })
    hlsPlayer.on(Hls.Events.MANIFEST_PARSED, () => {
      element.play().catch(fallbackToSnapshot)
      startLiveGuard()
    })
    return
  }

  if (playableHls) {
    element.src = hls
    element.play().catch(fallbackToSnapshot)
    startLiveGuard()
  }
}

onMounted(async () => {
  document.addEventListener('fullscreenchange', handleFullscreenChange)
  window.addEventListener('blur', stopHoldAction)
  document.addEventListener('visibilitychange', handleVisibilityChange)
  setupAlertStream()
  try {
    const [overviewData, robotData, templateData] = await Promise.all([
      fetchOverview(),
      fetchRobots(),
      fetchSpeechTemplates().catch(() => []),
    ])
    overview.value = overviewData
    robots.value = robotData
    speechTemplates.value = templateData
    const initialTemplate = templateData.find((item) => item.name === templateName.value) || templateData[0]
    if (initialTemplate) setSpeakerTemplate(initialTemplate)
    const initialRobotId = overviewData.latest_robot?.id || robotData[0]?.id
    if (initialRobotId) {
      await chooseRobot(initialRobotId, false)
    }
  } catch (error) {
    loadError.value = error?.message || '未能获取监测数据，请稍后重试。'
  } finally {
    loading.value = false
  }

  setupLivePlayer()
})

onBeforeUnmount(() => {
  if (takeoverActive.value) {
    void exitTakeover({ skipFullscreen: true, source: 'component_unmount' })
  }
  if (recording.value && mediaRecorder) mediaRecorder.stop()
  cleanupRecorder()
  resetRecording()
  if (previewPlayer) {
    previewPlayer.pause()
    previewPlayer = null
  }
  destroyVideoPlayers()
  closeAlertStream()
  document.removeEventListener('fullscreenchange', handleFullscreenChange)
  window.removeEventListener('blur', stopHoldAction)
  document.removeEventListener('visibilitychange', handleVisibilityChange)
})

watch(livePlayUrls, () => {
  streamUnavailable.value = false
  setupLivePlayer()
})

function handleFullscreenChange() {
  if (takeoverActive.value && !document.fullscreenElement) {
    void exitTakeover({ skipFullscreen: true, source: 'fullscreen_exit' })
  }
}

function handleVisibilityChange() {
  if (document.hidden) {
    stopHoldAction()
  }
}
</script>

<template>
  <section v-if="!loading && overview" class="page-grid">
    <div class="content-column">
      <section class="top-summary">
        <article class="status-pill online">
          <span class="dot"></span>
          设备在线 {{ latestRobot?.code || overview.header.device_code }}
        </article>
        <article class="status-pill warning">今日告警 {{ overview.header.today_alerts }} 条</article>
        <article class="status-pill">当前区域 {{ latestRobot?.location || overview.header.current_location }}</article>
      </section>

      <section class="panel video-panel" :class="{ 'takeover-active': takeoverActive }">
        <div class="panel-head">
          <div>
            <h3>实时视频流监控</h3>
            <p>支持平台方查看机器人前端画面、检测框与事件识别状态</p>
          </div>
          <span class="panel-badge">Live Stream</span>
        </div>

        <div ref="videoStageRef" class="video-stage">
          <video
            v-if="hasLiveStream"
            ref="videoRef"
            class="video-source"
            muted
            playsinline
            autoplay
            @pause="keepLivePlaying"
            @progress="seekLatestFrame"
            @timeupdate="seekLatestFrame"
            @loadedmetadata="keepLivePlaying"
            @waiting="keepLivePlaying"
          ></video>
          <div v-else class="video-source no-signal" role="img" aria-label="视频无信号">
            <strong>无信号</strong>
            <span>{{ latestRobot?.stream_id || '当前设备暂无可用视频源' }}</span>
          </div>

          <div class="video-overlay">
            <div class="overlay-card">
              <strong>巡检位置</strong>
              <span>{{ latestRobot?.location }}</span>
            </div>
            <div class="overlay-card" v-if="latestRobot?.stream_id">
              <strong>视频流</strong>
              <span>{{ latestRobot.stream_id }}</span>
            </div>
          </div>

          <div v-if="takeoverActive" class="takeover-layer">
            <div class="takeover-status">
              <strong>人工接管</strong>
              <span>{{ latestRobot?.code }} · {{ latestRobot?.location }}</span>
            </div>
            <button class="takeover-exit" type="button" @click="exitTakeover">退出</button>
            <div class="takeover-controls" aria-label="机器狗控制动作">
              <div class="motion-pad" aria-label="移动控制">
                <button
                  v-for="item in motionActions"
                  :key="item.action"
                  type="button"
                  :class="['gamepad-btn', item.position, { active: activeHoldAction === item.action }]"
                  :aria-label="item.label"
                  :disabled="commandSending"
                  @pointerdown.prevent="startHoldAction(item, $event)"
                  @pointerup.prevent="stopHoldAction($event)"
                  @pointercancel.prevent="stopHoldAction($event)"
                  @lostpointercapture="stopHoldAction($event)"
                  @contextmenu.prevent
                >
                  {{ item.arrow }}
                </button>
              </div>
              <div class="skill-strip" aria-label="动作控制">
                <button
                  v-for="item in skillActions"
                  :key="item.action"
                  type="button"
                  :class="['skill-btn', { primary: item.primary }]"
                  :disabled="commandSending || Boolean(activeHoldAction)"
                  @click="sendControlAction(item.action)"
                >
                  {{ item.label }}
                </button>
              </div>
              <div class="turn-pad" aria-label="转向控制">
                <button
                  v-for="item in turnActions"
                  :key="item.action"
                  type="button"
                  :class="['gamepad-btn', item.position, { active: activeHoldAction === item.action }]"
                  :aria-label="item.label"
                  :disabled="commandSending"
                  @pointerdown.prevent="startHoldAction(item, $event)"
                  @pointerup.prevent="stopHoldAction($event)"
                  @pointercancel.prevent="stopHoldAction($event)"
                  @lostpointercapture="stopHoldAction($event)"
                  @contextmenu.prevent
                >
                  {{ item.arrow }}
                </button>
              </div>
            </div>
            <div class="takeover-mobile-actions" aria-label="全部动作">
              <button
                v-for="item in allControlActions"
                :key="item.action"
                type="button"
                :class="['skill-btn', { primary: item.primary, active: activeHoldAction === item.action }]"
                :disabled="commandSending || (!item.hold && Boolean(activeHoldAction))"
                @pointerdown.prevent="item.hold && startHoldAction(item, $event)"
                @pointerup.prevent="item.hold && stopHoldAction($event)"
                @pointercancel.prevent="item.hold && stopHoldAction($event)"
                @lostpointercapture="item.hold && stopHoldAction($event)"
                @click="!item.hold && sendControlAction(item.action, item.payload)"
                @contextmenu.prevent
              >
                {{ item.arrow || item.label }}
              </button>
            </div>
          </div>

        </div>

        <div class="video-footer">
          <div class="footer-card">
            <strong>今日巡检时长</strong>
            <span>{{ overview.header.today_patrol_minutes || 0 }} 分钟</span>
          </div>
          <div class="footer-card area-card">
            <strong>当前巡检区域</strong>
            <span>{{ latestRobot?.area }}</span>
          </div>
          <button class="takeover-btn" :disabled="!hasLiveStream || commandSending" @click="enterTakeover">
            {{ commandSending ? '下发中...' : '接管' }}
          </button>
          <div class="footer-card">
            <strong>设备电量</strong>
            <span>{{ latestRobot?.battery_level }}%</span>
          </div>
          <button class="danger-btn" :disabled="commandSending" @click="emergencyStop">
            {{ commandSending ? '下发中...' : '紧急停止' }}
          </button>
        </div>
      </section>

      <section class="board-grid">
        <article class="panel compact-panel">
          <div class="panel-head">
            <div>
              <h3>运行概况</h3>
              <p>今日巡检与告警统计</p>
            </div>
          </div>
          <div class="metrics-grid">
            <div class="metric-card">
              <strong>{{ overview.summary.online_robot_count }}</strong>
              <span>在线机器人</span>
            </div>
            <div class="metric-card">
              <strong>{{ overview.summary.pending_event_count }}</strong>
              <span>待处理事件</span>
            </div>
            <div class="metric-card">
              <strong>{{ overview.summary.resolved_event_count }}</strong>
              <span>已处理事件</span>
            </div>
            <div class="metric-card">
              <strong>{{ overview.summary.today_alert_count }}</strong>
              <span>今日告警</span>
            </div>
          </div>
        </article>

        <article class="panel compact-panel">
          <div class="panel-head">
            <div>
              <h3>设备列表</h3>
              <p>{{ switchingRobot ? '正在切换设备...' : '点击设备后同步切换监控画面、控制台与历史事件' }}</p>
            </div>
          </div>
          <div class="robot-list">
            <button
              v-for="robot in robots"
              :key="robot.id"
              type="button"
              class="robot-item"
              :class="{ selected: latestRobot?.id === robot.id }"
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
        </article>
      </section>
    </div>

    <aside class="side-column">
      <section class="panel side-panel">
        <div class="panel-head">
          <div>
            <h3>远程控制台</h3>
            <p>支持快速切换播报模板，并同步查看现场联动状态</p>
          </div>
          <span class="panel-badge">Control</span>
        </div>
        <div class="speaker-box">
          <textarea v-model="speakerText" maxlength="500" placeholder="输入什么文字，机器狗就播报什么文字"></textarea>
          <div class="quick-actions">
            <button
              v-for="item in speechTemplates"
              :key="item.id"
              class="chip"
              :class="{ active: selectedSpeakerTemplateId === item.id }"
              @click="setSpeakerTemplate(item)"
            >
              {{ item.name }}
            </button>
          </div>
          <div class="template-editor">
            <input v-model="templateName" maxlength="64" placeholder="文案名称，例如：重点路段" />
            <button class="ghost-btn" :disabled="templateSaving" @click="saveSpeechTemplate">
              {{ selectedSpeakerTemplateId ? '更新文案' : '保存文案' }}
            </button>
            <button class="ghost-btn" :disabled="templateSaving" @click="startNewSpeechTemplate">新建</button>
            <button class="danger-btn" :disabled="!selectedSpeakerTemplateId || templateSaving" @click="removeSpeechTemplate">删除</button>
          </div>
          <div class="action-row">
            <button class="primary-btn" :disabled="audioCommandSending" @click="beginSpeak">
              {{ audioCommandSending ? '语音生成中' : '开始喊话' }}
            </button>
            <button class="ghost-btn" :disabled="audioCommandSending" @click="previewVoice">语音预览</button>
          </div>
          <div class="recording-box">
            <div class="recording-status" :class="{ active: recording }">
              <span>{{ recording ? '录音中' : recordedBlob ? '录音完成' : '未录音' }}</span>
              <strong>{{ recordingDurationLabel() }}</strong>
            </div>
            <div class="action-row">
              <button class="ghost-btn" :disabled="audioCommandSending || recording" @click="startRecording">录音</button>
              <button class="ghost-btn" :disabled="!recording" @click="stopRecording">停止</button>
              <button class="primary-btn" :disabled="!recordedBlob || audioCommandSending || recording" @click="playRecordedAudio">
                播放录音
              </button>
            </div>
            <audio v-if="recordedUrl" class="recording-preview" :src="recordedUrl" controls></audio>
          </div>
          <div class="mini-row">
            <div class="mini-card">当前音量 {{ latestRobot?.speaker_volume }}%</div>
            <div class="mini-card">喊话链路 已接入</div>
          </div>
        </div>
      </section>

      <section class="panel side-panel">
        <div class="panel-head">
          <div>
            <h3>历史事件识别</h3>
          </div>
          <span class="panel-badge">History</span>
        </div>
        <div class="event-list">
          <article v-for="(event, index) in latestRobot?.recent_events || []" :key="event.id" class="event-card">
            <div class="event-thumb" :style="eventThumbStyle(index)"></div>
            <div class="event-main">
              <strong>{{ event.title }}</strong>
              <span>{{ event.location }}</span>
              <small>{{ formatEventTime(event.detected_at) }}</small>
            </div>
            <div class="event-side">
              <span :class="['risk-chip', event.status === 'pending' ? event.status : 'review-chip']">
                {{ event.status === 'pending' ? event.status_label : event.review_result_label || '确认违规' }}
              </span>
            </div>
          </article>
        </div>
      </section>
    </aside>

    <AppToast :show="visible" :message="toastMessage" :variant="toastVariant" />
  </section>

  <section v-else-if="!loading" class="page-grid overview-error">
    <div class="error-card">
      <strong>数据加载失败</strong>
      <p>{{ loadError || '未能获取监测数据，请稍后重试。' }}</p>
    </div>
  </section>
</template>
