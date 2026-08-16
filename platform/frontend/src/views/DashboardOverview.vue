<script setup>
import Hls from 'hls.js'
import mpegts from 'mpegts.js'
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'

import AppToast from '../components/AppToast.vue'
import { useToast } from '../composables/useToast'
import {
  API_BASE,
  createSpeechCategory,
  createSpeechTemplate,
  deleteSpeechCategory,
  deleteRecordedAudio,
  deleteSpeechTemplate,
  fetchAlertSkills,
  fetchOverview,
  fetchRecordedAudios,
  fetchRobotDetail,
  fetchRobotPersonDetections,
  fetchRobots,
  fetchRobotStatus,
  fetchSpeechCategories,
  fetchSpeechTemplates,
  playSavedRecording,
  previewAlertSkill,
  sendRecordedAudioCommand,
  sendRobotCommand,
  sendTextToSpeechCommand,
  synthesizeSpeech,
  updateSpeechTemplate,
  updateAlertSkill,
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
const speechCategories = ref([])
const speechTemplates = ref([])
const alertSkills = ref([])
const alertSkillSaving = ref('')
const alertSkillPreviewing = ref('')
const robotAudioStatus = ref(null)
const selectedCategoryFilter = ref('all')
const selectedSpeakerTemplateId = ref(null)
const selectedRecordingId = ref(null)
const selectedTemplateCategoryId = ref(null)
const templateName = ref('驶离提醒')
const newCategoryName = ref('')
const templateSaving = ref(false)
const categorySaving = ref(false)
const audioCommandSending = ref(false)
const recording = ref(false)
const recordedBlob = ref(null)
const recordedUrl = ref('')
const recordingSeconds = ref(0)
const recordingTitle = ref('现场录音')
const recordingCategoryId = ref(null)
const savedRecordings = ref([])
const recordingSaving = ref(false)
const videoRef = ref(null)
const videoStageRef = ref(null)
const streamUnavailable = ref(false)
const liveDetectionState = ref({ detections: [] })
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
let audioStatusTimer = null
let liveDetectionTimer = null
const realtimeEventIds = new Set()
const HOLD_REPEAT_MS = 300
const AUDIO_COMMAND_COOLDOWN_MS = 3000
const { toastMessage, toastVariant, visible, showToast } = useToast()

const eventImages = ['/images/event-1.jpg', '/images/event-2.jpg', '/images/event-3.jpg']
const latestRobot = computed(() => selectedRobot.value || overview.value?.latest_robot || null)
const speaker3588Status = computed(() => robotAudioStatus.value?.speaker_3588 || null)
const speakerNxStatus = computed(() => robotAudioStatus.value?.speaker_nx || null)
const speechLibraryItems = computed(() => [
  ...speechTemplates.value.map((item) => ({ ...item, title: item.name, source_type: 'tts' })),
  ...savedRecordings.value.map((item) => ({ ...item, name: item.title, text: item.transcript, source_type: 'recording' })),
])
const filteredSpeechItems = computed(() => {
  if (selectedCategoryFilter.value === 'all') return speechLibraryItems.value
  if (selectedCategoryFilter.value === 'uncategorized') {
    return speechLibraryItems.value.filter((item) => !item.category)
  }
  return speechLibraryItems.value.filter((item) => item.category === selectedCategoryFilter.value)
})
const selectedSourceType = computed(() => selectedRecordingId.value ? 'recording' : 'tts')
const liveEvent = computed(() => latestRobot.value?.recent_events?.[0] || overview.value?.live_event || null)
const livePlayUrls = computed(() => latestRobot.value?.play_urls || {})
const hasLiveStream = computed(() => !streamUnavailable.value && Boolean(livePlayUrls.value.flv || livePlayUrls.value.hls))
const bicycleDetections = computed(() => (liveDetectionState.value?.detections || []).filter((item) =>
  ['bicycle', 'bike', '自行车'].includes(String(item.label || '').toLowerCase()),
))
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

async function refreshLiveDetections() {
  const robotId = latestRobot.value?.id
  if (!robotId) {
    liveDetectionState.value = { detections: [] }
    return
  }
  try {
    const state = await fetchRobotPersonDetections(robotId)
    if (latestRobot.value?.id === robotId) liveDetectionState.value = state
  } catch {
    liveDetectionState.value = { detections: [] }
  }
}

function bicycleDetectionStyle(item) {
  const stage = videoStageRef.value
  const frameWidth = Number(liveDetectionState.value?.frame_width || 1)
  const frameHeight = Number(liveDetectionState.value?.frame_height || 1)
  const stageWidth = Number(stage?.clientWidth || 1)
  const stageHeight = Number(stage?.clientHeight || 1)
  const scale = Math.max(stageWidth / frameWidth, stageHeight / frameHeight)
  const renderedWidth = frameWidth * scale
  const renderedHeight = frameHeight * scale
  const offsetX = (stageWidth - renderedWidth) / 2
  const offsetY = (stageHeight - renderedHeight) / 2
  const box = item.bbox || {}
  return {
    left: `${offsetX + Number(box.x || 0) * scale}px`,
    top: `${offsetY + Number(box.y || 0) * scale}px`,
    width: `${Number(box.width || 0) * scale}px`,
    height: `${Number(box.height || 0) * scale}px`,
  }
}

function setSpeakerTemplate(template) {
  speakerText.value = template.text
  templateName.value = template.name
  selectedSpeakerTemplateId.value = template.id
  selectedRecordingId.value = null
  selectedTemplateCategoryId.value = template.category || null
  showToast('已切换喊话模板')
}

function setSpeechItem(item) {
  if (item.source_type === 'recording') {
    selectedRecordingId.value = item.id
    selectedSpeakerTemplateId.value = null
    templateName.value = item.title
    speakerText.value = item.transcript || (item.asr_status === 'failed' ? '语音识别失败，可直接播放原录音' : '语音识别中')
    selectedTemplateCategoryId.value = item.category || null
    showToast('已切换到录音喊话，开始喊话将播放原录音')
    return
  }
  setSpeakerTemplate(item)
}

async function loadSpeechLibrary() {
  const [categories, templates] = await Promise.all([fetchSpeechCategories(), fetchSpeechTemplates()])
  speechCategories.value = categories
  speechTemplates.value = templates
  const selected = speechTemplates.value.find((item) => item.id === selectedSpeakerTemplateId.value) ||
    speechTemplates.value.find((item) => item.name === templateName.value) ||
    speechTemplates.value[0]
  if (selected && selectedSpeakerTemplateId.value === null) setSpeakerTemplate(selected)
}

function normalizeAlertSkills(items) {
  return items.map((item) => ({
    ...item,
    template_id: item.template?.id || null,
    saved_template_id: item.template?.id || null,
    saved_enabled: item.enabled,
  }))
}

function alertSkillHasUnsavedChanges(skill) {
  return skill.template_id !== skill.saved_template_id || skill.enabled !== skill.saved_enabled
}

async function saveAlertSkill(skill) {
  if (!skill.template_id || alertSkillSaving.value) return
  alertSkillSaving.value = skill.skill_key
  try {
    const saved = await updateAlertSkill(skill.skill_key, {
      template_id: skill.template_id,
      enabled: skill.enabled,
    })
    const index = alertSkills.value.findIndex((item) => item.skill_key === skill.skill_key)
    if (index >= 0) alertSkills.value[index] = normalizeAlertSkills([saved])[0]
    showToast(`${skill.display_name}播报模板已绑定`)
  } catch (error) {
    showToast(error.message || '告警播报绑定失败')
  } finally {
    alertSkillSaving.value = ''
  }
}

async function previewBoundAlertSkill(skill) {
  const robot = latestRobot.value
  if (!robot?.id || !skill.template_id || alertSkillPreviewing.value || alertSkillHasUnsavedChanges(skill)) return
  alertSkillPreviewing.value = skill.skill_key
  try {
    await previewAlertSkill(skill.skill_key, robot.id)
    showToast(`${skill.display_name}双音响试播已下发`)
  } catch (error) {
    showToast(error.message || '双音响试播下发失败')
  } finally {
    alertSkillPreviewing.value = ''
  }
}

async function loadRecordingLibrary() {
  savedRecordings.value = await fetchRecordedAudios()
}

async function beginSpeak() {
  const robot = latestRobot.value
  if (!robot?.id || audioCommandSending.value) return
  if (selectedRecordingId.value) {
    const recordingItem = savedRecordings.value.find((item) => item.id === selectedRecordingId.value)
    if (recordingItem) await replaySavedRecording(recordingItem)
    return
  }
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
  if (selectedRecordingId.value) {
    const recordingItem = savedRecordings.value.find((item) => item.id === selectedRecordingId.value)
    if (!recordingItem?.audio_url) return
    if (previewPlayer) previewPlayer.pause()
    previewPlayer = new Audio(recordingItem.audio_url)
    try {
      await previewPlayer.play()
      showToast('正在预览原录音')
    } catch (_error) {
      showToast('录音预览失败')
    }
    return
  }
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
      ? await updateSpeechTemplate(selectedSpeakerTemplateId.value, { name, text, category: selectedTemplateCategoryId.value })
      : await createSpeechTemplate({ name, text, category: selectedTemplateCategoryId.value })
    selectedSpeakerTemplateId.value = saved.id
    await loadSpeechLibrary()
    showToast('喊话文案已保存')
  } catch (error) {
    showToast(error.message || '保存文案失败')
  } finally {
    templateSaving.value = false
  }
}

function startNewSpeechTemplate() {
  selectedSpeakerTemplateId.value = null
  selectedRecordingId.value = null
  selectedTemplateCategoryId.value = typeof selectedCategoryFilter.value === 'number'
    ? selectedCategoryFilter.value
    : null
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
    await loadSpeechLibrary()
    showToast('喊话文案已删除')
  } catch (error) {
    showToast(error.message || '删除文案失败')
  } finally {
    templateSaving.value = false
  }
}

async function addSpeechCategory() {
  const name = newCategoryName.value.trim()
  if (!name || categorySaving.value) {
    if (!name) showToast('请输入分类名称')
    return
  }
  categorySaving.value = true
  try {
    const category = await createSpeechCategory(name)
    newCategoryName.value = ''
    selectedCategoryFilter.value = category.id
    selectedTemplateCategoryId.value = category.id
    await loadSpeechLibrary()
    showToast('播报分类已创建')
  } catch (error) {
    showToast(error.message || '创建分类失败')
  } finally {
    categorySaving.value = false
  }
}

async function removeSpeechCategory() {
  const categoryId = selectedCategoryFilter.value
  if (typeof categoryId !== 'number' || categorySaving.value) return
  categorySaving.value = true
  try {
    await deleteSpeechCategory(categoryId)
    if (selectedTemplateCategoryId.value === categoryId) selectedTemplateCategoryId.value = null
    selectedCategoryFilter.value = 'all'
    await loadSpeechLibrary()
    showToast('分类已删除，原有文案已转为未分类')
  } catch (error) {
    showToast(error.message || '删除分类失败')
  } finally {
    categorySaving.value = false
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

async function saveRecordedAudio(playNow = false) {
  const robot = latestRobot.value
  const title = recordingTitle.value.trim()
  if (!robot?.id || !recordedBlob.value || recordingSaving.value || audioCommandSending.value) return
  if (!title) {
    showToast('请输入录音标题')
    return
  }
  const extension = recordedBlob.value.type.includes('ogg') ? 'ogg' : 'webm'
  const file = new File([recordedBlob.value], `dashboard-recording.${extension}`, {
    type: recordedBlob.value.type || 'audio/webm',
  })
  const commandStartedAt = Date.now()
  recordingSaving.value = true
  if (playNow) audioCommandSending.value = true
  try {
    const result = await sendRecordedAudioCommand(robot.id, file, {
      title,
      category: recordingCategoryId.value,
      playNow,
    })
    await loadRecordingLibrary()
    const saved = savedRecordings.value.find((item) => item.id === result.recording?.id)
    if (saved) setSpeechItem({ ...saved, source_type: 'recording' })
    showToast(playNow ? '录音已保存并下发播放' : '录音已保存')
  } catch (error) {
    showToast(error.message || '录音保存失败')
  } finally {
    if (playNow) {
      const cooldownRemaining = AUDIO_COMMAND_COOLDOWN_MS - (Date.now() - commandStartedAt)
      if (cooldownRemaining > 0) {
        await new Promise((resolve) => window.setTimeout(resolve, cooldownRemaining))
      }
      audioCommandSending.value = false
    }
    recordingSaving.value = false
  }
}

async function replaySavedRecording(recordingItem) {
  const robot = latestRobot.value
  if (!robot?.id || audioCommandSending.value) return
  audioCommandSending.value = true
  try {
    await playSavedRecording(robot.id, recordingItem.id)
    showToast(`已下发录音“${recordingItem.title}”`)
  } catch (error) {
    showToast(error.message || '录音播放指令下发失败')
  } finally {
    window.setTimeout(() => { audioCommandSending.value = false }, AUDIO_COMMAND_COOLDOWN_MS)
  }
}

async function removeRecordedAudio(recordingItem) {
  if (recordingSaving.value || !window.confirm(`确定删除录音“${recordingItem.title}”吗？`)) return
  recordingSaving.value = true
  try {
    await deleteRecordedAudio(recordingItem.id)
    await loadRecordingLibrary()
    showToast('录音已删除')
  } catch (error) {
    showToast(error.message || '删除录音失败')
  } finally {
    recordingSaving.value = false
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
  liveDetectionState.value = { detections: [] }
  try {
    const [robotDetail, liveStatus] = await Promise.all([
      fetchRobotDetail(robotId),
      fetchRobotStatus(robotId).catch(() => null),
    ])
    selectedRobot.value = robotDetail
    robotAudioStatus.value = liveStatus?.status?.audio || null
    streamUnavailable.value = false
    if (announce) showToast(`已切换至 ${selectedRobot.value.name}`)
  } finally {
    switchingRobot.value = false
  }
}

async function refreshAudioStatus() {
  const robotId = latestRobot.value?.id
  if (!robotId) return
  try {
    const liveStatus = await fetchRobotStatus(robotId)
    if (latestRobot.value?.id === robotId) {
      robotAudioStatus.value = liveStatus?.status?.audio || null
    }
  } catch (_error) {
    // Retain the last valid device sample during a transient request failure.
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
    const [overviewData, robotData, categoryData, templateData, recordingData, alertSkillData] = await Promise.all([
      fetchOverview(),
      fetchRobots(),
      fetchSpeechCategories().catch(() => []),
      fetchSpeechTemplates().catch(() => []),
      fetchRecordedAudios().catch(() => []),
      fetchAlertSkills().catch(() => []),
    ])
    overview.value = overviewData
    robots.value = robotData
    speechCategories.value = categoryData
    speechTemplates.value = templateData
    savedRecordings.value = recordingData
    alertSkills.value = normalizeAlertSkills(alertSkillData)
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
  audioStatusTimer = window.setInterval(refreshAudioStatus, 5000)
  await refreshLiveDetections()
  liveDetectionTimer = window.setInterval(refreshLiveDetections, 400)
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
  if (audioStatusTimer) window.clearInterval(audioStatusTimer)
  if (liveDetectionTimer) window.clearInterval(liveDetectionTimer)
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

          <div
            v-for="detection in bicycleDetections"
            :key="detection.track_id"
            class="bicycle-detection-box"
            :style="bicycleDetectionStyle(detection)"
          >
            <span>自行车 · {{ Math.round(Number(detection.confidence || 0) * 100) }}%</span>
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

        <div class="alert-skill-config">
          <div class="alert-skill-heading">
            <div>
              <strong>告警技能</strong>
              <span>双音响同步播报，并应用高增益硬削波告警音效</span>
            </div>
            <div class="alert-skill-badges">
              <span class="panel-badge">双音响</span>
              <span class="panel-badge alert-effect-badge">炸麦</span>
            </div>
          </div>
          <div class="alert-audio-status" aria-label="告警音效与扬声器状态">
            <span><small>音频增益</small><strong>+12 dB</strong></span>
            <span><small>削波效果</small><strong>硬削波 55%</strong></span>
            <span><small>峰值保护</small><strong>98%</strong></span>
            <span :class="{ offline: !speaker3588Status?.online }">
              <small>3588 音响</small>
              <strong>{{ speaker3588Status?.online ? `${speaker3588Status.volume_percent}%` : '离线' }}</strong>
            </span>
            <span :class="{ offline: !speakerNxStatus?.online }">
              <small>NX 音响</small>
              <strong>{{ speakerNxStatus?.online ? `${speakerNxStatus.volume_percent}%` : '离线' }}</strong>
            </span>
          </div>
          <div class="alert-skill-list">
            <div v-for="skill in alertSkills" :key="skill.skill_key" class="alert-skill-row">
              <label class="alert-skill-toggle">
                <input v-model="skill.enabled" type="checkbox" />
                <span>{{ skill.display_name }}</span>
              </label>
              <select v-model="skill.template_id" :aria-label="`${skill.display_name}播报模板`">
                <option v-for="template in speechTemplates" :key="template.id" :value="template.id">
                  {{ template.name }}
                </option>
              </select>
              <span class="alert-skill-copy">
                {{ speechTemplates.find((template) => template.id === skill.template_id)?.text || '未配置播报模板' }}
              </span>
              <div class="alert-skill-actions">
                <button
                  type="button"
                  class="ghost-btn"
                  :disabled="!skill.template_id || Boolean(alertSkillSaving) || Boolean(alertSkillPreviewing)"
                  @click="saveAlertSkill(skill)"
                >
                  {{ alertSkillSaving === skill.skill_key ? '保存中' : '保存' }}
                </button>
                <button
                  type="button"
                  class="primary-btn"
                  :title="alertSkillHasUnsavedChanges(skill) ? '请先保存当前模板配置' : '在当前机器人上使用两个音响试播'"
                  :disabled="!skill.template_id || !latestRobot?.id || Boolean(alertSkillSaving) || Boolean(alertSkillPreviewing) || alertSkillHasUnsavedChanges(skill)"
                  @click="previewBoundAlertSkill(skill)"
                >
                  {{ alertSkillPreviewing === skill.skill_key ? '试播中' : '试播' }}
                </button>
              </div>
            </div>
          </div>
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
          <div class="speech-category-manager">
            <div class="speech-category-tabs">
              <button class="chip" :class="{ active: selectedCategoryFilter === 'all' }" @click="selectedCategoryFilter = 'all'">全部</button>
              <button
                v-for="category in speechCategories"
                :key="category.id"
                class="chip"
                :class="{ active: selectedCategoryFilter === category.id }"
                @click="selectedCategoryFilter = category.id"
              >
                {{ category.name }}（{{ (category.template_count || 0) + (category.recording_count || 0) }}）
              </button>
              <button class="chip" :class="{ active: selectedCategoryFilter === 'uncategorized' }" @click="selectedCategoryFilter = 'uncategorized'">未分类</button>
            </div>
            <div class="speech-category-create">
              <input v-model="newCategoryName" maxlength="64" placeholder="新分类名称" @keyup.enter="addSpeechCategory" />
              <button class="ghost-btn" :disabled="categorySaving" @click="addSpeechCategory">创建分类</button>
              <button class="danger-btn" :disabled="typeof selectedCategoryFilter !== 'number' || categorySaving" @click="removeSpeechCategory">删除当前分类</button>
            </div>
          </div>
          <div class="quick-actions">
            <button
              v-for="item in filteredSpeechItems"
              :key="`${item.source_type}-${item.id}`"
              class="chip"
              :class="[
                `speech-source-${item.source_type}`,
                { active: item.source_type === 'recording' ? selectedRecordingId === item.id : selectedSpeakerTemplateId === item.id },
              ]"
              @click="setSpeechItem(item)"
            >
              <span>{{ item.name }}</span>
              <small>{{ item.source_type === 'recording' ? '录音' : '文案' }}</small>
            </button>
          </div>
          <div class="template-editor">
            <input v-model="templateName" maxlength="64" :readonly="selectedSourceType === 'recording'" placeholder="新建标题，例如：重点路段" />
            <select v-model="selectedTemplateCategoryId" :disabled="selectedSourceType === 'recording'">
              <option :value="null">未分类</option>
              <option v-for="category in speechCategories" :key="category.id" :value="category.id">{{ category.name }}</option>
            </select>
            <button class="ghost-btn" :disabled="templateSaving || selectedSourceType === 'recording'" @click="saveSpeechTemplate">
              {{ selectedSpeakerTemplateId ? '更新文案' : '保存文案' }}
            </button>
            <button class="ghost-btn" :disabled="templateSaving" @click="startNewSpeechTemplate">新建</button>
            <button class="danger-btn" :disabled="!selectedSpeakerTemplateId || templateSaving || selectedSourceType === 'recording'" @click="removeSpeechTemplate">删除</button>
          </div>
          <textarea
            v-model="speakerText"
            maxlength="500"
            :readonly="selectedSourceType === 'recording'"
            :class="{ 'recording-transcript': selectedSourceType === 'recording' }"
            :placeholder="selectedSourceType === 'recording' ? '录音保存后会自动识别文字' : '输入什么文字，机器狗就播报什么文字'"
          ></textarea>
          <div class="action-row">
            <button class="primary-btn" :disabled="audioCommandSending" @click="beginSpeak">
              {{ audioCommandSending ? '下发中' : '开始喊话' }}
            </button>
            <button class="ghost-btn" :disabled="audioCommandSending" @click="previewVoice">
              {{ selectedSourceType === 'recording' ? '原录音预览' : '语音预览' }}
            </button>
          </div>
          <div class="recording-box">
            <div class="recording-status" :class="{ active: recording }">
              <span>{{ recording ? '录音中' : recordedBlob ? '录音完成' : '未录音' }}</span>
              <strong>{{ recordingDurationLabel() }}</strong>
            </div>
            <div class="recording-meta">
              <input v-model="recordingTitle" maxlength="128" placeholder="录音标题，例如：南门劝导" />
              <select v-model="recordingCategoryId">
                <option :value="null">未分类</option>
                <option v-for="category in speechCategories" :key="category.id" :value="category.id">{{ category.name }}</option>
              </select>
            </div>
            <div class="action-row">
              <button class="ghost-btn" :disabled="audioCommandSending || recordingSaving || recording" @click="startRecording">录音</button>
              <button class="ghost-btn" :disabled="!recording" @click="stopRecording">停止</button>
              <button class="ghost-btn" :disabled="!recordedBlob || recordingSaving || recording" @click="saveRecordedAudio(false)">
                {{ recordingSaving ? '保存中' : '仅保存' }}
              </button>
              <button class="primary-btn" :disabled="!recordedBlob || audioCommandSending || recordingSaving || recording" @click="saveRecordedAudio(true)">
                保存并播放
              </button>
            </div>
            <audio v-if="recordedUrl" class="recording-preview" :src="recordedUrl" controls></audio>
            <div v-if="savedRecordings.length" class="recording-library">
              <div class="recording-library-title">已保存录音</div>
              <article v-for="item in savedRecordings" :key="item.id" class="recording-item">
                <div>
                  <strong class="recording-title">{{ item.title }} <em>录音</em></strong>
                  <small class="recording-copy">
                    {{ item.transcript || (item.asr_status === 'failed' ? `识别失败：${item.asr_error || '未识别到文字'}` : '文案识别中') }}
                  </small>
                  <small>{{ item.category_name || '未分类' }} · {{ formatEventTime(item.created_at) }}</small>
                </div>
                <div class="recording-item-actions">
                  <button class="ghost-btn" :disabled="audioCommandSending" @click="replaySavedRecording(item)">开始喊话</button>
                  <button class="danger-btn" :disabled="recordingSaving" @click="removeRecordedAudio(item)">删除</button>
                </div>
              </article>
            </div>
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
