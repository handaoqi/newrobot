import { createTTLCache } from './cache'

export const API_BASE = (import.meta.env.VITE_API_BASE || '/api').replace(/\/$/, '')

export const listCache = createTTLCache({ ttlMs: 30_000 })
const ROBOT_LIST_TIMEOUT_MS = 8_000

function summaryPath(path) {
  return `${path}${path.includes('?') ? '&' : '?'}view=summary`
}

async function request(path, options = {}) {
  const { timeoutMs, signal: callerSignal, ...fetchOptions } = options
  const token = localStorage.getItem('inspection_token')
  const headers = { ...(fetchOptions.headers || {}) }
  if (!(fetchOptions.body instanceof FormData)) headers['Content-Type'] = 'application/json'

  if (token) {
    headers.Authorization = `Token ${token}`
  }

  let timeoutHandle = null
  let timeoutTriggered = false
  let requestController = null
  let removeCallerAbortListener = null
  let requestSignal = callerSignal
  if (timeoutMs > 0) {
    requestController = new AbortController()
    requestSignal = requestController.signal
    const abortFromCaller = () => requestController.abort()
    if (callerSignal) {
      if (callerSignal.aborted) requestController.abort()
      else {
        callerSignal.addEventListener('abort', abortFromCaller, { once: true })
        removeCallerAbortListener = () => callerSignal.removeEventListener('abort', abortFromCaller)
      }
    }
    timeoutHandle = setTimeout(() => {
      timeoutTriggered = true
      requestController.abort()
    }, timeoutMs)
  }

  let response
  try {
    response = await fetch(`${API_BASE}${path}`, {
      ...fetchOptions,
      headers,
      ...(requestSignal ? { signal: requestSignal } : {}),
    })
  } catch (error) {
    if (timeoutTriggered && !callerSignal?.aborted) {
      const timeoutError = new Error(`请求超时（${Math.round(timeoutMs / 1000)} 秒）`)
      timeoutError.code = 'REQUEST_TIMEOUT'
      throw timeoutError
    }
    throw error
  } finally {
    if (timeoutHandle !== null) clearTimeout(timeoutHandle)
    removeCallerAbortListener?.()
  }

  if (!response.ok) {
    const errorPayload = await response.json().catch(() => ({ detail: '请求失败' }))
    const error = new Error(errorPayload.detail || '请求失败')
    error.payload = errorPayload
    error.status = response.status
    // 会话过期/令牌失效：全站统一清理并跳转登录，避免各页因未捕获而白屏卡死。
    if (response.status === 401 && path !== '/auth/login/') {
      localStorage.removeItem('inspection_token')
      localStorage.removeItem('inspection_user')
      if (typeof window !== 'undefined' && !window.location.pathname.startsWith('/login')) {
        window.location.assign('/login')
      }
    }
    throw error
  }

  if (response.status === 204) return {}
  return response.json()
}

export async function login(payload) {
  return request('/auth/login/', {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchOverview() {
  return request('/dashboard/overview/')
}

export async function fetchAnalytics() {
  return request('/dashboard/analytics/')
}

export async function fetchEvents({
  status = '',
  page = 1,
  pageSize = 10,
  search = '',
  ordering = 'detected_desc',
  detectedFrom = '',
  detectedTo = '',
} = {}) {
  const params = new URLSearchParams({
    page: String(page),
    page_size: String(pageSize),
  })
  if (status) params.set('status', status)
  if (search.trim()) params.set('search', search.trim())
  if (ordering) params.set('ordering', ordering)
  if (detectedFrom) params.set('detected_from', detectedFrom)
  if (detectedTo) params.set('detected_to', detectedTo)
  const query = `?${params.toString()}`
  return request(`/events/${query}`)
}

export async function fetchRobots({ force = false } = {}) {
  const load = () => request('/robots/', { timeoutMs: ROBOT_LIST_TIMEOUT_MS })
  return force ? load() : listCache.get('robots', load)
}

export async function fetchRobotDetail(robotId, { signal } = {}) {
  return request(`/robots/${robotId}/`, { signal })
}

export async function fetchRobotPersonDetections(robotId, { signal } = {}) {
  return request(`/robots/${robotId}/person-detections/`, { signal })
}

export async function setRobotPersonDetection(robotId, enabled) {
  return request(`/robots/${robotId}/person-detections/`, {
    method: 'POST',
    body: JSON.stringify({ enabled }),
  })
}

export async function sendRobotCommand(robotId, payload) {
  return request(`/robots/${robotId}/commands/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function startRobotChargingDock(robotId, payload = {}) {
  return request(`/robots/${robotId}/charging-dock/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchRobotCommand(robotId, commandId) {
  return request(`/robots/${robotId}/commands/${commandId}/`)
}

export async function sendRecordedAudioCommand(robotId, file, { title = '现场录音', category = null, playNow = true } = {}) {
  const formData = new FormData()
  formData.append('file', file)
  formData.append('title', title)
  if (category) formData.append('category', String(category))
  formData.append('play_now', playNow ? 'true' : 'false')
  return request(`/robots/${robotId}/commands/audio-recording/`, {
    method: 'POST',
    body: formData,
  })
}

export async function setRobotStreamAudioCapture(robotId, enabled) {
  return request(`/robots/${robotId}/commands/stream-audio/`, {
    method: 'POST',
    body: JSON.stringify({ enabled }),
  })
}

export async function fetchRobotStreamAudioCommand(robotId, commandId) {
  return request(`/robots/${robotId}/commands/stream-audio/${commandId}/`)
}

export async function fetchRecordedAudios() {
  return request('/recorded-audio/')
}

export async function playSavedRecording(robotId, recordingId) {
  return request(`/robots/${robotId}/commands/recorded-audio/${recordingId}/`, {
    method: 'POST',
    body: '{}',
  })
}

export async function deleteRecordedAudio(recordingId) {
  return request(`/recorded-audio/${recordingId}/`, { method: 'DELETE' })
}

export async function sendTextToSpeechCommand(robotId, text, audioName = '实时文字喊话') {
  return request(`/robots/${robotId}/commands/tts/`, {
    method: 'POST',
    body: JSON.stringify({ text, audio_name: audioName }),
  })
}

export async function synthesizeSpeech(text) {
  return request('/speech/synthesize/', {
    method: 'POST',
    body: JSON.stringify({ text }),
  })
}

export async function fetchSpeechTemplates({ signal } = {}) {
  if (signal) return request('/speech-templates/', { signal })
  return listCache.get('speech-templates', () => request('/speech-templates/', { signal }))
}

export async function createSpeechTemplate(payload) {
  const result = await request('/speech-templates/', { method: 'POST', body: JSON.stringify(payload) })
  listCache.invalidate('speech-templates')
  return result
}

export async function updateSpeechTemplate(templateId, payload) {
  const result = await request(`/speech-templates/${templateId}/`, { method: 'PATCH', body: JSON.stringify(payload) })
  listCache.invalidate('speech-templates')
  return result
}

export async function deleteSpeechTemplate(templateId) {
  const result = await request(`/speech-templates/${templateId}/`, { method: 'DELETE' })
  listCache.invalidate('speech-templates')
  return result
}

export async function fetchAlertSkills() {
  return request('/alert-skills/')
}

export async function updateAlertSkill(skillKey, payload) {
  return request(`/alert-skills/${skillKey}/`, { method: 'PATCH', body: JSON.stringify(payload) })
}

export async function previewAlertSkill(skillKey, robotId) {
  return request(`/alert-skills/${skillKey}/preview/`, {
    method: 'POST',
    body: JSON.stringify({ robot_id: robotId }),
  })
}

export async function fetchValidationRecordings() {
  return request('/validation-recordings/')
}

export async function fetchValidationProfiles() {
  return request('/validation-profiles/')
}

export async function fetchValidationRunners() {
  return request('/validation-runners/')
}

export async function fetchValidationJobs() {
  return request('/validation-jobs/')
}

export async function fetchValidationJob(jobId) {
  return request(`/validation-jobs/${jobId}/`)
}

export async function createValidationJob(payload) {
  return request('/validation-jobs/', { method: 'POST', body: JSON.stringify(payload) })
}

export async function cancelValidationJob(jobId) {
  return request(`/validation-jobs/${jobId}/cancel/`, { method: 'POST', body: '{}' })
}

export async function approveValidationBaseline(jobId) {
  return request(`/validation-jobs/${jobId}/approve-baseline/`, { method: 'POST', body: '{}' })
}

export async function createValidationLiveTicket(jobId) {
  return request(`/validation-jobs/${jobId}/live-ticket/`, { method: 'POST', body: '{}' })
}

export async function uploadValidationRecording(file, { label = '', robot = null } = {}) {
  let initiation
  try {
    initiation = await request('/validation-recordings/uploads/initiate/', {
      method: 'POST',
      body: JSON.stringify({
        label: label || file.name,
        file_name: file.name,
        ...(robot ? { robot } : {}),
      }),
    })
  } catch (error) {
    if (error.status !== 409) throw error
    const form = new FormData()
    form.append('file', file)
    form.append('label', label || file.name)
    form.append('storage_format', 'mcap')
    if (robot) form.append('robot', String(robot))
    return request('/validation-recordings/', { method: 'POST', body: form })
  }

  const recording = initiation.recording
  const partSize = initiation.part_size_bytes
  const partCount = Math.ceil(file.size / partSize)
  const partNumbers = Array.from({ length: partCount }, (_, index) => index + 1)
  const spec = await request(`/validation-recordings/${recording.id}/uploads/parts/`, {
    method: 'POST',
    body: JSON.stringify({ part_numbers: partNumbers }),
  })
  const parts = []
  for (const item of spec.parts) {
    const start = (item.part_number - 1) * partSize
    const response = await fetch(item.url, {
      method: 'PUT',
      body: file.slice(start, Math.min(file.size, start + partSize)),
    })
    if (!response.ok) throw new Error(`第 ${item.part_number} 个分片上传失败`)
    const etag = response.headers.get('ETag')
    if (!etag) throw new Error('对象存储未暴露 ETag，请检查 MinIO CORS 配置')
    parts.push({ part_number: item.part_number, etag })
  }
  return request(`/validation-recordings/${recording.id}/uploads/complete/`, {
    method: 'POST',
    body: JSON.stringify({ parts }),
  })
}

export async function fetchSpeechCategories({ signal } = {}) {
  if (signal) return request('/speech-categories/', { signal })
  return listCache.get('speech-categories', () => request('/speech-categories/', { signal }))
}

export async function createSpeechCategory(name) {
  const result = await request('/speech-categories/', { method: 'POST', body: JSON.stringify({ name }) })
  listCache.invalidate('speech-categories')
  return result
}

export async function updateSpeechCategory(categoryId, name) {
  const result = await request(`/speech-categories/${categoryId}/`, { method: 'PATCH', body: JSON.stringify({ name }) })
  listCache.invalidate('speech-categories')
  return result
}

export async function deleteSpeechCategory(categoryId) {
  const result = await request(`/speech-categories/${categoryId}/`, { method: 'DELETE' })
  listCache.invalidate('speech-categories')
  return result
}

export async function fetchTasks() {
  return request('/tasks/')
}

export async function fetchPatrolTasks() {
  return request('/patrol-tasks/')
}

export async function createPatrolTask(payload) {
  return request('/patrol-tasks/', { method: 'POST', body: JSON.stringify(payload) })
}

export async function updatePatrolTask(taskId, payload) {
  return request(`/patrol-tasks/${taskId}/`, { method: 'PUT', body: JSON.stringify(payload) })
}

export async function deletePatrolTask(taskId, { force = false } = {}) {
  return request(`/patrol-tasks/${taskId}/${force ? '?force=true' : ''}`, { method: 'DELETE' })
}

export async function executePatrolTask(taskId, {
  recordRosbag = null,
  loopExecution = false,
  loopSessionId = null,
  roundNumber = 1,
} = {}) {
  const payload = {
    loop_execution: loopExecution,
    loop_session_id: loopSessionId,
    round_number: roundNumber,
  }
  if (typeof recordRosbag === 'boolean') payload.record_rosbag = recordRosbag
  return request(`/patrol-tasks/${taskId}/execute/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchPatrolSchedules(params = {}) {
  const query = new URLSearchParams()
  Object.entries(params).forEach(([key, value]) => {
    if (value !== undefined && value !== null && value !== '') query.set(key, value)
  })
  return request(`/patrol-schedules/${query.toString() ? `?${query}` : ''}`)
}

export async function createPatrolSchedule(payload) {
  return request('/patrol-schedules/', { method: 'POST', body: JSON.stringify(payload) })
}

export async function updatePatrolSchedule(scheduleId, payload) {
  return request(`/patrol-schedules/${scheduleId}/`, { method: 'PUT', body: JSON.stringify(payload) })
}

export async function deletePatrolSchedule(scheduleId) {
  return request(`/patrol-schedules/${scheduleId}/`, { method: 'DELETE' })
}

export async function setPatrolScheduleEnabled(scheduleId, enabled) {
  return request(`/patrol-schedules/${scheduleId}/${enabled ? 'enable' : 'disable'}/`, {
    method: 'POST',
    body: '{}',
  })
}

export async function runPatrolScheduleNow(scheduleId) {
  return request(`/patrol-schedules/${scheduleId}/run-now/`, { method: 'POST', body: '{}' })
}

export async function fetchCalendarDays() {
  return request('/calendar-days/')
}

export async function createCalendarDay(payload) {
  return request('/calendar-days/', { method: 'POST', body: JSON.stringify(payload) })
}

export async function updateCalendarDay(dayId, payload) {
  return request(`/calendar-days/${dayId}/`, { method: 'PUT', body: JSON.stringify(payload) })
}

export async function deleteCalendarDay(dayId) {
  return request(`/calendar-days/${dayId}/`, { method: 'DELETE' })
}

export async function fetchPatrolCalendar(params = {}) {
  const query = new URLSearchParams()
  Object.entries(params).forEach(([key, value]) => {
    if (value !== undefined && value !== null && value !== '') query.set(key, value)
  })
  return request(`/patrol-calendar/${query.toString() ? `?${query}` : ''}`)
}

export async function fetchScheduleRuns(params = {}) {
  const query = new URLSearchParams()
  Object.entries(params).forEach(([key, value]) => {
    if (value !== undefined && value !== null && value !== '') query.set(key, value)
  })
  return request(`/schedule-runs/${query.toString() ? `?${query}` : ''}`)
}

export async function fetchTaskExecution(executionId) {
  return request(`/task-executions/${executionId}/`)
}

export async function sendTaskExecutionAction(executionId, action) {
  return request(`/task-executions/${executionId}/${action}/`, {
    method: 'POST',
    body: JSON.stringify({ reason: 'operator_request' }),
  })
}

export async function fetchTaskTrajectory(executionId) {
  return request(`/task-executions/${executionId}/trajectory/`)
}

export async function fetchRobotStatus(robotId, { signal } = {}) {
  return request(`/robots/${robotId}/status/`, { signal })
}

export async function fetchRobotSessions(robotId) {
  return request(`/robots/${robotId}/sessions/`)
}

export async function fetchRobotMappingStatus(robotId, { signal } = {}) {
  return request(`/robots/${robotId}/mapping/status/`, { signal })
}

export async function fetchRobotNavigationStatus(robotId, { signal } = {}) {
  return request(`/robots/${robotId}/navigation/status/`, { signal })
}

export async function sendRobotNavigationCommand(robotId, action, payload = {}) {
  const allowed = new Set(['probe', 'start', 'restart', 'recover', 'relocalize', 'stop', 'initial-pose'])
  if (!allowed.has(action)) throw new Error('不支持的导航命令')
  return request(`/robots/${robotId}/navigation/${action}/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function restartRobotSensor(robotId, sensor) {
  return request(`/robots/${robotId}/sensors/restart/`, {
    method: 'POST',
    body: JSON.stringify({ sensor }),
  })
}

export async function startRobotMapping(robotId, payload) {
  return request(`/robots/${robotId}/mapping/start/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function startRobotMappingOrigin(robotId, payload) {
  return request(`/robots/${robotId}/mapping/origin/start/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function cancelRobotMappingOrigin(robotId, payload = {}) {
  return request(`/robots/${robotId}/mapping/origin/cancel/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function extractRobotMappingGlobalEnu(robotId, payload = {}) {
  return request(`/robots/${robotId}/mapping/origin/extract-global/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function startRobotMappingSlam(robotId, payload) {
  return request(`/robots/${robotId}/mapping/slam/start/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function beginRobotMapping(robotId, payload = {}) {
  return request(`/robots/${robotId}/mapping/begin/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function saveRobotMapping(robotId, payload) {
  return request(`/robots/${robotId}/mapping/save/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function cancelRobotMapping(robotId, payload = {}) {
  return request(`/robots/${robotId}/mapping/cancel/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function syncRobotMapping(robotId, payload = {}) {
  return request(`/robots/${robotId}/mapping/sync/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchEventTimeline(eventId) {
  return request(`/events/${eventId}/timeline/`)
}

export async function handleEvent(eventId, payload) {
  return request(`/events/${eventId}/handle/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchMaps({ signal } = {}) {
  return request('/maps/', { signal })
}

export async function fetchMapSummaries({ signal } = {}) {
  if (signal) return request(summaryPath('/maps/'), { signal })
  return listCache.get('maps-summary', () => request(summaryPath('/maps/'), { signal }))
}

export async function fetchMapSets({ signal } = {}) {
  return request('/map-sets/', { signal })
}

export async function fetchMapSetSummaries({ signal } = {}) {
  if (signal) return request(summaryPath('/map-sets/'), { signal })
  return listCache.get('map-sets-summary', () => request(summaryPath('/map-sets/'), { signal }))
}

export async function fetchMapDetail(mapId, { signal } = {}) {
  return request(`/maps/${mapId}/`, { signal })
}

export async function fetchMapMappingTrace(mapId) {
  return request(`/maps/${mapId}/mapping-trace/`)
}

export async function createMap(payload) {
  const formData = new FormData()
  Object.keys(payload).forEach(key => {
    if (payload[key] !== null && payload[key] !== undefined) {
      formData.append(key, payload[key])
    }
  })
  const result = await request('/maps/', {
    method: 'POST',
    body: formData,
    headers: {},
  })
  listCache.invalidate('maps-summary')
  listCache.invalidate('map-sets-summary')
  return result
}

export async function updateMap(mapId, payload) {
  const result = await request(`/maps/${mapId}/`, {
    method: 'PUT',
    body: JSON.stringify(payload),
  })
  listCache.invalidate('maps-summary')
  return result
}

export async function deleteMap(mapId, { force = false } = {}) {
  const result = await request(`/maps/${mapId}/${force ? '?force=true' : ''}`, {
    method: 'DELETE',
  })
  listCache.invalidate('maps-summary')
  listCache.invalidate('map-sets-summary')
  return result
}

export async function downloadMap(mapId) {
  const token = localStorage.getItem('inspection_token')
  const headers = {}
  if (token) {
    headers.Authorization = `Token ${token}`
  }
  const response = await fetch(`${API_BASE}/maps/${mapId}/download/`, {
    headers,
  })
  if (!response.ok) {
    throw new Error('下载失败')
  }
  return response.blob()
}

export async function setActiveMap(mapId) {
  const result = await request(`/maps/${mapId}/set_active/`, {
    method: 'POST',
  })
  listCache.invalidate('maps-summary')
  return result
}

export async function manuallyCleanMap(mapId, payload) {
  const result = await request(`/maps/${mapId}/manual-clean/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
  listCache.invalidate('maps-summary')
  return result
}

export async function fetchMapLoopReview(mapId, thresholds = {}) {
  const query = new URLSearchParams()
  Object.entries(thresholds).forEach(([key, value]) => {
    if (value !== '' && value !== null && value !== undefined) query.set(key, String(value))
  })
  return request(`/maps/${mapId}/loop-review/${query.size ? `?${query}` : ''}`)
}

export async function optimizeMapLoops(mapId, payload) {
  return request(`/maps/${mapId}/loop-optimize/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchRoutes({ signal } = {}) {
  return request('/routes/', { signal })
}

export async function fetchRouteSummaries({ signal } = {}) {
  if (signal) return request(summaryPath('/routes/'), { signal })
  return listCache.get('routes-summary', () => request(summaryPath('/routes/'), { signal }))
}

export async function fetchRouteDetail(routeId, { signal } = {}) {
  return request(`/routes/${routeId}/`, { signal })
}

export async function createRoute(payload) {
  const result = await request('/routes/', {
    method: 'POST',
    body: JSON.stringify(payload),
  })
  listCache.invalidate('routes-summary')
  return result
}

export async function updateRoute(routeId, payload) {
  const result = await request(`/routes/${routeId}/`, {
    method: 'PUT',
    body: JSON.stringify(payload),
  })
  listCache.invalidate('routes-summary')
  return result
}

export async function deleteRoute(routeId) {
  const result = await request(`/routes/${routeId}/`, {
    method: 'DELETE',
  })
  listCache.invalidate('routes-summary')
  return result
}

export async function executeRoute(routeId, {
  recordRosbag = false,
  loopExecution = false,
  loopSessionId = null,
  roundNumber = 1,
} = {}) {
  const result = await request(`/routes/${routeId}/execute/`, {
    method: 'POST',
    body: JSON.stringify({
      record_rosbag: recordRosbag,
      loop_execution: loopExecution,
      loop_session_id: loopSessionId,
      round_number: roundNumber,
    }),
  })
  listCache.invalidate('routes-summary')
  return result
}

export async function navigateSingleGoal(robotId, payload) {
  return request(`/robots/${robotId}/navigation/single-goal/`, {
    method: 'POST', body: JSON.stringify(payload),
  })
}

export async function fetchZones() {
  return request('/zones/')
}

export async function fetchZoneDetail(zoneId) {
  return request(`/zones/${zoneId}/`)
}

export async function createZone(payload) {
  return request('/zones/', {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function updateZone(zoneId, payload) {
  return request(`/zones/${zoneId}/`, {
    method: 'PUT',
    body: JSON.stringify(payload),
  })
}

export async function deleteZone(zoneId) {
  return request(`/zones/${zoneId}/`, {
    method: 'DELETE',
  })
}

export async function fetchTracks({ signal } = {}) {
  return request('/tracks/', { signal })
}

export async function fetchTrackSummaries({ signal } = {}) {
  if (signal) return request(summaryPath('/tracks/'), { signal })
  return listCache.get('tracks-summary', () => request(summaryPath('/tracks/'), { signal }))
}

export async function fetchTrackDetail(trackId, { signal } = {}) {
  return request(`/tracks/${trackId}/`, { signal })
}

export async function deleteTrack(trackId) {
  const result = await request(`/tracks/${trackId}/`, {
    method: 'DELETE',
  })
  listCache.invalidate('tracks-summary')
  return result
}

// 机器狗连接
export async function connectRobot(payload) {
  return request('/maps/robot/connect/', {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function downloadFromRobot(payload) {
  return request('/maps/robot/download/', {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchDevelopmentAgents() {
  return request('/development/agents/')
}

export async function fetchVoiceRecognitions(robotId, limit = 80) {
  const query = new URLSearchParams({
    robot: String(robotId),
    limit: String(limit),
    include_no_speech: 'true',
    _: String(Date.now()),
  })
  return request(`/development/voice-recognitions/?${query.toString()}`, { cache: 'no-store' })
}

export async function fetchDevelopmentTasks(robotId = '') {
  const query = robotId ? `?robot=${encodeURIComponent(robotId)}` : ''
  return request(`/development/tasks/${query}`)
}

export async function fetchDevelopmentConversation(robotId) {
  return request(`/development/conversations/main/?robot=${encodeURIComponent(robotId)}`)
}

export async function fetchDevelopmentConversationMode(robotId) {
  return request(`/development/conversations/main/mode/?robot=${encodeURIComponent(robotId)}`)
}

export async function setDevelopmentConversationMode(payload) {
  return request('/development/conversations/main/mode/', {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchDevelopmentTask(taskId) {
  return request(`/development/tasks/${taskId}/`)
}

export async function createDevelopmentTask(payload) {
  return request('/development/tasks/', {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function cancelDevelopmentTask(taskId) {
  return request(`/development/tasks/${taskId}/cancel/`, {
    method: 'POST',
    body: '{}',
  })
}
