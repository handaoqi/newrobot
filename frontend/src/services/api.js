export const API_BASE = (import.meta.env.VITE_API_BASE || '/api').replace(/\/$/, '')

async function request(path, options = {}) {
  const token = localStorage.getItem('inspection_token')
  const headers = { ...(options.headers || {}) }
  if (!(options.body instanceof FormData)) headers['Content-Type'] = 'application/json'

  if (token) {
    headers.Authorization = `Token ${token}`
  }

  const response = await fetch(`${API_BASE}${path}`, {
    ...options,
    headers,
  })

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

export async function fetchRobots() {
  return request('/robots/')
}

export async function fetchRobotDetail(robotId) {
  return request(`/robots/${robotId}/`)
}

export async function sendRobotCommand(robotId, payload) {
  return request(`/robots/${robotId}/commands/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function sendRecordedAudioCommand(robotId, file, audioName = '现场录音') {
  const formData = new FormData()
  formData.append('file', file)
  formData.append('audio_name', audioName)
  return request(`/robots/${robotId}/commands/audio-recording/`, {
    method: 'POST',
    body: formData,
  })
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

export async function deletePatrolTask(taskId, { force = false } = {}) {
  return request(`/patrol-tasks/${taskId}/${force ? '?force=true' : ''}`, { method: 'DELETE' })
}

export async function executePatrolTask(taskId) {
  return request(`/patrol-tasks/${taskId}/execute/`, { method: 'POST', body: '{}' })
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

export async function fetchRobotStatus(robotId) {
  return request(`/robots/${robotId}/status/`)
}

export async function fetchRobotSessions(robotId) {
  return request(`/robots/${robotId}/sessions/`)
}

export async function fetchRobotMappingStatus(robotId) {
  return request(`/robots/${robotId}/mapping/status/`)
}

export async function fetchRobotNavigationStatus(robotId) {
  return request(`/robots/${robotId}/navigation/status/`)
}

export async function sendRobotNavigationCommand(robotId, action, payload = {}) {
  const allowed = new Set(['probe', 'start', 'restart', 'recover', 'stop', 'initial-pose'])
  if (!allowed.has(action)) throw new Error('不支持的导航命令')
  return request(`/robots/${robotId}/navigation/${action}/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function startRobotMapping(robotId, payload) {
  return request(`/robots/${robotId}/mapping/start/`, {
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

export async function fetchMaps() {
  return request('/maps/')
}

export async function fetchMapSets() {
  return request('/map-sets/')
}

export async function fetchMapDetail(mapId) {
  return request(`/maps/${mapId}/`)
}

export async function createMap(payload) {
  const formData = new FormData()
  Object.keys(payload).forEach(key => {
    if (payload[key] !== null && payload[key] !== undefined) {
      formData.append(key, payload[key])
    }
  })
  return request('/maps/', {
    method: 'POST',
    body: formData,
    headers: {},
  })
}

export async function updateMap(mapId, payload) {
  return request(`/maps/${mapId}/`, {
    method: 'PUT',
    body: JSON.stringify(payload),
  })
}

export async function deleteMap(mapId, { force = false } = {}) {
  return request(`/maps/${mapId}/${force ? '?force=true' : ''}`, {
    method: 'DELETE',
  })
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
  return request(`/maps/${mapId}/set_active/`, {
    method: 'POST',
  })
}

export async function fetchRoutes() {
  return request('/routes/')
}

export async function fetchRouteDetail(routeId) {
  return request(`/routes/${routeId}/`)
}

export async function createRoute(payload) {
  return request('/routes/', {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function updateRoute(routeId, payload) {
  return request(`/routes/${routeId}/`, {
    method: 'PUT',
    body: JSON.stringify(payload),
  })
}

export async function deleteRoute(routeId) {
  return request(`/routes/${routeId}/`, {
    method: 'DELETE',
  })
}

export async function executeRoute(routeId) {
  return request(`/routes/${routeId}/execute/`, { method: 'POST', body: '{}' })
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

export async function fetchTracks() {
  return request('/tracks/')
}

export async function fetchTrackDetail(trackId) {
  return request(`/tracks/${trackId}/`)
}

export async function deleteTrack(trackId) {
  return request(`/tracks/${trackId}/`, {
    method: 'DELETE',
  })
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
