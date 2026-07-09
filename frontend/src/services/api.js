export const API_BASE = import.meta.env.VITE_API_BASE || '/api'

export class ApiError extends Error {
  constructor(message, status) {
    super(message)
    this.name = 'ApiError'
    this.status = status
  }
}

async function request(path, options = {}) {
  const token = localStorage.getItem('inspection_token')
  const isFormData = options.body instanceof FormData
  const headers = {
    Accept: 'application/json',
    ...(isFormData ? {} : { 'Content-Type': 'application/json' }),
    ...(options.headers || {}),
  }

  if (token) {
    headers.Authorization = `Token ${token}`
  }

  const response = await fetch(`${API_BASE}${path}`, {
    ...options,
    headers,
  })

  if (!response.ok) {
    const errorPayload = await response.json().catch(() => ({ detail: '请求失败' }))
    if (response.status === 401) {
      localStorage.removeItem('inspection_token')
      localStorage.removeItem('inspection_user')
      if (!window.location.pathname.startsWith('/login')) {
        window.location.assign('/login')
      }
    }
    throw new ApiError(errorPayload.detail || '请求失败', response.status)
  }

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

export async function handleEvent(eventId, payload) {
  return request(`/events/${eventId}/handle/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}

export async function fetchMaps() {
  return request('/maps/')
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

export async function deleteMap(mapId) {
  return request(`/maps/${mapId}/`, {
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
