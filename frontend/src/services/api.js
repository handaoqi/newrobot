export const API_BASE = 'http://127.0.0.1:8000/api'

async function request(path, options = {}) {
  const token = localStorage.getItem('inspection_token')
  const headers = {
    'Content-Type': 'application/json',
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
    throw new Error(errorPayload.detail || '请求失败')
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

export async function fetchTasks() {
  return request('/tasks/')
}

export async function handleEvent(eventId, payload) {
  return request(`/events/${eventId}/handle/`, {
    method: 'POST',
    body: JSON.stringify(payload),
  })
}
