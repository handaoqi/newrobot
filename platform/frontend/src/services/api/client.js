import { createTTLCache } from '../cache.js'

export const API_BASE = (import.meta.env?.VITE_API_BASE || '/api').replace(/\/$/, '')
export const listCache = createTTLCache({ ttlMs: 30_000 })

export async function request(path, options = {}) {
  const { timeoutMs, signal: callerSignal, traceId, ...fetchOptions } = options
  const token = localStorage.getItem('inspection_token')
  const headers = { ...(fetchOptions.headers || {}) }
  if (!(fetchOptions.body instanceof FormData)) headers['Content-Type'] = 'application/json'
  if (token) headers.Authorization = `Token ${token}`
  if (traceId) headers['X-Trace-Id'] = traceId

  let controller
  let timeoutHandle
  let timeoutTriggered = false
  let removeCallerAbortListener
  let signal = callerSignal
  if (timeoutMs > 0) {
    controller = new AbortController()
    signal = controller.signal
    const abortFromCaller = () => controller.abort()
    if (callerSignal?.aborted) controller.abort()
    else if (callerSignal) {
      callerSignal.addEventListener('abort', abortFromCaller, { once: true })
      removeCallerAbortListener = () => callerSignal.removeEventListener('abort', abortFromCaller)
    }
    timeoutHandle = setTimeout(() => { timeoutTriggered = true; controller.abort() }, timeoutMs)
  }

  let response
  try {
    response = await fetch(`${API_BASE}${path}`, { ...fetchOptions, headers, ...(signal ? { signal } : {}) })
  } catch (error) {
    if (timeoutTriggered && !callerSignal?.aborted) {
      const timeoutError = new Error(`请求超时（${Math.round(timeoutMs / 1000)} 秒）`)
      timeoutError.code = 'REQUEST_TIMEOUT'
      throw timeoutError
    }
    throw error
  } finally {
    if (timeoutHandle) clearTimeout(timeoutHandle)
    removeCallerAbortListener?.()
  }
  if (!response.ok) {
    const payload = await response.json().catch(() => ({ detail: '请求失败' }))
    const error = new Error(payload.detail || '请求失败')
    error.payload = payload
    error.status = response.status
    if (response.status === 401 && path !== '/auth/login/') {
      localStorage.removeItem('inspection_token')
      localStorage.removeItem('inspection_user')
      if (typeof window !== 'undefined' && !window.location.pathname.startsWith('/login')) window.location.assign('/login')
    }
    throw error
  }
  if (response.status === 204) return {}
  return response.json()
}
