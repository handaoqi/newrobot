/** Short backoff after a failed next-round start or nav repair. */
export const LOOP_FAILURE_RETRY_MS = 60_000
export const DEFAULT_LOOP_REST_SECONDS = 10
export const LOOP_REPAIR_SETTLE_TIMEOUT_MS = 5_000
export const NAV_STATUS_FETCH_ATTEMPTS = 3
export const NAV_STATUS_FETCH_RETRY_MS = 750

export function isTransientNavigationFetchError(error) {
  const message = String(error?.message || error || '')
  return error?.code === 'REQUEST_TIMEOUT'
    || /failed to fetch|network\s*error|network request failed|load failed|fetch failed|请求超时/i.test(message)
}

function navigationStatusUnavailableError(cause) {
  const error = new Error('网络连接暂时中断，未能获取导航状态')
  error.code = 'NAV_STATUS_UNREACHABLE'
  error.cause = cause
  return error
}

async function fetchNavigationStatusWithRetry(fetchStatus, {
  attempts = NAV_STATUS_FETCH_ATTEMPTS,
  retryMs = NAV_STATUS_FETCH_RETRY_MS,
  onProgress = () => {},
  sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms)),
} = {}) {
  const maxAttempts = Math.max(1, Number(attempts) || 1)
  for (let attempt = 1; attempt <= maxAttempts; attempt += 1) {
    try {
      return await fetchStatus()
    } catch (error) {
      if (!isTransientNavigationFetchError(error)) throw error
      if (attempt >= maxAttempts) throw navigationStatusUnavailableError(error)
      onProgress(`网络波动，正在重新获取导航状态（${attempt}/${maxAttempts - 1}）`)
      await sleep(Math.max(0, Number(retryMs) || 0))
    }
  }
  throw navigationStatusUnavailableError()
}

export function guardDutyLoopRepairFailureMessage(error, { duringRest = false } = {}) {
  const suffix = duringRest ? '，休息结束后将重试' : '，短间隔后重试'
  if (error?.code === 'NAV_STATUS_UNREACHABLE' || isTransientNavigationFetchError(error)) {
    return `网络连接暂时中断，导航状态获取失败${suffix}`
  }
  const prefix = duringRest ? '轮次休息维护失败' : '导航栈修复失败'
  return `${prefix}：${error?.message || '请检查导航栈'}${suffix}`
}

/**
 * Resolve how long the guard-duty loop should rest.
 * Normal round gaps use the operator-configured seconds. Start/repair failures
 * use a short retry so the dog is not idle for another full rest window while
 * Nav2 or localization is down.
 */
export function loopRestMilliseconds(restSeconds, { shortRetry = false } = {}) {
  const configured = Math.max(0, Number(restSeconds || 0) * 1000)
  if (!shortRetry) return configured
  if (configured > 0) return Math.min(LOOP_FAILURE_RETRY_MS, configured)
  return LOOP_FAILURE_RETRY_MS
}

export function restoreLoopRestSeconds(saved) {
  const value = Number(saved?.restSeconds)
  return Number.isFinite(value) && value >= 0 ? value : DEFAULT_LOOP_REST_SECONDS
}

/**
 * Make sure localization + Nav2 are ready before the next loop round.
 * When not ready, runs the injected repair (typically activateAndRelocalizeMap /
 * nav.start) and polls until ready or timeout.
 */
export async function ensureGuardDutyLoopNavigationReady({
  fetchStatus,
  isReady,
  repair,
  onProgress = () => {},
  readyTimeoutMs = 90_000,
  pollIntervalMs = 2_000,
  statusFetchAttempts = NAV_STATUS_FETCH_ATTEMPTS,
  statusFetchRetryMs = NAV_STATUS_FETCH_RETRY_MS,
  sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms)),
  now = () => Date.now(),
} = {}) {
  if (typeof fetchStatus !== 'function' || typeof isReady !== 'function') {
    throw new Error('ensureGuardDutyLoopNavigationReady requires fetchStatus and isReady')
  }

  const fetchStatusSafely = () => fetchNavigationStatusWithRetry(fetchStatus, {
    attempts: statusFetchAttempts,
    retryMs: statusFetchRetryMs,
    onProgress,
    sleep,
  })

  let status = await fetchStatusSafely()
  if (isReady(status)) {
    return { ok: true, repaired: false, status, reason: 'already_ready' }
  }
  if (typeof repair !== 'function') {
    return { ok: false, repaired: false, status, reason: 'repair_unavailable' }
  }

  onProgress('轮次维护：定位或导航栈未就绪，正在修复')
  const repairedStatus = await repair({ status, onProgress })
  status = repairedStatus || await fetchStatusSafely()
  if (isReady(status)) {
    onProgress('轮次维护：导航/定位已就绪')
    return { ok: true, repaired: true, status, reason: 'repaired' }
  }

  const deadline = now() + Math.max(0, Number(readyTimeoutMs) || 0)
  while (now() < deadline) {
    onProgress('轮次维护：等待定位与导航栈同步')
    await sleep(Math.max(100, Number(pollIntervalMs) || 1000))
    status = await fetchStatusSafely()
    if (isReady(status)) {
      onProgress('轮次维护：导航/定位已就绪')
      return { ok: true, repaired: true, status, reason: 'ready_after_wait' }
    }
  }

  return { ok: false, repaired: true, status, reason: 'not_ready_after_repair' }
}

/**
 * Do not let an old rest-period repair hold the next round in `starting`.
 * After a short grace period, refresh device truth. If the device is already
 * ready the caller may supersede the stale UI repair token and launch; if it
 * is not ready the caller should return to a visible short-retry state.
 */
export async function waitForGuardDutyLoopRepair({
  isBusy,
  fetchStatus,
  isReady,
  timeoutMs = LOOP_REPAIR_SETTLE_TIMEOUT_MS,
  pollIntervalMs = 250,
  statusFetchAttempts = NAV_STATUS_FETCH_ATTEMPTS,
  statusFetchRetryMs = NAV_STATUS_FETCH_RETRY_MS,
  onProgress = () => {},
  sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms)),
  now = () => Date.now(),
} = {}) {
  if (typeof isBusy !== 'function' || typeof fetchStatus !== 'function' || typeof isReady !== 'function') {
    throw new Error('waitForGuardDutyLoopRepair requires isBusy, fetchStatus and isReady')
  }
  const deadline = now() + Math.max(0, Number(timeoutMs) || 0)
  while (isBusy() && now() < deadline) {
    await sleep(Math.max(20, Number(pollIntervalMs) || 250))
  }
  if (!isBusy()) return { ok: true, supersede: false, status: null, reason: 'repair_finished' }

  const status = await fetchNavigationStatusWithRetry(fetchStatus, {
    attempts: statusFetchAttempts,
    retryMs: statusFetchRetryMs,
    onProgress,
    sleep,
  })
  if (isReady(status)) {
    return { ok: true, supersede: true, status, reason: 'device_already_ready' }
  }
  return { ok: false, supersede: false, status, reason: 'repair_still_busy' }
}
