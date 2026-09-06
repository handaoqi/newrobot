/** Short backoff after a failed next-round start or nav repair. */
export const LOOP_FAILURE_RETRY_MS = 60_000

/**
 * Resolve how long the guard-duty loop should rest.
 * Normal round gaps use the operator-configured minutes. Start/repair failures
 * use a short retry so the dog is not idle for another full rest window while
 * Nav2 or localization is down.
 */
export function loopRestMilliseconds(restMinutes, { shortRetry = false } = {}) {
  const configured = Math.max(0, Number(restMinutes || 0) * 60 * 1000)
  if (!shortRetry) return configured
  if (configured > 0) return Math.min(LOOP_FAILURE_RETRY_MS, configured)
  return LOOP_FAILURE_RETRY_MS
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
  sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms)),
  now = () => Date.now(),
} = {}) {
  if (typeof fetchStatus !== 'function' || typeof isReady !== 'function') {
    throw new Error('ensureGuardDutyLoopNavigationReady requires fetchStatus and isReady')
  }

  let status = await fetchStatus()
  if (isReady(status)) {
    return { ok: true, repaired: false, status, reason: 'already_ready' }
  }
  if (typeof repair !== 'function') {
    return { ok: false, repaired: false, status, reason: 'repair_unavailable' }
  }

  onProgress('轮次维护：定位或导航栈未就绪，正在修复')
  const repairedStatus = await repair({ status, onProgress })
  status = repairedStatus || await fetchStatus()
  if (isReady(status)) {
    onProgress('轮次维护：导航/定位已就绪')
    return { ok: true, repaired: true, status, reason: 'repaired' }
  }

  const deadline = now() + Math.max(0, Number(readyTimeoutMs) || 0)
  while (now() < deadline) {
    onProgress('轮次维护：等待定位与导航栈同步')
    await sleep(Math.max(100, Number(pollIntervalMs) || 1000))
    status = await fetchStatus()
    if (isReady(status)) {
      onProgress('轮次维护：导航/定位已就绪')
      return { ok: true, repaired: true, status, reason: 'ready_after_wait' }
    }
  }

  return { ok: false, repaired: true, status, reason: 'not_ready_after_repair' }
}
