import { onBeforeUnmount, onMounted } from 'vue'

export function createAsyncPoller(task, {
  intervalMs = 1_000,
  immediate = true,
  visibilityAware = true,
  onError = () => {},
  documentRef = typeof document === 'undefined' ? null : document,
  setTimeoutFn = typeof window === 'undefined' ? setTimeout : window.setTimeout,
  clearTimeoutFn = typeof window === 'undefined' ? clearTimeout : window.clearTimeout,
} = {}) {
  let running = false
  let paused = false
  let inFlight = false
  let timer = null
  let activeController = null

  const schedule = () => {
    if (!running || paused || timer !== null) return
    timer = setTimeoutFn(() => {
      timer = null
      void run()
    }, intervalMs)
  }

  const run = async () => {
    if (!running || paused || inFlight) return
    inFlight = true
    activeController = new AbortController()
    try {
      await task(activeController.signal)
    } catch (error) {
      if (error?.name !== 'AbortError') onError(error)
    } finally {
      inFlight = false
      activeController = null
      schedule()
    }
  }

  const handleVisibility = () => {
    paused = Boolean(documentRef?.hidden)
    if (paused) {
      if (timer !== null) clearTimeoutFn(timer)
      timer = null
      activeController?.abort()
    } else if (running) {
      void run()
    }
  }

  const start = () => {
    if (running) return
    running = true
    paused = visibilityAware && Boolean(documentRef?.hidden)
    if (visibilityAware) documentRef?.addEventListener('visibilitychange', handleVisibility)
    if (!paused && immediate) void run()
    else schedule()
  }

  const stop = () => {
    running = false
    paused = false
    if (timer !== null) clearTimeoutFn(timer)
    timer = null
    activeController?.abort()
    activeController = null
    if (visibilityAware) documentRef?.removeEventListener('visibilitychange', handleVisibility)
  }

  return {
    start,
    stop,
    run,
    get running() { return running },
    get inFlight() { return inFlight },
  }
}

export function useAsyncPoller(task, options) {
  const poller = createAsyncPoller(task, options)
  onMounted(poller.start)
  onBeforeUnmount(poller.stop)
  return poller
}
