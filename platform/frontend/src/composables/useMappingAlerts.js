import { onBeforeUnmount, onMounted, ref } from 'vue'
import { API_BASE, fetchRobotMappingStatus, fetchRobots } from '../services/api.js'

export function useMappingAlerts({ intervalMs = 2_000 } = {}) {
  const mappingAlert = ref(null)
  let timer = null
  let source = null
  let robots = []
  let refreshing = false
  let lastKey = ''

  const normalize = (status) => {
    const progress = status?.result?.save_progress || {}
    const health = progress.slam_health || {}
    const diverged = progress.error_code === 'SLAM_DIVERGED' || health.state === 'diverged'
    const degraded = health.state === 'degraded'
    if (!diverged && !degraded) return null
    return {
      robotId: status.robot_id,
      robotCode: status.robot_code || '',
      diverged,
      title: diverged ? '建图已停采，请立即停止遥控' : '建图质量正在恶化',
      message: diverged ? '定位已发散，关键帧不再记录。请停止走场，到地图页处理。' : (health.warning || progress.error || '位姿异常，请放慢或原地停下'),
    }
  }

  const apply = (alert) => {
    mappingAlert.value = alert
    if (alert) {
      const key = `${alert.robotId}:${alert.diverged}`
      if (key === lastKey) return
      lastKey = key
      try {
        const context = new window.AudioContext()
        const oscillator = context.createOscillator()
        const gain = context.createGain()
        oscillator.type = 'square'
        oscillator.frequency.value = alert.diverged ? 880 : 520
        gain.gain.value = 0.08
        oscillator.connect(gain); gain.connect(context.destination)
        oscillator.start(); oscillator.stop(context.currentTime + (alert.diverged ? 0.45 : 0.2))
      } catch {}
    } else lastKey = ''
  }

  async function refresh() {
    if (refreshing) return
    refreshing = true
    try {
      if (!robots.length) robots = await fetchRobots()
      const statuses = await Promise.all(robots.map((robot) => fetchRobotMappingStatus(robot.id).catch(() => null)))
      apply(statuses.map(normalize).find(Boolean) || null)
    } finally { refreshing = false }
  }

  function startStream() {
    const token = localStorage.getItem('inspection_token')
    if (!token || typeof EventSource === 'undefined') return
    source = new EventSource(`${API_BASE}/events/stream/?token=${encodeURIComponent(token)}`)
    source.addEventListener('inspection_event_created', (message) => {
      let payload
      try { payload = JSON.parse(message.data || '{}') } catch { return }
      const event = payload.event || {}
      if (event.event_type !== 'slam_diverged' && event.source_code !== 'SLAM_DIVERGED' && event.title !== '建图定位已发散') return
      apply({ robotId: event.robot || payload.robot?.id, robotCode: payload.robot?.code || '', diverged: true, title: '建图已停采，请立即停止遥控', message: event.description || '定位已发散，请停止走场并处理地图。' })
    })
  }

  onMounted(() => { refresh(); timer = window.setInterval(refresh, intervalMs); startStream() })
  onBeforeUnmount(() => { if (timer) clearInterval(timer); source?.close(); source = null })
  return { mappingAlert, refresh }
}
