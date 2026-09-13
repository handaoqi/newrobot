<script setup>
import { computed, nextTick, onBeforeUnmount, ref, watch } from 'vue'

import ImuRealtimeChart from '../components/replay/ImuRealtimeChart.vue'
import PatrolMapPanel from '../components/replay/PatrolMapPanel.vue'
import PointCloudPanel from '../components/replay/PointCloudPanel.vue'
import { openLiveMessageSource } from '../services/rosStream'
import { pointCloud2ToArrays } from '../services/sceneData'

const LIVE_URL_KEY = 'patrol_console_live_ws'
const defaultLiveUrl = `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/foxglove/ws`
const TOPICS = [
  '/odom/localization_odom', '/front_lidar', '/front_lidar/imu', '/localization_info', '/status',
  '/fix', '/perception/semantic_objects', '/battery_state', '/rosout',
]
const DEMO_WAYPOINTS = [
  { x: 1.8, y: 1.1 }, { x: 4.5, y: 2.2 }, { x: 7.4, y: 1.2 },
  { x: 9.2, y: -1.6 }, { x: 6.1, y: -3.6 }, { x: 2.4, y: -2.7 },
]

const dataMode = ref('sim')
const liveUrl = ref(localStorage.getItem(LIVE_URL_KEY) || defaultLiveUrl)
const backgroundMode = ref('grid')
const amapAvailable = ref(false)
const amapMessage = ref('')
const connectionState = ref('disconnected')
const running = ref(false)
const runtimeSeconds = ref(0)
const robotPose = ref({ x: 0, y: 0, yaw: 0, speed: 0 })
const trail = ref([])
const pointCloud = ref({ positions: new Float32Array(), colors: new Float32Array(), count: 0 })
const imuSamples = ref([])
const obstacles = ref([])
const geoReference = ref(null)
const battery = ref(null)
const localization = ref({ status: 0, converged: false, inlier: null, error: null })
const waypoints = ref(DEMO_WAYPOINTS.map(point => ({ ...point, status: 'waiting' })))
const currentWaypoint = ref(0)
const taskDispatched = ref(true)
const taskDrawerOpen = ref(false)
const waypointX = ref('')
const waypointY = ref('')
const logs = ref([])
const logLocked = ref(false)
const logBox = ref(null)
const portraitTip = ref(true)

let liveSource
let simTimer
let runtimeTimer
let simulationTick = 0

const connectionLabel = computed(() => ({
  disconnected: '未连接', connecting: '连接中', connected: '已连接', error: '连接异常',
}[connectionState.value]))
const connectionTone = computed(() => connectionState.value === 'connected' ? 'ok' : connectionState.value === 'connecting' ? 'warn' : 'bad')
const localizationLabel = computed(() => {
  if (localization.value.status === 4) return '定位丢失'
  if ([1, 2].includes(localization.value.status)) return '弱定位'
  if (localization.value.status === 3 || localization.value.converged) return '高精度'
  return '等待定位'
})
const localizationTone = computed(() => localization.value.status === 4 ? 'bad' : localization.value.status === 3 || localization.value.converged ? 'ok' : 'warn')
const completedWaypoints = computed(() => waypoints.value.filter(point => point.status === 'done').length)
const canStart = computed(() => dataMode.value === 'sim' || connectionState.value === 'connected')
const statusDetail = computed(() => {
  const parts = []
  if (localization.value.inlier != null) parts.push(`内点率 ${(localization.value.inlier * 100).toFixed(0)}%`)
  if (localization.value.error != null) parts.push(`误差 ${Number(localization.value.error).toFixed(3)}`)
  return parts.join(' · ') || 'NDT / RTK'
})

function addLog(message, level = 'INFO', module = 'console') {
  logs.value.push({ id: `${Date.now()}-${Math.random()}`, time: new Date(), level, module, message })
  if (logs.value.length > 150) logs.value.splice(0, logs.value.length - 150)
}

watch(logs, async () => {
  if (logLocked.value) return
  await nextTick()
  if (logBox.value) logBox.value.scrollTop = logBox.value.scrollHeight
}, { deep: true })

function stopTimers() {
  if (simTimer) window.clearInterval(simTimer)
  if (runtimeTimer) window.clearInterval(runtimeTimer)
  simTimer = null
  runtimeTimer = null
}

function closeLiveSource() {
  liveSource?.close?.()
  liveSource = null
}

function connect() {
  if (connectionState.value === 'connected' || connectionState.value === 'connecting') return
  if (dataMode.value === 'sim') {
    connectionState.value = 'connected'
    addLog('模拟数据源已就绪')
    return
  }
  const url = liveUrl.value.trim()
  if (!url) { addLog('请输入 Foxglove WebSocket 地址', 'WARN'); return }
  localStorage.setItem(LIVE_URL_KEY, url)
  closeLiveSource()
  connectionState.value = 'connecting'
  addLog(`正在连接 ${url}`)
  liveSource = openLiveMessageSource(url, TOPICS, {
    onReady: ({ subscribed }) => {
      connectionState.value = 'connected'
      addLog(`Foxglove 已连接，订阅 ${subscribed.length}/${TOPICS.length} 个话题`)
    },
    onMessage: consumeLiveMessage,
    onError: error => {
      connectionState.value = 'error'
      running.value = false
      addLog(error?.message || 'Foxglove 连接异常', 'ERROR', 'connection')
    },
  })
}

function disconnect() {
  running.value = false
  stopTimers()
  closeLiveSource()
  connectionState.value = 'disconnected'
  robotPose.value = { ...robotPose.value, speed: 0 }
  addLog('数据源已断开', 'WARN')
}

function setMode(mode) {
  if (dataMode.value === mode) return
  if (connectionState.value !== 'disconnected') disconnect()
  dataMode.value = mode
  addLog(mode === 'sim' ? '已切换至模拟调试' : '已切换至 ROS 真实数据')
}

function startRuntimeClock() {
  if (runtimeTimer) return
  runtimeTimer = window.setInterval(() => { if (running.value) runtimeSeconds.value += 1 }, 1000)
}

function start() {
  if (running.value) { pause(); return }
  if (dataMode.value === 'sim' && connectionState.value !== 'connected') connect()
  if (!canStart.value) { addLog('请先连接真实数据源', 'WARN'); return }
  running.value = true
  startRuntimeClock()
  if (dataMode.value === 'sim' && !simTimer) simTimer = window.setInterval(updateSimulation, 50)
  addLog(dataMode.value === 'sim' ? '巡检模拟开始运行' : '开始接收并绘制 ROS 数据')
}

function pause() {
  running.value = false
  if (simTimer) window.clearInterval(simTimer)
  simTimer = null
  addLog('画面与数据更新已暂停', 'WARN')
}

function stop() {
  if (!running.value && !simTimer) return
  running.value = false
  if (simTimer) window.clearInterval(simTimer)
  simTimer = null
  robotPose.value = { ...robotPose.value, speed: 0 }
  addLog('实时数据展示已停止', 'WARN')
}

function emptyCloud() {
  return { positions: new Float32Array(), colors: new Float32Array(), count: 0 }
}

function resetScene() {
  const wasRunning = running.value
  stopTimers()
  running.value = false
  runtimeSeconds.value = 0
  simulationTick = 0
  robotPose.value = { x: 0, y: 0, yaw: 0, speed: 0 }
  trail.value = []
  pointCloud.value = emptyCloud()
  imuSamples.value = []
  obstacles.value = []
  battery.value = dataMode.value === 'sim' ? 92 : null
  localization.value = { status: 0, converged: false, inlier: null, error: null }
  waypoints.value = DEMO_WAYPOINTS.map((point, index) => ({ ...point, status: index === 0 ? 'current' : 'waiting' }))
  currentWaypoint.value = 0
  taskDispatched.value = true
  logs.value = []
  addLog('巡检场景已重置')
  if (wasRunning) start()
}

function clearData() {
  trail.value = []
  pointCloud.value = emptyCloud()
  imuSamples.value = []
  logs.value = []
  addLog('轨迹、点云、IMU 与日志缓存已清空')
}

function createSimulationCloud(time) {
  const points = []
  const colors = []
  const push = (x, y, z) => {
    points.push(x, y, z)
    const normalized = Math.max(0, Math.min(1, (z + .2) / 2.5))
    colors.push(normalized, Math.max(.25, 1 - Math.abs(normalized - .45) * 1.5), 1 - normalized)
  }
  for (let i = -110; i <= 110; i += 1) {
    const x = i / 10
    for (let level = 0; level < 7; level += 1) {
      push(x, -5.5 + Math.sin(i * .07) * .08, level * .32)
      push(x, 5.5 + Math.cos(i * .06) * .08, level * .32)
    }
  }
  for (let index = 0; index < 320; index += 1) {
    const angle = index * .39
    const distance = 1.5 + index % 90 / 10
    push(Math.cos(angle) * distance, Math.sin(angle) * distance, -.12 + (index % 5) * .025)
  }
  for (const obstacle of [{ x: 4.2, y: 1.1 }, { x: -3.5, y: -2.4 }, { x: 1.8 + Math.sin(time) * 2, y: -1.6 }]) {
    for (let index = 0; index < 90; index += 1) {
      const angle = index / 90 * Math.PI * 2
      push(obstacle.x + Math.cos(angle) * .35, obstacle.y + Math.sin(angle) * .35, .2 + (index % 10) * .12)
    }
  }
  return { positions: new Float32Array(points), colors: new Float32Array(colors), count: points.length / 3 }
}

function updateSimulation() {
  if (!running.value) return
  simulationTick += 1
  const time = simulationTick * .05
  const target = waypoints.value[currentWaypoint.value]
  let pose = { ...robotPose.value }
  if (target && taskDispatched.value) {
    const dx = target.x - pose.x
    const dy = target.y - pose.y
    const distance = Math.hypot(dx, dy)
    if (distance < .12) {
      waypoints.value[currentWaypoint.value].status = 'done'
      addLog(`WP${currentWaypoint.value + 1} 已抵达`, 'INFO', 'waypoint')
      currentWaypoint.value += 1
      if (waypoints.value[currentWaypoint.value]) waypoints.value[currentWaypoint.value].status = 'current'
      else { taskDispatched.value = false; addLog('演示巡检任务已完成', 'INFO', 'task') }
    } else {
      const speed = .72
      const yaw = Math.atan2(dy, dx)
      pose = { x: pose.x + Math.cos(yaw) * speed * .05, y: pose.y + Math.sin(yaw) * speed * .05, yaw, speed }
    }
  } else {
    pose.speed = 0
  }
  robotPose.value = pose
  trail.value.push({ x: pose.x, y: pose.y })
  if (trail.value.length > 800) trail.value.shift()
  imuSamples.value.push({
    label: time.toFixed(1), gx: Number((.07 * Math.sin(time * 2.3)).toFixed(4)),
    gy: Number((.06 * Math.cos(time * 1.8)).toFixed(4)), gz: Number((pose.speed ? .035 + .025 * Math.sin(time) : 0).toFixed(4)),
  })
  if (imuSamples.value.length > 200) imuSamples.value.shift()
  if (simulationTick % 4 === 0) pointCloud.value = createSimulationCloud(time)
  obstacles.value = [
    { x: 4.2, y: 1.1, dynamic: false }, { x: 6.8, y: -2.8, dynamic: false },
    { x: pose.x + 2.4 + Math.sin(time * .7), y: pose.y - 1.1, dynamic: true },
  ]
  battery.value = Math.max(18, 92 - time / 180)
  localization.value = { status: 3, converged: true, inlier: .88 + Math.sin(time * .2) * .035, error: .028 + Math.abs(Math.sin(time)) * .012 }
  if (simulationTick % 120 === 0) addLog(`NDT 匹配稳定，MPPI 输出速度 ${pose.speed.toFixed(2)} m/s`, 'INFO', 'navigation')
}

function quaternionYaw(quaternion) {
  if (!quaternion) return 0
  return Math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))
}

function consumeLiveMessage({ topic, message, timestamp }) {
  if (!running.value) return
  if (topic === '/odom/localization_odom') {
    const pose = message?.pose?.pose
    const linear = message?.twist?.twist?.linear || {}
    if (!pose) return
    const next = {
      x: Number(pose.position?.x) || 0, y: Number(pose.position?.y) || 0,
      yaw: quaternionYaw(pose.orientation), speed: Math.hypot(Number(linear.x) || 0, Number(linear.y) || 0),
    }
    robotPose.value = next
    trail.value.push({ x: next.x, y: next.y })
    if (trail.value.length > 800) trail.value.shift()
    return
  }
  if (topic === '/front_lidar') {
    pointCloud.value = pointCloud2ToArrays(message, { maxPoints: 18_000, colorMode: 'height' })
    return
  }
  if (topic === '/front_lidar/imu') {
    const velocity = message?.angular_velocity || {}
    imuSamples.value.push({
      label: Number(timestamp).toFixed(1), gx: Number(velocity.x) || 0,
      gy: Number(velocity.y) || 0, gz: Number(velocity.z) || 0,
    })
    if (imuSamples.value.length > 200) imuSamples.value.shift()
    return
  }
  if (topic === '/localization_info') {
    localization.value = { ...localization.value, status: Number(message?.status) || 0 }
    return
  }
  if (topic === '/status') {
    localization.value = {
      ...localization.value, converged: Boolean(message?.has_converged),
      inlier: Number.isFinite(Number(message?.inlier_fraction)) ? Number(message.inlier_fraction) : localization.value.inlier,
      error: Number.isFinite(Number(message?.matching_error)) ? Number(message.matching_error) : localization.value.error,
    }
    return
  }
  if (topic === '/fix' && !geoReference.value?.available) {
    const latitude = Number(message?.latitude)
    const longitude = Number(message?.longitude)
    if (Number.isFinite(latitude) && Number.isFinite(longitude)) {
      geoReference.value = { available: true, origin_latitude: latitude, origin_longitude: longitude, map_offset_x: robotPose.value.x, map_offset_y: robotPose.value.y, enu_to_map_yaw: 0 }
      addLog(`GNSS 原点已锁定 ${longitude.toFixed(6)}, ${latitude.toFixed(6)}`, 'INFO', 'localization')
    }
    return
  }
  if (topic === '/battery_state') {
    const percentage = Number(message?.percentage)
    if (Number.isFinite(percentage)) battery.value = Math.max(0, Math.min(100, percentage <= 1 ? percentage * 100 : percentage))
    return
  }
  if (topic === '/perception/semantic_objects') {
    const items = message?.objects || message?.detections || []
    obstacles.value = items.map((item, index) => {
      const position = item?.position || item?.pose?.position || item?.pose?.pose?.position || {}
      return { id: item?.id || index, x: Number(position.x) || 0, y: Number(position.y) || 0, dynamic: item?.dynamic !== false }
    })
    return
  }
  if (topic === '/rosout') {
    const numericLevel = Number(message?.level)
    const level = numericLevel >= 40 ? 'ERROR' : numericLevel >= 30 ? 'WARN' : 'INFO'
    addLog(message?.msg || message?.message || 'ROS 日志', level, message?.name || 'ros')
  }
}

function mapAvailability({ available, message }) {
  amapAvailable.value = available
  amapMessage.value = message || ''
  if (!available && backgroundMode.value !== 'grid') backgroundMode.value = 'grid'
}

function addWaypoint() {
  const x = Number(waypointX.value)
  const y = Number(waypointY.value)
  if (!Number.isFinite(x) || !Number.isFinite(y)) { addLog('航点 X/Y 必须为有效数字', 'WARN', 'waypoint'); return }
  waypoints.value.push({ x, y, status: 'waiting' })
  waypointX.value = ''
  waypointY.value = ''
  addLog(`已添加航点 X ${x.toFixed(2)}, Y ${y.toFixed(2)}`, 'INFO', 'waypoint')
}

function removeWaypoint(index) {
  waypoints.value.splice(index, 1)
  currentWaypoint.value = Math.min(currentWaypoint.value, Math.max(0, waypoints.value.length - 1))
  addLog(`已删除 WP${index + 1}`, 'WARN', 'waypoint')
}

function clearTask() {
  waypoints.value = []
  currentWaypoint.value = 0
  taskDispatched.value = false
  addLog('演示任务已清空', 'WARN', 'task')
}

function resetTaskProgress() {
  currentWaypoint.value = 0
  waypoints.value.forEach((point, index) => { point.status = index === 0 ? 'current' : 'waiting' })
  taskDispatched.value = waypoints.value.length > 0
  addLog('演示任务进度已重置', 'INFO', 'task')
}

function dispatchTask() {
  if (!waypoints.value.length) { addLog('演示任务没有航点', 'WARN', 'task'); return }
  resetTaskProgress()
  taskDrawerOpen.value = false
  addLog(`演示任务已载入，共 ${waypoints.value.length} 个航点；未向机器人发送命令`, 'INFO', 'task')
}

function formatRuntime(seconds) {
  const hours = Math.floor(seconds / 3600)
  const minutes = Math.floor(seconds % 3600 / 60)
  const rest = seconds % 60
  return [hours, minutes, rest].map(value => String(value).padStart(2, '0')).join(':')
}

resetScene()

onBeforeUnmount(() => {
  stopTimers()
  closeLiveSource()
})
</script>

<template>
  <main class="patrol-console">
    <div v-if="portraitTip" class="portrait-tip">
      <span>建议切换横屏获得完整巡检监控体验</span>
      <button type="button" @click="portraitTip = false">知道了</button>
    </div>

    <header class="console-header">
      <div class="console-title">
        <p>ROBOT PATROL VISUALIZATION</p>
        <h1>机器狗巡检可视化控制台 <small>V3.4</small></h1>
      </div>
      <div class="metrics">
        <div><span>连接状态</span><strong :class="connectionTone"><i />{{ connectionLabel }}</strong></div>
        <div><span>定位质量</span><strong :class="localizationTone">{{ localizationLabel }}</strong><small>{{ statusDetail }}</small></div>
        <div><span>实时速度</span><strong>{{ Number(robotPose.speed || 0).toFixed(2) }} m/s</strong></div>
        <div><span>航点进度</span><strong>{{ completedWaypoints }}/{{ waypoints.length }}</strong></div>
        <div><span>剩余电量</span><strong :class="battery != null && battery < 20 ? 'bad' : ''">{{ battery == null ? '—' : `${battery.toFixed(0)}%` }}</strong></div>
        <div><span>运行时长</span><strong>{{ formatRuntime(runtimeSeconds) }}</strong></div>
      </div>
    </header>

    <section class="control-strip">
      <div class="mode-switch">
        <button type="button" :class="{ active: dataMode === 'sim' }" @click="setMode('sim')">模拟调试</button>
        <button type="button" :class="{ active: dataMode === 'ros' }" @click="setMode('ros')">ROS 真实数据</button>
      </div>
      <input v-if="dataMode === 'ros'" v-model="liveUrl" class="ws-input" aria-label="Foxglove WebSocket 地址" spellcheck="false" />
      <select v-model="backgroundMode" aria-label="地图底图">
        <option value="grid">本地坐标</option>
        <option value="satellite" :disabled="!amapAvailable">高德卫星</option>
        <option value="road" :disabled="!amapAvailable">高德路网</option>
      </select>
      <button type="button" class="control primary" :disabled="connectionState === 'connecting'" @click="connect">连接</button>
      <button type="button" class="control success" :disabled="!canStart" @click="start">{{ running ? '暂停' : '启动实时' }}</button>
      <button type="button" class="control" :disabled="!running" @click="stop">停止</button>
      <button type="button" class="control" :disabled="connectionState === 'disconnected'" @click="disconnect">断开</button>
      <button type="button" class="control" @click="resetScene">场景重置</button>
      <button type="button" class="control" @click="clearData">数据清空</button>
      <button type="button" class="control task-button" @click="taskDrawerOpen = true">演示任务 <b>{{ waypoints.length }}</b></button>
      <small v-if="!amapAvailable" class="map-config-tip">{{ amapMessage }}</small>
    </section>

    <section class="visual-grid">
      <PatrolMapPanel
        class="global-map" :background-mode="backgroundMode" :robot-pose="robotPose" :trail="trail"
        :waypoints="waypoints" :obstacles="obstacles" :point-cloud="pointCloud" :geo-reference="geoReference"
        @amap-availability="mapAvailability"
      />
      <div class="sensor-stack">
        <PointCloudPanel :cloud="pointCloud" :yaw="robotPose.yaw" />
        <ImuRealtimeChart :samples="imuSamples" />
      </div>
    </section>

    <section class="log-panel">
      <header>
        <div><strong>系统运行日志</strong><span>ROS / NDT / MPPI / WAYPOINT</span></div>
        <small>{{ logLocked ? '已锁定历史位置' : '自动跟随最新记录' }}</small>
      </header>
      <div ref="logBox" class="log-list" @mouseenter="logLocked = true" @mouseleave="logLocked = false">
        <div v-for="item in logs" :key="item.id" :class="`log-${item.level.toLowerCase()}`">
          <time>{{ item.time.toLocaleTimeString('zh-CN', { hour12: false }) }}</time>
          <b>{{ item.level }}</b><span>[{{ item.module }}] {{ item.message }}</span>
        </div>
      </div>
    </section>

    <div v-if="taskDrawerOpen" class="drawer-backdrop" @click="taskDrawerOpen = false" />
    <aside class="task-drawer" :class="{ open: taskDrawerOpen }" aria-label="演示任务管理">
      <header><div><small>LOCAL DEMO</small><h2>巡检演示任务</h2></div><button type="button" @click="taskDrawerOpen = false">×</button></header>
      <p class="demo-notice">此处仅演示航点与进度，不会向机器狗发送控制命令。</p>
      <div class="task-stats">
        <div><span>总航点</span><strong>{{ waypoints.length }}</strong></div>
        <div><span>已完成</span><strong>{{ completedWaypoints }}</strong></div>
        <div><span>当前</span><strong>{{ waypoints.length ? Math.min(currentWaypoint + 1, waypoints.length) : '—' }}</strong></div>
      </div>
      <div class="waypoint-form">
        <input v-model="waypointX" type="number" step="0.1" placeholder="X (m)" />
        <input v-model="waypointY" type="number" step="0.1" placeholder="Y (m)" />
        <button type="button" @click="addWaypoint">添加航点</button>
      </div>
      <div class="task-actions">
        <button type="button" class="success" @click="dispatchTask">载入演示任务</button>
        <button type="button" @click="resetTaskProgress">重置进度</button>
        <button type="button" class="danger" @click="clearTask">清空任务</button>
      </div>
      <div class="waypoint-list">
        <div v-for="(point, index) in waypoints" :key="`${point.x}-${point.y}-${index}`" :class="point.status">
          <i>{{ index + 1 }}</i>
          <span><strong>WP{{ index + 1 }}</strong><small>X {{ point.x.toFixed(2) }} · Y {{ point.y.toFixed(2) }}</small></span>
          <em>{{ point.status === 'done' ? '已完成' : point.status === 'current' ? '当前' : '等待' }}</em>
          <button type="button" @click="removeWaypoint(index)">删除</button>
        </div>
      </div>
    </aside>
  </main>
</template>

<style scoped>
.patrol-console { --console-bg: #f4f8fc; --console-panel: #ffffff; --console-line: #d3e0eb; --console-text: #18324b; --console-muted: #6b8298; position: relative; display: grid; gap: 10px; min-width: 0; padding: 10px; overflow: hidden; border: 1px solid #cbdbe8; border-radius: 16px; color: var(--console-text); background: linear-gradient(145deg, #f8fbff, #edf5fb 55%, #e6f0f7); box-shadow: 0 24px 80px rgba(71, 111, 145, .16); }
.portrait-tip { display: none; align-items: center; justify-content: space-between; gap: 12px; padding: 9px 12px; border-radius: 9px; color: #1b1405; background: #ffb84d; font-size: 12px; }
.portrait-tip button { min-height: 32px; padding: 4px 10px; border: 0; border-radius: 6px; color: #fff; background: #322819; }
.console-header { display: grid; grid-template-columns: minmax(250px, .7fr) minmax(650px, 1.6fr); gap: 18px; align-items: center; padding: 12px 14px; border: 1px solid var(--console-line); border-radius: 12px; background: rgba(255, 255, 255, .9); }
.console-title p { margin: 0 0 5px; color: #1686b8; font: 10px ui-monospace, monospace; letter-spacing: .16em; }
.console-title h1 { margin: 0; font-size: clamp(17px, 1.45vw, 23px); }
.console-title small { color: #7890a5; font-size: 11px; }
.metrics { display: grid; grid-template-columns: repeat(6, minmax(88px, 1fr)); gap: 7px; }
.metrics > div { display: grid; gap: 3px; min-width: 0; padding: 8px 9px; border: 1px solid rgba(83, 125, 166, .2); border-radius: 8px; background: rgba(245, 250, 255, .9); }
.metrics span, .metrics small { overflow: hidden; color: var(--console-muted); font-size: 9px; text-overflow: ellipsis; white-space: nowrap; }
.metrics strong { color: #1f3d58; font: 600 13px ui-monospace, monospace; white-space: nowrap; }
.metrics strong i { display: inline-block; width: 6px; height: 6px; margin-right: 5px; border-radius: 50%; background: currentColor; box-shadow: 0 0 8px currentColor; }
.ok { color: #4be29a !important; } .warn { color: #ffd166 !important; } .bad { color: #ff6677 !important; }
.control-strip { display: flex; flex-wrap: wrap; align-items: center; gap: 7px; padding: 9px 11px; border: 1px solid var(--console-line); border-radius: 11px; background: rgba(255, 255, 255, .82); }
.mode-switch { display: flex; padding: 3px; border: 1px solid #c8d8e5; border-radius: 8px; background: #edf5fb; }
.mode-switch button { min-height: 34px; padding: 5px 10px; border: 0; border-radius: 6px; color: #6b8298; background: transparent; }
.mode-switch button.active { color: #fff; background: #168fbd; font-weight: 700; }
.control-strip input, .control-strip select, .control { min-height: 40px; border: 1px solid #c6d6e2; border-radius: 8px; color: #25445f; background: #f8fbfe; }
.ws-input { flex: 1 1 260px; min-width: 220px; padding: 0 10px; font: 11px ui-monospace, monospace; }
.control-strip select { padding: 0 9px; }
.control { padding: 0 11px; }
.control.primary { border-color: #2586ac; color: #fff; background: #167ca7; }
.control.success, .task-actions .success { border-color: #23855f; color: #fff; background: #16835b; }
.task-button { margin-left: auto; }
.task-button b { display: inline-grid; min-width: 18px; height: 18px; margin-left: 4px; place-content: center; border-radius: 9px; background: #42bfe5; color: #12364d; font-size: 10px; }
.map-config-tip { color: #788fa3; font-size: 10px; }
.visual-grid { display: grid; grid-template-columns: minmax(0, 1.22fr) minmax(350px, .78fr); gap: 10px; min-height: min(670px, calc(100vh - 375px)); }
.sensor-stack { display: grid; grid-template-rows: 55fr 45fr; gap: 10px; min-width: 0; min-height: 0; }
.global-map { min-height: 100%; }
.log-panel { display: grid; grid-template-rows: auto minmax(0, 1fr); height: 138px; overflow: hidden; border: 1px solid var(--console-line); border-radius: 11px; background: #f8fbfe; }
.log-panel header { display: flex; align-items: center; justify-content: space-between; gap: 12px; padding: 9px 12px; border-bottom: 1px solid #d9e5ee; }
.log-panel header div { display: flex; align-items: baseline; gap: 10px; }
.log-panel header strong { font-size: 12px; } .log-panel header span, .log-panel header small { color: var(--console-muted); font: 9px ui-monospace, monospace; letter-spacing: .05em; }
.log-list { overflow-y: auto; padding: 6px 11px 10px; font: 11px/1.65 ui-monospace, monospace; }
.log-list > div { display: grid; grid-template-columns: 68px 44px minmax(0, 1fr); gap: 8px; }
.log-list time { color: #7892a7; } .log-list b { font-size: 10px; } .log-info b { color: #148ebc; } .log-warn b { color: #ad7100; } .log-error b { color: #d13c52; }
.log-list span { color: #405d74; overflow-wrap: anywhere; }
.drawer-backdrop { position: fixed; z-index: 119; inset: 0; background: rgba(0, 0, 0, .45); }
.task-drawer { position: fixed; z-index: 120; top: 0; right: 0; width: min(410px, 100vw); height: 100vh; padding: 18px; overflow-y: auto; border-left: 1px solid #c7d9e6; color: var(--console-text); background: #f7fbfe; box-shadow: -24px 0 70px rgba(71, 111, 145, .24); transform: translateX(101%); transition: transform .25s ease; }
.task-drawer.open { transform: translateX(0); }
.task-drawer > header { display: flex; align-items: center; justify-content: space-between; padding-bottom: 13px; border-bottom: 1px solid #d3e0eb; }
.task-drawer h2 { margin: 3px 0 0; font-size: 19px; } .task-drawer header small { color: #1689b5; font: 9px ui-monospace, monospace; letter-spacing: .15em; }
.task-drawer header button { width: 38px; height: 38px; border: 1px solid #c7d9e6; border-radius: 8px; color: #34536b; background: #eef6fb; font-size: 22px; }
.demo-notice { margin: 12px 0; padding: 10px 11px; border: 1px solid #e6c878; border-radius: 8px; color: #8a6100; background: #fff8df; font-size: 11px; line-height: 1.5; }
.task-stats { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-bottom: 12px; }
.task-stats div { display: grid; gap: 5px; padding: 10px; border: 1px solid #d3e0eb; border-radius: 8px; background: #fff; }
.task-stats span { color: #6b8298; font-size: 10px; } .task-stats strong { font: 18px ui-monospace, monospace; }
.waypoint-form { display: grid; grid-template-columns: 1fr 1fr auto; gap: 7px; }
.waypoint-form input, .waypoint-form button, .task-actions button { min-width: 0; min-height: 42px; padding: 0 9px; border: 1px solid #c6d6e2; border-radius: 8px; color: #25445f; background: #fff; }
.waypoint-form button { color: #fff; background: #167ca7; }
.task-actions { display: flex; flex-wrap: wrap; gap: 7px; margin: 9px 0 14px; }
.task-actions .danger { border-color: #e0aeb8; color: #b8324a; background: #fff1f3; }
.waypoint-list { display: grid; gap: 7px; }
.waypoint-list > div { display: grid; grid-template-columns: 30px minmax(0, 1fr) auto auto; gap: 8px; align-items: center; padding: 9px; border: 1px solid #d3e0eb; border-radius: 8px; background: #fff; }
.waypoint-list > div.current { border-color: #2e91bd; } .waypoint-list > div.done { opacity: .72; }
.waypoint-list i { display: grid; width: 26px; height: 26px; place-content: center; border-radius: 50%; color: #fff; background: #317be0; font-size: 10px; font-style: normal; }
.waypoint-list .done i { background: #28a66e; }
.waypoint-list span { display: grid; gap: 2px; } .waypoint-list span strong { font-size: 11px; } .waypoint-list span small { color: #7895ac; font: 9px ui-monospace, monospace; }
.waypoint-list em { color: #4f7795; font-size: 9px; font-style: normal; }
.waypoint-list button { min-height: 32px; padding: 4px 7px; border: 1px solid #e0aeb8; border-radius: 6px; color: #b8324a; background: #fff1f3; font-size: 10px; }
@media (max-width: 1250px) {
  .console-header { grid-template-columns: 1fr; }
  .metrics { grid-template-columns: repeat(6, 1fr); }
  .visual-grid { grid-template-columns: minmax(0, 1.15fr) minmax(320px, .85fr); }
}
@media (max-width: 900px) {
  .metrics { grid-template-columns: repeat(3, 1fr); }
  .visual-grid { grid-template-columns: 1fr; min-height: auto; }
  .global-map { min-height: 420px; }
  .sensor-stack { grid-template-rows: 280px 260px; }
  .task-button { margin-left: 0; }
}
@media (orientation: portrait) {
  .patrol-console { overflow: visible; }
  .portrait-tip { display: flex; }
  .visual-grid { display: flex; flex-direction: column; }
  .global-map { min-height: 360px; }
  .sensor-stack { display: flex; flex-direction: column; }
  .log-panel { height: 180px; }
}
@media (max-width: 560px) {
  .metrics { grid-template-columns: repeat(2, 1fr); }
  .control-strip > .control, .control-strip select { flex: 1 1 auto; }
  .waypoint-form { grid-template-columns: 1fr 1fr; }
  .waypoint-form button { grid-column: 1 / -1; }
}
@media (prefers-reduced-motion: reduce) { .task-drawer { transition: none; } }
</style>
