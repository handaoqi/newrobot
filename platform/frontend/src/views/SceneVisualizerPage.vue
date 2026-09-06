<script setup>
import { computed, onBeforeUnmount, onMounted, reactive, ref, watch } from 'vue'

import SceneViewport from '../components/scene/SceneViewport.vue'
import SystemLogPanel from '../components/SystemLogPanel.vue'
import {
  fetchMapScene, fetchMapSceneCloud, fetchMapSummaries, fetchRobotNavigationStatus,
  fetchRobotPersonDetections, fetchRobotStatus, fetchRobots, fetchRouteDetail,
  fetchRouteSummaries,
} from '../services/api'
import { openLiveMessageSource, scanBagMessages } from '../services/rosStream'
import {
  ASSET_REGISTRY, SCENE_LAYER_DEFAULTS, SCENE_TOPICS, assetForClass,
  createTfTree, decodeJsonString, localizationProcess, normalizeSceneAssetInstance, normalizeSemanticObjects,
  occupancyGridToPoints, pointCloud2ToArrays, transformPointData, transformPoseTo2D,
} from '../services/sceneData'

const LIVE_URL_KEY = 'scene_visualizer_live_ws'
const sourceMode = ref('live')
const robots = ref([])
const maps = ref([])
const routes = ref([])
const selectedRobotId = ref('')
const selectedMapId = ref('')
const selectedRouteId = ref('')
const liveUrl = ref(localStorage.getItem(LIVE_URL_KEY) || `${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/foxglove/ws`)
const manifest = ref(null)
const cloudBuffer = ref(null)
const liveCloud = ref(null)
const obstacles = ref(null)
const streamPose = ref(null)
const trail = ref([])
const semanticObjects = ref([])
const projectionStatus = ref({})
const viewerStatus = ref({})
const liveDecision = ref({})
const robotStatus = ref(null)
const navigationStatus = ref(null)
const selectedRoute = ref(null)
const personDetections = ref(null)
const loading = ref(true)
const sceneLoading = ref(false)
const error = ref('')
const connectionState = ref('idle')
const viewMode = ref('2d')
const cameraPreset = ref('overview')
const panel = ref('localization')
const layers = reactive({ ...SCENE_LAYER_DEFAULTS })
const renderStats = ref({ fps: 0, calls: 0, points: 0 })
const bagPicker = ref(null)
const bagName = ref('')
const bagProgress = ref(0)
const bagDuration = ref(0)
const bagTime = ref(0)
const bagEvents = ref([])
let liveSource = null
let pollTimer = null
let pollController = null
let lastLiveCloudAt = -Infinity
let pendingCloud = null
let pendingCostmap = null
const tfTree = createTfTree('map')

const selectedRobot = computed(() => robots.value.find(item => String(item.id) === String(selectedRobotId.value)))
const selectedMap = computed(() => maps.value.find(item => String(item.id) === String(selectedMapId.value)))
const routeWaypoints = computed(() => selectedRoute.value?.waypoints || [])
const status = computed(() => navigationStatus.value?.status || robotStatus.value?.status || {})
const robotPose = computed(() => streamPose.value || ({ x: Number(status.value.x), y: Number(status.value.y), z: Number(status.value.z || 0), yaw: Number(status.value.yaw || 0) }))
const processSteps = computed(() => localizationProcess(status.value))
const quality = computed(() => status.value.localization_quality || {})
const decision = computed(() => ({ ...(quality.value.decision || {}), ...liveDecision.value }))
const sensors = computed(() => status.value.sensors || {})
const timeDiagnostics = computed(() => status.value.time_diagnostics || {})
const dataAge = computed(() => {
  const stamp = Date.parse(status.value.received_at || status.value.sampled_at || '')
  return Number.isFinite(stamp) ? Math.max(0, (Date.now() - stamp) / 1000).toFixed(1) : '—'
})
const sourceLabel = computed(() => ({ live: '实时机器狗', map: '平台离线地图包', bag: '本地 MCAP' }[sourceMode.value]))
const visibleObjects = computed(() => [
  ...(manifest.value?.static_assets || []).map((item, index) => normalizeSceneAssetInstance({ ...item, dynamic: false }, index)),
  ...semanticObjects.value,
].map(item => ({ ...item, asset: assetForClass(item.className || item.assetId) })))
const viewerRejection = computed(() => Object.values(viewerStatus.value).filter(Boolean).join('；'))
const correction = computed(() => {
  const drift = decision.value.ndt_drift
  const pose = robotPose.value
  if (!drift || !Number.isFinite(Number(drift.dx_m)) || !Number.isFinite(Number(pose.x))) return null
  return { from: { x: Number(pose.x) - Number(drift.dx_m), y: Number(pose.y) - Number(drift.dy_m || 0) }, to: pose }
})

function mapTransformFor(message) {
  const frame = message?.header?.frame_id
  return frame ? tfTree.matrixFrom(frame) : null
}

function consumeCloud(message, at) {
  const matrix = mapTransformFor(message)
  if (!matrix) {
    pendingCloud = { message, at }
    viewerStatus.value = { ...viewerStatus.value, localCloud: `局部点云 tf_missing:${message?.header?.frame_id || 'unknown'}→map` }
    return false
  }
  liveCloud.value = transformPointData(pointCloud2ToArrays(message, { maxPoints: 80_000, colorMode: 'height' }), matrix)
  pendingCloud = null
  const { localCloud: _cleared, ...remaining } = viewerStatus.value
  viewerStatus.value = remaining
  lastLiveCloudAt = at
  return true
}

function consumeCostmap(message) {
  const matrix = mapTransformFor(message)
  if (!matrix) {
    pendingCostmap = message
    viewerStatus.value = { ...viewerStatus.value, obstacles: `障碍物 tf_missing:${message?.header?.frame_id || 'unknown'}→map` }
    return false
  }
  obstacles.value = occupancyGridToPoints(message, { frameTransform: matrix })
  pendingCostmap = null
  const { obstacles: _cleared, ...remaining } = viewerStatus.value
  viewerStatus.value = remaining
  return true
}

function semanticObjectsInMap(message, at) {
  const matrix = mapTransformFor(message)
  if (!matrix) {
    viewerStatus.value = { ...viewerStatus.value, semantic: `语义对象 tf_missing:${message?.header?.frame_id || 'unknown'}→map` }
    return []
  }
  const items = normalizeSemanticObjects(message, at).map(item => {
    const pose = transformPoseTo2D(item.position, item.orientation, matrix)
    return pose ? { ...item, position: { x: pose.x, y: pose.y, z: pose.z }, orientation: { x: 0, y: 0, z: Math.sin(pose.yaw / 2), w: Math.cos(pose.yaw / 2) } } : item
  })
  const { semantic: _cleared, ...remaining } = viewerStatus.value
  viewerStatus.value = remaining
  return items
}

function poseFromOdometry(message) {
  const pose = message?.pose?.pose
  if (!pose) return null
  const matrix = mapTransformFor(message)
  if (!matrix) return null
  return transformPoseTo2D(pose.position, pose.orientation, matrix)
}

function number(value, digits = 2, suffix = '') {
  const parsed = Number(value)
  return Number.isFinite(parsed) ? `${parsed.toFixed(digits)}${suffix}` : '—'
}

function timestamp(value) {
  if (!value) return '—'
  const date = new Date(value)
  return Number.isNaN(date.getTime()) ? String(value) : date.toLocaleTimeString('zh-CN', { hour12: false, fractionalSecondDigits: 3 })
}

function statusTone(value) {
  if (value === true || ['normal', 'relocalized', 'online', 'ok', 'ndt_imu', 'rtk_imu', 'lio_imu'].includes(value)) return 'ok'
  if (['lost', 'offline', 'unavailable', false].includes(value)) return 'bad'
  return 'warn'
}

async function loadCatalogs() {
  loading.value = true
  error.value = ''
  try {
    const [robotRows, mapRows, routeRows] = await Promise.all([fetchRobots(), fetchMapSummaries(), fetchRouteSummaries()])
    robots.value = robotRows
    maps.value = mapRows
    routes.value = routeRows
    selectedRobotId.value ||= String(robotRows.find(item => item.connection_status === 'online')?.id || robotRows[0]?.id || '')
    selectedMapId.value ||= String(mapRows.find(item => item.active)?.id || mapRows[0]?.id || '')
    const firstRoute = routeRows.find(item => String(item.map_data) === String(selectedMapId.value)) || routeRows[0]
    selectedRouteId.value ||= String(firstRoute?.id || '')
  } catch (cause) {
    error.value = cause.message || '场景目录加载失败'
  } finally {
    loading.value = false
  }
}

async function loadScene() {
  cloudBuffer.value = null
  manifest.value = null
  if (!selectedMapId.value) return
  sceneLoading.value = true
  try {
    manifest.value = await fetchMapScene(selectedMapId.value)
    if (manifest.value.cloud?.available) cloudBuffer.value = await fetchMapSceneCloud(selectedMapId.value)
  } catch (cause) {
    error.value = cause.message || '地图场景加载失败'
  } finally {
    sceneLoading.value = false
  }
}

async function loadRoute() {
  selectedRoute.value = null
  if (!selectedRouteId.value) return
  try { selectedRoute.value = await fetchRouteDetail(selectedRouteId.value) } catch (cause) { error.value = cause.message }
}

async function pollStatus() {
  if (!selectedRobotId.value || sourceMode.value !== 'live') return
  pollController?.abort()
  pollController = new AbortController()
  try {
    const [robot, navigation, detections] = await Promise.all([
      fetchRobotStatus(selectedRobotId.value, { signal: pollController.signal }),
      fetchRobotNavigationStatus(selectedRobotId.value, { signal: pollController.signal }),
      fetchRobotPersonDetections(selectedRobotId.value, { signal: pollController.signal }),
    ])
    robotStatus.value = robot
    navigationStatus.value = navigation
    personDetections.value = detections
  } catch (cause) {
    if (cause.name !== 'AbortError') error.value = cause.message || '实时状态刷新失败'
  }
}

function stopLive() {
  liveSource?.close?.()
  liveSource = null
  connectionState.value = 'idle'
}

function handleMessage({ topic, timestamp: at, message }) {
  if (topic === '/tf' || topic === '/tf_static') {
    tfTree.update(message)
    if (pendingCloud) consumeCloud(pendingCloud.message, pendingCloud.at)
    if (pendingCostmap) consumeCostmap(pendingCostmap)
  } else if (topic === '/front_lidar') {
    if (at - lastLiveCloudAt >= .2) {
      consumeCloud(message, at)
    }
  } else if (topic === '/local_costmap/costmap_raw') {
    consumeCostmap(message)
  } else if (topic === '/odom/localization_odom') {
    const pose = poseFromOdometry(message)
    if (pose && [pose.x, pose.y].every(Number.isFinite)) {
      streamPose.value = pose
      const last = trail.value.at(-1)
      if (!last || Math.hypot(pose.x - last.x, pose.y - last.y) >= .05) trail.value = [...trail.value.slice(-499), pose]
    }
  } else if (topic === '/perception/semantic_objects') {
    semanticObjects.value = semanticObjectsInMap(message, at)
  } else if (topic === '/perception/projection_status') {
    projectionStatus.value = decodeJsonString(message) || {}
  } else if (topic === '/localization/decision') {
    const next = decodeJsonString(message)
    if (next) liveDecision.value = next
  } else if (topic === '/status') {
    Object.assign(quality.value, {
      has_converged: Boolean(message.has_converged),
      matching_error: Number(message.matching_error),
      inlier_fraction: Number(message.inlier_fraction),
    })
  }
}

function connectLive() {
  stopLive()
  if (sourceMode.value !== 'live') return
  localStorage.setItem(LIVE_URL_KEY, liveUrl.value)
  connectionState.value = 'connecting'
  liveSource = openLiveMessageSource(liveUrl.value, SCENE_TOPICS, {
    onMessage: handleMessage,
    onReady: () => { connectionState.value = 'online' },
    onError: cause => { connectionState.value = 'warning'; error.value = cause.message || '实时场景连接异常' },
  })
}

function selectSource(mode) {
  sourceMode.value = mode
  semanticObjects.value = []
  liveCloud.value = null
  obstacles.value = null
  streamPose.value = null
  trail.value = []
  viewerStatus.value = {}
  pendingCloud = null
  pendingCostmap = null
  lastLiveCloudAt = -Infinity
  tfTree.clear()
  if (mode === 'live') connectLive()
  else stopLive()
  if (mode === 'bag') bagPicker.value?.click()
}

function selectCamera(preset) {
  cameraPreset.value = preset
  if (preset === 'dog') viewMode.value = '3d'
}

async function openBag(event) {
  const file = event.target.files?.[0]
  event.target.value = ''
  if (!file) return
  bagName.value = file.name
  bagProgress.value = 0
  bagEvents.value = []
  error.value = ''
  let lastCloud = -Infinity
  try {
    const result = await scanBagMessages(file, SCENE_TOPICS, {
      onProgress: value => { bagProgress.value = value },
      onMessage: item => {
        if (item.topic === '/tf' || item.topic === '/tf_static') {
          tfTree.update(item.message)
        } else if (item.topic === '/front_lidar') {
          if (item.timestamp - lastCloud < 1 || bagEvents.value.filter(row => row.kind === 'cloud').length >= 120) return
          lastCloud = item.timestamp
          const matrix = mapTransformFor(item.message)
          if (matrix) bagEvents.value.push({ at: item.timestamp, kind: 'cloud', value: transformPointData(pointCloud2ToArrays(item.message, { maxPoints: 20_000 }), matrix) })
        } else if (item.topic === '/perception/semantic_objects') {
          bagEvents.value.push({ at: item.timestamp, kind: 'objects', value: semanticObjectsInMap(item.message, item.timestamp) })
        } else if (item.topic === '/localization/decision') {
          bagEvents.value.push({ at: item.timestamp, kind: 'decision', value: decodeJsonString(item.message) || {} })
        } else if (item.topic === '/odom/localization_odom') {
          const pose = item.message?.pose?.pose
          const normalized = poseFromOdometry(item.message)
          if (normalized) bagEvents.value.push({ at: item.timestamp, kind: 'pose', value: normalized })
        } else if (item.topic === '/local_costmap/costmap_raw') {
          const matrix = mapTransformFor(item.message)
          if (matrix) bagEvents.value.push({ at: item.timestamp, kind: 'obstacles', value: occupancyGridToPoints(item.message, { frameTransform: matrix }) })
        }
      },
    })
    bagDuration.value = result.duration
    bagTime.value = 0
    applyBagTime()
  } catch (cause) {
    error.value = `MCAP解析失败：${cause.message}`
  }
}

function applyBagTime() {
  const latest = kind => bagEvents.value.filter(item => item.kind === kind && item.at <= bagTime.value).at(-1)?.value
  liveCloud.value = latest('cloud') || null
  semanticObjects.value = latest('objects') || []
  projectionStatus.value = latest('projection') || {}
  liveDecision.value = latest('decision') || {}
  obstacles.value = latest('obstacles') || null
  const pose = latest('pose')
  streamPose.value = pose || null
  trail.value = bagEvents.value.filter(item => item.kind === 'pose' && item.at <= bagTime.value).slice(-500).map(item => item.value)
  if (pose) navigationStatus.value = { status: { ...(navigationStatus.value?.status || {}), ...pose, localization_status: 'replay' } }
}

function locateLog(item) {
  cameraPreset.value = 'overview'
  if (Number.isFinite(Number(item.x))) navigationStatus.value = { status: { ...status.value, x: item.x, y: item.y, yaw: item.yaw || 0 } }
}

watch(selectedMapId, () => {
  loadScene()
  const matching = routes.value.find(item => String(item.map_data) === String(selectedMapId.value))
  if (matching) selectedRouteId.value = String(matching.id)
})
watch(selectedRouteId, loadRoute)
watch(selectedRobotId, () => { pollStatus(); if (sourceMode.value === 'live') connectLive() })
watch(bagTime, applyBagTime)

onMounted(async () => {
  await loadCatalogs()
  await Promise.all([loadScene(), loadRoute(), pollStatus()])
  connectLive()
  pollTimer = window.setInterval(pollStatus, 1000)
})

onBeforeUnmount(() => {
  stopLive()
  pollController?.abort()
  if (pollTimer) window.clearInterval(pollTimer)
})
</script>

<template>
  <section class="scene-page">
    <header class="scene-head">
      <div>
        <p class="eyebrow">HUMAN VIEW · READ ONLY</p>
        <h2>场景视角调试</h2>
        <p>把机器狗看到的点云、定位决策、障碍物和语义目标还原到人的场景视角。</p>
      </div>
      <div class="source-tabs" aria-label="数据源">
        <button v-for="item in [['live','实时机器狗'],['map','离线地图包'],['bag','本地 MCAP']]" :key="item[0]" :class="{ active: sourceMode === item[0] }" @click="selectSource(item[0])">{{ item[1] }}</button>
      </div>
    </header>

    <div v-if="error" class="error-banner"><span>{{ error }}</span><button @click="error = ''">×</button></div>

    <section class="control-bar">
      <label>机器人<select v-model="selectedRobotId"><option v-for="robot in robots" :key="robot.id" :value="String(robot.id)">{{ robot.code }} · {{ robot.name }}</option></select></label>
      <label>世界/场景地图<select v-model="selectedMapId"><option v-for="map in maps" :key="map.id" :value="String(map.id)">{{ map.active ? '● ' : '' }}{{ map.name }}</option></select></label>
      <label>巡检路线<select v-model="selectedRouteId"><option value="">不叠加路线</option><option v-for="route in routes.filter(item => String(item.map_data) === String(selectedMapId))" :key="route.id" :value="String(route.id)">{{ route.name }}</option></select></label>
      <div class="runtime-state"><i :class="statusTone(connectionState)"></i><strong>{{ sourceLabel }}</strong><span>{{ sourceMode === 'live' ? `${dataAge}s 前` : sourceMode === 'bag' ? bagName || '未选文件' : '静态' }}</span></div>
    </section>

    <section class="scene-workspace">
      <article class="viewport-card">
        <div class="viewport-toolbar">
          <div class="segmented"><button :class="{ active: viewMode === '2d' }" @click="viewMode = '2d'">2D</button><button :class="{ active: viewMode === '3d' }" @click="viewMode = '3d'">3D</button><span>滚轮/双指自动切换</span></div>
          <div class="segmented"><button v-for="item in [['overview','俯视'],['follow','跟随'],['dog','机器狗视角']]" :key="item[0]" :class="{ active: cameraPreset === item[0] }" @click="selectCamera(item[0])">{{ item[1] }}</button></div>
          <span class="render-stats">{{ renderStats.fps }} FPS · {{ renderStats.points.toLocaleString() }} 点</span>
        </div>
        <div class="viewport-wrap">
          <SceneViewport :manifest="manifest" :cloud-buffer="cloudBuffer" :live-cloud="liveCloud" :obstacles="obstacles" :trail="trail" :correction="correction" :robot-pose="robotPose" :waypoints="routeWaypoints" :semantic-objects="semanticObjects" :layers="layers" :mode="viewMode" :camera-preset="cameraPreset" @mode-change="viewMode = $event" @stats="renderStats = $event" @error="error = $event" />
          <div v-if="loading || sceneLoading" class="scene-loading">{{ loading ? '正在加载设备与地图…' : '正在生成/加载三维点云预览…' }}</div>
          <div class="scene-legend"><span><i class="robot"></i>机器狗</span><span><i class="route"></i>规划路线</span><span><i class="cloud"></i>局部点云</span><span><i class="object"></i>识别资产</span></div>
        </div>
        <div v-if="sourceMode === 'bag'" class="bag-timeline"><strong>{{ bagName || '请选择MCAP' }}</strong><input v-model.number="bagTime" type="range" min="0" :max="bagDuration || 1" step="0.1" :disabled="!bagDuration"/><span>{{ number(bagTime,1,'s') }} / {{ number(bagDuration,1,'s') }}</span><small v-if="bagProgress < 1">解析 {{ Math.round(bagProgress * 100) }}%</small></div>
      </article>

      <aside class="diagnostic-card">
        <nav class="diagnostic-tabs">
          <button v-for="item in [['localization','定位/重定位'],['fusion','融合校正'],['waypoint','航点/算法'],['layers','图层/资产']]" :key="item[0]" :class="{ active: panel === item[0] }" @click="panel = item[0]">{{ item[1] }}</button>
        </nav>

        <div v-if="panel === 'localization'" class="diagnostic-body">
          <div class="process-list"><div v-for="step in processSteps" :key="step.key" :class="['process-step', step.tone]"><i></i><span>{{ step.label }}</span><strong>{{ step.tone === 'ok' ? '完成' : step.tone === 'active' ? '处理中' : step.tone === 'warning' ? '异常' : '等待' }}</strong></div></div>
          <h3>实时定位证据</h3>
          <dl class="metric-list">
            <div><dt>定位状态</dt><dd :class="statusTone(status.localization_status)">{{ status.localization_status || '未上报' }}</dd></div>
            <div><dt>当前定位源</dt><dd :class="statusTone(decision.active_source)">{{ decision.active_source || 'unavailable' }}</dd></div>
            <div><dt>地图 / 坐标系</dt><dd>{{ status.map_id || selectedMapId || '—' }} / {{ manifest?.frame_id || 'map' }}</dd></div>
            <div><dt>位姿 X / Y / Yaw</dt><dd>{{ number(status.x) }} / {{ number(status.y) }} / {{ number(status.yaw,3) }}</dd></div>
            <div><dt>全局重定位</dt><dd>{{ decision.global_relocalization?.state || 'idle' }}</dd></div>
            <div><dt>稳定确认</dt><dd>{{ decision.absolute_stable ? '已稳定' : `${decision.absolute_stable_samples || 0} 帧` }}</dd></div>
            <div><dt>最近状态时间</dt><dd>{{ timestamp(status.sampled_at) }}</dd></div>
          </dl>
        </div>

        <div v-else-if="panel === 'fusion'" class="diagnostic-body">
          <div class="fusion-flow"><span>IMU预测</span><b>→</b><span>LIO里程计</span><b>→</b><span>NDT / RTK门控</span><b>→</b><strong>{{ decision.active_source || '无可用源' }}</strong></div>
          <h3>IMU + RTK + NDT</h3>
          <dl class="metric-list">
            <div><dt>IMU</dt><dd :class="statusTone(sensors.imu?.online)">{{ sensors.imu?.online ? number(sensors.imu.frequency_hz,1,' Hz') : '离线/未上报' }}</dd></div>
            <div><dt>RTK</dt><dd :class="statusTone(sensors.rtk?.online)">{{ sensors.rtk?.quality || decision.rtk_quality || '未上报' }}</dd></div>
            <div><dt>NDT 收敛 / 分数</dt><dd :class="statusTone(quality.has_converged)">{{ quality.has_converged ? '是' : '否' }} / {{ number(quality.matching_error,3) }}</dd></div>
            <div><dt>NDT 内点率</dt><dd>{{ number(Number(quality.inlier_fraction) * 100,1,'%') }}</dd></div>
            <div><dt>激光→IMU时间差</dt><dd>{{ number(timeDiagnostics.lidar_to_imu_delta_ms,1,' ms') }}</dd></div>
            <div><dt>激光→RTK时间差</dt><dd>{{ number(timeDiagnostics.lidar_to_rtk_delta_ms,1,' ms') }}</dd></div>
            <div><dt>RTK阻塞原因</dt><dd>{{ decision.rtk_blocked_reason || '无' }}</dd></div>
            <div><dt>校正候选 / 平滑</dt><dd>{{ decision.correction_candidate_source || 'none' }} / {{ decision.correction_smoothing_active ? '进行中' : '未启用' }}</dd></div>
            <div><dt>投影同步 / 点数</dt><dd>{{ number(projectionStatus.sync_delta_ms,1,' ms') }} / {{ projectionStatus.selected_points ?? '—' }}</dd></div>
            <div><dt>投影拒绝原因</dt><dd>{{ projectionStatus.rejection_reason || '无' }}</dd></div>
            <div><dt>视图坐标链</dt><dd :class="viewerRejection ? 'bad' : 'ok'">{{ viewerRejection || 'map 对齐' }}</dd></div>
          </dl>
        </div>

        <div v-else-if="panel === 'waypoint'" class="diagnostic-body">
          <div class="readonly-note">只读检查：这里显示保存配置与后端返回的实际生效算法，不提供下发按钮。</div>
          <h3>{{ selectedRoute?.name || '未选择路线' }}</h3>
          <div class="waypoint-list">
            <article v-for="(point, index) in routeWaypoints" :key="point.id || index"><header><b>{{ index + 1 }}</b><strong>{{ selectedRoute?.waypoint_names?.[index] || point.name || `航点${index + 1}` }}</strong><span>{{ number(point.x) }}, {{ number(point.y) }}</span></header><div><span>Yaw {{ number(point.yaw,2) }}</span><span>定位 {{ point.localization_mode || 'ndt' }}</span><span>全局 {{ point.global_controller || selectedRoute?.global_controller || 'theta_star' }}</span><span>局部 {{ point.local_controller || 'mppi' }}</span><span>避障 {{ point.avoidance_to_next === false ? '关闭' : '开启' }}</span><span>到点朝向 {{ point.require_yaw === false ? '不要求' : '要求' }}</span></div></article>
            <p v-if="!routeWaypoints.length" class="empty">当前路线没有航点。</p>
          </div>
        </div>

        <div v-else class="diagnostic-body">
          <h3>场景图层</h3>
          <div class="layer-list"><label v-for="(_, key) in layers" :key="key"><input v-model="layers[key]" type="checkbox"/><span>{{ {occupancy:'2D占据图',globalCloud:'3D伪彩地图',localCloud:'实时局部点云',obstacles:'障碍物',route:'路线/航点',trail:'定位尾迹',corrections:'融合校正',semantic:'语义资产',boundary:'导航边界'}[key] }}</span></label></div>
          <div class="readonly-note boundary-note">边界仅用于可视化核对：草稿 v{{ manifest?.boundary?.revision || 0 }} / 生效 v{{ manifest?.boundary?.active_revision || 0 }} · {{ manifest?.boundary?.apply_status || '未配置' }}。编辑、校验和发布请到<a href="/dashboard/tasks/routes">路径规划</a>。</div>
          <h3>基础资产与当前识别</h3>
          <div class="asset-grid"><span v-for="(asset,key) in ASSET_REGISTRY" :key="key"><i :style="{background:asset.color}"></i>{{ asset.label }}</span></div>
          <div class="object-list"><article v-for="item in visibleObjects" :key="item.id"><i :style="{background:item.asset.color}"></i><div><strong>{{ item.asset.label }} · {{ item.id }}</strong><small>{{ number(item.position.x) }}, {{ number(item.position.y) }}, {{ number(item.position.z) }} · {{ number(item.confidence * 100,0,'%') }}</small></div></article><p v-if="!visibleObjects.length" class="empty">尚未收到经过标定和点云验证的三维目标；不会用二维框伪造地图坐标。</p></div>
          <small v-if="personDetections?.detections?.length" class="projection-pending">收到 {{ personDetections.detections.length }} 个二维YOLO框，等待 `/perception/semantic_objects` 三维投影。</small>
        </div>
      </aside>
    </section>

    <SystemLogPanel :robot-id="selectedRobotId" :map-id="selectedMapId" @locate="locateLog" />
    <input ref="bagPicker" class="file-picker" type="file" accept=".mcap" @change="openBag" />
  </section>
</template>

<style scoped>
.scene-page { display: grid; gap: 14px; min-width: 0; }
.scene-head { display: flex; align-items: flex-end; justify-content: space-between; gap: 18px; }
.scene-head h2 { margin: 2px 0 0; font-size: 24px; }.scene-head p { margin: 5px 0 0; color: var(--muted); }.eyebrow { color: var(--cyan)!important; font-size: 11px; font-weight: 800; letter-spacing: .12em; }
.source-tabs,.segmented { display: flex; align-items: center; gap: 4px; padding: 4px; border: 1px solid var(--line); border-radius: 11px; background: var(--panel-soft); }.source-tabs button,.segmented button { border: 0; border-radius: 8px; padding: 8px 11px; color: var(--muted); background: transparent; cursor: pointer; }.source-tabs button.active,.segmented button.active { color: #fff; background: #087aa0; }.segmented span { padding: 0 6px; color: var(--muted); font-size: 11px; }
.error-banner { display: flex; justify-content: space-between; padding: 10px 13px; border: 1px solid #dc6060; border-radius: 10px; color: #ffb3b3; background: #351318; }.error-banner button { border: 0; color: inherit; background: transparent; font-size: 18px; }
.control-bar { display: grid; grid-template-columns: repeat(3,minmax(180px,1fr)) auto; gap: 10px; align-items: end; padding: 11px 13px; border: 1px solid var(--line); border-radius: 12px; background: var(--panel); }.control-bar label { display: grid; gap: 4px; color: var(--muted); font-size: 11px; }.control-bar select { min-width: 0; padding: 8px 9px; border: 1px solid var(--line); border-radius: 8px; color: var(--text); background: var(--input-bg); }.runtime-state { display: grid; grid-template-columns: auto auto; gap: 2px 7px; align-items: center; min-width: 130px; }.runtime-state i { grid-row: 1 / 3; width: 9px; height: 9px; border-radius: 50%; background: #eab308; }.runtime-state i.ok { background: #22c55e; }.runtime-state i.bad { background: #ef4444; }.runtime-state span { color: var(--muted); font-size: 11px; }
.scene-workspace { display: grid; grid-template-columns: minmax(0,1.75fr) minmax(350px,.75fr); gap: 14px; min-height: min(720px,calc(100vh - 250px)); }.viewport-card,.diagnostic-card { min-width: 0; overflow: hidden; border: 1px solid var(--line); border-radius: 15px; background: var(--panel); }.viewport-card { display: grid; grid-template-rows: auto minmax(0,1fr) auto; }.viewport-toolbar { display: flex; flex-wrap: wrap; align-items: center; gap: 8px; padding: 8px; border-bottom: 1px solid var(--line); }.render-stats { margin-left: auto; color: var(--muted); font: 11px ui-monospace,monospace; }.viewport-wrap { position: relative; min-height: 0; background: #07111f; }.scene-loading { position: absolute; inset: 0; display: grid; place-content: center; color: #d9edff; background: rgba(4,12,23,.72); backdrop-filter: blur(4px); }.scene-legend { position: absolute; left: 12px; bottom: 11px; display: flex; flex-wrap: wrap; gap: 10px; padding: 7px 9px; border: 1px solid #29415a; border-radius: 9px; color: #dbeafe; background: rgba(5,15,28,.82); font-size: 10px; pointer-events: none; }.scene-legend span { display: flex; gap: 5px; align-items: center; }.scene-legend i { width: 12px; height: 3px; }.scene-legend .robot { background:#22d3ee }.scene-legend .route { background:#38bdf8 }.scene-legend .cloud { background:#7dd3fc }.scene-legend .object { background:#f59e0b }
.bag-timeline { display: grid; grid-template-columns: auto minmax(120px,1fr) auto auto; gap: 9px; align-items: center; padding: 9px 12px; border-top: 1px solid var(--line); font-size: 11px; }.bag-timeline input { width: 100%; }
.diagnostic-card { display: grid; grid-template-rows: auto minmax(0,1fr); }.diagnostic-tabs { display: grid; grid-template-columns: repeat(4,1fr); border-bottom: 1px solid var(--line); }.diagnostic-tabs button { min-width: 0; padding: 12px 4px; border: 0; border-bottom: 2px solid transparent; color: var(--muted); background: transparent; cursor: pointer; font-size: 11px; }.diagnostic-tabs button.active { color: var(--cyan); border-bottom-color: var(--cyan); background: var(--panel-soft); }.diagnostic-body { min-height: 0; padding: 14px; overflow: auto; }.diagnostic-body h3 { margin: 17px 0 8px; font-size: 13px; }.process-list { display: grid; grid-template-columns: repeat(3,1fr); gap: 7px; }.process-step { display: grid; grid-template-columns: auto 1fr; gap: 2px 6px; padding: 8px; border: 1px solid var(--line); border-radius: 8px; }.process-step i { grid-row: 1/3; width: 8px; height: 8px; margin-top: 3px; border-radius: 50%; background: #94a3b8; }.process-step span,.process-step strong { font-size: 10px; }.process-step strong { color: var(--muted); }.process-step.ok i{background:#22c55e}.process-step.active i{background:#38bdf8}.process-step.warning i{background:#ef4444}
.metric-list { margin: 0; }.metric-list div { display: grid; grid-template-columns: minmax(110px,.8fr) minmax(0,1.2fr); gap: 10px; padding: 7px 2px; border-bottom: 1px solid var(--line); font-size: 11px; }.metric-list dt { color: var(--muted); }.metric-list dd { margin: 0; text-align: right; overflow-wrap: anywhere; font-family: ui-monospace,monospace; }.ok{color:#22c55e!important}.warn{color:#eab308!important}.bad{color:#ef4444!important}.fusion-flow { display: flex; flex-wrap: wrap; gap: 5px; align-items: center; padding: 9px; border: 1px solid #245b72; border-radius: 9px; background: rgba(8,122,160,.09); font-size: 10px; }.fusion-flow span,.fusion-flow strong { padding: 5px; border-radius: 6px; background: var(--panel-soft); }.fusion-flow b { color: var(--cyan); }.readonly-note { padding: 8px 10px; border-left: 3px solid #38bdf8; color: var(--muted); background: var(--panel-soft); font-size: 11px; }.waypoint-list { display: grid; gap: 7px; }.waypoint-list article { padding: 8px; border: 1px solid var(--line); border-radius: 8px; }.waypoint-list header { display: grid; grid-template-columns: 22px 1fr auto; align-items: center; gap: 7px; font-size: 11px; }.waypoint-list header b { display:grid;place-content:center;width:20px;height:20px;border-radius:50%;color:#fff;background:#087aa0 }.waypoint-list header span { color: var(--muted); font-family:ui-monospace,monospace }.waypoint-list article>div { display:flex;flex-wrap:wrap;gap:5px;margin-top:7px }.waypoint-list article>div span { padding:3px 5px;border-radius:5px;color:var(--muted);background:var(--panel-soft);font-size:9px }
.boundary-note { margin-top:10px;line-height:1.55 }.boundary-note a { margin-left:3px;color:var(--cyan) }
.layer-list { display:grid;grid-template-columns:repeat(2,1fr);gap:6px }.layer-list label { display:flex;gap:7px;align-items:center;padding:7px;border:1px solid var(--line);border-radius:7px;font-size:10px }.asset-grid { display:grid;grid-template-columns:repeat(2,1fr);gap:6px }.asset-grid span { display:flex;gap:7px;align-items:center;font-size:10px }.asset-grid i,.object-list i { width:9px;height:9px;border-radius:2px }.object-list { display:grid;gap:6px;margin-top:12px }.object-list article { display:flex;gap:8px;align-items:center;padding:7px;border:1px solid var(--line);border-radius:7px }.object-list div { display:grid;gap:2px }.object-list strong,.object-list small { font-size:10px }.object-list small,.empty,.projection-pending { color:var(--muted) }.empty { font-size:11px;line-height:1.5 }.projection-pending { display:block;margin-top:9px;font-size:10px }.file-picker { position:absolute;width:1px;height:1px;opacity:0;pointer-events:none }
@media (max-width: 1250px) { .scene-workspace { grid-template-columns: minmax(0,1.35fr) minmax(330px,.85fr); }.control-bar { grid-template-columns: repeat(2,1fr); } }
@media (max-width: 900px) { .scene-head { align-items:stretch;flex-direction:column }.source-tabs { align-self:flex-start }.scene-workspace { grid-template-columns:1fr;min-height:0 }.viewport-wrap { min-height:430px }.diagnostic-card { max-height:600px }.control-bar { grid-template-columns:1fr 1fr }.runtime-state { grid-column:1/-1 } }
</style>
