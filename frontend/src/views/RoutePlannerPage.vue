<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref } from 'vue'
import {
  fetchMaps,
  fetchMapSets,
  fetchRobotNavigationStatus,
  fetchRobotStatus,
  fetchRobots,
  fetchRoutes,
  createRoute,
  updateRoute,
  deleteRoute,
  executeRoute,
  sendRobotNavigationCommand,
} from '../services/api'
import { API_BASE } from '../services/api'

const maps = ref([])
const mapSets = ref([])
const robots = ref([])

const getFullUrl = (relativeUrl) => {
  if (!relativeUrl) return null
  if (relativeUrl.startsWith('http')) return relativeUrl
  return `${API_BASE.replace('/api', '')}${relativeUrl}`
}
const routes = ref([])
const selectedMap = ref(null)
const selectedRoute = ref(null)
const waypoints = ref([])
const waypointNames = ref([])
const showRouteDialog = ref(false)
const loading = ref(false)
const mapImageRef = ref(null)
const imageReadyTick = ref(0)
const navStatus = ref(null)
const navCommandBusy = ref('')
const navError = ref('')
const routeExecuteBusy = ref(false)
const lastExecution = ref(null)
const initialPoseMode = ref(false)
const manualInitialPose = ref(null)
const initialPoseStep = ref('position')
const initialPoseHeadingTarget = ref(null)
const localizationInitState = ref('idle')
const localizationInitMessage = ref('')
const poseHistory = ref([])
const showPoseTrail = ref(true)
const lastPoseSampleKey = ref('')
let navTimer = null

const routeForm = ref({
  name: '',
  map_data: null,
  map_set: null,
  robot: null,
  description: '',
})

onMounted(async () => {
  await loadData()
  await refreshNavigationStatus()
  navTimer = setInterval(refreshNavigationStatus, 2000)
  window.addEventListener('resize', refreshImageGeometry)
})

onBeforeUnmount(() => {
  if (navTimer) clearInterval(navTimer)
  window.removeEventListener('resize', refreshImageGeometry)
})

async function loadData() {
  loading.value = true
  try {
    const [mapsResult, mapSetsResult, routesResult, robotsResult] = await Promise.allSettled([fetchMaps(), fetchMapSets(), fetchRoutes(), fetchRobots()])
    if (mapsResult.status === 'fulfilled') maps.value = mapsResult.value
    else console.error('加载地图失败:', mapsResult.reason)
    if (mapSetsResult.status === 'fulfilled') mapSets.value = mapSetsResult.value
    else console.error('加载地图集失败:', mapSetsResult.reason)
    if (routesResult.status === 'fulfilled') routes.value = routesResult.value
    else console.error('加载路线失败:', routesResult.reason)
    if (robotsResult.status === 'fulfilled') robots.value = robotsResult.value
    else console.error('加载机器人失败:', robotsResult.reason)
    seedRobotsFromMapsAndRoutes()
    if (!selectedMap.value && maps.value.length) {
      handleMapSelect(maps.value.find(map => map.active) || maps.value[0])
    }
  } catch (error) {
    console.error('加载数据失败:', error)
  } finally {
    loading.value = false
  }
}

const selectedRobot = computed(() => {
  const robotId = selectedRoute.value?.robot || routeForm.value.robot || selectedMap.value?.robot || robots.value[0]?.id
  const robot = robots.value.find(item => String(item.id) === String(robotId))
  if (robot) return robot
  if (!robotId) return null
  return {
    id: robotId,
    name: selectedRoute.value?.robot_name || selectedMap.value?.robot_name || '机器狗',
    code: selectedRoute.value?.robot_code || selectedMap.value?.robot_code || String(robotId),
  }
})

function seedRobotsFromMapsAndRoutes() {
  const known = new Map(robots.value.map(robot => [String(robot.id), robot]))
  for (const source of [...maps.value, ...routes.value]) {
    if (!source.robot || known.has(String(source.robot))) continue
    known.set(String(source.robot), {
      id: source.robot,
      name: source.robot_name || source.robot_code || `机器狗 ${source.robot}`,
      code: source.robot_code || String(source.robot),
    })
  }
  robots.value = Array.from(known.values())
}

function handleMapSelect(map) {
  selectedMap.value = map
  selectedRoute.value = null
  waypoints.value = []
  waypointNames.value = []
  clearPoseHistory()
  routeForm.value = {
    name: '',
    map_data: map?.id || null,
    map_set: null,
    robot: map?.robot || 1,
    description: '',
  }
  refreshImageGeometry()
  refreshNavigationStatus()
}

function handleMapClick(event) {
  if (!selectedMap.value) return

  const image = mapImageRef.value || event.currentTarget
  const geometry = getMapGeometry()
  if (!geometry) return
  const rect = geometry.rect
  const displayX = event.clientX - rect.left
  const displayY = event.clientY - rect.top
  if (displayX < 0 || displayY < 0 || displayX > rect.width || displayY > rect.height) return

  const imagePoint = displayToImagePoint(displayX, displayY, geometry)
  if (initialPoseMode.value) {
    const clickedPose = imagePointToWaypoint(imagePoint, geometry, Number(manualInitialPose.value?.yaw || 0))
    if (initialPoseStep.value === 'position' || !manualInitialPose.value) {
      manualInitialPose.value = clickedPose
      initialPoseHeadingTarget.value = null
      initialPoseStep.value = 'heading'
      navError.value = '已设置初始位置，请再点击狗头朝向'
      return
    }
    const dx = clickedPose.x - manualInitialPose.value.x
    const dy = clickedPose.y - manualInitialPose.value.y
    if (Math.hypot(dx, dy) < 0.05) {
      navError.value = '朝向点离初始位置太近，请点远一点'
      return
    }
    manualInitialPose.value = {
      ...manualInitialPose.value,
      yaw: Number(Math.atan2(dy, dx).toFixed(4)),
    }
    initialPoseHeadingTarget.value = clickedPose
    navError.value = '已设置初始朝向，可以下发初始定位'
    return
  }
  waypoints.value.push(imagePointToWaypoint(imagePoint, geometry))
  waypointNames.value.push(`点${waypoints.value.length}`)
}

function refreshImageGeometry() {
  imageReadyTick.value += 1
}

function removeWaypoint(index) {
  waypoints.value.splice(index, 1)
  waypointNames.value.splice(index, 1)
}

function clearWaypoints() {
  waypoints.value = []
  waypointNames.value = []
}

async function handleSaveRoute() {
  if (!selectedMap.value || waypoints.value.length === 0) {
    alert('请选择地图并添加途经点')
    return
  }

  const payload = {
    name: routeForm.value.name || `路线-${new Date().toLocaleString()}`,
    map_data: selectedMap.value.id,
    map_set: routeForm.value.map_set || null,
    robot: routeForm.value.robot || selectedMap.value.robot || 1,
    waypoints: withWaypointYaw(waypoints.value),
    waypoint_names: waypointNames.value,
    description: routeForm.value.description,
  }

  try {
    if (selectedRoute.value?.id) {
      await updateRoute(selectedRoute.value.id, payload)
    } else {
      await createRoute(payload)
    }
    await loadData()
    selectedRoute.value = null
    routeForm.value.name = ''
    routeForm.value.description = ''
    alert('保存成功')
  } catch (error) {
    console.error('保存失败:', error)
    alert('保存失败')
  }
}

async function handleLoadRoute(route) {
  selectedRoute.value = route
  selectedMap.value = maps.value.find(m => String(m.id) === String(route.map_data)) || selectedMap.value
  waypoints.value = (route.waypoints || []).map(point => ({ ...point }))
  waypointNames.value = route.waypoint_names?.length
    ? [...route.waypoint_names]
    : waypoints.value.map((_, index) => `点${index + 1}`)
  routeForm.value.name = route.name
  routeForm.value.description = route.description
  routeForm.value.robot = route.robot
  routeForm.value.map_data = route.map_data
  routeForm.value.map_set = route.map_set || null
  await nextTick()
  refreshImageGeometry()
  refreshNavigationStatus()
}

function normalizeStoredWaypoint(point, map = selectedMap.value) {
  const geometry = getMapGeometry(map)
  if (Array.isArray(point)) {
    const first = Number(point[0])
    const second = Number(point[1])
    const yaw = Number(point[2] || 0)
    if (geometry && looksLikeLegacyImagePoint(first, second, geometry)) {
      return imagePointToWaypoint({ imageX: first, imageY: second }, geometry, yaw)
    }
    return imagePointToWaypoint(mapPointToImagePoint(first, second, geometry), geometry, yaw)
  }

  const normalized = {
    x: Number(point.x),
    y: Number(point.y),
    yaw: Number(point.yaw || 0),
    frame_id: point.frame_id || 'map',
  }
  if (Number.isFinite(Number(point.image_x)) && Number.isFinite(Number(point.image_y)) && geometry) {
    return imagePointToWaypoint(
      { imageX: Number(point.image_x), imageY: Number(point.image_y) },
      geometry,
      normalized.yaw,
    )
  }
  if (Number.isFinite(Number(point.u)) && Number.isFinite(Number(point.v)) && geometry) {
    return imagePointToWaypoint(
      { imageX: Number(point.u) * geometry.mapWidth, imageY: Number(point.v) * geometry.mapHeight },
      geometry,
      normalized.yaw,
    )
  }
  return {
    ...normalized,
    ...mapPointToImagePoint(normalized.x, normalized.y, geometry),
  }
}

function withWaypointYaw(points) {
  return points.map((point, index) => {
    const current = normalizeStoredWaypoint(point)
    const yaw = Number(current.yaw || 0)
    return {
      frame_id: 'map',
      x: Number(current.x.toFixed(4)),
      y: Number(current.y.toFixed(4)),
      yaw: Number(yaw.toFixed(4)),
      image_x: Number(current.image_x.toFixed(4)),
      image_y: Number(current.image_y.toFixed(4)),
      u: Number(current.u.toFixed(8)),
      v: Number(current.v.toFixed(8)),
    }
  })
}

function waypointDisplayText(point) {
  const normalized = normalizeStoredWaypoint(point)
  return `(${Number(normalized.x).toFixed(2)}, ${Number(normalized.y).toFixed(2)})`
}

function waypointDisplayPosition(point) {
  imageReadyTick.value
  const normalized = normalizeStoredWaypoint(point)
  const geometry = getMapGeometry()
  if (!geometry) return { left: '0px', top: '0px' }
  const imageX = Number.isFinite(normalized.image_x) ? normalized.image_x : mapPointToImagePoint(normalized.x, normalized.y, geometry).imageX
  const imageY = Number.isFinite(normalized.image_y) ? normalized.image_y : mapPointToImagePoint(normalized.x, normalized.y, geometry).imageY
  return {
    left: `${imageX * (geometry.rect.width / geometry.mapWidth)}px`,
    top: `${imageY * (geometry.rect.height / geometry.mapHeight)}px`,
  }
}

function pathPolylinePoints() {
  return waypoints.value
    .map(point => {
      const pos = waypointDisplayPosition(point)
      return `${Number.parseFloat(pos.left)},${Number.parseFloat(pos.top)}`
    })
    .join(' ')
}

function pointDisplayPositionFromMap(x, y, geometry = getMapGeometry()) {
  if (!geometry) return null
  const point = mapPointToImagePoint(Number(x), Number(y), geometry)
  return {
    x: point.imageX * (geometry.rect.width / geometry.mapWidth),
    y: point.imageY * (geometry.rect.height / geometry.mapHeight),
  }
}

function poseTrailPoints() {
  imageReadyTick.value
  const geometry = getMapGeometry()
  if (!geometry) return ''
  return poseHistory.value
    .map(point => pointDisplayPositionFromMap(point.x, point.y, geometry))
    .filter(Boolean)
    .map(point => `${point.x},${point.y}`)
    .join(' ')
}

function initialPoseHeadingLinePoints() {
  imageReadyTick.value
  const geometry = getMapGeometry()
  if (!geometry || !manualInitialPose.value || !initialPoseHeadingTarget.value) return ''
  const start = pointDisplayPositionFromMap(manualInitialPose.value.x, manualInitialPose.value.y, geometry)
  const end = pointDisplayPositionFromMap(initialPoseHeadingTarget.value.x, initialPoseHeadingTarget.value.y, geometry)
  if (!start || !end) return ''
  return `${start.x},${start.y} ${end.x},${end.y}`
}

function clearPoseHistory() {
  poseHistory.value = []
  lastPoseSampleKey.value = ''
}

function recordPoseSample() {
  const status = navStatus.value?.status
  if (!status || !robotMapMatches()) return
  const x = Number(status.x)
  const y = Number(status.y)
  if (!Number.isFinite(x) || !Number.isFinite(y)) return

  const sampleKey = status.sampled_at || status.received_at || `${x.toFixed(4)},${y.toFixed(4)},${status.yaw || 0}`
  if (sampleKey === lastPoseSampleKey.value) return

  const lastPoint = poseHistory.value[poseHistory.value.length - 1]
  if (lastPoint) {
    const distance = Math.hypot(x - lastPoint.x, y - lastPoint.y)
    if (distance < 0.005 && status.sampled_at === lastPoint.sampled_at) return
  }

  lastPoseSampleKey.value = sampleKey
  poseHistory.value = [
    ...poseHistory.value,
    {
      x,
      y,
      yaw: Number(status.yaw || 0),
      speed_mps: Number(status.speed_mps || 0),
      sampled_at: status.sampled_at || status.received_at || new Date().toISOString(),
      map_id: status.map_id || navStatus.value?.current_map_id || null,
    },
  ].slice(-120)
}

async function refreshNavigationStatus() {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot || 1
  if (!robotId) return
  try {
    const [status, navigation] = await Promise.all([
      fetchRobotStatus(robotId),
      fetchRobotNavigationStatus(robotId),
    ])
    navStatus.value = {
      ...navigation,
      status: status.status || navigation.status,
      connection_status: status.connection_status || navigation.connection_status,
      last_seen_at: status.last_seen_at || navigation.last_seen_at,
    }
    recordPoseSample()
    navError.value = ''
    refreshImageGeometry()
  } catch (error) {
    navError.value = error.message || '导航状态获取失败'
  }
}

async function sendNavigationCommand(action) {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot
  if (!robotId) {
    navError.value = '未选择机器人'
    return
  }
  navCommandBusy.value = action
  navError.value = ''
  try {
    await sendRobotNavigationCommand(robotId, action, {
      map_id: selectedMap.value?.id,
      map_version: selectedMap.value?.description || '',
    })
    await refreshNavigationStatus()
  } catch (error) {
    navError.value = error.message || '导航命令下发失败'
  } finally {
    navCommandBusy.value = ''
  }
}

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms))
}

async function initializeLocalization() {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot
  if (!robotId) {
    navError.value = '未选择机器人'
    return
  }
  if (navStatus.value?.connection_status !== 'online') {
    navError.value = '机器人未在线'
    return
  }

  navCommandBusy.value = 'localization-init'
  localizationInitState.value = 'restarting'
  localizationInitMessage.value = '正在重启导航/定位栈'
  navError.value = ''
  try {
    await sendRobotNavigationCommand(robotId, 'restart', {
      map_id: selectedMap.value?.id,
      map_version: selectedMap.value?.description || '',
    })
    await sleep(2500)
    await refreshNavigationStatus()

    if (!manualInitialPose.value) {
      initialPoseMode.value = true
      initialPoseStep.value = 'position'
      localizationInitState.value = 'waiting_pose'
      localizationInitMessage.value = '请在地图点击机器狗真实位置，再点击狗头朝向'
      navError.value = localizationInitMessage.value
      return
    }

    localizationInitState.value = 'sending_pose'
    localizationInitMessage.value = '正在下发初始定位'
    await publishInitialPose(false, false)
    localizationInitState.value = 'waiting_convergence'
    localizationInitMessage.value = '等待定位收敛和 NDT 质量更新'
    for (let index = 0; index < 6; index += 1) {
      await sleep(2000)
      await refreshNavigationStatus()
      if (!localizationSampleStale() && localizationLabel() === 'normal' && !localizationQualityStale()) {
        localizationInitState.value = 'done'
        localizationInitMessage.value = '定位初始化完成'
        return
      }
    }
    localizationInitState.value = 'failed'
    localizationInitMessage.value = '未看到新的定位/NDT上报，请检查定位节点是否启动'
    navError.value = localizationInitMessage.value
  } catch (error) {
    localizationInitState.value = 'failed'
    localizationInitMessage.value = error.message || '定位初始化失败'
    navError.value = localizationInitMessage.value
  } finally {
    navCommandBusy.value = ''
  }
}

async function handleExecuteRoute() {
  if (!selectedRoute.value?.id) {
    navError.value = '请先保存并选择一条路线'
    return
  }
  if (navStatus.value?.connection_status !== 'online') {
    navError.value = '机器人未在线'
    return
  }
  if (!navStatus.value?.status?.nav_ready) {
    navError.value = '导航栈未就绪，请先启动导航栈'
    return
  }
  if (!confirm(`确定执行路线 "${selectedRoute.value.name}" 吗？请确认现场路径安全。`)) return
  routeExecuteBusy.value = true
  navError.value = ''
  try {
    lastExecution.value = await executeRoute(selectedRoute.value.id)
    await refreshNavigationStatus()
  } catch (error) {
    navError.value = error.message || '路线执行失败'
  } finally {
    routeExecuteBusy.value = false
  }
}

function toggleInitialPoseMode() {
  initialPoseMode.value = !initialPoseMode.value
  if (initialPoseMode.value) {
    initialPoseStep.value = manualInitialPose.value ? 'heading' : 'position'
    navError.value = manualInitialPose.value ? '请点击狗头朝向，或重新选择位置' : '请点击机器狗真实位置'
  } else {
    navError.value = ''
  }
}

function resetInitialPosePosition() {
  manualInitialPose.value = null
  initialPoseHeadingTarget.value = null
  initialPoseStep.value = 'position'
  initialPoseMode.value = true
  navError.value = '请点击机器狗真实位置'
}

function adjustInitialPoseYaw(delta) {
  if (!manualInitialPose.value) return
  manualInitialPose.value = {
    ...manualInitialPose.value,
    yaw: Number((Number(manualInitialPose.value.yaw || 0) + delta).toFixed(4)),
  }
}

async function sendInitialPose() {
  await publishInitialPose(true, true)
}

async function publishInitialPose(confirmRequired = true, manageBusy = true) {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot
  if (!robotId) {
    navError.value = '未选择机器人'
    return
  }
  if (!manualInitialPose.value) {
    navError.value = '请先在地图上点击机器狗真实位置'
    return
  }
  if (confirmRequired && !confirm(`确认把初始定位设置为 ${waypointDisplayText(manualInitialPose.value)} / yaw ${Number(manualInitialPose.value.yaw || 0).toFixed(2)}？`)) return
  if (manageBusy) navCommandBusy.value = 'initial-pose'
  navError.value = ''
  try {
    await sendRobotNavigationCommand(robotId, 'initial-pose', {
      frame_id: 'map',
      x: Number(manualInitialPose.value.x),
      y: Number(manualInitialPose.value.y),
      yaw: Number(manualInitialPose.value.yaw || 0),
      map_id: selectedMap.value?.id,
      map_version: selectedMap.value?.description || '',
    })
    initialPoseMode.value = false
    await refreshNavigationStatus()
  } catch (error) {
    navError.value = error.message || '初始定位下发失败'
  } finally {
    if (manageBusy && navCommandBusy.value === 'initial-pose') navCommandBusy.value = ''
  }
}

function robotMapMatches() {
  const mapId = navStatus.value?.status?.map_id || navStatus.value?.current_map_id
  if (!mapId || !selectedMap.value?.id) return true
  return String(mapId) === String(selectedMap.value.id)
}

function robotDisplayPosition() {
  imageReadyTick.value
  const status = navStatus.value?.status
  const geometry = getMapGeometry()
  if (!robotPoseUsable() || !status || !geometry || status.x === null || status.y === null || !robotMapMatches()) return null
  const point = mapPointToImagePoint(Number(status.x), Number(status.y), geometry)
  return {
    left: `${point.imageX * (geometry.rect.width / geometry.mapWidth)}px`,
    top: `${point.imageY * (geometry.rect.height / geometry.mapHeight)}px`,
  }
}

function robotHeadingStyle() {
  const yaw = Number(navStatus.value?.status?.yaw || 0)
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - yaw}rad)` }
}

function initialPoseHeadingStyle() {
  const yaw = Number(manualInitialPose.value?.yaw || 0)
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - yaw}rad)` }
}

function navReadyLabel() {
  const status = navStatus.value?.status
  if (!navStatus.value) return '未知'
  return status?.nav_ready ? 'Nav2 ready' : 'Nav2 not ready'
}

function localizationLabel() {
  return navStatus.value?.status?.localization_status || navStatus.value?.localization_status || 'unknown'
}

function robotPoseText() {
  const status = navStatus.value?.status
  if (!status || status.x === null || status.y === null) return '暂无定位'
  return `x ${Number(status.x).toFixed(2)} / y ${Number(status.y).toFixed(2)} / yaw ${Number(status.yaw || 0).toFixed(2)}`
}

function formatNumber(value, digits = 3) {
  if (value === null || value === undefined || value === '') return '—'
  const number = Number(value)
  if (!Number.isFinite(number)) return String(value)
  return number.toFixed(digits)
}

function formatDateTime(value) {
  if (!value) return '—'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return String(value)
  return date.toLocaleTimeString()
}

function formatDateTimeWithAge(value) {
  if (!value) return '—'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return String(value)
  const ageSeconds = Math.max(0, Math.floor((Date.now() - date.getTime()) / 1000))
  if (ageSeconds < 60) return `${date.toLocaleTimeString()} / ${ageSeconds}s前`
  return `${date.toLocaleTimeString()} / ${Math.floor(ageSeconds / 60)}分钟前`
}

function sampleIsStale(value, thresholdMs = 10000) {
  if (!value) return false
  const sampledTime = new Date(value).getTime()
  if (Number.isNaN(sampledTime)) return false
  return Date.now() - sampledTime > thresholdMs
}

function localizationSampleStale() {
  return sampleIsStale(navStatus.value?.status?.sampled_at)
}

function localizationQualityStale(quality = localizationQuality()) {
  if (!quality?.sampled_at) return localizationSampleStale()
  return sampleIsStale(quality.sampled_at)
}

function localizationRefreshLabel() {
  if (!navStatus.value) return '等待状态'
  return localizationSampleStale() ? '2s 拉取平台状态 / 定位未更新' : '2s 拉取平台状态'
}

function mapMatchLabel() {
  if (!selectedMap.value) return '未选择页面地图'
  if (robotMapMatches()) return '一致'
  return '不一致'
}

function commandText(command = navStatus.value?.command) {
  if (!command) return '—'
  const finished = command.finished_at ? ` / ${formatDateTime(command.finished_at)}` : ''
  const error = command.error_code ? ` / ${command.error_code}` : ''
  return `${command.command_type} · ${command.status}${error}${finished}`
}

function localizationQuality() {
  return navStatus.value?.status?.localization_quality || null
}

function ndtQualityLabel(quality = localizationQuality()) {
  if (!quality) return '无数据'
  if (localizationQualityStale(quality)) return '已过期'
  if (!ndtQualityValid(quality)) return '无效'
  const error = Number(quality.matching_error)
  if (!Number.isFinite(error)) return '无分数'
  if (error <= 0.5 && quality.has_converged !== false) return '正常'
  return '偏差大'
}

function ndtQualityValid(quality = localizationQuality()) {
  if (!quality) return false
  const error = Number(quality.matching_error)
  const inlier = Number(quality.inlier_fraction)
  const translation = Number(quality.relative_translation_m)
  return Number.isFinite(error)
    && error < 100
    && (!Number.isFinite(inlier) || inlier >= 0.05)
    && (!Number.isFinite(translation) || translation < 20)
}

function ndtConvergedText(quality = localizationQuality()) {
  if (!quality) return '—'
  if (localizationQualityStale(quality)) return '已过期'
  return quality.has_converged && ndtQualityValid(quality) ? '是' : '否'
}

function robotPoseUsable() {
  const status = navStatus.value?.status
  if (!status) return false
  const localizationStatus = status.localization_status || navStatus.value?.localization_status
  return localizationStatus === 'normal'
    && !localizationSampleStale()
    && robotMapMatches()
    && Boolean(localizationQuality())
    && !localizationQualityStale()
    && ndtQualityValid()
}

function predictionErrorText(quality = localizationQuality()) {
  const errors = Array.isArray(quality?.prediction_errors) ? quality.prediction_errors : []
  if (!errors.length) return '—'
  return errors
    .map(item => `${item.label || 'pred'}:${formatNumber(item.translation_m, 2)}m`)
    .join(' / ')
}

function localizationDebugRows() {
  const status = navStatus.value?.status || {}
  const command = navStatus.value?.command
  const quality = localizationQuality()
  const qualityFresh = quality && !localizationQualityStale(quality)
  return [
    ['页面地图', selectedMap.value ? `${selectedMap.value.id} / ${selectedMap.value.name}` : '—'],
    ['机器人地图', `${status.map_id || navStatus.value?.current_map_id || '—'} / ${status.map_version || navStatus.value?.current_map_version || '—'}`],
    ['地图一致', mapMatchLabel()],
    ['定位状态', status.localization_status || navStatus.value?.localization_status || 'unknown'],
    ['定位源状态', status.localization_source_status ?? '—'],
    ['NDT质量', ndtQualityLabel(quality)],
    ['NDT分数', qualityFresh ? formatNumber(quality.matching_error, 3) : (quality ? `已过期 ${formatNumber(quality.matching_error, 3)}` : '—')],
    ['NDT收敛', qualityFresh ? ndtConvergedText(quality) : (quality ? '已过期' : '—')],
    ['内点率', qualityFresh ? formatNumber(quality.inlier_fraction, 3) : (quality ? `已过期 ${formatNumber(quality.inlier_fraction, 3)}` : '—')],
    ['匹配位移', qualityFresh ? `${formatNumber(quality.relative_translation_m, 3)} m` : '—'],
    ['预测误差', qualityFresh ? predictionErrorText(quality) : '—'],
    ['质量时间', quality ? formatDateTimeWithAge(quality.sampled_at) : '—'],
    ['初始化状态', localizationInitMessage.value || localizationInitState.value],
    ['导航栈', status.nav_ready ? 'ready' : 'not ready'],
    ['ROS', status.ros_ready ? 'ready' : 'not ready'],
    ['连接', navStatus.value?.connection_status || 'unknown'],
    ['x / y', `${formatNumber(status.x)} / ${formatNumber(status.y)}`],
    ['z / yaw', `${formatNumber(status.z)} / ${formatNumber(status.yaw)}`],
    ['速度', `${formatNumber(status.speed_mps)} m/s`],
    ['定位尾迹', `${poseHistory.value.length} 点`],
    ['采样时间', formatDateTimeWithAge(status.sampled_at)],
    ['接收时间', formatDateTimeWithAge(status.received_at)],
    ['最近命令', commandText(command)],
  ]
}

function stateTone(state) {
  if (['ok', 'active', 'normal', 'ready'].includes(state)) return 'ok'
  if (['warn', 'initializing', 'unknown', 'stale'].includes(state)) return 'warn'
  if (['bad', 'lost', 'offline', 'blocked'].includes(state)) return 'bad'
  return 'idle'
}

function stateMachineSteps() {
  const status = navStatus.value?.status || {}
  const quality = localizationQuality()
  const mapMatches = robotMapMatches()
  const localizationStatus = status.localization_status || navStatus.value?.localization_status || 'unknown'
  const connection = navStatus.value?.connection_status || 'unknown'
  const command = navStatus.value?.command
  const taskId = status.task_execution_id
  const ndtError = Number(quality?.matching_error)
  const inlier = Number(quality?.inlier_fraction)
  const qualityStale = localizationQualityStale(quality)
  const goodNdt = !qualityStale && ndtQualityValid(quality) && Number.isFinite(ndtError) && ndtError <= 0.5 && (!Number.isFinite(inlier) || inlier >= 0.65)

  return [
    {
      key: 'connection',
      title: '平台连接',
      value: connection,
      detail: `状态版本 ${status.state_version ?? '—'} · ${formatDateTimeWithAge(status.received_at)}`,
      state: connection === 'online' ? 'ok' : 'bad',
    },
    {
      key: 'map',
      title: '地图一致',
      value: mapMatches ? '一致' : '不一致',
      detail: `页面 ${selectedMap.value?.id || '—'} / 机器人 ${status.map_id || navStatus.value?.current_map_id || '—'}`,
      state: mapMatches ? 'ok' : 'bad',
    },
    {
      key: 'localization',
      title: '定位状态',
      value: localizationStatus,
      detail: `源 ${status.localization_source_status ?? '—'} · ${robotPoseText()}`,
      state: localizationStatus === 'normal' ? 'ok' : localizationStatus === 'lost' ? 'bad' : 'warn',
    },
    {
      key: 'localization-init',
      title: '定位初始化',
      value: localizationInitState.value,
      detail: localizationInitMessage.value || '可点击初始化定位重启栈并下发初始位',
      state: localizationInitState.value === 'done' ? 'ok' : localizationInitState.value === 'failed' ? 'bad' : localizationInitState.value === 'idle' ? 'idle' : 'warn',
    },
    {
      key: 'ndt',
      title: 'NDT匹配',
      value: ndtQualityLabel(quality),
      detail: quality ? `score ${formatNumber(quality.matching_error, 3)} · inlier ${formatNumber(quality.inlier_fraction, 3)} · ${formatDateTimeWithAge(quality.sampled_at)}` : '未收到 /status 质量上报',
      state: goodNdt ? 'ok' : qualityStale ? 'warn' : quality ? 'bad' : 'warn',
    },
    {
      key: 'ros',
      title: 'ROS运行',
      value: status.ros_ready ? 'ready' : 'not ready',
      detail: `采样 ${formatDateTimeWithAge(status.sampled_at)}${localizationSampleStale() ? ' · 未持续更新' : ''}`,
      state: status.ros_ready && !localizationSampleStale() ? 'ok' : 'warn',
    },
    {
      key: 'nav',
      title: '导航栈',
      value: status.nav_ready ? 'ready' : 'not ready',
      detail: commandText(command),
      state: status.nav_ready ? 'ok' : 'warn',
    },
    {
      key: 'task',
      title: '任务执行',
      value: taskId ? '执行中/上报中' : lastExecution.value ? lastExecution.value.state : '空闲',
      detail: taskId || lastExecution.value?.id || '暂无任务上下文',
      state: taskId ? 'active' : 'idle',
    },
  ]
}

function sensorStateRows() {
  const status = navStatus.value?.status || {}
  const quality = localizationQuality()
  const localizationStatus = status.localization_status || navStatus.value?.localization_status || 'unknown'
  const qualityFresh = quality?.sampled_at && !localizationQualityStale(quality)
  const hasPose = status.x !== null && status.x !== undefined && status.y !== null && status.y !== undefined
  const navReady = Boolean(status.nav_ready)

  return [
    {
      name: '激光雷达 /laser_scan',
      value: quality ? '参与定位/避障' : '未见质量上报',
      detail: qualityFresh ? `NDT ${formatNumber(quality.matching_error, 3)} · ${formatDateTimeWithAge(quality.sampled_at)}` : '前端依赖定位质量侧证，未直接订阅 ROS',
      state: qualityFresh && localizationStatus !== 'lost' ? 'ok' : quality ? 'warn' : 'unknown',
    },
    {
      name: 'IMU /front_lidar/imu',
      value: quality ? '参与定位融合' : '未上报',
      detail: '当前接口未拆分 IMU 频率，只能通过定位质量间接判断',
      state: quality ? 'ok' : 'unknown',
    },
    {
      name: '里程计 /odom/mc_odom',
      value: hasPose ? '有位姿输出' : '无位姿',
      detail: `速度 ${formatNumber(status.speed_mps)} m/s · 控制模式 ${status.control_mode || '—'}`,
      state: hasPose ? 'ok' : 'warn',
    },
    {
      name: 'RTK/GNSS',
      value: '未接入当前室内导航状态',
      detail: '当前状态接口没有 RTK fix/卫星数/差分状态字段',
      state: 'idle',
    },
    {
      name: '视觉',
      value: '未上报',
      detail: '当前导航链路未见视觉定位/避障状态字段',
      state: 'idle',
    },
    {
      name: 'Local costmap',
      value: navReady ? '随导航栈运行' : '导航栈未就绪',
      detail: '障碍源应来自 /laser_scan，前端暂未收到 costmap 统计字段',
      state: navReady ? 'ok' : 'warn',
    },
    {
      name: 'Collision monitor',
      value: navReady ? '应随导航栈运行' : '导航栈未就绪',
      detail: '用于 cmd_vel_raw -> cmd_vel 的实时减速/刹停链路',
      state: navReady ? 'ok' : 'warn',
    },
  ]
}

function statusBadgeClass(state) {
  return `state-${stateTone(state)}`
}

function getMapGeometry(map = selectedMap.value) {
  if (!map) return null
  const image = mapImageRef.value
  const naturalWidth = image?.naturalWidth || Number(map.width) || 0
  const naturalHeight = image?.naturalHeight || Number(map.height) || 0
  const mapWidth = Number(map.width) || naturalWidth
  const mapHeight = Number(map.height) || naturalHeight
  const resolution = Number(map.resolution || 0.05)
  const origin = Array.isArray(map.origin) && map.origin.length >= 2
    ? [Number(map.origin[0]), Number(map.origin[1]), Number(map.origin[2] || 0)]
    : [0, 0, 0]
  const rect = image?.getBoundingClientRect() || { width: naturalWidth, height: naturalHeight, left: 0, top: 0 }
  if (!mapWidth || !mapHeight || !resolution || !rect.width || !rect.height) return null
  return { mapWidth, mapHeight, naturalWidth, naturalHeight, resolution, origin, rect }
}

function displayToImagePoint(displayX, displayY, geometry) {
  return {
    imageX: clamp(displayX * (geometry.mapWidth / geometry.rect.width), 0, geometry.mapWidth),
    imageY: clamp(displayY * (geometry.mapHeight / geometry.rect.height), 0, geometry.mapHeight),
  }
}

function imagePointToWaypoint({ imageX, imageY }, geometry, yaw = 0) {
  const x = geometry.origin[0] + imageX * geometry.resolution
  const y = geometry.origin[1] + (geometry.mapHeight - imageY) * geometry.resolution
  return {
    frame_id: 'map',
    x: Number(x.toFixed(4)),
    y: Number(y.toFixed(4)),
    yaw: Number(yaw || 0),
    image_x: Number(imageX.toFixed(4)),
    image_y: Number(imageY.toFixed(4)),
    u: Number((imageX / geometry.mapWidth).toFixed(8)),
    v: Number((imageY / geometry.mapHeight).toFixed(8)),
  }
}

function mapPointToImagePoint(x, y, geometry) {
  if (!geometry) return { image_x: 0, image_y: 0, imageX: 0, imageY: 0, u: 0, v: 0 }
  const imageX = (Number(x) - geometry.origin[0]) / geometry.resolution
  const imageY = geometry.mapHeight - ((Number(y) - geometry.origin[1]) / geometry.resolution)
  return {
    image_x: Number(imageX.toFixed(4)),
    image_y: Number(imageY.toFixed(4)),
    imageX,
    imageY,
    u: Number((imageX / geometry.mapWidth).toFixed(8)),
    v: Number((imageY / geometry.mapHeight).toFixed(8)),
  }
}

function looksLikeLegacyImagePoint(x, y, geometry) {
  return x >= 0 && y >= 0 && x <= geometry.mapWidth * 8 && y <= geometry.mapHeight * 8
    && (x > geometry.origin[0] + geometry.mapWidth * geometry.resolution || y > geometry.origin[1] + geometry.mapHeight * geometry.resolution)
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value))
}

async function handleDeleteRoute(route) {
  if (!confirm(`确定要删除路线 "${route.name}" 吗？`)) return
  try {
    await deleteRoute(route.id)
    await loadData()
    if (selectedRoute.value?.id === route.id) {
      clearWaypoints()
      selectedRoute.value = null
    }
  } catch (error) {
    console.error('删除失败:', error)
    alert('删除失败')
  }
}
</script>

<template>
  <section class="page-section">
    <section class="panel detail-panel">
      <div class="panel-header">
        <h2>路径规划</h2>
        <button class="btn btn-primary" @click="handleSaveRoute" :disabled="!selectedMap || waypoints.length === 0">
          保存路线
        </button>
      </div>

      <div class="route-planner-layout">
        <!-- 左侧面板 -->
        <div class="side-panel">
          <div class="panel-section">
            <h3>1. 选择地图</h3>
            <select v-model="selectedMap" @change="handleMapSelect(selectedMap)">
              <option :value="null">请选择地图</option>
              <option v-for="map in maps" :key="map.id" :value="map">
                {{ map.name }} {{ map.active ? '(活动)' : '' }}
              </option>
            </select>
          </div>

          <div class="panel-section">
            <h3>2. 路线信息</h3>
            <div class="form-group" v-if="mapSets.length">
              <label>跨图地图集</label>
              <select v-model="routeForm.map_set">
                <option :value="null">单地图路线</option>
                <option v-for="mapSet in mapSets" :key="mapSet.id" :value="mapSet.id">
                  {{ mapSet.name }} ({{ mapSet.members.length }} 子图)
                </option>
              </select>
            </div>
            <div class="form-group">
              <label>路线名称</label>
              <input v-model="routeForm.name" type="text" placeholder="输入路线名称" />
            </div>
            <div class="form-group">
              <label>描述</label>
              <textarea v-model="routeForm.description" rows="2" placeholder="输入路线描述"></textarea>
            </div>
          </div>

          <div class="panel-section">
            <h3>3. 途经点列表</h3>
            <div v-if="waypoints.length === 0" class="empty-hint">点击地图添加途经点</div>
            <div v-else class="waypoint-list">
              <div v-for="(point, index) in waypoints" :key="index" class="waypoint-item">
                <span>{{ waypointNames[index] }}: {{ waypointDisplayText(point) }}</span>
                <button class="btn-close" @click="removeWaypoint(index)">×</button>
              </div>
            </div>
            <div class="waypoint-actions">
              <button class="btn btn-sm" @click="clearWaypoints" :disabled="waypoints.length === 0">清空</button>
            </div>
          </div>

          <div class="panel-section">
            <h3>4. 已保存路线</h3>
            <div v-if="routes.length === 0" class="empty-hint">暂无保存的路线</div>
            <div v-else class="route-list">
              <div v-for="route in routes" :key="route.id" class="route-item" :class="{ active: selectedRoute?.id === route.id }">
                <div @click="handleLoadRoute(route)">
                  <strong>{{ route.name }}</strong>
                  <small>{{ route.waypoints.length }} 个途经点</small>
                </div>
                <button class="btn btn-sm btn-danger" @click="handleDeleteRoute(route)">删除</button>
              </div>
            </div>
          </div>

          <div class="panel-section test-panel">
            <h3>5. 导航测试</h3>
            <div class="status-grid">
              <div>
                <span>机器人</span>
                <strong>{{ selectedRobot?.name || '未选择' }}</strong>
              </div>
              <div>
                <span>活动地图</span>
                <strong>{{ selectedMap?.name || '未选择' }}</strong>
              </div>
              <div>
                <span>连接</span>
                <strong>{{ navStatus?.connection_status || 'unknown' }}</strong>
              </div>
              <div>
                <span>定位</span>
                <strong>{{ localizationLabel() }}</strong>
              </div>
              <div>
                <span>导航栈</span>
                <strong>{{ navReadyLabel() }}</strong>
              </div>
              <div>
                <span>当前位置</span>
                <strong>{{ robotPoseText() }}</strong>
              </div>
            </div>
            <p v-if="!robotMapMatches()" class="form-error">当前机器人上报地图与页面地图不一致，暂不显示位置。</p>
            <p v-if="localizationSampleStale()" class="form-error">定位数据未持续更新，请检查导航/定位栈是否启动。</p>
            <p v-if="navError" class="form-error">{{ navError }}</p>
            <div class="nav-actions">
              <button class="btn btn-sm" :disabled="!!navCommandBusy" @click="refreshNavigationStatus">刷新状态</button>
              <button class="btn btn-sm btn-primary" :disabled="!!navCommandBusy || navStatus?.connection_status !== 'online'" @click="sendNavigationCommand('start')">启动导航栈</button>
              <button class="btn btn-sm" :disabled="!!navCommandBusy || navStatus?.connection_status !== 'online'" @click="sendNavigationCommand('restart')">重启</button>
              <button class="btn btn-sm" :disabled="!!navCommandBusy || navStatus?.connection_status !== 'online'" @click="sendNavigationCommand('recover')">恢复</button>
              <button class="btn btn-sm btn-danger" :disabled="!!navCommandBusy || navStatus?.connection_status !== 'online'" @click="sendNavigationCommand('stop')">停止</button>
              <button class="btn btn-sm" :class="{ 'btn-primary': initialPoseMode }" :disabled="!!navCommandBusy || navStatus?.connection_status !== 'online'" @click="toggleInitialPoseMode">
                {{ initialPoseMode ? '正在选初始位' : '设初始定位' }}
              </button>
              <button class="btn btn-sm btn-primary" :disabled="!!navCommandBusy || navStatus?.connection_status !== 'online'" @click="initializeLocalization">
                {{ navCommandBusy === 'localization-init' ? '初始化中...' : '初始化定位' }}
              </button>
              <button class="btn btn-sm btn-primary" :disabled="routeExecuteBusy || !selectedRoute?.id || navStatus?.connection_status !== 'online' || !navStatus?.status?.nav_ready" @click="handleExecuteRoute">
                {{ routeExecuteBusy ? '执行中...' : '执行当前路线' }}
              </button>
              <button class="btn btn-sm" :class="{ 'btn-primary': showPoseTrail }" @click="showPoseTrail = !showPoseTrail">
                {{ showPoseTrail ? '隐藏尾迹' : '显示尾迹' }}
              </button>
              <button class="btn btn-sm" :disabled="poseHistory.length === 0" @click="clearPoseHistory">清空尾迹</button>
            </div>
            <div v-if="initialPoseMode || manualInitialPose" class="initial-pose-panel">
              <div class="initial-pose-guide">
                <span>初始定位</span>
                <strong>{{ initialPoseStep === 'position' ? '第1步：点击机器狗位置' : '第2步：点击狗头朝向' }}</strong>
                <small>分数越低越好，NDT 大于阈值时不会完成初始化。</small>
              </div>
              <div>
                <span>初始位</span>
                <strong>{{ manualInitialPose ? waypointDisplayText(manualInitialPose) : '点击地图选择' }}</strong>
              </div>
              <label>
                朝向
                <input
                  v-if="manualInitialPose"
                  v-model.number="manualInitialPose.yaw"
                  type="number"
                  step="0.1"
                />
                <input v-else type="number" step="0.1" disabled />
              </label>
              <div v-if="manualInitialPose" class="yaw-actions">
                <button class="btn btn-sm" @click="adjustInitialPoseYaw(Math.PI / 12)">左转15°</button>
                <button class="btn btn-sm" @click="adjustInitialPoseYaw(-Math.PI / 12)">右转15°</button>
                <button class="btn btn-sm" @click="resetInitialPosePosition">重选位置</button>
                <span>{{ Math.round(Number(manualInitialPose.yaw || 0) * 180 / Math.PI) }}°</span>
              </div>
              <button class="btn btn-sm btn-primary" :disabled="!manualInitialPose || !!navCommandBusy" @click="sendInitialPose">下发初始定位</button>
            </div>
            <small v-if="navStatus?.command" class="command-note">
              最近命令 {{ navStatus.command.command_type }} · {{ navStatus.command.status }}
            </small>
            <small v-if="lastExecution" class="command-note">
              最近执行 {{ lastExecution.id }} · {{ lastExecution.state }}
            </small>
            <small v-if="localizationInitMessage" class="command-note">
              定位初始化 {{ localizationInitState }} · {{ localizationInitMessage }}
            </small>
            <div class="state-machine-panel">
              <div class="debug-header">
                <strong>导航状态机</strong>
                <span>{{ localizationRefreshLabel() }}</span>
              </div>
              <div class="state-machine-grid">
                <div v-for="step in stateMachineSteps()" :key="step.key" class="state-card" :class="statusBadgeClass(step.state)">
                  <span>{{ step.title }}</span>
                  <strong>{{ step.value }}</strong>
                  <small>{{ step.detail }}</small>
                </div>
              </div>
              <div class="sensor-state-list">
                <div class="sensor-state-head">
                  <strong>传感器与避障链路</strong>
                  <span>上报 / 推断 / 未上报</span>
                </div>
                <div v-for="row in sensorStateRows()" :key="row.name" class="sensor-state-row">
                  <span>{{ row.name }}</span>
                  <strong :class="statusBadgeClass(row.state)">{{ row.value }}</strong>
                  <small>{{ row.detail }}</small>
                </div>
              </div>
            </div>

            <div class="localization-debug-panel">
              <div class="debug-header">
                <strong>原始状态详情</strong>
                <span>平台最近一次上报</span>
              </div>
              <div class="debug-grid">
                <div v-for="[label, value] in localizationDebugRows()" :key="label" class="debug-row">
                  <span>{{ label }}</span>
                  <strong :class="{ danger: (label === '地图一致' && value === '不一致') || (label === '采样时间' && localizationSampleStale()) || (label === '质量时间' && localizationQualityStale()) || (label === 'NDT质量' && ['偏差大', '已过期', '无效'].includes(value)) || (label === 'NDT收敛' && ['否', '已过期'].includes(value)) || (label === 'NDT分数' && String(value).startsWith('已过期')) || (label === '内点率' && String(value).startsWith('已过期')) }">{{ value }}</strong>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- 右侧地图预览区 -->
        <div class="map-preview-area">
          <div v-if="!selectedMap" class="map-placeholder">
            请先选择地图
          </div>
          <div v-else class="map-container">
            <div v-if="selectedMap.thumbnail_url" class="map-image-layer">
              <img ref="mapImageRef" :src="getFullUrl(selectedMap.thumbnail_url)" alt="地图预览" @load="refreshImageGeometry" @click="handleMapClick" />

              <!-- 定位尾迹 -->
              <svg v-if="showPoseTrail && poseHistory.length > 1" class="pose-trail-lines">
                <polyline :points="poseTrailPoints()" fill="none" stroke="#f97316" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" />
              </svg>

              <!-- 途经点标记 -->
              <div class="waypoint-markers">
                <div v-for="(point, index) in waypoints" :key="index" class="waypoint-marker" :style="waypointDisplayPosition(point)">
                  {{ index + 1 }}
                </div>
                <div v-if="robotDisplayPosition()" class="robot-marker" :style="robotDisplayPosition()">
                  <span :style="robotHeadingStyle()"></span>
                </div>
                <div v-if="manualInitialPose" class="initial-pose-marker" :style="waypointDisplayPosition(manualInitialPose)">
                  <span :style="initialPoseHeadingStyle()"></span>
                </div>
              </div>

              <svg v-if="initialPoseHeadingLinePoints()" class="initial-pose-heading-line">
                <polyline :points="initialPoseHeadingLinePoints()" fill="none" stroke="#f97316" stroke-width="2.5" stroke-linecap="round" />
              </svg>

              <!-- 路径连线 -->
              <svg v-if="waypoints.length > 1" class="path-lines">
                <polyline :points="pathPolylinePoints()" fill="none" stroke="#1976d2" stroke-width="2" />
              </svg>
            </div>
            <div v-else class="map-placeholder">地图预览不可用</div>
          </div>

          <div class="map-hint" v-if="selectedMap">
            点击地图添加途经点；设初始定位时先点机器狗位置，再点狗头朝向。
          </div>
        </div>
      </div>
    </section>
  </section>
</template>

<style scoped>
.route-planner-layout {
  display: grid;
  grid-template-columns: 300px 1fr;
  gap: 1rem;
  height: calc(100vh - 200px);
}

.side-panel {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  overflow-y: auto;
  padding-right: 0.5rem;
}

.panel-section {
  background: #f9f9f9;
  padding: 1rem;
  border-radius: 4px;
}

.panel-section h3 {
  margin: 0 0 0.75rem 0;
  font-size: 0.875rem;
  color: #666;
}

.form-group {
  margin-bottom: 0.75rem;
}

.form-group label {
  display: block;
  margin-bottom: 0.25rem;
  font-size: 0.75rem;
  color: #666;
}

.form-group input,
.form-group textarea,
.form-group select {
  width: 100%;
  padding: 0.5rem;
  border: 1px solid #e0e0e0;
  border-radius: 4px;
  font-size: 0.875rem;
}

.empty-hint {
  color: #999;
  font-size: 0.875rem;
  text-align: center;
  padding: 1rem;
}

.waypoint-list {
  max-height: 200px;
  overflow-y: auto;
}

.waypoint-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.5rem;
  background: #fff;
  border-radius: 4px;
  margin-bottom: 0.5rem;
  font-size: 0.875rem;
}

.waypoint-actions {
  display: flex;
  gap: 0.5rem;
  margin-top: 0.5rem;
}

.test-panel {
  border: 1px solid #dbe4ef;
  background: #fff;
}

.status-grid {
  display: grid;
  grid-template-columns: 1fr;
  gap: 0.5rem;
}

.status-grid div {
  display: grid;
  grid-template-columns: 74px 1fr;
  gap: 0.5rem;
  align-items: center;
  font-size: 0.75rem;
}

.status-grid span {
  color: #667085;
}

.status-grid strong {
  color: #1f2937;
  font-size: 0.78rem;
  font-weight: 700;
  overflow-wrap: anywhere;
}

.nav-actions {
  display: flex;
  flex-wrap: wrap;
  gap: 0.5rem;
  margin-top: 0.75rem;
}

.command-note {
  display: block;
  margin-top: 0.5rem;
  color: #667085;
}

.state-machine-panel {
  margin-top: 0.75rem;
  border: 1px solid #cfd8e3;
  border-radius: 4px;
  background: #fff;
  overflow: hidden;
}

.state-machine-grid {
  display: grid;
  grid-template-columns: 1fr;
  gap: 0.5rem;
  padding: 0.65rem;
}

.state-card {
  display: grid;
  gap: 0.2rem;
  min-width: 0;
  padding: 0.55rem 0.6rem;
  border: 1px solid #e5e7eb;
  border-left-width: 4px;
  border-radius: 4px;
  background: #f9fafb;
}

.state-card span,
.sensor-state-row span {
  color: #667085;
  font-size: 0.7rem;
}

.state-card strong,
.sensor-state-row strong {
  color: #101828;
  font-size: 0.78rem;
  overflow-wrap: anywhere;
}

.state-card small,
.sensor-state-row small,
.sensor-state-head span {
  color: #667085;
  font-size: 0.68rem;
  line-height: 1.35;
  overflow-wrap: anywhere;
}

.state-ok {
  border-left-color: #12b76a;
}

.state-ok strong {
  color: #027a48;
}

.state-warn {
  border-left-color: #f79009;
}

.state-warn strong {
  color: #b54708;
}

.state-bad {
  border-left-color: #f04438;
}

.state-bad strong {
  color: #b42318;
}

.state-idle {
  border-left-color: #98a2b3;
}

.state-idle strong {
  color: #475467;
}

.sensor-state-list {
  border-top: 1px solid #eaecf0;
}

.sensor-state-head,
.sensor-state-row {
  display: grid;
  grid-template-columns: minmax(86px, 0.9fr) minmax(74px, 0.8fr);
  gap: 0.35rem 0.5rem;
  padding: 0.5rem 0.65rem;
  border-bottom: 1px solid #f2f4f7;
}

.sensor-state-head {
  background: #f8fafc;
}

.sensor-state-head strong {
  color: #101828;
  font-size: 0.76rem;
}

.sensor-state-row small {
  grid-column: 1 / -1;
}

.localization-debug-panel {
  margin-top: 0.75rem;
  border: 1px solid #d0d5dd;
  border-radius: 4px;
  background: #fcfcfd;
  overflow: hidden;
}

.debug-header {
  display: flex;
  justify-content: space-between;
  gap: 0.75rem;
  align-items: center;
  padding: 0.55rem 0.65rem;
  border-bottom: 1px solid #eaecf0;
  background: #f8fafc;
}

.debug-header strong {
  font-size: 0.8rem;
  color: #101828;
}

.debug-header span {
  font-size: 0.7rem;
  color: #667085;
}

.debug-grid {
  display: grid;
  grid-template-columns: 1fr;
}

.debug-row {
  display: grid;
  grid-template-columns: 76px minmax(0, 1fr);
  gap: 0.5rem;
  padding: 0.42rem 0.65rem;
  border-bottom: 1px solid #f2f4f7;
  font-size: 0.72rem;
}

.debug-row:last-child {
  border-bottom: 0;
}

.debug-row span {
  color: #667085;
}

.debug-row strong {
  color: #1f2937;
  font-weight: 700;
  overflow-wrap: anywhere;
}

.debug-row strong.danger {
  color: #b42318;
}

.initial-pose-panel {
  display: grid;
  gap: 0.5rem;
  margin-top: 0.75rem;
  padding: 0.75rem;
  border: 1px solid #bcd7ff;
  border-radius: 4px;
  background: #f5f9ff;
}

.initial-pose-panel div,
.initial-pose-panel label {
  display: grid;
  grid-template-columns: 60px 1fr;
  gap: 0.5rem;
  align-items: center;
  font-size: 0.75rem;
  color: #667085;
}

.initial-pose-panel .initial-pose-guide {
  grid-template-columns: 60px 1fr;
  align-items: start;
}

.initial-pose-guide small {
  grid-column: 2;
  color: #667085;
  line-height: 1.35;
}

.initial-pose-panel .yaw-actions {
  display: flex;
  flex-wrap: wrap;
  gap: 0.4rem;
  align-items: center;
}

.yaw-actions span {
  color: #475467;
  font-weight: 700;
  font-size: 0.75rem;
}

.initial-pose-panel input {
  width: 100%;
  padding: 0.35rem 0.45rem;
  border: 1px solid #d0d5dd;
  border-radius: 4px;
}

.form-error {
  margin: 0.6rem 0 0;
  color: #b42318;
  font-size: 0.75rem;
}

.route-list {
  max-height: 200px;
  overflow-y: auto;
}

.route-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.75rem;
  background: #fff;
  border-radius: 4px;
  margin-bottom: 0.5rem;
  cursor: pointer;
  transition: background 0.2s;
}

.route-item:hover {
  background: #f0f0f0;
}

.route-item.active {
  background: #e3f2fd;
  border: 1px solid #1976d2;
}

.route-item strong {
  display: block;
  font-size: 0.875rem;
}

.route-item small {
  color: #666;
  font-size: 0.75rem;
}

.map-preview-area {
  background: #f5f5f5;
  border-radius: 4px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  align-items: stretch;
  justify-content: flex-start;
  position: relative;
  overflow: hidden;
}

.map-container {
  flex: 1;
  position: relative;
  width: 100%;
  min-height: 520px;
  max-height: calc(100% - 40px);
  overflow: auto;
  display: flex;
  align-items: flex-start;
  justify-content: center;
  padding: 1rem;
  background:
    linear-gradient(45deg, #eef1f6 25%, transparent 25%),
    linear-gradient(-45deg, #eef1f6 25%, transparent 25%),
    linear-gradient(45deg, transparent 75%, #eef1f6 75%),
    linear-gradient(-45deg, transparent 75%, #eef1f6 75%);
  background-size: 24px 24px;
  background-position: 0 0, 0 12px, 12px -12px, -12px 0;
}

.map-image-layer {
  position: relative;
  width: min(100%, 1200px);
  min-width: 760px;
  background: #fff;
  box-shadow: 0 12px 30px rgba(15, 23, 42, 0.14);
}

.map-container img {
  width: 100%;
  height: auto;
  cursor: crosshair;
  display: block;
  user-select: none;
}

.map-placeholder {
  color: #999;
  font-size: 1rem;
  text-align: center;
  padding: 2rem;
}

.waypoint-markers {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
  z-index: 4;
}

.waypoint-marker {
  position: absolute;
  width: 24px;
  height: 24px;
  background: #1976d2;
  color: #fff;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.75rem;
  font-weight: bold;
  transform: translate(-50%, -50%);
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
}

.robot-marker {
  position: absolute;
  width: 30px;
  height: 30px;
  transform: translate(-50%, -50%);
  pointer-events: none;
  z-index: 3;
}

.robot-marker::before {
  content: "";
  position: absolute;
  inset: 4px;
  border-radius: 50%;
  background: #10b981;
  border: 3px solid #fff;
  box-shadow: 0 3px 8px rgba(16, 185, 129, 0.35);
}

.robot-marker span {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 0;
  height: 0;
  border-left: 6px solid transparent;
  border-right: 6px solid transparent;
  border-bottom: 17px solid #065f46;
  transform-origin: 50% 72%;
  z-index: 4;
}

.initial-pose-marker {
  position: absolute;
  transform: translate(-50%, -50%);
  width: 28px;
  height: 28px;
  border: 2px solid #f97316;
  border-radius: 50%;
  background: rgba(249, 115, 22, 0.12);
  color: #c2410c;
  font-size: 24px;
  line-height: 22px;
  text-align: center;
  font-weight: 800;
  pointer-events: none;
  z-index: 6;
}

.initial-pose-marker span {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 0;
  height: 0;
  border-left: 6px solid transparent;
  border-right: 6px solid transparent;
  border-bottom: 18px solid #c2410c;
  transform-origin: 50% 72%;
}

.pose-trail-lines,
.path-lines,
.initial-pose-heading-line {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.pose-trail-lines {
  z-index: 2;
}

.path-lines {
  z-index: 3;
}

.initial-pose-heading-line {
  z-index: 5;
}

.map-hint {
  margin-top: 0.5rem;
  font-size: 0.875rem;
  color: #666;
  text-align: center;
}

@media (max-width: 1100px) {
  .route-planner-layout {
    grid-template-columns: 1fr;
    height: auto;
  }

  .map-container {
    min-height: 420px;
  }
}

.btn {
  padding: 0.5rem 1rem;
  border: 1px solid #e0e0e0;
  border-radius: 4px;
  background: #fff;
  cursor: pointer;
  font-size: 0.875rem;
}

.btn-primary {
  background: #1976d2;
  color: #fff;
  border-color: #1976d2;
}

.btn-sm {
  padding: 0.25rem 0.5rem;
  font-size: 0.75rem;
}

.btn-danger {
  background: #d32f2f;
  color: #fff;
  border-color: #d32f2f;
}

.btn-close {
  background: none;
  border: none;
  font-size: 1.25rem;
  cursor: pointer;
  padding: 0;
  width: 1.5rem;
  height: 1.5rem;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #666;
}

.btn-close:hover {
  color: #d32f2f;
}

.btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}
</style>
