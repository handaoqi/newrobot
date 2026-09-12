<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useAsyncPoller } from '../composables/useAsyncPoller'
import {
  fetchMapSummaries,
  fetchMapDetail,
  fetchMapMappingTrace,
  fetchMapSetSummaries,
  fetchRouteDetail,
  fetchRobotNavigationStatus,
  fetchRobotStatus,
  fetchTaskExecution,
  fetchTaskTrajectory,
  fetchRobots,
  fetchRouteSummaries,
  fetchSpeechCategories,
  fetchSpeechTemplates,
  createRoute,
  updateRoute,
  deleteRoute,
  executeRoute,
  navigateSingleGoal,
  sendRobotNavigationCommand,
  restartRobotSensor,
  fetchMapNavigationBoundary,
  saveMapNavigationBoundary,
  publishMapNavigationBoundary,
  synthesizeSpeech,
} from '../services/api'
import { API_BASE } from '../services/api'
import RobotDogIcon from '../components/RobotDogIcon.vue'
import SystemLogPanel from '../components/SystemLogPanel.vue'
import {
  KEYFRAME_PAGE_SIZE,
  MAP_ZOOM_MAX,
  MAP_ZOOM_MIN,
  MAP_ZOOM_STEP,
  appendConfirmedInspectionPoint,
  clampMapZoom,
  headingBetweenMapPoints,
  headingDegreesToRadians,
  initialPoseCommandOutcome,
  normalizeHeadingDegrees,
  normalizeRoutePlannerTelemetry,
  paginateKeyframes,
  resolveMapClickAction,
  removeConfirmedInspectionPoint,
  rtkFixStatusLabel,
  rtkPositionTypeLabel,
  rtkQualityLabel,
  rtkSolutionStatusLabel,
  shouldShowBoundaryPolicyStatus,
} from '../services/routePlannerState'
import {
  buildLocalizationLossMarkers,
  currentRobotMapPose,
  localizationRecoveryLabel,
} from '../services/taskMapState'
import { activateAndRelocalizeMap, activateRouteMap, waitForRobotCommand } from '../services/mapActivationFlow'
import { expectedLegacyMapVersion } from '../services/mapActivationState'
import {
  buildProgressiveLocalizationPayload,
  initializeProgressiveLocalization,
  progressiveLocalizationTimeoutMs,
} from '../services/progressiveLocalization'
import {
  attemptMarkerPose,
  attemptRejectReasonLabel,
  attemptStatusClass,
  attemptStatusLabel,
  clearStoredAttemptSession,
  emptyAttemptSession,
  formatAttemptMetric,
  formatAttemptPose,
  localizationAttemptTimeline,
  localizationAttemptFailureMessage,
  isAttemptSessionTerminal,
  localizationAttemptSessionFromCommand,
  readStoredAttemptSession,
  shouldShowAttemptMarkers,
  withAttemptMarkerExpiry,
  writeStoredAttemptSession,
} from '../services/localizationAttemptSession'
import {
  appendRelocalizationMarker,
  readStoredRelocalizationMarkers,
  relocalizationMarkerTitle,
  writeStoredRelocalizationMarkers,
} from '../services/relocalizationMarkers'
import { preferredExecutedItem } from '../utils/executionSelection'
import { resolveBatteryPercent } from '../utils/battery'
import { isLowBatteryBlocked, lowBatteryGuardMessage } from '../utils/guardDutyLowBattery'
import {
  buildTaskExecutionTimeline,
  resolveTaskExecutionWaypointProgress,
  taskExecutionIsActive,
} from '../services/taskExecutionWaypointProgress'

const maps = ref([])
const mapSets = ref([])
const robots = ref([])

const getFullUrl = (relativeUrl) => {
  if (!relativeUrl) return null
  if (relativeUrl.startsWith('http')) return relativeUrl
  return `${API_BASE.replace('/api', '')}${relativeUrl}`
}
const routes = ref([])
const speechCategories = ref([])
const speechTemplates = ref([])
const selectedMap = ref(null)
const selectedMapVersion = () => expectedLegacyMapVersion(selectedMap.value?.id)
const mapOriginDisplay = computed(() => selectedMap.value?.origin_display || null)
const mapFrameOrigin = computed(() => mapOriginDisplay.value?.map || { x: 0, y: 0, yaw: 0 })
const occupancyGridOrigin = computed(() => mapOriginDisplay.value?.occupancy_grid || null)
const rtkEnuOrigin = computed(() => mapOriginDisplay.value?.rtk_enu || null)
const selectedRoute = ref(null)
const waypoints = ref([])
const waypointNames = ref([])
const showRouteDialog = ref(false)
const loading = ref(false)
const mapImageRef = ref(null)
const mapViewportRef = ref(null)
const drillTimelineListRef = ref(null)
const imageReadyTick = ref(0)
const navStatus = ref(null)
const navCommandBusy = ref('')
const sensorCommandBusy = ref('')
const navError = ref('')
const routeLoadError = ref('')
const routeLoading = ref(false)
const routeExecuteBusy = ref(false)
const lastExecution = ref(null)
const taskMapExecution = ref(null)
const taskMapTrajectory = ref([])
const initialPoseMode = ref(false)
const manualInitialPose = ref(null)
const initialPoseStep = ref('position')
const initialPoseHeadingTarget = ref(null)
const localizationInitState = ref('idle')
const localizationInitMessage = ref('')
const localizationAttemptSession = ref(null)
const localizationAttemptCardOpen = ref(true)
const attemptMarkerTick = ref(0)
const relocalizationMarkers = ref([])
const lastLocalizationStatus = ref('')
const poseHistory = ref([])
const showPoseTrail = ref(true)
const showLocalizationAttemptPoints = ref(true)
const mappingTrace = ref([])
const mappingTraceLoading = ref(false)
const showMappingTrace = ref(true)
const inspectedMapPoint = ref(null)
const confirmedInspectedPoints = ref([])
const mapClickMode = ref('waypoint')
const inspectPoseStep = ref('position')
const inspectedHeadingTarget = ref(null)
const mapInteractionError = ref('')
const mapZoom = ref(1)
const mapImageNaturalWidth = ref(0)
const waypointYawDrafts = ref([])
const waypointYawErrors = ref([])
const waypointYawConfirmed = ref([])
const singleGoalIndex = ref(null)
const selectedSingleGoalIndex = ref(0)
const selectedLogPoint = ref(null)
const boundaryConfig = ref(null)
const boundaryEditing = ref(false)
const boundaryDrawType = ref('outer')
const boundaryDraftPoints = ref([])
const boundaryBusy = ref('')
const boundaryError = ref('')
const localizationLossMarkers = computed(() => buildLocalizationLossMarkers(
  taskMapExecution.value,
  taskMapTrajectory.value,
  selectedMap.value?.id,
))
const visibleAttemptMarkers = computed(() => {
  attemptMarkerTick.value
  if (!showLocalizationAttemptPoints.value) return []
  const session = localizationAttemptSession.value
  if (!shouldShowAttemptMarkers(session)) return []
  return session.attempts
})
const globalPlanPoints = computed(() => {
  imageReadyTick.value
  if (!robotMapMatches()) return []
  const plan = navStatus.value?.status?.navigation?.global_plan
  if (!plan?.updated || !Array.isArray(plan.points) || plan.points.length < 2) return []
  return plan.points
})
const keyframePanelOpen = ref(false)
const keyframePage = ref(1)
const selectedKeyframeIndex = ref(null)
const lastPoseSampleKey = ref('')
const drillRunning = ref(false)
const drillPosition = ref(null)
const drillCurrentIndex = ref(null)
const drillMessage = ref('')
const drillTimeline = ref([])
const drillTimelineOpen = ref(false)
const drillTimelineMode = ref('simulation')
const drillElapsedSeconds = ref(0)
const drillCurrentSpeed = ref(0)
const routePreviewRequestedAt = ref(0)
const routePreviewError = ref('')
const expandedWaypoints = ref(new Set())
const routeListOpen = ref(false)
const navigationPoller = useAsyncPoller((signal) => refreshNavigationStatus({ signal }), {
  intervalMs: 2_000,
  immediate: false,
})
let drillClockTimer = null
let drillStartedAt = null
let drillEventSequence = 0
let drillAnimationFrame = null
let drillCancelled = false
let drillAudio = null
let drillAudioResolve = null
let navigationStatusRefreshing = false
let inspectionPointSequence = 0
let initialSelectionApplied = false
let attemptMarkerTimer = null
let routeLoadSequence = 0

const GLOBAL_CONTROLLER_OPTIONS = [
  { value: 'theta_star', label: 'Theta*（ThetaStar）' },
  { value: 'navfn', label: 'NavFn（A*）' },
  { value: 'smac_hybrid', label: 'Smac Hybrid A*（高精度）' },
]
const LOCAL_CONTROLLER_OPTIONS = [
  { value: 'mppi', label: 'MPPI（FollowPath）' },
  { value: 'rpp', label: 'RPP（Regulated Pure Pursuit）' },
  { value: 'ilqr', label: 'iLQR（高精度）' },
]
const ARRIVAL_POLICY_OPTIONS = [
  { value: 'pass_through', label: '通过（不停留）' },
  { value: 'stop_and_confirm', label: '停车校正确认' },
  { value: 'precision', label: '精确到点' },
  { value: 'dock', label: '停靠确认' },
]
const DEFAULT_GLOBAL_CONTROLLER = 'theta_star'
const NEW_ROUTE_SELECTION = '__new__'

const routeForm = ref({
  name: '',
  map_data: null,
  map_set: null,
  robot: null,
  description: '',
  scene_scope: 'indoor',
  record_rosbag: false,
})
const allWaypointsExpanded = computed(() => (
  waypoints.value.length > 0
  && waypoints.value.every((_, index) => expandedWaypoints.value.has(index))
))
const routeExecutionProgress = computed(() => resolveTaskExecutionWaypointProgress(taskMapExecution.value))
const routeExecutionTimeline = computed(() => buildTaskExecutionTimeline(taskMapExecution.value, {
  requestedAt: routePreviewRequestedAt.value,
  requestError: routePreviewError.value,
}))
const displayedDrillTimeline = computed(() => (
  drillTimelineMode.value === 'execution' ? routeExecutionTimeline.value : drillTimeline.value
))
const routeExecutionRunning = computed(() => (
  drillTimelineMode.value === 'execution'
  && (routeExecuteBusy.value || taskExecutionIsActive(taskMapExecution.value))
))
const timelineRunning = computed(() => (
  drillTimelineMode.value === 'execution' ? routeExecutionRunning.value : drillRunning.value
))
const drillTimelineSourceLabel = computed(() => (
  drillTimelineMode.value === 'execution' ? '真实预演' : '模拟演练'
))
const drillTimelineEmptyText = computed(() => (
  drillTimelineMode.value === 'execution'
    ? '路线下发后，这里会按顺序显示航点目标下发和实际到达过程。'
    : '点击“演练开始”后，这里会记录模拟移动、到达点位和播报内容。'
))
const drillTimelineSummary = computed(() => {
  if (drillTimelineMode.value === 'execution') {
    const progress = routeExecutionProgress.value
    const execution = taskMapExecution.value
    const currentTarget = progress.activeTarget
    const currentLabel = currentTarget?.payload?.waypoint?.map_point_number
      ?? progress.currentMapPointNumber
    const speed = Number(navStatus.value?.status?.speed_mps || 0)
    return [
      { label: '用时', value: formatDrillElapsed(drillElapsedSeconds.value) },
      { label: '当前目标', value: currentLabel !== null && currentLabel !== undefined ? `${currentLabel}号点` : (execution ? '等待目标' : '等待下发') },
      { label: '进度', value: `${Number(execution?.completed_waypoints || 0)} / ${progress.totalWaypoints || waypoints.value.length}` },
      { label: '当前速度', value: `${speed.toFixed(2)} m/s` },
    ]
  }
  return [
    { label: '用时', value: formatDrillElapsed(drillElapsedSeconds.value) },
    { label: '当前点位', value: drillCurrentIndex.value == null ? '未开始' : (waypointNames.value[drillCurrentIndex.value] || `点${drillCurrentIndex.value + 1}`) },
    { label: '当前速度', value: `${drillCurrentSpeed.value.toFixed(2)} m/s` },
    { label: '事件', value: String(displayedDrillTimeline.value.length) },
  ]
})

onMounted(async () => {
  await loadData()
  await navigationPoller.run()
  restoreAttemptSessionFromStatus()
  restoreRelocalizationMarkers()
  window.addEventListener('resize', refreshImageGeometry)
})

onBeforeUnmount(() => {
  routeLoadSequence += 1
  window.removeEventListener('resize', refreshImageGeometry)
  stopDrill(false)
  if (attemptMarkerTimer) {
    clearTimeout(attemptMarkerTimer)
    attemptMarkerTimer = null
  }
})

async function loadData() {
  loading.value = true
  try {
    const [mapsResult, mapSetsResult, routesResult, robotsResult, categoriesResult, templatesResult] = await Promise.allSettled([
      fetchMapSummaries(),
      fetchMapSetSummaries(),
      fetchRouteSummaries(),
      fetchRobots(),
      fetchSpeechCategories(),
      fetchSpeechTemplates(),
    ])
    if (mapsResult.status === 'fulfilled') maps.value = mapsResult.value
    else console.error('加载地图失败:', mapsResult.reason)
    if (mapSetsResult.status === 'fulfilled') mapSets.value = mapSetsResult.value
    else console.error('加载地图集失败:', mapSetsResult.reason)
    if (routesResult.status === 'fulfilled') routes.value = routesResult.value
    else console.error('加载路线失败:', routesResult.reason)
    if (robotsResult.status === 'fulfilled') robots.value = robotsResult.value
    else console.error('加载机器人失败:', robotsResult.reason)
    if (categoriesResult.status === 'fulfilled') speechCategories.value = categoriesResult.value
    else console.error('加载播报分类失败:', categoriesResult.reason)
    if (templatesResult.status === 'fulfilled') speechTemplates.value = templatesResult.value
    else console.error('加载播报文案失败:', templatesResult.reason)
    seedRobotsFromMapsAndRoutes()
    if (!initialSelectionApplied) {
      initialSelectionApplied = true
      const preferredRoute = preferredExecutedItem(routes.value)
      if (preferredRoute) {
        await handleLoadRoute(preferredRoute)
      } else if (maps.value.length) {
        await handleMapSelect(maps.value.find(map => map.active) || maps.value[0])
      }
    }
  } catch (error) {
    console.error('加载数据失败:', error)
  } finally {
    loading.value = false
  }
}

const selectedRobot = computed(() => {
  const robotId = routeForm.value.robot || selectedRoute.value?.robot || selectedMap.value?.robot || robots.value[0]?.id
  const robot = robots.value.find(item => String(item.id) === String(robotId))
  if (robot) return robot
  if (!robotId) return null
  return {
    id: robotId,
    name: selectedRoute.value?.robot_name || selectedMap.value?.robot_name || '机器狗',
    code: selectedRoute.value?.robot_code || selectedMap.value?.robot_code || String(robotId),
  }
})

watch(() => selectedRobot.value?.id, () => {
  restoreRelocalizationMarkers()
  lastLocalizationStatus.value = ''
})

watch(() => displayedDrillTimeline.value.at(-1)?.id, () => {
  nextTick(() => {
    if (drillTimelineListRef.value) {
      drillTimelineListRef.value.scrollTop = drillTimelineListRef.value.scrollHeight
    }
  })
})

watch(() => selectedMap.value?.id, async (mapId) => {
  boundaryEditing.value = false
  boundaryDraftPoints.value = []
  selectedLogPoint.value = null
  boundaryError.value = ''
  if (!mapId) {
    boundaryConfig.value = null
    return
  }
  try {
    boundaryConfig.value = await fetchMapNavigationBoundary(mapId)
  } catch (error) {
    boundaryConfig.value = null
    boundaryError.value = error.message || '导航边界加载失败'
  }
})

function startBoundaryDrawing(type) {
  ensureBoundaryConfig()
  boundaryEditing.value = true
  boundaryDrawType.value = type
  boundaryDraftPoints.value = []
  initialPoseMode.value = false
  boundaryError.value = ''
}

function ensureBoundaryConfig() {
  if (boundaryConfig.value) return boundaryConfig.value
  boundaryConfig.value = {
    revision: 0,
    active_revision: 0,
    apply_status: 'unconfigured',
    outer_polygon: [],
    safety_margin_m: 0.2,
    zones: [],
  }
  return boundaryConfig.value
}

function toggleBoundaryEditing() {
  if (!boundaryEditing.value) ensureBoundaryConfig()
  boundaryEditing.value = !boundaryEditing.value
  if (!boundaryEditing.value) boundaryDraftPoints.value = []
  boundaryError.value = ''
}

function cancelBoundaryDrawing() {
  boundaryDraftPoints.value = []
  boundaryError.value = ''
}

function finishBoundaryDrawing() {
  if (boundaryDraftPoints.value.length < 3) {
    boundaryError.value = '至少需要三个边界点'
    return
  }
  const polygon = boundaryDraftPoints.value.map(point => [Number(point.x), Number(point.y)])
  if (!boundaryConfig.value) boundaryConfig.value = { revision: 0, outer_polygon: [], safety_margin_m: 0.2, zones: [] }
  if (boundaryDrawType.value === 'outer') boundaryConfig.value.outer_polygon = polygon
  else {
    const labels = { forbidden: '禁入区', restricted: '限速区', warning: '警告区' }
    boundaryConfig.value.zones.push({
      name: `${labels[boundaryDrawType.value]}${boundaryConfig.value.zones.length + 1}`,
      zone_type: boundaryDrawType.value,
      polygon,
      active: true,
      speed_limit_mps: boundaryDrawType.value === 'restricted' ? 0.15 : null,
      warning_distance_m: 0.5,
      description: '',
    })
  }
  boundaryDraftPoints.value = []
  boundaryError.value = ''
}

function removeBoundaryZone(index) {
  boundaryConfig.value.zones.splice(index, 1)
}

async function saveBoundaryDraft() {
  if (!selectedMap.value || !boundaryConfig.value?.outer_polygon?.length) {
    boundaryError.value = '请先绘制可行驶外边界'
    return
  }
  boundaryBusy.value = 'save'
  boundaryError.value = ''
  try {
    boundaryConfig.value = await saveMapNavigationBoundary(selectedMap.value.id, boundaryConfig.value)
  } catch (error) {
    boundaryError.value = error.message || '导航边界保存失败'
  } finally {
    boundaryBusy.value = ''
  }
}

async function publishBoundary() {
  if (!selectedMap.value || !boundaryConfig.value?.revision) {
    boundaryError.value = '请先保存边界草稿'
    return
  }
  boundaryBusy.value = 'publish'
  boundaryError.value = ''
  try {
    const published = await publishMapNavigationBoundary(selectedMap.value.id, selectedRobot.value?.id)
    boundaryConfig.value = published
    if (published.command_id && selectedRobot.value?.id) {
      await waitForRobotCommand(
        selectedRobot.value.id,
        { id: published.command_id, status: 'created' },
        { timeoutMs: 240_000 },
      )
      boundaryConfig.value = await fetchMapNavigationBoundary(selectedMap.value.id)
    }
  } catch (error) {
    boundaryError.value = error.message || '导航边界发布失败'
    try {
      boundaryConfig.value = await fetchMapNavigationBoundary(selectedMap.value.id)
    } catch {
      // Keep the publish error as the operator-facing diagnosis.
    }
  } finally {
    boundaryBusy.value = ''
  }
}

function boundaryPolygonPoints(polygon = []) {
  return polygon.map(point => pointDisplayPositionFromMap(point[0], point[1])).filter(Boolean).map(point => `${point.x},${point.y}`).join(' ')
}

function boundaryDraftPolyline() {
  return boundaryDraftPoints.value.map(point => pointDisplayPositionFromMap(point.x, point.y)).filter(Boolean).map(point => `${point.x},${point.y}`).join(' ')
}

function boundaryStatusPresentation() {
  const boundary = boundaryConfig.value
  if (!boundary?.outer_polygon?.length) {
    return { tone: 'warning', title: '导航边界未配置', detail: '当前地图按兼容模式运行；建议绘制外边界并发布后再执行任务。' }
  }
  if (boundary.apply_status === 'failed') {
    return { tone: 'error', title: `边界 v${boundary.revision} 发布失败`, detail: boundary.apply_error || '请检查 Edge 与 Nav2 日志后重试。' }
  }
  if (boundary.apply_status === 'applying') {
    return { tone: 'applying', title: `边界 v${boundary.revision} 正在发布`, detail: 'Edge 确认并加载 Nav2 过滤图层后才会标记为生效。' }
  }
  if (Number(boundary.active_revision || 0) < Number(boundary.revision || 0)) {
    return { tone: 'warning', title: `草稿 v${boundary.revision} 尚未生效`, detail: `机器人继续使用已生效版本 v${boundary.active_revision || 0}。` }
  }
  return { tone: 'active', title: `导航边界 v${boundary.active_revision} 已生效`, detail: '外边界、禁入区和安全距离已参与 Nav2 规划与 Edge 运行时保护。' }
}

function locateSystemLog(log) {
  selectedLogPoint.value = { x: Number(log.x), y: Number(log.y), yaw: Number(log.yaw || 0) }
  nextTick(() => centerMapOnPoint(selectedLogPoint.value))
}

function resetWaypointExpansion() {
  expandedWaypoints.value = new Set(waypoints.value.map((_, index) => index))
}

function isWaypointExpanded(index) {
  return expandedWaypoints.value.has(index)
}

function toggleWaypointExpanded(index) {
  const next = new Set(expandedWaypoints.value)
  if (next.has(index)) next.delete(index)
  else next.add(index)
  expandedWaypoints.value = next
}

function toggleAllWaypoints() {
  expandedWaypoints.value = allWaypointsExpanded.value
    ? new Set()
    : new Set(waypoints.value.map((_, index) => index))
}

async function handleRouteSelect(routeId) {
  if (routeId === NEW_ROUTE_SELECTION) {
    startNewRouteEditor()
    return
  }
  const route = routes.value.find(item => String(item.id) === String(routeId))
  if (route) await handleLoadRoute(route)
}

function startNewRouteEditor() {
  stopDrill(false)
  routeLoadSequence += 1
  routeLoading.value = false
  routeLoadError.value = ''
  selectedRoute.value = null
  lastExecution.value = null
  taskMapExecution.value = null
  taskMapTrajectory.value = []
  routeForm.value = {
    ...routeForm.value,
    name: '',
    description: '',
    map_data: selectedMap.value?.id || routeForm.value.map_data || null,
    map_set: null,
  }
  waypoints.value = []
  waypointNames.value = []
  resetWaypointExpansion()
  resetWaypointYawEditors()
  resetDrillTimelineView()
}

function toggleRouteList() {
  routeListOpen.value = !routeListOpen.value
}

const inspectionSpeechCategory = computed(() => speechCategories.value.find(item => item.name === '巡检智能播报') || null)
const inspectionSpeechTemplates = computed(() => {
  if (!inspectionSpeechCategory.value) return []
  return speechTemplates.value.filter(item => String(item.category) === String(inspectionSpeechCategory.value.id))
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

async function handleMapSelect(map) {
  resetDrillTimelineView()
  routeLoadSequence += 1
  routeLoading.value = false
  routeLoadError.value = ''
  selectedMap.value = map
  selectedRoute.value = null
  lastExecution.value = null
  taskMapExecution.value = null
  taskMapTrajectory.value = []
  resetWaypointExpansion()
  waypoints.value = []
  waypointNames.value = []
  resetWaypointYawEditors()
  clearPoseHistory()
  clearInspectedMapPoints()
  selectedKeyframeIndex.value = null
  keyframePage.value = 1
  mapZoom.value = 1
  routeForm.value = {
    name: '',
    map_data: map?.id || null,
    map_set: null,
    robot: routeForm.value.robot || robots.value[0]?.id || map?.robot || 1,
    description: '',
    scene_scope: map?.scene_scope || 'indoor',
    record_rosbag: false,
  }
  refreshImageGeometry()
  refreshNavigationStatus()
  await Promise.all([
    loadMappingTrace(map),
    hydrateSelectedMap(map),
  ])
}

async function handleMapIdSelect(mapId) {
  const map = maps.value.find((item) => String(item.id) === String(mapId)) || null
  if (String(map?.id || '') === String(selectedMap.value?.id || '')) return
  await handleMapSelect(map)
}

async function hydrateSelectedMap(map) {
  if (!map?.id || Object.prototype.hasOwnProperty.call(map, 'description')) return map
  try {
    const detail = await fetchMapDetail(map.id)
    if (String(selectedMap.value?.id) === String(map.id)) selectedMap.value = detail
    return detail
  } catch (error) {
    console.warn('加载地图详情失败:', error)
    return map
  }
}

async function loadMappingTrace(map = selectedMap.value) {
  mappingTrace.value = []
  keyframePage.value = 1
  selectedKeyframeIndex.value = null
  if (!map?.id) return
  mappingTraceLoading.value = true
  try {
    const result = await fetchMapMappingTrace(map.id)
    if (String(selectedMap.value?.id) === String(map.id)) mappingTrace.value = result.samples || []
  } catch (error) {
    console.error('加载建图轨迹失败:', error)
  } finally {
    mappingTraceLoading.value = false
  }
}

function handleMapClick(event) {
  if (!selectedMap.value) return
  if (drillRunning.value) return

  const image = mapImageRef.value || event.currentTarget
  const geometry = getMapGeometry()
  if (!geometry) return
  const rect = geometry.rect
  const displayX = event.clientX - rect.left
  const displayY = event.clientY - rect.top
  if (displayX < 0 || displayY < 0 || displayX > rect.width || displayY > rect.height) return

  const imagePoint = displayToImagePoint(displayX, displayY, geometry)
  const clickedMapPoint = imagePointToWaypoint(imagePoint, geometry)
  if (boundaryEditing.value) {
    boundaryDraftPoints.value.push({ x: clickedMapPoint.x, y: clickedMapPoint.y })
    boundaryError.value = ''
    return
  }
  const clickAction = resolveMapClickAction(mapClickMode.value, initialPoseMode.value)
  if (clickAction === 'initial_pose') {
    const clickedPose = imagePointToWaypoint(imagePoint, geometry, Number(manualInitialPose.value?.yaw || 0))
    if (initialPoseStep.value === 'position' || !manualInitialPose.value) {
      manualInitialPose.value = clickedPose
      initialPoseHeadingTarget.value = null
      initialPoseStep.value = 'heading'
      navError.value = '已设置初始位置，请再点击狗头朝向'
      return
    }
    const yaw = headingBetweenMapPoints(manualInitialPose.value, clickedPose)
    if (yaw === null) {
      navError.value = '朝向点离初始位置太近，请点远一点'
      return
    }
    manualInitialPose.value = {
      ...manualInitialPose.value,
      yaw,
    }
    initialPoseHeadingTarget.value = clickedPose
    navError.value = '已设置初始朝向，可以下发初始定位'
    return
  }
  if (clickAction === 'inspect') {
    if (inspectPoseStep.value === 'position' || !inspectedMapPoint.value) {
      inspectedMapPoint.value = {
        point: { ...clickedMapPoint, yaw: 0 },
        sample: nearestMappingSample(clickedMapPoint),
      }
      inspectedHeadingTarget.value = null
      inspectPoseStep.value = 'heading'
      mapInteractionError.value = ''
      selectedKeyframeIndex.value = inspectedMapPoint.value.sample?.index ?? null
      return
    }
    const yaw = headingBetweenMapPoints(inspectedMapPoint.value.point, clickedMapPoint)
    if (yaw === null) {
      mapInteractionError.value = '方向点离选定位置太近，请移动后再点击'
      return
    }
    inspectedMapPoint.value = {
      ...inspectedMapPoint.value,
      point: { ...inspectedMapPoint.value.point, yaw },
    }
    inspectedHeadingTarget.value = clickedMapPoint
    inspectPoseStep.value = 'complete'
    mapInteractionError.value = ''
    return
  }
  resetInspectedMapPoint()
  const waypoint = imagePointToWaypoint(imagePoint, geometry)
  waypoints.value.push(waypoint)
  expandedWaypoints.value = new Set([...expandedWaypoints.value, waypoints.value.length - 1])
  waypointNames.value.push(`点${waypoints.value.length}`)
  waypointYawDrafts.value.push(waypointYawDegrees(waypoint).toFixed(1))
  waypointYawErrors.value.push('')
  waypointYawConfirmed.value.push(true)
}

function setMapClickMode(mode) {
  const nextMode = mode === 'inspect' ? 'inspect' : 'waypoint'
  if (mapClickMode.value === nextMode) return
  mapClickMode.value = nextMode
  resetInspectedMapPoint()
}

function resetInspectedMapPoint() {
  inspectedMapPoint.value = null
  inspectedHeadingTarget.value = null
  inspectPoseStep.value = 'position'
  mapInteractionError.value = ''
  selectedKeyframeIndex.value = null
}

function clearInspectedMapPoints() {
  resetInspectedMapPoint()
  confirmedInspectedPoints.value = []
  inspectionPointSequence = 0
}

function confirmInspectedMapPoint() {
  if (!inspectedMapPoint.value || inspectPoseStep.value !== 'complete') return
  confirmedInspectedPoints.value = appendConfirmedInspectionPoint(
    confirmedInspectedPoints.value,
    inspectedMapPoint.value,
    ++inspectionPointSequence,
  )
  resetInspectedMapPoint()
}

function removeConfirmedInspectedPoint(pointId) {
  confirmedInspectedPoints.value = removeConfirmedInspectionPoint(confirmedInspectedPoints.value, pointId)
}

function mapModeHintText() {
  if (mapClickMode.value === 'waypoint') return '点击地图直接添加途经点。'
  if (inspectPoseStep.value === 'position') return '第1步：点击地图获得 XY 位置。'
  if (inspectPoseStep.value === 'heading') return '第2步：移动到朝向位置并再次点击，确定方向。'
  return '位置和方向已获取；点击“确认”固定该点，然后继续选择下一个点。'
}

function refreshImageGeometry() {
  if (mapImageRef.value?.naturalWidth) mapImageNaturalWidth.value = mapImageRef.value.naturalWidth
  imageReadyTick.value += 1
}

function removeWaypoint(index) {
  waypoints.value.splice(index, 1)
  waypointNames.value.splice(index, 1)
  waypointYawDrafts.value.splice(index, 1)
  waypointYawErrors.value.splice(index, 1)
  waypointYawConfirmed.value.splice(index, 1)
  resetWaypointExpansion()
}

function setWaypointSpeech(index, templateId) {
  const template = inspectionSpeechTemplates.value.find(item => String(item.id) === String(templateId))
  const current = waypoints.value[index]
  waypoints.value[index] = {
    ...current,
    speech_template_id: template?.id || null,
    speech_template_name: template?.name || '',
    speech_text: template?.text || '',
  }
}

function normalizeWaypointLocalizationMode(mode) {
  const normalized = String(mode || 'ndt').trim().toLowerCase()
  return ['ndt', 'rtk', 'ukf'].includes(normalized) ? normalized : 'ndt'
}

function normalizeLocalController(mode) {
  const normalized = String(mode || 'mppi').trim().toLowerCase()
  return ['mppi', 'rpp', 'ilqr'].includes(normalized) ? normalized : 'mppi'
}

function normalizeGlobalController(mode) {
  const normalized = String(mode || 'theta_star').trim().toLowerCase()
  return ['theta_star', 'navfn', 'smac_hybrid'].includes(normalized) ? normalized : 'theta_star'
}

function normalizeArrivalPolicy(value, point = {}) {
  const normalized = String(value || '').trim().toLowerCase()
  if (ARRIVAL_POLICY_OPTIONS.some(option => option.value === normalized)) return normalized
  if (Number(point.dwell_seconds || 0) > 0 || point.require_yaw === true || (point.actions || []).length) {
    return 'stop_and_confirm'
  }
  return 'stop_and_confirm'
}

function normalizeSpeechMode(value, point = {}) {
  const normalized = String(value || '').trim().toLowerCase()
  if (['blocking', 'non_blocking', 'disabled'].includes(normalized)) return normalized
  return point.speech_template_id ? 'non_blocking' : 'disabled'
}

function setWaypointArrivalPolicy(index, policy) {
  waypoints.value[index] = {
    ...waypoints.value[index],
    arrival_policy: normalizeArrivalPolicy(policy, waypoints.value[index]),
  }
}

function setWaypointLocalization(index, mode) {
  const normalized = normalizeWaypointLocalizationMode(mode)
  const allowed = mapIsLocalOnly.value && normalized === 'rtk' ? 'ndt' : normalized
  waypoints.value[index] = {
    ...waypoints.value[index],
    localization_mode: allowed,
  }
}

function setWaypointLocalController(index, mode) {
  waypoints.value[index] = {
    ...waypoints.value[index],
    local_controller: normalizeLocalController(mode),
  }
}

function setWaypointGlobalController(index, mode) {
  waypoints.value[index] = {
    ...waypoints.value[index],
    global_controller: normalizeGlobalController(mode),
  }
}

const mapIsLocalOnly = computed(() => {
  const mode = selectedMap.value?.coordinate_mode || ''
  return mode === 'local_only'
})

function resetWaypointYawEditors() {
  waypointYawDrafts.value = waypoints.value.map(point => waypointYawDegrees(point).toFixed(1))
  waypointYawErrors.value = waypoints.value.map(() => '')
  waypointYawConfirmed.value = waypoints.value.map(() => true)
}

function setWaypointYawDraft(index, value) {
  waypointYawDrafts.value[index] = value
  waypointYawErrors.value[index] = ''
  waypointYawConfirmed.value[index] = false
}

function confirmWaypointYaw(index) {
  const normalizedDegrees = normalizeHeadingDegrees(waypointYawDrafts.value[index])
  if (normalizedDegrees === null) {
    waypointYawErrors.value[index] = '请输入有效方向角'
    waypointYawConfirmed.value[index] = false
    return
  }
  const yaw = headingDegreesToRadians(normalizedDegrees)
  waypoints.value[index] = {
    ...waypoints.value[index],
    yaw,
  }
  waypointYawDrafts.value[index] = normalizedDegrees.toFixed(1)
  waypointYawErrors.value[index] = ''
  waypointYawConfirmed.value[index] = true
}

function waypointYawDegrees(point) {
  return normalizeHeadingDegrees(Number(point?.yaw || 0) * 180 / Math.PI) ?? 0
}

async function executeSingleGoal(index) {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot
  const point = waypoints.value[index]
  if (!robotId || !selectedMap.value?.id || !point || navCommandBusy.value) return
  if (!waypointYawConfirmed.value[index]) { navError.value = '请先确认该途经点方向角'; return }
  if (!confirm(`确认通过云平台导航到${waypointNames.value[index] || `点${index + 1}`}？`)) return
  navCommandBusy.value = 'single-goal'; singleGoalIndex.value = index; navError.value = ''
  try {
    await navigateSingleGoal(robotId, {
      frame_id: 'map',
      x: Number(point.x),
      y: Number(point.y),
      yaw: Number(point.yaw || 0),
      require_yaw: Boolean(point.require_yaw),
      global_controller: normalizeGlobalController(point.global_controller || DEFAULT_GLOBAL_CONTROLLER),
      map_id: selectedMap.value.id,
      map_version: selectedMapVersion(),
    })
    navError.value = `已下发单点导航：${waypointNames.value[index] || `点${index + 1}`}`
  } catch (error) { navError.value = error.message || '单点导航下发失败' }
  finally { navCommandBusy.value = ''; singleGoalIndex.value = null; await refreshNavigationStatus() }
}

function setWaypointBoolean(index, field, value) {
  const next = { ...waypoints.value[index], [field]: Boolean(value) }
  if (field === 'avoidance_to_next') {
    next.detour_enabled = Boolean(value)
    next.collision_slowdown_enabled = Boolean(value)
    next.collision_stop_enabled = true
  }
  waypoints.value[index] = next
}

function setWaypointDwell(index, value) {
  const seconds = Math.min(3600, Math.max(0, Number(value) || 0))
  waypoints.value[index] = {
    ...waypoints.value[index],
    dwell_seconds: Number(seconds.toFixed(1)),
  }
}

async function adjustMapZoom(delta) {
  const viewport = mapViewportRef.value
  const centerX = viewport?.scrollWidth ? (viewport.scrollLeft + viewport.clientWidth / 2) / viewport.scrollWidth : 0.5
  const centerY = viewport?.scrollHeight ? (viewport.scrollTop + viewport.clientHeight / 2) / viewport.scrollHeight : 0.5
  mapZoom.value = clampMapZoom(mapZoom.value + delta)
  await nextTick()
  refreshImageGeometry()
  if (viewport) {
    viewport.scrollLeft = Math.max(0, centerX * viewport.scrollWidth - viewport.clientWidth / 2)
    viewport.scrollTop = Math.max(0, centerY * viewport.scrollHeight - viewport.clientHeight / 2)
  }
}

function resetMapZoom() {
  mapZoom.value = 1
  nextTick(refreshImageGeometry)
}

const mapImageLayerStyle = computed(() => {
  const baseWidth = Math.min(1200, mapImageNaturalWidth.value || 1200)
  return { width: `${Math.max(320, Math.round(baseWidth * mapZoom.value))}px` }
})

const keyframePageData = computed(() => paginateKeyframes(mappingTrace.value, keyframePage.value, KEYFRAME_PAGE_SIZE))

function changeKeyframePage(delta) {
  keyframePage.value = Math.min(
    keyframePageData.value.pageCount,
    Math.max(1, keyframePageData.value.page + delta),
  )
}

async function selectKeyframe(sample, rowIndex) {
  const slam = sample?.slam
  if (!Number.isFinite(Number(slam?.x)) || !Number.isFinite(Number(slam?.y))) return
  const absoluteIndex = keyframePageData.value.start + rowIndex
  selectedKeyframeIndex.value = sample.index ?? absoluteIndex
  inspectedMapPoint.value = {
    point: normalizeStoredWaypoint({ x: Number(slam.x), y: Number(slam.y), yaw: Number(slam.yaw || 0) }),
    sample: { ...sample, index: selectedKeyframeIndex.value, distance_m: 0 },
  }
  mapClickMode.value = 'inspect'
  inspectPoseStep.value = 'complete'
  inspectedHeadingTarget.value = null
  mapInteractionError.value = ''
  await nextTick()
  centerMapOnPoint(slam)
}

function centerMapOnPoint(point) {
  const viewport = mapViewportRef.value
  const geometry = getMapGeometry()
  if (!viewport || !geometry) return
  const display = pointDisplayPositionFromMap(point.x, point.y, geometry)
  if (!display) return
  const imageRect = mapImageRef.value.getBoundingClientRect()
  const viewportRect = viewport.getBoundingClientRect()
  const targetX = imageRect.left - viewportRect.left + viewport.scrollLeft + display.x
  const targetY = imageRect.top - viewportRect.top + viewport.scrollTop + display.y
  viewport.scrollTo({
    left: Math.max(0, targetX - viewport.clientWidth / 2),
    top: Math.max(0, targetY - viewport.clientHeight / 2),
    behavior: 'smooth',
  })
}

function clearWaypoints() {
  stopDrill(false)
  waypoints.value = []
  waypointNames.value = []
  resetWaypointExpansion()
  resetWaypointYawEditors()
}

function drillDisplayPosition() {
  if (!drillPosition.value) return null
  return waypointDisplayPosition(drillPosition.value)
}

function formatDrillClock(value) {
  return new Date(value).toLocaleTimeString('zh-CN', { hour12: false })
}

function formatDrillElapsed(seconds) {
  const total = Math.max(0, Math.floor(Number(seconds) || 0))
  const minutes = String(Math.floor(total / 60)).padStart(2, '0')
  const remaining = String(total % 60).padStart(2, '0')
  return `${minutes}:${remaining}`
}

function recordDrillEvent(type, title, detail = '', metadata = {}) {
  const occurredAt = Date.now()
  drillTimeline.value.push({
    id: ++drillEventSequence,
    type,
    title,
    detail,
    occurredAt,
    elapsedSeconds: drillStartedAt ? (occurredAt - drillStartedAt) / 1000 : 0,
    ...metadata,
  })
  nextTick(() => {
    if (drillTimelineListRef.value) {
      drillTimelineListRef.value.scrollTop = drillTimelineListRef.value.scrollHeight
    }
  })
}

function startDrillClock() {
  if (drillClockTimer) window.clearInterval(drillClockTimer)
  drillClockTimer = window.setInterval(() => {
    drillElapsedSeconds.value = drillStartedAt ? (Date.now() - drillStartedAt) / 1000 : 0
  }, 250)
}

function stopDrillClock() {
  if (drillClockTimer) {
    window.clearInterval(drillClockTimer)
    drillClockTimer = null
  }
}

function syncExecutionTimelineClock(execution = taskMapExecution.value) {
  if (drillTimelineMode.value !== 'execution') return
  const createdAt = Date.parse(execution?.created_at || execution?.started_at || '')
  const startedAt = routePreviewRequestedAt.value
    || (Number.isFinite(createdAt) ? createdAt : Date.now())
  drillStartedAt = startedAt
  const finishedAt = Date.parse(execution?.finished_at || '')
  const terminal = execution && !taskExecutionIsActive(execution)
  const endedAt = terminal && Number.isFinite(finishedAt) ? finishedAt : Date.now()
  drillElapsedSeconds.value = Math.max(0, (endedAt - startedAt) / 1000)
  if (terminal || routePreviewError.value) stopDrillClock()
  else startDrillClock()
}

function openExecutionTimeline(execution = null, requestedAt = Date.now()) {
  if (drillRunning.value) stopDrill(false)
  drillTimelineMode.value = 'execution'
  drillTimelineOpen.value = true
  routePreviewRequestedAt.value = Number(requestedAt) || Date.now()
  routePreviewError.value = ''
  syncExecutionTimelineClock(execution)
}

function clearDisplayedDrillTimeline() {
  if (timelineRunning.value) return
  if (drillTimelineMode.value === 'execution') {
    routePreviewRequestedAt.value = 0
    routePreviewError.value = ''
    drillTimelineMode.value = 'simulation'
    drillElapsedSeconds.value = 0
    drillStartedAt = null
    stopDrillClock()
    drillTimelineOpen.value = false
    return
  }
  clearDrillTimeline()
}

function resetDrillTimelineView() {
  if (drillRunning.value) stopDrill(false)
  routePreviewRequestedAt.value = 0
  routePreviewError.value = ''
  drillTimelineMode.value = 'simulation'
  drillTimeline.value = []
  drillTimelineOpen.value = false
  drillElapsedSeconds.value = 0
  drillStartedAt = null
  stopDrillClock()
}

function clearDrillTimeline() {
  if (drillRunning.value) return
  drillTimeline.value = []
  drillTimelineOpen.value = false
  drillElapsedSeconds.value = 0
  drillMessage.value = ''
  drillStartedAt = null
}

function drillSegmentMetrics(fromPoint, toPoint) {
  const from = normalizeStoredWaypoint(fromPoint)
  const to = normalizeStoredWaypoint(toPoint)
  const distance = Math.hypot(to.x - from.x, to.y - from.y)
  const duration = Math.max(1000, Math.min(3200, distance * 550))
  return { from, to, distance, duration, speed: distance / (duration / 1000) }
}

function stopDrill(showMessage = true) {
  const wasRunning = drillRunning.value
  drillCancelled = true
  if (drillAnimationFrame) {
    cancelAnimationFrame(drillAnimationFrame)
    drillAnimationFrame = null
  }
  if (drillAudio) {
    drillAudio.pause()
    drillAudio = null
  }
  if (drillAudioResolve) {
    drillAudioResolve()
    drillAudioResolve = null
  }
  window.speechSynthesis?.cancel()
  drillRunning.value = false
  drillCurrentSpeed.value = 0
  drillPosition.value = null
  drillCurrentIndex.value = null
  if (wasRunning && showMessage) {
    drillMessage.value = '演练已停止'
    recordDrillEvent('stop', '演练停止', '用户手动停止演练', { speed: 0 })
  }
  stopDrillClock()
}

function animateDrillSegment(fromPoint, toPoint) {
  const { from, to, duration } = drillSegmentMetrics(fromPoint, toPoint)
  return new Promise((resolve) => {
    const startedAt = performance.now()
    const tick = (now) => {
      if (drillCancelled) {
        resolve(false)
        return
      }
      const progress = Math.min(1, (now - startedAt) / duration)
      const eased = progress < 0.5 ? 2 * progress * progress : 1 - ((-2 * progress + 2) ** 2) / 2
      drillPosition.value = {
        frame_id: 'map',
        x: from.x + (to.x - from.x) * eased,
        y: from.y + (to.y - from.y) * eased,
        yaw: Math.atan2(to.y - from.y, to.x - from.x),
      }
      if (progress >= 1) {
        drillAnimationFrame = null
        resolve(true)
        return
      }
      drillAnimationFrame = requestAnimationFrame(tick)
    }
    drillAnimationFrame = requestAnimationFrame(tick)
  })
}

function fallbackBrowserSpeech(text) {
  return new Promise((resolve) => {
    if (!window.speechSynthesis || drillCancelled) {
      resolve()
      return
    }
    const utterance = new SpeechSynthesisUtterance(text)
    utterance.lang = 'zh-CN'
    utterance.rate = 0.95
    utterance.onend = resolve
    utterance.onerror = resolve
    window.speechSynthesis.speak(utterance)
  })
}

async function playDrillSpeech(point, index) {
  const text = String(point.speech_text || '').trim()
  if (!text || drillCancelled) return
  drillMessage.value = `到达点${index + 1}，正在播报：${point.speech_template_name || text}`
  recordDrillEvent('speech', `点${index + 1} 开始播报`, text, {
    pointIndex: index,
    pointName: waypointNames.value[index] || `点${index + 1}`,
    speechTitle: point.speech_template_name || '',
    speed: 0,
  })
  try {
    const result = await synthesizeSpeech(text)
    if (drillCancelled) return
    drillAudio = new Audio(result.audio_url)
    await drillAudio.play()
    await new Promise((resolve) => {
      const finish = () => {
        drillAudioResolve = null
        resolve()
      }
      drillAudioResolve = finish
      drillAudio.addEventListener('ended', finish, { once: true })
      drillAudio.addEventListener('error', finish, { once: true })
    })
    drillAudio = null
  } catch (_error) {
    await fallbackBrowserSpeech(text)
  }
  if (!drillCancelled) {
    recordDrillEvent('speech-end', `点${index + 1} 播报完成`, point.speech_template_name || text, {
      pointIndex: index,
      speed: 0,
    })
  }
}

async function startDrill() {
  if (drillRunning.value) {
    stopDrill()
    return
  }
  if (waypoints.value.length < 2) {
    alert('演练至少需要起点和终点两个途经点')
    return
  }
  drillTimelineMode.value = 'simulation'
  routePreviewRequestedAt.value = 0
  routePreviewError.value = ''
  drillCancelled = false
  drillTimeline.value = []
  drillTimelineOpen.value = true
  drillEventSequence = 0
  drillStartedAt = Date.now()
  drillElapsedSeconds.value = 0
  drillCurrentSpeed.value = 0
  startDrillClock()
  drillRunning.value = true
  drillCurrentIndex.value = 0
  drillPosition.value = normalizeStoredWaypoint(waypoints.value[0])
  drillMessage.value = '演练开始：机器狗位于起点'
  recordDrillEvent('start', '演练开始', `起点：${waypointNames.value[0] || '点1'}`, {
    pointIndex: 0,
    pointName: waypointNames.value[0] || '点1',
    speed: 0,
  })
  recordDrillEvent('arrival', '到达点1', waypointDisplayText(waypoints.value[0]), {
    pointIndex: 0,
    pointName: waypointNames.value[0] || '点1',
    speed: 0,
  })
  await playDrillSpeech(waypoints.value[0], 0)
  for (let index = 1; index < waypoints.value.length && !drillCancelled; index += 1) {
    drillMessage.value = `正在前往点${index + 1}`
    const metrics = drillSegmentMetrics(waypoints.value[index - 1], waypoints.value[index])
    drillCurrentSpeed.value = metrics.speed
    recordDrillEvent(
      'move',
      `前往点${index + 1}`,
      `距离 ${metrics.distance.toFixed(2)} m，模拟速度 ${metrics.speed.toFixed(2)} m/s`,
      {
        pointIndex: index,
        pointName: waypointNames.value[index] || `点${index + 1}`,
        distance: metrics.distance,
        speed: metrics.speed,
      },
    )
    const completed = await animateDrillSegment(waypoints.value[index - 1], waypoints.value[index])
    if (!completed) return
    drillCurrentSpeed.value = 0
    drillCurrentIndex.value = index
    drillPosition.value = normalizeStoredWaypoint(waypoints.value[index])
    recordDrillEvent('arrival', `到达点${index + 1}`, waypointDisplayText(waypoints.value[index]), {
      pointIndex: index,
      pointName: waypointNames.value[index] || `点${index + 1}`,
      speed: 0,
    })
    await playDrillSpeech(waypoints.value[index], index)
  }
  if (!drillCancelled) {
    drillRunning.value = false
    drillCurrentSpeed.value = 0
    drillMessage.value = '演练完成：机器狗已到达终点'
    recordDrillEvent('complete', '演练完成', `终点：${waypointNames.value.at(-1) || `点${waypoints.value.length}`}`, { speed: 0 })
    drillElapsedSeconds.value = (Date.now() - drillStartedAt) / 1000
    stopDrillClock()
  }
}

function buildRoutePayload() {
  return {
    name: String(routeForm.value.name || '').trim(),
    map_data: selectedMap.value.id,
    map_set: routeForm.value.map_set || null,
    robot: routeForm.value.robot || selectedMap.value.robot || 1,
    waypoints: withWaypointYaw(waypoints.value).map((point, index) => ({
      ...point,
      map_point_number: index + 1,
      localization_mode: mapIsLocalOnly.value && normalizeWaypointLocalizationMode(point.localization_mode) === 'rtk'
        ? 'ndt'
        : normalizeWaypointLocalizationMode(point.localization_mode),
    })),
    waypoint_names: waypointNames.value,
    description: routeForm.value.description,
    scene_scope: mapIsLocalOnly.value ? 'indoor' : (routeForm.value.scene_scope || selectedMap.value.scene_scope || 'indoor'),
    global_controller: DEFAULT_GLOBAL_CONTROLLER,
    record_rosbag: Boolean(routeForm.value.record_rosbag),
  }
}

function validateRouteDraft({ createOnly = false } = {}) {
  if (!selectedMap.value || waypoints.value.length === 0) {
    alert('请选择地图并添加途经点')
    return false
  }
  const pendingYawIndex = waypointYawConfirmed.value.findIndex(confirmed => !confirmed)
  if (pendingYawIndex >= 0) {
    alert(`请先确认点${pendingYawIndex + 1}的方向`)
    return false
  }
  const name = String(routeForm.value.name || '').trim()
  if (!name) {
    alert('请输入路线名称')
    return false
  }
  const duplicate = routes.value.find(route => (
    String(route.name || '').trim() === name
    && (createOnly || !selectedRoute.value?.id || String(route.id) !== String(selectedRoute.value.id))
  ))
  if (duplicate) {
    alert('路线名称已存在，请修改名称后再新建')
    return false
  }
  return true
}

async function persistRoute({ createOnly = false } = {}) {
  if (!validateRouteDraft({ createOnly })) return

  const traceId = newPlannerTraceId()
  const payload = buildRoutePayload()

  let savedRoute
  try {
    savedRoute = !createOnly && selectedRoute.value?.id
      ? await updateRoute(selectedRoute.value.id, payload, { traceId })
      : await createRoute(payload, { traceId })
    await loadData()
    const savedSummary = routes.value.find(route => String(route.id) === String(savedRoute.id))
    await handleLoadRoute(savedSummary || savedRoute)
  } catch (error) {
    console.error('保存失败:', error)
    const errorCode = Array.isArray(error?.payload?.code) ? error.payload.code[0] : error?.payload?.code
    alert(errorCode === 'ROUTE_NAME_EXISTS'
      ? '路线名称已存在，请修改名称后再新建'
      : (error.message || '保存失败'))
    return
  }

  navCommandBusy.value = 'map-activate'
  navError.value = ''
  localizationInitState.value = 'waiting_convergence'
  localizationInitMessage.value = '路线已保存，正在准备地图、定位与导航栈'
  beginLocalizationAttemptSession({ phase: 'transfer', commandType: 'map.activate' })
  try {
    const result = await activateAndRelocalizeMap({
      mapId: savedRoute.map_data,
      robotId: savedRoute.robot,
      sceneScope: routeForm.value.scene_scope || selectedMap.value?.scene_scope || 'indoor',
      coordinateMode: selectedMap.value?.coordinate_mode || 'local_only',
      waypoints: payload.waypoints,
      traceId,
      onProgress: message => { localizationInitMessage.value = message },
      onCommand: event => applyLocalizationAttemptCommand(event.command, event),
    })
    navStatus.value = result.navigationStatus
    localizationInitState.value = 'done'
    localizationInitMessage.value = '路线已保存并选中，地图、定位与导航栈均已就绪'
    alert('路线保存成功，地图、定位与导航栈均已就绪')
  } catch (error) {
    localizationInitState.value = 'failed'
    localizationInitMessage.value = `路线已保存，但导航准备失败：${error.message || '未知错误'}`
    navError.value = localizationInitMessage.value
    alert(localizationInitMessage.value)
  } finally {
    navCommandBusy.value = ''
    await refreshNavigationStatus()
  }
}

async function handleSaveRoute() {
  await persistRoute({ createOnly: false })
}

async function handleCreateRoute() {
  await persistRoute({ createOnly: true })
}

async function handleLoadRoute(route) {
  const routeSummary = { ...route }
  const loadSequence = ++routeLoadSequence
  routeLoading.value = true
  routeLoadError.value = ''

  let hydratedRoute
  try {
    const routeDetail = routeSummary.id
      ? await fetchRouteDetail(routeSummary.id)
      : routeSummary
    if (loadSequence !== routeLoadSequence) return
    if (!Array.isArray(routeDetail?.waypoints) || routeDetail.waypoints.length === 0) {
      const expected = routeSummary.waypoint_count
      if (typeof expected === 'number' && expected > 0) {
        throw new Error(`路线摘要为 ${expected} 个点，详情未返回途经点数据`)
      }
      if (!Array.isArray(routeDetail?.waypoints)) {
        throw new Error('路线详情未返回途经点数据')
      }
    }
    if (typeof routeSummary.waypoint_count === 'number'
      && routeSummary.waypoint_count !== routeDetail.waypoints.length) {
      throw new Error(`路线摘要为 ${routeSummary.waypoint_count} 个点，详情返回 ${routeDetail.waypoints.length} 个点`)
    }
    hydratedRoute = {
      ...routeSummary,
      ...routeDetail,
      latest_execution: routeSummary.latest_execution || routeDetail.latest_execution || null,
    }
  } catch (error) {
    if (loadSequence === routeLoadSequence) {
      routeLoadError.value = `路线“${routeSummary.name || routeSummary.id || ''}”加载失败：${error.message || '未知错误'}`
    }
    return
  } finally {
    if (loadSequence === routeLoadSequence) routeLoading.value = false
  }

  if (loadSequence !== routeLoadSequence) return
  if (String(selectedRoute.value?.id || '') !== String(hydratedRoute.id || '')) {
    resetDrillTimelineView()
  }
  selectedRoute.value = hydratedRoute
  lastExecution.value = hydratedRoute.latest_execution || null
  const routeGlobalController = normalizeGlobalController(hydratedRoute.global_controller)
  let routeMap = maps.value.find(m => String(m.id) === String(hydratedRoute.map_data))
    || (String(selectedMap.value?.id || '') === String(hydratedRoute.map_data) ? selectedMap.value : null)
  if (!routeMap && hydratedRoute.map_data) {
    try {
      routeMap = await fetchMapDetail(hydratedRoute.map_data)
    } catch (error) {
      console.warn('加载路线关联地图失败:', error)
    }
  }
  if (loadSequence !== routeLoadSequence) return
  const mapChanged = String(routeMap?.id) !== String(selectedMap.value?.id)
  selectedMap.value = routeMap
  waypoints.value = hydratedRoute.waypoints.map(point => {
    const normalizedPoint = Array.isArray(point)
      ? { x: point[0], y: point[1], yaw: point[2] || 0 }
      : { ...point }
    return {
      ...normalizedPoint,
      global_controller: normalizeGlobalController(normalizedPoint.global_controller || routeGlobalController),
    }
  })
  resetWaypointExpansion()
  waypointNames.value = hydratedRoute.waypoint_names?.length
    ? [...hydratedRoute.waypoint_names]
    : waypoints.value.map((_, index) => `点${index + 1}`)
  resetWaypointYawEditors()
  routeForm.value.name = hydratedRoute.name
  routeForm.value.description = hydratedRoute.description
  routeForm.value.robot = hydratedRoute.robot || robots.value[0]?.id || null
  routeForm.value.map_data = hydratedRoute.map_data
  routeForm.value.map_set = hydratedRoute.map_set || null
  routeForm.value.scene_scope = hydratedRoute.scene_scope || routeMap?.scene_scope || 'indoor'
  routeForm.value.record_rosbag = Boolean(hydratedRoute.record_rosbag)
  if (mapChanged) clearInspectedMapPoints()
  await nextTick()
  const [, detailedMap] = await Promise.all([
    mapChanged ? loadMappingTrace(routeMap) : Promise.resolve(),
    hydrateSelectedMap(routeMap),
  ])
  routeMap = detailedMap || routeMap
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

  const speechFields = {
    speech_template_id: point.speech_template_id || null,
    speech_template_name: point.speech_template_name || '',
    speech_text: point.speech_text || '',
    localization_mode: normalizeWaypointLocalizationMode(point.localization_mode),
    local_controller: normalizeLocalController(point.local_controller),
    global_controller: normalizeGlobalController(point.global_controller || DEFAULT_GLOBAL_CONTROLLER),
    arrival_policy: normalizeArrivalPolicy(point.arrival_policy, point),
    avoidance_to_next: point.avoidance_to_next !== false,
    detour_enabled: point.detour_enabled !== false && point.avoidance_to_next !== false,
    collision_slowdown_enabled: point.collision_slowdown_enabled !== false && point.avoidance_to_next !== false,
    collision_stop_enabled: point.collision_stop_enabled !== false,
    require_yaw: point.require_yaw === true,
    dwell_seconds: Math.max(0, Number(point.dwell_seconds || 0)),
    speech_mode: normalizeSpeechMode(point.speech_mode, point),
  }
  const normalized = {
    x: Number(point.x),
    y: Number(point.y),
    yaw: Number(point.yaw || 0),
    frame_id: point.frame_id || 'map',
  }
  if (Number.isFinite(Number(point.image_x)) && Number.isFinite(Number(point.image_y)) && geometry) {
    return {
      ...imagePointToWaypoint(
      { imageX: Number(point.image_x), imageY: Number(point.image_y) },
      geometry,
      normalized.yaw,
      ),
      ...speechFields,
    }
  }
  if (Number.isFinite(Number(point.u)) && Number.isFinite(Number(point.v)) && geometry) {
    return {
      ...imagePointToWaypoint(
      { imageX: Number(point.u) * geometry.mapWidth, imageY: Number(point.v) * geometry.mapHeight },
      geometry,
      normalized.yaw,
      ),
      ...speechFields,
    }
  }
  return {
    ...normalized,
    ...mapPointToImagePoint(normalized.x, normalized.y, geometry),
    ...speechFields,
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
      speech_template_id: current.speech_template_id || null,
      speech_template_name: current.speech_template_name || '',
      localization_mode: normalizeWaypointLocalizationMode(current.localization_mode),
      local_controller: normalizeLocalController(current.local_controller),
      global_controller: normalizeGlobalController(current.global_controller || DEFAULT_GLOBAL_CONTROLLER),
      arrival_policy: normalizeArrivalPolicy(current.arrival_policy, current),
      avoidance_to_next: current.avoidance_to_next !== false,
      detour_enabled: current.detour_enabled !== false && current.avoidance_to_next !== false,
      collision_slowdown_enabled: current.collision_slowdown_enabled !== false && current.avoidance_to_next !== false,
      collision_stop_enabled: current.collision_stop_enabled !== false,
      require_yaw: current.require_yaw === true,
      dwell_seconds: Math.max(0, Number(current.dwell_seconds || 0)),
      speech_mode: normalizeSpeechMode(current.speech_mode, current),
      speech_text: current.speech_text || '',
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

function originPoseText(origin, xKey = 'x', yKey = 'y') {
  if (!origin) return '未配置'
  const x = Number(origin[xKey])
  const y = Number(origin[yKey])
  if (!Number.isFinite(x) || !Number.isFinite(y)) return '未配置'
  return `map (${x.toFixed(3)}, ${y.toFixed(3)})`
}

function rtkOriginTitle() {
  const origin = rtkEnuOrigin.value
  if (!origin) return '当前地图未锁定 RTK ENU 原点'
  const latitude = Number(origin.latitude)
  const longitude = Number(origin.longitude)
  return `RTK ENU 原点 · ${latitude.toFixed(8)}, ${longitude.toFixed(8)} · ${originPoseText(origin, 'map_x', 'map_y')}`
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

function mappingTracePoints() {
  imageReadyTick.value
  const geometry = getMapGeometry()
  if (!geometry) return ''
  return mappingTrace.value
    .map(sample => pointDisplayPositionFromMap(sample.slam?.x, sample.slam?.y, geometry))
    .filter(Boolean)
    .map(point => `${point.x},${point.y}`)
    .join(' ')
}

function globalPlanPolylinePoints() {
  imageReadyTick.value
  const geometry = getMapGeometry()
  if (!geometry) return ''
  return globalPlanPoints.value
    .map(point => pointDisplayPositionFromMap(Number(point.x), Number(point.y), geometry))
    .filter(Boolean)
    .map(point => `${point.x},${point.y}`)
    .join(' ')
}

function nearestMappingSample(point) {
  const x = Number(point?.x)
  const y = Number(point?.y)
  if (!Number.isFinite(x) || !Number.isFinite(y) || !mappingTrace.value.length) return null
  let nearest = null
  let nearestDistance = Number.POSITIVE_INFINITY
  for (const sample of mappingTrace.value) {
    const sx = Number(sample.slam?.x)
    const sy = Number(sample.slam?.y)
    if (!Number.isFinite(sx) || !Number.isFinite(sy)) continue
    const distance = Math.hypot(sx - x, sy - y)
    if (distance < nearestDistance) {
      nearest = sample
      nearestDistance = distance
    }
  }
  return nearest ? { ...nearest, distance_m: nearestDistance } : null
}

const waypointMappingSamples = computed(() => waypoints.value.map(point => nearestMappingSample(point)))

function poseText(pose) {
  if (!pose || !Number.isFinite(Number(pose.x)) || !Number.isFinite(Number(pose.y))) return '无记录'
  const yaw = Number.isFinite(Number(pose.yaw)) ? `${waypointYawDegrees(pose).toFixed(1)} deg` : '无方向'
  return `${Number(pose.x).toFixed(2)}, ${Number(pose.y).toFixed(2)} / ${yaw}`
}

function rtkPoseText(rtk) {
  const pose = poseText(rtk)
  if (pose === '无记录') return rtk?.reason === 'not_recorded' ? '旧地图无记录' : '无有效记录'
  const quality = { 2: '固定解', 1: '浮点解', 0: '单点解' }[Number(rtk.status)] || '状态未知'
  const precision = Number.isFinite(Number(rtk.horizontal_std_m)) ? ` / 精度 ${Number(rtk.horizontal_std_m).toFixed(2)}m` : ''
  return `${pose} / ${quality}${precision}`
}

function mappingSampleTime(sample) {
  const stamp = Number(sample?.stamp)
  if (!Number.isFinite(stamp) || stamp <= 0) return '旧地图无时间记录'
  return new Date(stamp * 1000).toLocaleString()
}

function waypointHeadingStyle(point) {
  return { transform: `rotate(${-Number(point?.yaw || 0)}rad)` }
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

function inspectedHeadingLinePoints() {
  imageReadyTick.value
  const geometry = getMapGeometry()
  if (!geometry || !inspectedMapPoint.value || !inspectedHeadingTarget.value) return ''
  const start = pointDisplayPositionFromMap(inspectedMapPoint.value.point.x, inspectedMapPoint.value.point.y, geometry)
  const end = pointDisplayPositionFromMap(inspectedHeadingTarget.value.x, inspectedHeadingTarget.value.y, geometry)
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

  const localizationStatus = status.localization_status || navStatus.value?.localization_status || 'unknown'
  lastLocalizationStatus.value = localizationStatus

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

async function refreshNavigationStatus({ signal } = {}) {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot || 1
  if (!robotId || navigationStatusRefreshing) return
  navigationStatusRefreshing = true
  try {
    const [status, navigation] = await Promise.all([
      fetchRobotStatus(robotId, { signal }),
      fetchRobotNavigationStatus(robotId, { signal }),
    ])
    navStatus.value = {
      ...(navigation || {}),
      status: status?.status || navigation?.status || null,
      connection_status: status?.connection_status || navigation?.connection_status,
      last_seen_at: status?.last_seen_at || navigation?.last_seen_at,
    }
    await refreshTaskMapExecution()
    recordPoseSample()
    restoreAttemptSessionFromStatus()
    refreshImageGeometry()
  } catch (error) {
    if (error?.name === 'AbortError') return
    navError.value = error.message || '导航状态获取失败'
  } finally {
    navigationStatusRefreshing = false
  }
}

async function refreshTaskMapExecution() {
  const executionId = navStatus.value?.status?.task_execution_id || lastExecution.value?.id
  if (!executionId) {
    taskMapExecution.value = null
    taskMapTrajectory.value = []
    return
  }
  try {
    const [detail, track] = await Promise.all([
      fetchTaskExecution(executionId),
      fetchTaskTrajectory(executionId),
    ])
    taskMapExecution.value = detail
    taskMapTrajectory.value = track.points || []
    const executionMatchesRoute = !detail?.route
      || !selectedRoute.value?.id
      || String(detail.route) === String(selectedRoute.value.id)
    if (taskExecutionIsActive(detail) && executionMatchesRoute && !drillRunning.value) {
      if (drillTimelineMode.value !== 'execution') {
        const createdAt = Date.parse(detail.created_at || detail.started_at || '')
        openExecutionTimeline(detail, Number.isFinite(createdAt) ? createdAt : Date.now())
      } else {
        syncExecutionTimelineClock(detail)
      }
    } else if (drillTimelineMode.value === 'execution') {
      syncExecutionTimelineClock(detail)
    }
  } catch {
    // Navigation status remains useful even if a historical execution was removed.
  }
}

async function sendNavigationCommand(action) {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot
  if (!robotId) {
    navError.value = '未选择机器人'
    return
  }
  if (navCommandBusy.value) return
  navCommandBusy.value = action
  const traceId = newPlannerTraceId()
  navError.value = ''
  try {
    const command = await sendRobotNavigationCommand(robotId, action, {
      map_id: selectedMap.value?.id,
      map_version: selectedMapVersion(),
    }, { traceId })
    await waitForRobotCommand(robotId, command, {
      timeoutMs: ['start', 'restart', 'recover'].includes(action) ? 240_000 : 90_000,
      onProgress: latest => {
        localizationInitMessage.value = `导航${action} · ${latest.status || 'created'}`
      },
    })
    await refreshNavigationStatus()
  } catch (error) {
    navError.value = error.message || '导航命令下发失败'
  } finally {
    navCommandBusy.value = ''
  }
}

async function handleRestartSensor(sensor) {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot
  if (!robotId) {
    navError.value = '未选择机器人'
    return
  }
  sensorCommandBusy.value = sensor
  navError.value = ''
  try {
    await restartRobotSensor(robotId, sensor)
    await sleep(3000)
    await refreshNavigationStatus()
  } catch (error) {
    navError.value = error.message || '传感器重启失败'
  } finally {
    sensorCommandBusy.value = ''
  }
}

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms))
}

function newPlannerTraceId() {
  return globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random().toString(16).slice(2)}`
}

function beginLocalizationAttemptSession({ phase = 'localization', commandType = '', commandId = '' } = {}) {
  const robotId = selectedRobot.value?.id
  if (robotId) clearStoredAttemptSession(robotId)
  localizationAttemptSession.value = emptyAttemptSession({ phase, commandType, commandId })
  localizationAttemptCardOpen.value = true
  scheduleAttemptMarkerRefresh(localizationAttemptSession.value)
}

function scheduleAttemptMarkerRefresh(session) {
  if (attemptMarkerTimer) {
    clearTimeout(attemptMarkerTimer)
    attemptMarkerTimer = null
  }
  const visibleUntil = Number(session?.markersVisibleUntil)
  if (!Number.isFinite(visibleUntil)) return
  const delay = visibleUntil - Date.now()
  if (delay <= 0) {
    attemptMarkerTick.value += 1
    return
  }
  attemptMarkerTimer = setTimeout(() => {
    attemptMarkerTick.value += 1
    attemptMarkerTimer = null
  }, delay + 50)
}

function registerRelocalizationMarker(pose, meta = {}) {
  const robotId = selectedRobot.value?.id
  if (!robotId || !robotMapMatches()) return
  relocalizationMarkers.value = appendRelocalizationMarker(relocalizationMarkers.value, pose, meta)
  writeStoredRelocalizationMarkers(robotId, relocalizationMarkers.value)
}

function restoreRelocalizationMarkers() {
  const robotId = selectedRobot.value?.id
  relocalizationMarkers.value = readStoredRelocalizationMarkers(robotId)
}

function maybeRegisterAttemptRelocalizationMarker(session) {
  if (!session || !isAttemptSessionTerminal(session) || !session.bestMatchPose || !session.optimalVerified) return
  const commandId = String(session.commandId || '')
  if (commandId && relocalizationMarkers.value.some(item => item.commandId === commandId)) return
  registerRelocalizationMarker(session.bestMatchPose, {
    source: session.source || session.commandType || '定位尝试',
    commandType: session.commandType || '',
    commandId,
    verification: session.rtkDrift?.verified
      ? `RTK漂移 ${formatAttemptMetric(session.rtkDrift.xy_m)}m`
      : `NDT ${formatAttemptMetric(session.bestNdtCandidate?.matching_error)} < 0.01`,
  })
}

function applyLocalizationAttemptCommand(command, extras = {}) {
  if (!command) return
  const robotId = selectedRobot.value?.id
  const previousSession = localizationAttemptSession.value || readStoredAttemptSession(robotId)
  const timelineHistory = previousSession
    ? localizationAttemptTimeline(previousSession)
    : []
  const session = localizationAttemptSessionFromCommand(command, {
    ...extras,
    timelineHistory,
  })
  if (!session) return
  localizationAttemptSession.value = session
  if (robotId) writeStoredAttemptSession(robotId, session)
  maybeRegisterAttemptRelocalizationMarker(session)
  scheduleAttemptMarkerRefresh(session)
}

function localizationAttemptProgressText(session) {
  const candidateCount = Number(session?.candidateCount || 0)
  const evaluated = Number(session?.evaluatedCandidateCount || 0)
  const activeAttempt = (session?.attempts || []).find(attempt => (
    ['verifying', 'started', 'running', 'executing', 'in_progress'].includes(attempt.status)
  ))
  const committingAttempt = (session?.attempts || []).find(attempt => attempt.status === 'committing')
  if (session?.globalSearchStarted) return '全局关键帧匹配阶段'
  if (session?.commandType === 'nav.initial_pose') {
    return session?.source === 'rtk' ? 'RTK 固定解验证阶段' : '正在验证手选初始位姿'
  }
  if (candidateCount <= 0) return '正在准备定位候选列表'
  if (activeAttempt) {
    return `正在尝试候选 #${activeAttempt.candidateNumber} · 已完成 ${evaluated} / ${candidateCount} 个候选`
  }
  if (committingAttempt) {
    return `正在提交候选 #${committingAttempt.candidateNumber} · 已完成 ${evaluated} / ${candidateCount} 个候选`
  }
  return `已完成 ${evaluated} / ${candidateCount} 个候选 · 原点/航点候选阶段`
}

function restoreAttemptSessionFromStatus() {
  if (navCommandBusy.value) return
  const robotId = selectedRobot.value?.id
  const command = navStatus.value?.localization_command
  if (command?.result_payload) {
    applyLocalizationAttemptCommand(command, {
      phase: 'localization',
      showCandidates: true,
    })
    return
  }
  const stored = readStoredAttemptSession(robotId)
  if (stored?.attempts?.length) {
    localizationAttemptSession.value = withAttemptMarkerExpiry(stored)
    scheduleAttemptMarkerRefresh(localizationAttemptSession.value)
  }
}

function relocalizationHeadingStyle(marker) {
  const yaw = Number(marker?.yaw || 0)
  return { transform: `translate(-50%, -100%) rotate(${yaw}rad)` }
}

function attemptMarkerPosition(attempt) {
  const pose = attemptMarkerPose(attempt)
  return pose ? waypointDisplayPosition(pose) : null
}

function applyInitialPoseOutcome(command, submittedPose = manualInitialPose.value) {
  const outcome = initialPoseCommandOutcome(command, submittedPose)
  if (outcome.pose) {
    manualInitialPose.value = {
      ...(manualInitialPose.value || {}),
      ...outcome.pose,
    }
    initialPoseHeadingTarget.value = null
  }
  return outcome
}

function initialPoseOutcomeMessage(prefix, outcome) {
  if (!outcome?.pose) return prefix
  const pose = outcome.pose
  const score = outcome.matchingError === null ? '' : ` · NDT ${outcome.matchingError.toFixed(3)}`
  const inlier = outcome.inlierFraction === null ? '' : ` · 内点 ${(outcome.inlierFraction * 100).toFixed(1)}%`
  const committed = outcome.bestNdtCommitted ? ' · 已提交最优候选' : ''
  return `${prefix}：x ${pose.x.toFixed(3)} / y ${pose.y.toFixed(3)} / yaw ${pose.yaw.toFixed(3)}${score}${inlier}${committed}`
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
  const traceId = newPlannerTraceId()
  localizationInitState.value = 'restarting'
  localizationInitMessage.value = '正在下发当前地图，并按地图类型重新初始化定位'
  beginLocalizationAttemptSession({ phase: 'transfer', commandType: 'map.activate' })
  navError.value = ''
  try {
    const initialization = await initializeProgressiveLocalization({
      mapId: selectedMap.value?.id,
      robotId,
      mapVersion: selectedMapVersion(),
      sceneScope: routeForm.value.scene_scope || selectedMap.value?.scene_scope || 'indoor',
      coordinateMode: selectedMap.value?.coordinate_mode || '',
      waypoints: waypoints.value.map(point => {
        const normalized = normalizeStoredWaypoint(point)
        return { x: Number(normalized.x), y: Number(normalized.y), yaw: Number(normalized.yaw || 0) }
      }),
      onProgress: message => { localizationInitMessage.value = message },
      onCommand: event => applyLocalizationAttemptCommand(event.command, event),
      dependencies: {
        activateRouteMap,
        sendRobotNavigationCommand,
        waitForRobotCommand,
      },
      traceId,
    })
    applyLocalizationAttemptCommand(initialization.command, { phase: 'localization', showCandidates: true })
    navStatus.value = initialization.activation.navigationStatus
    const completedLocalization = initialization.command
    const outcome = applyInitialPoseOutcome(completedLocalization)
    localizationInitState.value = 'waiting_convergence'
    localizationInitMessage.value = initialPoseOutcomeMessage('命令已完成，等待定位状态同步', outcome)
    for (let index = 0; index < 35; index += 1) {
      await sleep(2000)
      await refreshNavigationStatus()
      if (!localizationSampleStale() && localizationLabel() === 'normal' && !localizationQualityStale()) {
        localizationInitState.value = 'done'
        localizationInitMessage.value = initialPoseOutcomeMessage('定位初始化完成', outcome)
        return
      }
    }
    localizationInitState.value = 'failed'
    localizationInitMessage.value = '未看到新的定位/NDT上报，请检查定位节点是否启动'
    navError.value = localizationInitMessage.value
  } catch (error) {
    localizationInitState.value = 'failed'
    if (error?.command) applyLocalizationAttemptCommand(error.command, { phase: 'localization', showCandidates: true })
    const outcome = error?.command ? applyInitialPoseOutcome(error.command) : null
    localizationInitMessage.value = outcome?.bestNdtCommitted
      ? initialPoseOutcomeMessage('最优NDT位姿已提交，但FAST-LIO接管未完成', outcome)
      : localizationAttemptFailureMessage(
        localizationAttemptSession.value,
        error.message || '定位初始化失败',
      )
    navError.value = localizationInitMessage.value
  } finally {
    navCommandBusy.value = ''
  }
}

async function activeRelocalize() {
  const robotId = selectedRobot.value?.id || selectedMap.value?.robot
  if (!robotId) {
    navError.value = '未选择机器人'
    return
  }
  navCommandBusy.value = 'relocalize'
  const traceId = newPlannerTraceId()
  localizationInitState.value = 'waiting_convergence'
  localizationInitMessage.value = '正在按统一流程搜索定位候选'
  beginLocalizationAttemptSession({ phase: 'localization', commandType: 'nav.relocalize' })
  navError.value = ''
  try {
    const sceneScope = routeForm.value.scene_scope || selectedMap.value?.scene_scope || 'indoor'
    const coordinateMode = selectedMap.value?.coordinate_mode || ''
    const manuallySelected = manualInitialPose.value
      ? [{
        x: Number(manualInitialPose.value.x),
        y: Number(manualInitialPose.value.y),
        yaw: Number(manualInitialPose.value.yaw || 0),
      }]
      : []
    const payload = buildProgressiveLocalizationPayload({
      mapId: selectedMap.value?.id,
      mapVersion: selectedMapVersion(),
      waypoints: [...manuallySelected, ...waypoints.value],
      sceneScope,
      coordinateMode,
    })
    payload.seed_source = 'quick_then_global'
    payload.scene_scope = sceneScope
    payload.coordinate_mode = coordinateMode
    const command = await sendRobotNavigationCommand(robotId, 'relocalize', payload, { traceId })
    const completed = await waitForRobotCommand(robotId, command, {
      timeoutMs: progressiveLocalizationTimeoutMs(payload),
      onProgress: latest => {
        localizationInitMessage.value = `正在按原点/航点/全局顺序搜索定位候选 · ${latest.status || 'created'}`
        applyLocalizationAttemptCommand(latest, { phase: 'localization', showCandidates: true })
      },
    })
    applyLocalizationAttemptCommand(completed, { phase: 'localization', showCandidates: true })
    const outcome = applyInitialPoseOutcome(completed, manualInitialPose.value)
    for (let index = 0; index < 35; index += 1) {
      await sleep(2000)
      await refreshNavigationStatus()
      if (!localizationSampleStale() && localizationLabel() === 'normal' && !localizationQualityStale()) {
        localizationInitState.value = 'done'
        localizationInitMessage.value = initialPoseOutcomeMessage('主动重定位完成', outcome)
        return
      }
    }
    throw new Error('主动重定位未在限定时间内收敛')
  } catch (error) {
    localizationInitState.value = 'failed'
    if (error?.command) applyLocalizationAttemptCommand(error.command, { phase: 'localization', showCandidates: true })
    const outcome = error?.command ? applyInitialPoseOutcome(error.command) : null
    localizationInitMessage.value = outcome?.bestNdtCommitted
      ? initialPoseOutcomeMessage('最优NDT位姿已提交，但FAST-LIO接管未完成', outcome)
      : localizationAttemptFailureMessage(
        localizationAttemptSession.value,
        error.message || '主动重定位失败',
      )
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
  const batteryPercent = resolveBatteryPercent(null, selectedRobot.value)
  if (isLowBatteryBlocked(batteryPercent)) {
    navError.value = lowBatteryGuardMessage(batteryPercent)
    return
  }
  if (!confirm(`确定执行路线 "${selectedRoute.value.name}" 吗？请确认现场路径安全。`)) return
  openExecutionTimeline(null, Date.now())
  const traceId = newPlannerTraceId()
  taskMapExecution.value = null
  taskMapTrajectory.value = []
  lastExecution.value = null
  routeExecuteBusy.value = true
  navError.value = ''
  try {
    lastExecution.value = await executeRoute(selectedRoute.value.id, {
      recordRosbag: Boolean(routeForm.value.record_rosbag),
      traceId,
    })
    taskMapExecution.value = lastExecution.value
    taskMapTrajectory.value = []
    syncExecutionTimelineClock(lastExecution.value)
    await refreshNavigationStatus()
  } catch (error) {
    navError.value = error.message || '路线执行失败'
    routePreviewError.value = navError.value
    syncExecutionTimelineClock(null)
  } finally {
    routeExecuteBusy.value = false
  }
}

async function handleActivateSelectedMap() {
  const robotId = selectedRobot.value?.id
  if (!selectedMap.value?.id || !robotId || navCommandBusy.value) return
  navCommandBusy.value = 'map-activate'
  const traceId = newPlannerTraceId()
  navError.value = ''
  localizationInitState.value = 'waiting_convergence'
  localizationInitMessage.value = '正在检查机器狗地图'
  beginLocalizationAttemptSession({ phase: 'transfer', commandType: 'map.activate' })
  try {
    const result = await activateAndRelocalizeMap({
      mapId: selectedMap.value.id,
      robotId,
      mapVersion: selectedMapVersion(),
      sceneScope: routeForm.value.scene_scope || selectedMap.value.scene_scope || 'indoor',
      coordinateMode: selectedMap.value.coordinate_mode || 'local_only',
      waypoints: waypoints.value,
      traceId,
      onProgress: message => { localizationInitMessage.value = message },
      onCommand: event => applyLocalizationAttemptCommand(event.command, event),
    })
    navStatus.value = result.navigationStatus
    localizationInitState.value = 'done'
    localizationInitMessage.value = '地图下发完成，定位与导航已就绪'
  } catch (error) {
    localizationInitState.value = 'failed'
    localizationInitMessage.value = error.message || '地图下发失败'
    navError.value = localizationInitMessage.value
  } finally {
    navCommandBusy.value = ''
    await refreshNavigationStatus()
  }
}

function formatExecutionCreatedAt(value) {
  if (!value) return '未执行'
  const date = new Date(value)
  return Number.isNaN(date.getTime())
    ? value
    : date.toLocaleString('zh-CN', { hour12: false })
}

function routeExecutionOptionText(route) {
  const latest = route.latest_execution
  return latest ? ` · ${formatExecutionCreatedAt(latest.created_at)}` : ' · 未执行'
}

function toggleInitialPoseMode() {
  if (initialPoseMode.value) {
    initialPoseMode.value = false
    navError.value = ''
    return
  }
  resetInitialPosePosition()
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
  const submittedPose = {
    x: Number(manualInitialPose.value.x),
    y: Number(manualInitialPose.value.y),
    yaw: Number(manualInitialPose.value.yaw || 0),
  }
  navError.value = ''
  localizationInitState.value = 'sending_pose'
  localizationInitMessage.value = '正在下发初始种子并等待NDT最优位姿'
  const traceId = newPlannerTraceId()
  try {
    const command = await sendRobotNavigationCommand(robotId, 'initial-pose', {
      frame_id: 'map',
      ...submittedPose,
      map_id: selectedMap.value?.id,
      map_version: selectedMapVersion(),
    }, { traceId })
    const completed = await waitForRobotCommand(robotId, command, {
      timeoutMs: 120_000,
      onProgress: latest => {
        localizationInitMessage.value = `NDT初始定位 · ${latest.status || 'created'}`
        applyLocalizationAttemptCommand(latest, { phase: 'localization', showCandidates: true })
      },
    })
    const outcome = applyInitialPoseOutcome(completed, submittedPose)
    initialPoseMode.value = false
    localizationInitState.value = 'done'
    localizationInitMessage.value = initialPoseOutcomeMessage('初始定位已确认', outcome)
    await refreshNavigationStatus()
  } catch (error) {
    const outcome = error?.command ? applyInitialPoseOutcome(error.command, submittedPose) : null
    localizationInitState.value = 'failed'
    localizationInitMessage.value = outcome?.bestNdtCommitted
      ? initialPoseOutcomeMessage('最优NDT位姿已提交，但FAST-LIO接管未完成', outcome)
      : (error.message || '初始定位下发失败')
    navError.value = localizationInitMessage.value
  } finally {
    if (manageBusy && navCommandBusy.value === 'initial-pose') navCommandBusy.value = ''
  }
}

function robotMapMatches() {
  const mapId = navStatus.value?.status?.map_id || navStatus.value?.current_map_id
  if (!mapId || !selectedMap.value?.id) return true
  const mapVersion = navStatus.value?.status?.map_version || navStatus.value?.current_map_version
  return String(mapId) === String(selectedMap.value.id)
    && (!mapVersion || String(mapVersion) === selectedMapVersion())
}

function robotDisplayPosition() {
  imageReadyTick.value
  const geometry = getMapGeometry()
  const status = robotMapPoint()
  if (!status || !geometry) return null
  const point = mapPointToImagePoint(Number(status.x), Number(status.y), geometry)
  return {
    left: `${point.imageX * (geometry.rect.width / geometry.mapWidth)}px`,
    top: `${point.imageY * (geometry.rect.height / geometry.mapHeight)}px`,
  }
}

function robotMapPoint() {
  return currentRobotMapPose(navStatus.value, selectedMap.value?.id, localizationLossMarkers.value)
}

function robotHeadingStyle() {
  const yaw = Number(robotMapPoint()?.yaw || 0)
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - yaw}rad)` }
}

function robotMarkerTitle() {
  const point = robotMapPoint()
  if (!point) return '暂无定位'
  return `${point.trusted ? '机器狗当前定位' : '机器狗最新上报位置（定位不可信）'}\nx=${Number(point.x).toFixed(2)}, y=${Number(point.y).toFixed(2)}, yaw=${Number(point.yaw || 0).toFixed(3)}`
}

function lossHeadingStyle(point) {
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - Number(point?.yaw || 0)}rad)` }
}

function lossMarkerTitle(point) {
  const score = Number(point.quality?.matching_error)
  const target = point.waypoint?.map_point_number || (Number.isFinite(Number(point.waypointIndex)) ? Number(point.waypointIndex) + 1 : '—')
  return `第 ${point.sequence} 次定位丢失\n最后可信位置 x=${Number(point.x).toFixed(2)}, y=${Number(point.y).toFixed(2)}\n目标 ${target}号点 · NDT ${Number.isFinite(score) ? score.toFixed(3) : '—'}\n${localizationRecoveryLabel(point.recoveryState)}\n${formatDateTime(point.occurredAt)}`
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
  return formatClock(date)
}

function formatDateTimeWithAge(value) {
  if (!value) return '—'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return String(value)
  const ageSeconds = Math.max(0, Math.floor((Date.now() - date.getTime()) / 1000))
  if (ageSeconds < 60) return `${formatClock(date)} / ${ageSeconds}s前`
  return `${formatClock(date)} / ${Math.floor(ageSeconds / 60)}分钟前`
}

function formatClock(date) {
  const pad = value => String(value).padStart(2, '0')
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`
}

function sampleIsStale(value, thresholdMs = 10000) {
  if (!value) return false
  const sampledTime = new Date(value).getTime()
  if (Number.isNaN(sampledTime)) return false
  return Date.now() - sampledTime > thresholdMs
}

function localizationSampleStale() {
  const status = navStatus.value?.status || {}
  const quality = status.localization_quality || {}
  if (quality.localization_fresh === false) return true
  return sampleIsStale(
    quality.localization_sampled_at
    || quality.sampled_at
    || status.sampled_at,
  )
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

function localizationSourceLabel(source) {
  const labels = {
    lio_imu: 'LIO + IMU',
    ndt_imu: 'NDT + IMU',
    rtk_imu: 'RTK + IMU',
    imu_odom_bridge: 'IMU + 里程计桥接',
    unavailable: '无可用定位源',
  }
  return labels[source] || source || '决策数据未上报'
}

function localizationStatusCodeLabel(code) {
  const labels = {
    0: '初始化中',
    1: '全局重定位中',
    2: '重定位完成',
    3: '正常',
    4: '定位丢失',
  }
  if (code === null || code === undefined || code === '') return '—'
  return `${code}（${labels[Number(code)] || '未知状态'}）`
}

function sensorOnlineLabel(sensor) {
  if (!sensor) return '未上报'
  if (typeof sensor.online !== 'boolean') return '未上报'
  return sensor.online ? '在线' : '离线'
}

function sensorFrequencyLabel(sensor) {
  const hz = Number(sensor?.frequency_hz)
  return Number.isFinite(hz) ? `${hz.toFixed(1)} Hz` : '—'
}

function sensorTimeOffsetLabel(sensor) {
  const offset = Number(sensor?.measurement_time_offset_ms)
  if (!Number.isFinite(offset)) return '—'
  const validity = sensor?.measurement_time_valid === false ? '异常' : '正常'
  return `${offset.toFixed(1)} ms（${validity}）`
}

function rtkBlockedReasonLabel(reason) {
  const labels = {
    none: '无阻塞',
    gnss_fusion_disabled: 'GNSS融合未启用',
    gnss_origin_not_loaded: 'GNSS原点未加载',
    no_fix: '未收到有效定位解',
    quality_insufficient: '解质量不满足融合门限',
    position_stale: 'RTK位置数据过期',
    horizontal_error_exceeded: '水平误差超限',
    heading_unavailable: '双天线航向不可用',
    heading_stale: '双天线航向过期',
    time_invalid: 'RTK时间戳无效',
    not_initialized: '定位尚未初始化',
  }
  return labels[reason] || reason || '未上报'
}

function sensorAgeLabel(sensor) {
  const directAgeValue = Number(sensor?.sample_age_seconds ?? sensor?.age_seconds)
  const directAge = Number.isFinite(directAgeValue) && directAgeValue >= 0 ? directAgeValue : NaN
  const receivedUnix = Number(sensor?.received_at_unix)
  const receivedAt = Number.isFinite(receivedUnix) && receivedUnix > 946684800
    ? receivedUnix * 1000
    : Date.parse(sensor?.received_at || sensor?.sampled_at || '')
  const measurementStamp = Number(sensor?.measurement_stamp)
  const measurementTime = Number.isFinite(measurementStamp) && measurementStamp > 946684800
    ? measurementStamp * 1000
    : NaN
  const age = Number.isFinite(directAge)
    ? directAge
    : Number.isFinite(receivedAt)
      ? Math.max(0, (Date.now() - receivedAt) / 1000)
      : Number.isFinite(measurementTime)
        ? Math.max(0, (Date.now() - measurementTime) / 1000)
        : NaN
  if (!Number.isFinite(age)) return '无数据年龄'
  return age > 3 ? `数据过期 ${age.toFixed(1)} 秒` : `${age.toFixed(1)} 秒前`
}

function sensorMeasurementTimeLabel(sensor) {
  const rawStamp = sensor?.measurement_stamp ?? sensor?.heading?.measurement_stamp
  const stamp = Number(rawStamp)
  if (Number.isFinite(stamp) && stamp > 946684800) {
    return formatDateTimeWithAge(new Date(stamp * 1000).toISOString())
  }
  if (typeof rawStamp === 'string' && rawStamp) return formatDateTimeWithAge(rawStamp)
  return '未上报'
}

function odomTimeSourceLabel(decision) {
  if (decision?.odom_time_source === 'not_used' || decision?.odom_time_source === 'unavailable') {
    return '控制器里程计预测未启用'
  }
  if (decision?.odom_time_valid === false || decision?.odom_time_source === 'invalid') {
    return '时间戳异常/未同步'
  }
  if (decision?.odom_time_source === 'ros_reception_monotonic') {
    return 'ROS接收时间单调有效'
  }
  return decision?.odom_time_source || '未上报'
}

function predictionErrorBySource(quality, source) {
  const errors = Array.isArray(quality?.prediction_errors) ? quality.prediction_errors : []
  const error = errors.find(item => String(item.label || '').toLowerCase().includes(source))
  return error ? `${formatNumber(error.translation_m, 3)} m` : '—'
}

function localizationDecisionBasis(quality = localizationQuality()) {
  const decision = quality?.decision || {}
  const { rawRtk } = normalizeRoutePlannerTelemetry(navStatus.value?.status)
  const source = decision.active_source || ''
  if (!source) return '定位决策数据未上报。'
  const preferred = String(decision.preferred_source || 'ndt').toLowerCase()
  const correctionPolicy = String(decision.correction_policy || preferred).toUpperCase()
  const rtkUsable = decision.rtk_usable === true
  const rtkQuality = rtkQualityLabel(rawRtk.quality || decision.rtk_quality)
  const decisionQuality = rtkQualityLabel(decision.rtk_quality)
  const blockedReason = rtkBlockedReasonLabel(decision.rtk_blocked_reason)
  const decisionScore = Number(decision.ndt_score)
  const qualityScore = Number(quality?.matching_error)
  const score = Number.isFinite(decisionScore) ? decisionScore : qualityScore
  const ndtHealthy = typeof decision.ndt_healthy === 'boolean'
    ? decision.ndt_healthy
    : Number.isFinite(score) && score < 0.5 && quality?.has_converged !== false
  const ndtDetail = Number.isFinite(score)
    ? `NDT${ndtHealthy ? '健康' : '不健康'}（分数 ${score.toFixed(3)}）`
    : 'NDT质量未上报'
  const rtkDetail = `原始RTK ${rtkQuality}；融合${rtkUsable ? '可用' : '不可用'}（决策${decisionQuality}，${blockedReason}）`

  if (source === 'lio_imu') {
    const lioHealth = decision.lio_healthy === true
      ? 'LIO健康'
      : decision.lio_healthy === false ? 'LIO数据异常' : 'LIO状态未上报'
    const anchor = decision.lio_anchored === true ? '已完成地图锚定' : '等待地图锚定'
    const stable = decision.absolute_stable === true
      ? `定位稳定（${Number(decision.absolute_stable_samples || 0)}帧）`
      : `等待定位稳定（${Number(decision.absolute_stable_samples || 0)}帧）`
    const policyReady = decision.policy_source_ready
    let correction = `${correctionPolicy}校正源${policyReady === false ? '未就绪' : '已就绪'}`
    if (decision.correction_smoothing_active === true) {
      correction = `正在平滑应用${decision.correction_source || '外部'}修正`
    } else if (decision.correction_candidate_source && decision.correction_candidate_source !== 'none') {
      correction = `${correctionPolicy}已选择${String(decision.correction_candidate_source).toUpperCase()}校正候选`
    } else if (decision.ndt_drift_decision === 'suppressed_low_drift') {
      correction = '当前漂移较小，无需校正'
    }
    return `LIO + IMU主定位，${lioHealth}，${anchor}，${stable}；${ndtDetail}，${correction}；${rtkDetail}。`
  }
  if (source === 'rtk_imu') {
    return preferred === 'rtk'
      ? `RTK优先，${rtkDetail}；采用RTK + IMU。`
      : `NDT不健康，${rtkDetail}；切换RTK + IMU兜底。`
  }
  if (source === 'ndt_imu') {
    return preferred === 'rtk' && !rtkUsable
      ? `RTK优先但${rtkDetail}；${ndtDetail}，采用NDT + IMU。`
      : `NDT优先，${ndtDetail}；${rtkDetail}。`
  }
  if (source === 'imu_odom_bridge') {
    return `NDT不健康且${rtkDetail}；进入受限桥接 ${Number(decision.bridge_distance_m || 0).toFixed(2)}m / ${Number(decision.bridge_elapsed_s || 0).toFixed(1)}s。`
  }
  const rejection = decision.bridge_rejection_reason ? `桥接拒绝：${decision.bridge_rejection_reason}。` : ''
  if (source === 'unavailable') {
    return `${ndtDetail}，${rtkDetail}；暂无绝对定位源。${rejection}`
  }
  return `当前定位源 ${localizationSourceLabel(source)}；${ndtDetail}；${rtkDetail}。${rejection}`
}

function localizationDecisionTone(decision = {}) {
  const source = decision.active_source || ''
  if (['ndt_imu', 'rtk_imu'].includes(source)) return 'ok'
  if (source === 'lio_imu') {
    return decision.lio_healthy === true
      && decision.lio_anchored === true
      && decision.absolute_stable === true ? 'ok' : 'warn'
  }
  if (source === 'imu_odom_bridge') return 'warn'
  if (source === 'unavailable') return 'bad'
  return source ? 'warn' : 'unknown'
}

function ndtQualityLabel(quality = localizationQuality()) {
  if (!quality) return '无数据'
  if (localizationQualityStale(quality)) return '已过期'
  if (!ndtQualityValid(quality)) return '无效'
  const error = Number(quality.matching_error)
  if (!Number.isFinite(error)) return '无分数'
  if (error < 0.5 && quality.has_converged !== false) return '正常'
  return '偏差大'
}

function ndtQualityValid(quality = localizationQuality()) {
  if (!quality) return false
  const error = Number(quality.matching_error)
  const inlier = Number(quality.inlier_fraction)
  const translation = Number(quality.relative_translation_m)
  return Number.isFinite(error)
    && error < 0.5
    && (!Number.isFinite(inlier) || inlier >= 0.05)
    && (!Number.isFinite(translation) || translation < 20)
}

function ndtConvergedText(quality = localizationQuality()) {
  if (!quality) return '—'
  if (localizationQualityStale(quality)) return '已过期'
  return quality.has_converged && ndtQualityValid(quality) ? '是' : '否'
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
  const decision = quality?.decision || {}
  const ndtDrift = decision.ndt_drift || {}
  const initialization = decision.initialization || {}
  const globalRelocalization = decision.global_relocalization || {}
  const telemetry = normalizeRoutePlannerTelemetry(status)
  const sensors = telemetry.sensors
  const rtk = telemetry.rtk
  const rawRtk = telemetry.rawRtk
  const timeDiagnostics = telemetry.timeDiagnostics
  const imu = sensors.imu
  const odometry = sensors.odometry
  const qualityFresh = quality && !localizationQualityStale(quality)
  const hasRtkCoordinate = value => value !== null && value !== undefined && value !== '' && Number.isFinite(Number(value))
  const rtkMapPosition = hasRtkCoordinate(decision.rtk_x) && hasRtkCoordinate(decision.rtk_y)
    ? `${formatNumber(decision.rtk_x)} / ${formatNumber(decision.rtk_y)}`
    : '未形成地图坐标'
  const rtkMapHeading = hasRtkCoordinate(decision.rtk_yaw)
    ? `${formatNumber(decision.rtk_yaw)} rad`
    : '未形成航向'
  return [
    ['页面地图', selectedMap.value ? `${selectedMap.value.id} / ${selectedMap.value.name}` : '—'],
    ['机器人地图', `${status.map_id || navStatus.value?.current_map_id || '—'} / ${status.map_version || navStatus.value?.current_map_version || '—'}`],
    ['地图一致', mapMatchLabel()],
    ['定位状态', status.localization_status || navStatus.value?.localization_status || 'unknown'],
    ['定位状态码', localizationStatusCodeLabel(status.localization_source_status)],
    ['当前定位源', localizationSourceLabel(decision.active_source)],
    ['定位决策依据', localizationDecisionBasis(quality)],
    ['航点定位校正方式', String(decision.correction_policy || decision.preferred_source || 'ndt').toUpperCase()],
    ['校正源就绪', decision.policy_source_ready === true ? '是' : '否'],
    ['本次校正候选', String(decision.correction_candidate_source || 'none').toUpperCase()],
    ['NDT · 质量', ndtQualityLabel(quality)],
    ['NDT · 分数', qualityFresh ? formatNumber(quality.matching_error, 3) : (quality ? `已过期 ${formatNumber(quality.matching_error, 3)}` : '—')],
    ['NDT · 收敛', qualityFresh ? ndtConvergedText(quality) : (quality ? '已过期' : '—')],
    ['NDT · 内点率', qualityFresh ? formatNumber(quality.inlier_fraction, 3) : (quality ? `已过期 ${formatNumber(quality.inlier_fraction, 3)}` : '—')],
    ['NDT · 匹配位移', qualityFresh ? `${formatNumber(quality.relative_translation_m, 3)} m` : '—'],
    ['NDT · 漂移XY', `${formatNumber(ndtDrift.xy_m, 3)} m（dx ${formatNumber(ndtDrift.dx_m, 3)} / dy ${formatNumber(ndtDrift.dy_m, 3)}）`],
    ['NDT · 航向漂移', `${formatNumber(ndtDrift.yaw_deg, 2)}° / 阈值 ${formatNumber(ndtDrift.threshold_yaw_deg, 1)}°`],
    ['NDT · 漂移校正门', `${ndtDrift.decision || decision.ndt_drift_decision || '—'} · ${Number(ndtDrift.stable_frames || 0)}/${Number(ndtDrift.required_stable_frames || 3)}帧`],
    ['初始化 · 验证', `${initialization.state || '未上报'} · ${initialization.verified === true ? '已验证' : '未验证'} · ${Number(initialization.stable_frames || 0)}/${Number(initialization.required_stable_frames || 3)}帧`],
    ['初始化 · 航向匹配', `种子 ${formatNumber(initialization.seed_yaw_deg, 2)}° → 匹配 ${formatNumber(initialization.matched_yaw_deg, 2)}°（修正 ${formatNumber(initialization.yaw_correction_deg, 2)}°）`],
    ['全局重定位', `${globalRelocalization.mode || 'disabled'} / ${globalRelocalization.state || 'idle'}${globalRelocalization.keyframe >= 0 ? ` / KF ${globalRelocalization.keyframe}` : ''}`],
    ['IMU · 状态', sensorOnlineLabel(imu)],
    ['IMU · 频率', sensorFrequencyLabel(imu)],
    ['IMU · 时间偏差', sensorTimeOffsetLabel(imu)],
    ['IMU · 预测误差', qualityFresh ? predictionErrorBySource(quality, 'imu') : '—'],
    ['RTK · 状态', sensorOnlineLabel(rtk)],
    ['RTK · 原始解状态', rtkQualityLabel(rawRtk.quality || rtk?.quality)],
    ['RTK · 原始 fix 状态', rtkFixStatusLabel(rawRtk.fix_status)],
    ['RTK · 原始位置类型', rtkPositionTypeLabel(rawRtk.position_type)],
    ['RTK · 原始解算状态', rtkSolutionStatusLabel(rawRtk.solution_status)],
    ['RTK · 原始卫星数', rawRtk.solution_satellites === null || rawRtk.solution_satellites === undefined ? '—' : String(rawRtk.solution_satellites)],
    ['RTK · 定位决策解状态', rtkQualityLabel(decision.rtk_quality)],
    ['RTK · 融合可用', decision.rtk_usable === true || rtk?.fusion_usable === true ? '是' : '否'],
    ['RTK · 阻塞原因', rtkBlockedReasonLabel(decision.rtk_blocked_reason)],
    ['RTK · 地图坐标', rtkMapPosition],
    ['RTK · 航向', rtkMapHeading],
    ['RTK · 原始航向', rawRtk.heading?.heading_deg === null || rawRtk.heading?.heading_deg === undefined ? '—' : `${formatNumber(rawRtk.heading.heading_deg, 2)}° / std ${formatNumber(rawRtk.heading.heading_std_deg, 2)}°`],
    ['RTK · 原始测量时间', sensorMeasurementTimeLabel(rawRtk)],
    ['RTK · 原始数据年龄', sensorAgeLabel(rawRtk)],
    ['RTK · 水平误差', rtk?.horizontal_std_m === null || rtk?.horizontal_std_m === undefined ? '—' : `${formatNumber(rtk.horizontal_std_m, 2)} m`],
    ['RTK · 时间偏差', sensorTimeOffsetLabel(rtk)],
    ['里程计 · 状态', sensorOnlineLabel(odometry)],
    ['里程计 · 频率', sensorFrequencyLabel(odometry)],
    ['里程计 · 时间源', odomTimeSourceLabel(decision)],
    ['时间 · 激光→RTK', Number.isFinite(Number(timeDiagnostics.lidar_to_rtk_delta_ms)) ? `${formatNumber(timeDiagnostics.lidar_to_rtk_delta_ms, 1)} ms` : '—'],
    ['时间 · 激光→里程计', Number.isFinite(Number(timeDiagnostics.lidar_to_odom_delta_ms)) ? `${formatNumber(timeDiagnostics.lidar_to_odom_delta_ms, 1)} ms` : '—'],
    ['时间 · 总体状态', timeDiagnostics.all_time_valid === true ? '有效' : timeDiagnostics.warning || '未形成诊断'],
    ['里程计 · 预测误差', qualityFresh ? predictionErrorBySource(quality, 'odom') : '—'],
    ['桥接 · 状态', decision.active_source === 'imu_odom_bridge' ? '当前使用' : (imu?.online && odometry?.online ? '待命' : '不可用')],
    ['桥接 · 距离/时长', `${formatNumber(decision.bridge_distance_m, 2)} m / ${formatNumber(decision.bridge_elapsed_s, 1)} s`],
    ['桥接 · 拒绝原因', decision.bridge_rejection_reason || '无'],
    ['综合 · 预测误差', qualityFresh ? predictionErrorText(quality) : '—'],
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
  const decision = quality?.decision || {}
  const telemetry = normalizeRoutePlannerTelemetry(status)
  const sensors = telemetry.sensors
  const activeSource = decision.active_source || ''
  const mapMatches = robotMapMatches()
  const localizationStatus = status.localization_status || navStatus.value?.localization_status || 'unknown'
  const connection = navStatus.value?.connection_status || 'unknown'
  const command = navStatus.value?.command
  const taskId = status.task_execution_id
  const ndtError = Number(quality?.matching_error)
  const inlier = Number(quality?.inlier_fraction)
  const qualityStale = localizationQualityStale(quality)
  const goodNdt = !qualityStale && ndtQualityValid(quality) && Number.isFinite(ndtError) && ndtError < 0.5 && (!Number.isFinite(inlier) || inlier >= 0.65)
  const ndtHealthy = typeof decision.ndt_healthy === 'boolean' ? decision.ndt_healthy : goodNdt
  const rtkOnline = sensors.rtk?.online === true
  const rtkUsable = decision.rtk_usable === true || sensors.rtk?.fusion_usable === true
  const rawRtk = telemetry.rawRtk
  const bridgeReady = sensors.imu?.online === true && sensors.odometry?.online === true
  const bridgeRejected = Boolean(decision.bridge_rejection_reason)
  const hasRtkCoordinate = value => value !== null && value !== undefined && value !== '' && Number.isFinite(Number(value))
  const rtkPositionText = hasRtkCoordinate(decision.rtk_x) && hasRtkCoordinate(decision.rtk_y)
    ? `地图 ${formatNumber(decision.rtk_x)} / ${formatNumber(decision.rtk_y)}`
    : '地图坐标未形成'
  const rtkHeadingText = hasRtkCoordinate(decision.rtk_yaw)
    ? `航向 ${formatNumber(decision.rtk_yaw)} rad`
    : '航向未形成'

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
      detail: `状态码 ${localizationStatusCodeLabel(status.localization_source_status)} · ${robotPoseText()}`,
      state: localizationStatus === 'normal' ? 'ok' : localizationStatus === 'lost' ? 'bad' : 'warn',
    },
    {
      key: 'localization-decision',
      title: '定位决策',
      value: localizationSourceLabel(activeSource),
      detail: localizationDecisionBasis(quality),
      state: localizationDecisionTone(decision),
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
      value: activeSource === 'ndt_imu' ? '当前使用' : (ndtHealthy ? '健康待命' : ndtQualityLabel(quality)),
      detail: quality ? `score ${formatNumber(quality.matching_error, 3)} · inlier ${formatNumber(quality.inlier_fraction, 3)} · ${formatDateTimeWithAge(quality.sampled_at)}` : '未收到 /status 质量上报',
      state: ndtHealthy ? 'ok' : qualityStale ? 'warn' : quality ? 'bad' : 'warn',
    },
    {
      key: 'rtk',
      title: 'RTK定位',
      value: activeSource === 'rtk_imu' ? '当前使用' : rtkUsable ? '可用待命' : rtkOnline ? '不可用于融合' : '离线',
      detail: `原始${rtkQualityLabel(rawRtk.quality || sensors.rtk?.quality)} · ${rtkPositionTypeLabel(rawRtk.position_type)} · ${rtkSolutionStatusLabel(rawRtk.solution_status)} · 决策${rtkQualityLabel(decision.rtk_quality)} · ${rtkPositionText} · ${rtkHeadingText} · ${rtkBlockedReasonLabel(decision.rtk_blocked_reason)} · 水平误差 ${sensors.rtk?.horizontal_std_m === null || sensors.rtk?.horizontal_std_m === undefined ? '—' : `${formatNumber(sensors.rtk.horizontal_std_m, 2)} m`} · ${sensorAgeLabel(sensors.rtk)}`,
      state: activeSource === 'rtk_imu' || rtkUsable ? 'ok' : rtkOnline ? 'warn' : 'bad',
    },
    {
      key: 'bridge',
      title: 'IMU+里程计桥接',
      value: activeSource === 'imu_odom_bridge' ? '当前使用' : bridgeRejected ? '被拒绝' : bridgeReady ? '待命' : '不可用',
      detail: bridgeRejected
        ? decision.bridge_rejection_reason
        : `${formatNumber(decision.bridge_distance_m, 2)} m / ${formatNumber(decision.bridge_elapsed_s, 1)} s · ${decision.odom_time_source || '时间源未上报'}`,
      state: activeSource === 'imu_odom_bridge' ? 'warn' : bridgeRejected || !bridgeReady ? 'bad' : 'idle',
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
  const telemetry = normalizeRoutePlannerTelemetry(status)
  const sensors = telemetry.sensors
  const hasPose = status.x !== null && status.x !== undefined && status.y !== null && status.y !== undefined
  const navReady = Boolean(status.nav_ready)
  const sensorRow = (key, name, fallbackDetail, restartSensor = '') => {
    const sensor = sensors[key]
    if (!sensor) {
      return {
        key,
        name,
        value: '未上报',
        detail: `状态 未上报 · 数据年龄 未上报 · ${fallbackDetail}`,
        state: 'unknown',
        restartSensor,
      }
    }
    const hz = Number(sensor.frequency_hz)
    const age = Number(sensor.sample_age_seconds)
    if (key === 'rtk') {
      const modeLabels = {
        rtk_primary: 'RTK 主定位',
        hybrid: '融合定位',
        lidar_fallback: '激光兜底',
      }
      const std = Number(sensor.horizontal_std_m)
      const detail = `${sensorOnlineLabel(sensor)} · ${rtkQualityLabel(sensor.quality)} · ${rtkPositionTypeLabel(sensor.position_type)} · ${rtkSolutionStatusLabel(sensor.solution_status)} · 水平误差 ${Number.isFinite(std) ? `${std.toFixed(2)} m` : '—'} · ${Number.isFinite(hz) ? `${hz.toFixed(1)} Hz` : '—'} · 数据年龄 ${sensorAgeLabel(sensor)}`
      return {
        key,
        name,
        value: sensor.online ? `在线 · ${modeLabels[sensor.fusion_mode] || '质量未知'}` : '离线',
        detail,
        state: sensor.online && sensor.fusion_usable ? 'ok' : (sensor.online ? 'warn' : 'bad'),
        restartSensor: sensor.online ? '' : restartSensor,
      }
    }
    return {
      key,
      name,
      value: sensorOnlineLabel(sensor),
      detail: sensor.sampled_at || sensor.received_at || Number.isFinite(age)
        ? `${sensorOnlineLabel(sensor)} · ${Number.isFinite(hz) ? `${hz.toFixed(1)} Hz` : '—'} · 数据年龄 ${sensorAgeLabel(sensor)}`
        : `状态 ${sensorOnlineLabel(sensor)} · 数据年龄 ${sensorAgeLabel(sensor)} · ${fallbackDetail}`,
      state: sensor.online ? 'ok' : 'bad',
      restartSensor,
    }
  }

  return [
    sensorRow('lidar', '激光雷达 /front_lidar', '等待 Edge Agent 实时频率上报', 'lidar_imu'),
    sensorRow('imu', 'IMU /front_lidar/imu', '等待 Edge Agent 实时频率上报', 'lidar_imu'),
    sensorRow('odometry', '里程计 /odom/localization_odom', hasPose ? '定位里程计位姿已上报' : '等待定位里程计数据'),
    sensorRow('rtk', 'RTK/GNSS /fix', '未收到 GNSS 数据', 'rtk'),
    sensorRow('laser_scan', '避障扫描 /laser_scan', '点云转二维扫描链路无数据', 'lidar_imu'),
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
    localization_mode: 'ndt',
    local_controller: 'mppi',
    global_controller: DEFAULT_GLOBAL_CONTROLLER,
    avoidance_to_next: true,
    detour_enabled: true,
    collision_slowdown_enabled: true,
    collision_stop_enabled: true,
    require_yaw: false,
    dwell_seconds: 0,
    arrival_policy: 'stop_and_confirm',
    speech_mode: 'disabled',
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
    <section class="panel detail-panel route-planner-panel">
      <div class="route-planner-layout">
        <section class="panel-section route-config-panel">
          <div class="route-config-grid">
            <label>
              <span>选择地图</span>
              <select :value="selectedMap?.id || ''" @change="handleMapIdSelect($event.target.value)">
                <option value="">请选择地图</option>
                <option v-for="map in maps" :key="map.id" :value="map.id">
                  {{ map.name }} {{ map.active ? '(活动)' : '' }}
                </option>
              </select>
            </label>
            <label>
              <span>选择机器狗</span>
              <select v-model="routeForm.robot">
                <option :value="null">请选择机器狗</option>
                <option v-for="robot in robots" :key="robot.id" :value="robot.id">
                  {{ robot.name || robot.code }}{{ robot.code ? `（${robot.code}）` : '' }}
                </option>
              </select>
            </label>
            <label>
              <span>路线名称</span>
              <input v-model="routeForm.name" type="text" placeholder="输入路线名称" />
            </label>
            <label>
              <span>场景范围</span>
              <select v-model="routeForm.scene_scope" :disabled="mapIsLocalOnly">
                <option value="indoor">室内</option>
                <option value="transition" :disabled="mapIsLocalOnly">室内外过渡</option>
                <option value="outdoor" :disabled="mapIsLocalOnly">室外</option>
              </select>
            </label>
          </div>
          <p v-if="mapIsLocalOnly" class="empty-hint">当前地图无 RTK 原点，只能用于室内 NDT 定位，不能绑定室外或过渡区任务。</p>
        </section>

        <section class="panel-section route-drill-panel">
          <div class="route-drill-actions">
            <button
              class="btn drill-btn route-main-action"
              :class="{ running: drillRunning }"
              :disabled="!drillRunning && (!selectedMap || waypoints.length < 2)"
              @click="startDrill"
            >
              {{ drillRunning ? '■ 停止演练' : '▶ 演练开始' }}
            </button>
            <div v-if="selectedRoute" class="route-preview-actions">
              <button
                class="btn drill-btn route-preview-btn route-main-action"
                :disabled="routeExecuteBusy || navStatus?.connection_status !== 'online' || !navStatus?.status?.nav_ready"
                @click="handleExecuteRoute"
              >
                {{ routeExecuteBusy ? '■ 下发中...' : '▶ 预演' }}
              </button>
            </div>
          </div>
        </section>

        <!-- 途径点和路线信息 -->
        <div class="side-panel">
          <div class="route-waypoint-column">
            <div class="panel-section route-step-panel route-step-3">
              <div class="route-step-heading">
                <div>
                  <h3>途经点列表</h3>
                  <small>{{ waypoints.length }} 个点位</small>
                </div>
                <button type="button" class="route-step-toggle" :disabled="waypoints.length === 0" @click="toggleAllWaypoints">
                  {{ allWaypointsExpanded ? '全部收起' : '全部展开' }}
                </button>
              </div>
              <div class="route-step-content waypoint-panel-content">
                <div v-if="routeLoading" class="empty-hint">正在加载路线途经点...</div>
                <div v-if="routeLoadError" class="empty-hint waypoint-load-error">{{ routeLoadError }}</div>
                <div v-if="!routeLoading && !routeLoadError && waypoints.length === 0" class="empty-hint">点击地图添加途经点</div>
                <div v-if="waypoints.length > 0" class="waypoint-list">
                  <div v-for="(point, index) in waypoints" :key="index" class="waypoint-item">
                    <div class="waypoint-title-row">
                      <button type="button" class="waypoint-expand-toggle" @click="toggleWaypointExpanded(index)">
                        {{ isWaypointExpanded(index) ? '收起' : '展开' }}
                      </button>
                      <span>{{ waypointNames[index] }}: {{ waypointDisplayText(point) }}</span>
                      <button type="button" class="btn btn-sm btn-danger waypoint-delete-btn" @click="removeWaypoint(index)">删除</button>
                    </div>
                    <div v-if="isWaypointExpanded(index)" class="waypoint-main waypoint-details">
                      <div class="waypoint-pose-grid">
                        <small>NDT：{{ poseText(waypointMappingSamples[index]?.slam) }}</small>
                        <small>RTK：{{ rtkPoseText(waypointMappingSamples[index]?.rtk) }}</small>
                      </div>
                      <label class="waypoint-heading-row">
                        <span>方向</span>
                        <div class="waypoint-heading-input">
                          <input
                            type="number"
                            step="1"
                            inputmode="decimal"
                            :value="waypointYawDrafts[index]"
                            @input="setWaypointYawDraft(index, $event.target.value)"
                            @keydown.enter.prevent="confirmWaypointYaw(index)"
                          />
                          <span class="heading-unit">°</span>
                          <button
                            type="button"
                            class="btn btn-sm heading-confirm-btn"
                            :class="{ confirmed: waypointYawConfirmed[index] }"
                            @click="confirmWaypointYaw(index)"
                          >{{ waypointYawConfirmed[index] ? '已确认' : '确认' }}</button>
                        </div>
                      </label>
                      <small v-if="waypointYawErrors[index]" class="waypoint-field-error">{{ waypointYawErrors[index] }}</small>
                      <label class="waypoint-check">
                        <input type="checkbox" :checked="point.require_yaw === true" @change="setWaypointBoolean(index, 'require_yaw', $event.target.checked)" />
                        <span>到点转向</span>
                      </label>
                      <label>
                        <span>到点停留（秒）</span>
                        <input
                          type="number"
                          min="0"
                          max="3600"
                          step="0.5"
                          inputmode="decimal"
                          :value="point.dwell_seconds || 0"
                          @input="setWaypointDwell(index, $event.target.value)"
                        />
                      </label>
                      <label>
                        <span>到点策略</span>
                        <select
                          :value="point.arrival_policy || 'stop_and_confirm'"
                          @change="setWaypointArrivalPolicy(index, $event.target.value)"
                        >
                          <option v-for="option in ARRIVAL_POLICY_OPTIONS" :key="option.value" :value="option.value">
                            {{ option.label }}
                          </option>
                        </select>
                      </label>
                      <label v-if="index < waypoints.length - 1" class="waypoint-check">
                        <input type="checkbox" :checked="point.avoidance_to_next !== false" @change="setWaypointBoolean(index, 'avoidance_to_next', $event.target.checked)" />
                        <span>到下个点避障（绕行+减速；硬急停）</span>
                      </label>
                      <label>
                        <span>语音模式</span>
                        <select
                          :value="point.speech_mode || (point.speech_template_id ? 'non_blocking' : 'disabled')"
                          @change="waypoints[index] = { ...point, speech_mode: $event.target.value }"
                        >
                          <option value="blocking">阻塞下一段</option>
                          <option value="non_blocking">非阻塞（推荐）</option>
                          <option value="disabled">关闭</option>
                        </select>
                      </label>
                      <label>
                        <span>局部控制器</span>
                        <select :value="point.local_controller || 'mppi'" @change="setWaypointLocalController(index, $event.target.value)">
                          <option v-for="option in LOCAL_CONTROLLER_OPTIONS" :key="option.value" :value="option.value">
                            {{ option.label }}
                          </option>
                        </select>
                      </label>
                      <label>
                        <span>全局规划器</span>
                        <select
                          :value="point.global_controller || DEFAULT_GLOBAL_CONTROLLER"
                          @change="setWaypointGlobalController(index, $event.target.value)"
                        >
                          <option v-for="option in GLOBAL_CONTROLLER_OPTIONS" :key="option.value" :value="option.value">
                            {{ option.label }}
                          </option>
                        </select>
                      </label>
                      <label>
                        <span>定位校正方式</span>
                        <select :value="point.localization_mode || 'ndt'" @change="setWaypointLocalization(index, $event.target.value)">
                          <option value="ndt">NDT / FastVGICP 校正</option>
                          <option value="ukf">UKF（NDT / RTK 动态择优校正）</option>
                          <option value="rtk" :disabled="mapIsLocalOnly">RTK（固定解校正）</option>
                        </select>
                      </label>
                      <label>
                        <span>巡检智能播报</span>
                        <select :value="point.speech_template_id || ''" @change="setWaypointSpeech(index, $event.target.value)">
                          <option value="">到点不播报</option>
                          <option v-for="template in inspectionSpeechTemplates" :key="template.id" :value="template.id">
                            {{ template.name }}
                          </option>
                        </select>
                      </label>
                      <small v-if="point.speech_text">{{ point.speech_text }}</small>
                      <small v-else-if="!inspectionSpeechTemplates.length" class="waypoint-speech-empty">
                        “巡检智能播报”分类下暂无文案
                      </small>
                    </div>
                  </div>
                </div>
                <label class="route-description-under-waypoints">
                  <span>描述</span>
                  <textarea v-model="routeForm.description" rows="2" placeholder="输入路线描述"></textarea>
                </label>
                <label class="diagnostic-record-toggle route-record-toggle">
                  <input v-model="routeForm.record_rosbag" type="checkbox" />
                  <span>录制导航调试包</span>
                </label>
                <div class="waypoint-actions">
                  <button class="btn btn-primary route-main-action route-save-action" @click="handleSaveRoute" :disabled="!selectedMap || waypoints.length === 0 || drillRunning">
                    保存路线
                  </button>
                  <button class="btn btn-secondary route-save-action" @click="handleCreateRoute" :disabled="!selectedMap || waypoints.length === 0 || drillRunning">
                    新建路线
                  </button>
                  <button class="btn btn-sm btn-danger" @click="clearWaypoints" :disabled="waypoints.length === 0">清空</button>
                </div>
                <div class="route-save-hints" aria-label="路线操作提示">
                  <small class="route-preview-note">选择地图并添加至少两个途经点后开始演练。</small>
                  <small v-if="selectedRoute" class="route-preview-note route-save-preview-note">
                    {{ selectedRobot?.name || '机器狗' }}将实际执行“{{ selectedRoute.name }}”，请确认现场安全
                  </small>
                  <div v-if="drillMessage" class="drill-status route-save-status" :class="{ active: drillRunning }">
                    <span class="drill-status-dot"></span>
                    {{ drillMessage }}
                  </div>
                </div>
              </div>
            </div>

            <aside class="drill-timeline-panel route-timeline-column" :class="{ open: drillTimelineOpen }">
              <div class="drill-timeline-header">
                <div>
                  <span>演练记录</span>
                  <strong>时间轴 · {{ drillTimelineSourceLabel }}</strong>
                </div>
                <div class="drill-timeline-actions">
                  <button class="btn btn-sm" :disabled="timelineRunning || !displayedDrillTimeline.length" @click="clearDisplayedDrillTimeline">
                    {{ drillTimelineMode === 'execution' ? '清空视图' : '清空' }}
                  </button>
                  <button type="button" class="btn btn-sm" @click="drillTimelineOpen = !drillTimelineOpen">
                    {{ drillTimelineOpen ? '折叠' : '展开' }}
                  </button>
                </div>
              </div>
              <div v-if="drillTimelineOpen" class="drill-timeline-summary">
                <div v-for="item in drillTimelineSummary" :key="item.label">
                  <span>{{ item.label }}</span><strong :title="item.value">{{ item.value }}</strong>
                </div>
              </div>
              <div v-if="drillTimelineOpen && !displayedDrillTimeline.length" class="drill-timeline-empty">
                {{ drillTimelineEmptyText }}
              </div>
              <div v-else-if="drillTimelineOpen" ref="drillTimelineListRef" class="drill-timeline-list">
                <article v-for="event in displayedDrillTimeline" :key="event.id" class="drill-timeline-item" :class="`event-${event.type}`">
                  <div class="timeline-node"></div>
                  <div class="timeline-content">
                    <div class="timeline-time">
                      <span>{{ formatDrillClock(event.occurredAt) }}</span>
                      <em>+{{ formatDrillElapsed(event.elapsedSeconds) }}</em>
                    </div>
                    <strong>{{ event.title }}</strong>
                    <p v-if="event.detail">{{ event.detail }}</p>
                    <div class="timeline-meta">
                      <span v-if="event.pointName">📍 {{ event.pointName }}</span>
                      <span v-if="event.speed !== undefined">速度 {{ Number(event.speed).toFixed(2) }} m/s</span>
                    </div>
                  </div>
                </article>
              </div>
            </aside>
          </div>

          <div class="panel-section route-step-panel route-step-4">
            <div class="route-step-content route-select-content">
              <select class="route-selector" :value="selectedRoute?.id || NEW_ROUTE_SELECTION" @change="handleRouteSelect($event.target.value)">
                <option :value="NEW_ROUTE_SELECTION">新建或选择已保存线路</option>
                <option v-for="route in routes" :key="route.id" :value="route.id">
                  {{ route.name }}（{{ route.waypoint_count }} 个途经点）{{ routeExecutionOptionText(route) }}
                </option>
              </select>
              <small v-if="selectedRoute" class="route-last-execution">
                上次执行：{{ formatExecutionCreatedAt(selectedRoute.latest_execution?.created_at) }}
              </small>
              <button type="button" class="route-list-toggle" @click="toggleRouteList">
                {{ routeListOpen ? '收起路线列表' : '展开路线列表' }}
              </button>
            <div v-if="routeListOpen" class="route-list">
              <div v-if="routes.length === 0" class="empty-hint">暂无保存的路线</div>
              <div v-for="route in routes" :key="route.id" class="route-item" :class="{ active: selectedRoute?.id === route.id }">
                <div @click="handleLoadRoute(route)">
                  <strong>{{ route.name }}</strong>
                  <small>{{ route.waypoint_count }} 个途经点</small>
                </div>
                <button class="btn btn-sm btn-danger" @click="handleDeleteRoute(route)">删除</button>
              </div>
            </div>
            </div>
          </div>

          <div class="panel-section route-step-panel route-step-5">
            <h3>5. 导航测试</h3>
            <div class="status-grid">
              <div>
                <span>任务对象</span>
                <strong :title="`机器狗：${selectedRobot?.name || '未选择'}；地图：${selectedMap?.name || '未选择'}`">
                  机器狗 {{ selectedRobot?.name || '未选择' }} · 地图 {{ selectedMap?.name || '未选择' }}
                </strong>
              </div>
              <div>
                <span>运行链路</span>
                <strong :title="`连接：${navStatus?.connection_status || 'unknown'}；定位：${localizationLabel()}；导航栈：${navReadyLabel()}`">
                  连接 {{ navStatus?.connection_status || 'unknown' }} · 定位 {{ localizationLabel() }} · 导航 {{ navReadyLabel() }}
                </strong>
              </div>
              <div>
                <span>当前位置</span>
                <strong>{{ robotPoseText() }}</strong>
              </div>
              <div>
                <span>地图原点M</span>
                <strong>{{ originPoseText(mapFrameOrigin) }}</strong>
              </div>
              <div>
                <span>栅格左下角G</span>
                <strong>{{ originPoseText(occupancyGridOrigin) }}</strong>
              </div>
              <div>
                <span>RTK原点R</span>
                <strong :title="rtkOriginTitle()">{{ rtkEnuOrigin ? originPoseText(rtkEnuOrigin, 'map_x', 'map_y') : '未锁定' }}</strong>
              </div>
            </div>
            <p v-if="!robotMapMatches()" class="form-error">当前机器人上报地图与页面地图不一致，暂不显示位置。</p>
            <p v-if="localizationSampleStale()" class="form-error">定位数据未持续更新，请检查导航/定位栈是否启动。</p>
            <p v-if="navError" class="form-error">{{ navError }}</p>
            <div class="nav-actions">
              <label class="single-goal-selector">
                <span>目标航点</span>
                <select v-model.number="selectedSingleGoalIndex" :disabled="!!navCommandBusy || waypoints.length === 0">
                  <option v-for="(point, index) in waypoints" :key="index" :value="index">{{ waypointNames[index] || `点${index + 1}` }}（{{ waypointDisplayText(point) }}）</option>
                </select>
              </label>
              <button class="btn btn-sm btn-primary" :disabled="!!navCommandBusy || !selectedMap || !selectedRobot || !waypoints.length || navStatus?.connection_status !== 'online'" @click="executeSingleGoal(selectedSingleGoalIndex)">
                {{ navCommandBusy === 'single-goal' ? '单点导航下发中...' : '单点导航' }}
              </button>
              <button class="btn btn-sm" :disabled="!!navCommandBusy" @click="refreshNavigationStatus">刷新状态</button>
              <button
                class="btn btn-sm btn-primary"
                :disabled="!!navCommandBusy || !selectedMap || !selectedRobot || navStatus?.connection_status !== 'online'"
                @click="handleActivateSelectedMap"
              >{{ navCommandBusy === 'map-activate' ? '下发并定位中...' : '下发地图' }}</button>
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
              <button class="btn btn-sm" :disabled="!!navCommandBusy || navStatus?.connection_status !== 'online'" @click="activeRelocalize">
                {{ navCommandBusy === 'relocalize' ? '搜索中...' : '主动重定位' }}
              </button>
              <button class="btn btn-sm" :class="{ 'btn-primary': showPoseTrail }" @click="showPoseTrail = !showPoseTrail">
                {{ showPoseTrail ? '隐藏尾迹' : '显示尾迹' }}
              </button>
              <button class="btn btn-sm" :class="{ 'btn-primary': showLocalizationAttemptPoints }" @click="showLocalizationAttemptPoints = !showLocalizationAttemptPoints">
                {{ showLocalizationAttemptPoints ? '隐藏尝试点及原点' : '显示尝试点及原点' }}
              </button>
              <button class="btn btn-sm" :disabled="poseHistory.length === 0" @click="clearPoseHistory">清空尾迹</button>
            </div>
            <div class="localization-algorithm-note">
              <span><strong>下发地图</strong> 目标地图已就绪时直接复用；否则应用地图并恢复定位，局部候选均无合格结果时才进入全图位置与航向匹配。</span>
              <span><strong>初始化定位</strong> 室外/过渡且使用非本地坐标时优先验证 RTK 固定解；航向可用且漂移连续 3 个新样本严格小于 0.30 m 才通过，否则转入建图原点、航点和全局流程。</span>
              <span><strong>主动重定位</strong> 静止搜索建图原点及周边，有合格候选即提交该阶段最优，其中稳定 NDT 分数严格小于 0.01 时提前结束；原点阶段无合格候选才尝试手选点/路线航点，仍无合格候选才全局匹配。</span>
              <span><i class="legend-relocalization-dot"></i> 紫色标记仅记录终态中的严格优选位置：RTK 漂移验证通过，或最优 NDT 位姿已提交且分数严格小于 0.01；不表示 FAST-LIO 已完成稳定接管。</span>
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
              <button class="btn btn-sm btn-primary" :disabled="!manualInitialPose || !!navCommandBusy" @click="sendInitialPose">
                {{ navCommandBusy === 'initial-pose' ? 'NDT定位中...' : '下发初始定位' }}
              </button>
            </div>
            <small v-if="navStatus?.command" class="command-note">
              最近命令 {{ navStatus.command.command_type }} · {{ navStatus.command.status }}
            </small>
                  <small v-if="lastExecution" class="command-note">
                最近执行 {{ lastExecution.id }} · 第 {{ lastExecution.round_number || 1 }} 轮 · {{ lastExecution.state }} · {{ formatExecutionCreatedAt(lastExecution.created_at) }}
            </small>
            <small v-if="localizationInitMessage" class="command-note">
              定位初始化 {{ localizationInitState }} · {{ localizationInitMessage }}
            </small>
            <div v-if="localizationAttemptSession" class="localization-attempt-card">
              <button type="button" class="localization-attempt-toggle" @click="localizationAttemptCardOpen = !localizationAttemptCardOpen">
                <strong>定位尝试过程</strong>
                <span>{{ localizationAttemptSession.phase === 'transfer' ? '地图下发中' : (localizationAttemptSession.status || localizationInitState) }}</span>
                <small>{{ localizationAttemptCardOpen ? '收起' : '展开' }}</small>
              </button>
              <div v-if="localizationAttemptCardOpen" class="localization-attempt-body">
                <p v-if="localizationAttemptSession.phase === 'transfer'" class="command-note">文件传输阶段只显示命令进度，进入地图定位后再展示候选点。</p>
                <p v-if="localizationAttemptSession.livePose">实时位姿 {{ formatAttemptPose(localizationAttemptSession.livePose) }}</p>
                <p v-if="localizationAttemptSession">
                  {{ localizationAttemptProgressText(localizationAttemptSession) }}
                  <span v-if="localizationAttemptSession.earlyStopped"> · 已达到最优阈值并提前停止</span>
                </p>
                <p v-if="localizationAttemptSession.rtkDrift">
                  RTK/Fast-LIO 漂移 {{ formatAttemptMetric(localizationAttemptSession.rtkDrift.xy_m) }} m
                  · 阈值 &lt; {{ formatAttemptMetric(localizationAttemptSession.rtkDrift.threshold_xy_m, 2) }} m
                  · {{ localizationAttemptSession.rtkDrift.verified ? '通过' : '验证中' }}
                </p>
                <p v-if="localizationAttemptSession.bestMatchPose">
                  最优位姿 {{ formatAttemptPose(localizationAttemptSession.bestMatchPose) }}
                  <span v-if="localizationAttemptSession.bestNdtCandidate">
                    · 来源 {{ localizationAttemptSession.source || 'NDT' }}
                    · NDT {{ formatAttemptMetric(localizationAttemptSession.bestNdtCandidate.matching_error) }}
                    · 内点 {{ localizationAttemptSession.bestNdtCandidate.inlier_fraction == null ? '—' : `${(Number(localizationAttemptSession.bestNdtCandidate.inlier_fraction) * 100).toFixed(1)}%` }}
                  </span>
                </p>
                <ol v-if="localizationAttemptSession" class="localization-attempt-timeline">
                  <li
                    v-for="(step, stepIndex) in localizationAttemptTimeline(localizationAttemptSession)"
                    :key="step.key"
                    class="localization-timeline-step"
                    :class="step.status"
                  >
                    <span class="localization-timeline-node">{{ stepIndex + 1 }}</span>
                    <div class="localization-timeline-content">
                      <div class="localization-timeline-heading">
                        <strong>{{ step.title }}</strong>
                        <span>{{ step.statusLabel }}</span>
                      </div>
                      <small>{{ step.detail }}</small>
                      <div class="localization-timeline-time">
                        开始 {{ formatDateTime(step.startedAt) }} · 完成 {{ formatDateTime(step.finishedAt) }}
                      </div>
                      <div v-if="step.attempts.length" class="localization-timeline-attempts">
                        <div
                          v-for="attempt in step.attempts"
                          :key="`${step.key}-${attempt.candidateNumber}`"
                          class="localization-timeline-attempt"
                          :class="attemptStatusClass(attempt.status)"
                        >
                          <b>#{{ attempt.candidateNumber }}</b>
                          <span>{{ attemptStatusLabel(attempt.status) }}</span>
                          <small>{{ formatAttemptPose(attempt.seedPose) }}</small>
                          <small v-if="attempt.matchingError !== null || ['rejected', 'failed'].includes(attempt.status)">NDT {{ formatAttemptMetric(attempt.matchingError) }}</small>
                          <small v-if="attempt.inlierFraction !== null || ['rejected', 'failed'].includes(attempt.status)">内点 {{ attempt.inlierFraction === null ? '—' : `${(attempt.inlierFraction * 100).toFixed(1)}%` }}</small>
                          <small v-if="attempt.hasConverged !== null">收敛 {{ attempt.hasConverged ? '是' : '否' }}</small>
                          <small v-if="attempt.rejectReason">{{ attemptRejectReasonLabel(attempt.rejectReason) }}</small>
                        </div>
                      </div>
                    </div>
                  </li>
                </ol>
              </div>
            </div>
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
                <div v-for="row in sensorStateRows()" :key="row.key || row.name" class="sensor-state-row" :class="{ 'has-action': row.restartSensor && row.state !== 'ok' }">
                  <span>{{ row.name }}</span>
                  <strong :class="statusBadgeClass(row.state)">{{ row.value }}</strong>
                  <button
                    v-if="row.restartSensor && row.state !== 'ok'"
                    class="btn btn-sm sensor-restart-btn"
                    :disabled="!!sensorCommandBusy || navStatus?.connection_status !== 'online'"
                    @click="handleRestartSensor(row.restartSensor)"
                  >{{ sensorCommandBusy === row.restartSensor ? '重启中' : '重启' }}</button>
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

        <!-- 地图主区域 -->
        <div class="map-preview-area">
          <div class="map-stage-layout">
            <div v-if="!selectedMap" class="map-placeholder">
              请先选择地图
            </div>
            <div v-else class="map-workspace">
              <div class="map-toolbar" role="toolbar" aria-label="地图显示控制">
                <div class="map-click-mode" aria-label="地图点击模式">
                  <button type="button" :class="{ active: mapClickMode === 'waypoint' }" @click="setMapClickMode('waypoint')">添加途经点</button>
                  <button type="button" :class="{ active: mapClickMode === 'inspect' }" @click="setMapClickMode('inspect')">查看位置</button>
                  <button type="button" :class="{ active: boundaryEditing }" @click="toggleBoundaryEditing">边界编辑</button>
                </div>
                <div class="map-display-controls">
                  <button type="button" title="缩小" aria-label="缩小" :disabled="mapZoom <= MAP_ZOOM_MIN" @click="adjustMapZoom(-MAP_ZOOM_STEP)">−</button>
                  <button type="button" class="map-zoom-value" title="恢复 100%" @click="resetMapZoom">{{ Math.round(mapZoom * 100) }}%</button>
                  <button type="button" title="放大" aria-label="放大" :disabled="mapZoom >= MAP_ZOOM_MAX" @click="adjustMapZoom(MAP_ZOOM_STEP)">+</button>
                  <button type="button" :class="{ active: showMappingTrace }" @click="showMappingTrace = !showMappingTrace">
                    {{ showMappingTrace ? '隐藏轨迹' : '显示轨迹' }}
                  </button>
                </div>
              </div>
              <div
                class="map-mode-hint"
                :class="{ error: mapInteractionError }"
                :title="boundaryEditing ? '选择区域类型后在地图上依次点击顶点，至少三个点后闭合' : (mapInteractionError || mapModeHintText())"
              >
                <strong>{{ boundaryEditing ? '边界编辑' : (mapClickMode === 'waypoint' ? '添加途经点' : '查看位置') }}</strong>
                <span class="map-mode-hint-text">{{ boundaryEditing ? '依次点击顶点后闭合' : (mapInteractionError || mapModeHintText()) }}</span>
                <span v-if="showLocalizationAttemptPoints" class="map-origin-legend" aria-label="坐标原点图例">
                  <span class="map-origin-legend-item"><i class="map-origin-legend-icon map-origin-legend-map"><b>M</b></i>地图原点M</span>
                  <span v-if="occupancyGridOrigin" class="map-origin-legend-item"><i class="map-origin-legend-icon map-origin-legend-grid"><b>G</b></i>栅格左下角G</span>
                  <span v-if="rtkEnuOrigin" class="map-origin-legend-item"><i class="map-origin-legend-icon map-origin-legend-rtk"><b>R</b></i>RTK原点R</span>
                </span>
              </div>
              <div
                v-if="shouldShowBoundaryPolicyStatus(boundaryEditing, boundaryConfig)"
                :class="['boundary-policy-status', boundaryStatusPresentation().tone]"
              >
                <strong>{{ boundaryStatusPresentation().title }}</strong>
                <span>{{ boundaryStatusPresentation().detail }}</span>
                <div class="boundary-legend" aria-label="导航区域图例">
                  <i class="outer"></i>外边界 <i class="forbidden"></i>禁入 <i class="restricted"></i>限速 <i class="warning"></i>警告
                </div>
              </div>
              <section v-if="boundaryEditing && boundaryConfig" class="boundary-editor">
                <div class="boundary-editor-actions">
                  <button v-for="item in [['outer','外边界'],['forbidden','禁入区'],['restricted','限速区'],['warning','警告区']]" :key="item[0]" type="button" class="btn btn-sm" :class="{ 'btn-primary': boundaryDrawType === item[0] }" @click="startBoundaryDrawing(item[0])">{{ item[1] }}</button>
                  <button type="button" class="btn btn-sm" :disabled="boundaryDraftPoints.length < 3" @click="finishBoundaryDrawing">闭合区域</button>
                  <button type="button" class="btn btn-sm" :disabled="!boundaryDraftPoints.length" @click="boundaryDraftPoints.pop()">撤销一点</button>
                  <button type="button" class="btn btn-sm" @click="cancelBoundaryDrawing">取消绘制</button>
                  <label>安全距离 <input v-model.number="boundaryConfig.safety_margin_m" type="number" min="0" max="5" step="0.05" /></label>
                  <button type="button" class="btn btn-sm btn-primary" :disabled="!!boundaryBusy" @click="saveBoundaryDraft">{{ boundaryBusy === 'save' ? '保存中' : '保存草稿' }}</button>
                  <button type="button" class="btn btn-sm btn-primary" :disabled="!!boundaryBusy || !boundaryConfig?.revision" @click="publishBoundary">{{ boundaryBusy === 'publish' ? '发布中' : '发布到机器人' }}</button>
                  <span class="boundary-revision">草稿 v{{ boundaryConfig?.revision || 0 }} · 生效 v{{ boundaryConfig?.active_revision || 0 }} · {{ boundaryConfig?.apply_status || 'unconfigured' }}</span>
                </div>
                <p v-if="boundaryError" class="form-error">{{ boundaryError }}</p>
                <div v-if="boundaryConfig?.zones?.length" class="boundary-zone-list">
                  <div v-for="(zone, index) in boundaryConfig.zones" :key="zone.id || index" :class="`zone-${zone.zone_type}`">
                    <input v-model="zone.name" />
                    <span>{{ zone.zone_type === 'forbidden' ? '禁入' : zone.zone_type === 'restricted' ? '限速' : '警告' }}</span>
                    <label><input v-model="zone.active" type="checkbox" />启用</label>
                    <label v-if="zone.zone_type === 'restricted'">上限 <input v-model.number="zone.speed_limit_mps" type="number" min="0.05" max="0.3" step="0.05" /> m/s</label>
                    <label v-if="zone.zone_type === 'warning'">提前 <input v-model.number="zone.warning_distance_m" type="number" min="0" max="10" step="0.1" /> m</label>
                    <button type="button" class="btn btn-sm btn-danger" @click="removeBoundaryZone(index)">删除</button>
                  </div>
                </div>
              </section>
              <div ref="mapViewportRef" class="map-container">
                <div
                  v-if="confirmedInspectedPoints.length || inspectedMapPoint"
                  class="map-inspection-list"
                  aria-label="已选择的位置信息"
                >
                  <div
                    v-for="(item, index) in confirmedInspectedPoints"
                    :key="item.id"
                    class="map-inspection-row confirmed"
                  >
                    <div class="inspection-row-actions">
                      <button type="button" class="btn btn-sm inspection-reset-btn" @click="removeConfirmedInspectedPoint(item.id)">重选位置</button>
                    </div>
                    <strong>点{{ index + 1 }}</strong>
                    <span :title="waypointDisplayText(item.point)">XY {{ waypointDisplayText(item.point) }}</span>
                    <span>方向 {{ waypointYawDegrees(item.point).toFixed(1) }}°</span>
                    <span :title="poseText(item.sample?.slam)">NDT {{ poseText(item.sample?.slam) }}</span>
                    <span :title="rtkPoseText(item.sample?.rtk)">RTK {{ rtkPoseText(item.sample?.rtk) }}</span>
                    <small v-if="item.sample">关键帧 {{ item.sample.distance_m.toFixed(2) }}m · {{ mappingSampleTime(item.sample) }}</small>
                    <small v-else>附近无建图轨迹记录</small>
                  </div>
                  <div v-if="inspectedMapPoint" class="map-inspection-row draft">
                    <div class="inspection-row-actions">
                      <button
                        type="button"
                        class="btn btn-sm btn-primary"
                        :disabled="inspectPoseStep !== 'complete'"
                        @click="confirmInspectedMapPoint"
                      >确认</button>
                      <button type="button" class="btn btn-sm inspection-reset-btn" @click="resetInspectedMapPoint">重选位置</button>
                    </div>
                    <strong>点{{ confirmedInspectedPoints.length + 1 }}</strong>
                    <span :title="waypointDisplayText(inspectedMapPoint.point)">XY {{ waypointDisplayText(inspectedMapPoint.point) }}</span>
                    <span>{{ inspectPoseStep === 'complete' ? `方向 ${waypointYawDegrees(inspectedMapPoint.point).toFixed(1)}°` : '方向待选' }}</span>
                    <span :title="poseText(inspectedMapPoint.sample?.slam)">NDT {{ poseText(inspectedMapPoint.sample?.slam) }}</span>
                    <span :title="rtkPoseText(inspectedMapPoint.sample?.rtk)">RTK {{ rtkPoseText(inspectedMapPoint.sample?.rtk) }}</span>
                    <small v-if="inspectedMapPoint.sample">关键帧 {{ inspectedMapPoint.sample.distance_m.toFixed(2) }}m · {{ mappingSampleTime(inspectedMapPoint.sample) }}</small>
                    <small v-else>附近无建图轨迹记录</small>
                  </div>
                </div>
                <div v-if="selectedMap.thumbnail_url" class="map-image-layer" :style="mapImageLayerStyle">
                  <img ref="mapImageRef" :src="getFullUrl(selectedMap.thumbnail_url)" alt="地图预览" @load="refreshImageGeometry" @click="handleMapClick" />

                  <svg v-if="boundaryConfig?.outer_polygon?.length || boundaryDraftPoints.length" class="navigation-boundary-layer">
                    <polygon v-if="boundaryConfig?.outer_polygon?.length" :points="boundaryPolygonPoints(boundaryConfig.outer_polygon)" class="boundary-outer" />
                    <polygon v-for="(zone, index) in (boundaryConfig?.zones || [])" :key="`boundary-zone-${zone.id || index}`" :points="boundaryPolygonPoints(zone.polygon)" :class="`boundary-zone boundary-${zone.zone_type}`" />
                    <polyline v-if="boundaryDraftPoints.length" :points="boundaryDraftPolyline()" class="boundary-draft" />
                  </svg>

                  <svg v-if="showMappingTrace && mappingTrace.length > 1" class="mapping-trace-lines">
                    <polyline :points="mappingTracePoints()" fill="none" stroke="#0f766e" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" />
                  </svg>

                  <svg v-if="globalPlanPolylinePoints()" class="global-plan-lines">
                    <polyline :points="globalPlanPolylinePoints()" fill="none" stroke="#7c3aed" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" stroke-dasharray="8 5" />
                  </svg>

                  <svg v-if="showPoseTrail && poseHistory.length > 1" class="pose-trail-lines">
                    <polyline :points="poseTrailPoints()" fill="none" stroke="#f97316" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" />
                  </svg>

                  <div class="waypoint-markers">
                    <div v-if="showLocalizationAttemptPoints" class="planner-map-origin-marker" :style="waypointDisplayPosition(mapFrameOrigin)" title="地图坐标原点 map (0, 0)">M</div>
                    <div
                      v-if="showLocalizationAttemptPoints && occupancyGridOrigin"
                      class="planner-grid-origin-marker"
                      :style="waypointDisplayPosition(occupancyGridOrigin)"
                      title="map.yaml origin：栅格图左下角在 map 坐标中的位置"
                    >G</div>
                    <div
                      v-if="showLocalizationAttemptPoints && rtkEnuOrigin"
                      class="planner-rtk-origin-marker"
                      :style="waypointDisplayPosition({ x: rtkEnuOrigin.map_x, y: rtkEnuOrigin.map_y })"
                      :title="rtkOriginTitle()"
                    >R</div>
                    <div
                      v-for="(point, index) in waypoints"
                      :key="index"
                      class="waypoint-marker"
                      :class="{ 'drill-arrived': drillCurrentIndex === index && drillPosition }"
                      :style="waypointDisplayPosition(point)"
                    >
                      {{ index + 1 }}
                      <span v-if="point.require_yaw === true" class="waypoint-heading-arrow" :style="waypointHeadingStyle(point)"></span>
                    </div>
                    <div v-if="drillDisplayPosition()" class="drill-robot-marker" :style="drillDisplayPosition()">
                      <RobotDogIcon class="drill-dog-icon" :size="24" />
                      <strong>演练</strong>
                    </div>
                    <div
                      v-for="point in localizationLossMarkers"
                      :key="`planner-loss-${point.eventId}`"
                      class="planner-localization-loss"
                      :style="waypointDisplayPosition(point)"
                      :title="lossMarkerTitle(point)"
                    >
                      <i :style="lossHeadingStyle(point)"></i><small>{{ point.sequence }}</small>
                    </div>
                    <div
                      v-for="attempt in visibleAttemptMarkers"
                      :key="`attempt-${attempt.candidateNumber}`"
                      class="localization-attempt-marker"
                      :class="attemptStatusClass(attempt.status)"
                      :style="attemptMarkerPosition(attempt)"
                      :title="`候选 ${attempt.candidateNumber} ${attemptStatusLabel(attempt.status)} ${formatAttemptPose(attempt.seedPose)}`"
                    >
                      {{ attempt.candidateNumber }}
                    </div>
                    <div
                      v-for="marker in (showLocalizationAttemptPoints ? relocalizationMarkers : [])"
                      :key="`relocalization-${marker.id}`"
                      class="planner-relocalization-marker"
                      :style="waypointDisplayPosition(marker)"
                      :title="relocalizationMarkerTitle(marker)"
                    >
                      <i :style="relocalizationHeadingStyle(marker)"></i><small>{{ marker.sequence }}</small>
                    </div>
                    <div v-if="robotDisplayPosition()" class="robot-marker" :class="{ untrusted: !robotMapPoint()?.trusted }" :style="robotDisplayPosition()" :title="robotMarkerTitle()">
                      <RobotDogIcon :size="28" />
                      <span :style="robotHeadingStyle()"></span>
                      <small>{{ robotMapPoint()?.trusted ? '机器狗' : '定位不可信' }}</small>
                    </div>
                    <div v-if="manualInitialPose" class="initial-pose-marker" :style="waypointDisplayPosition(manualInitialPose)">
                      <span :style="initialPoseHeadingStyle()"></span>
                    </div>
                    <div v-if="selectedLogPoint" class="system-log-map-marker" :style="waypointDisplayPosition(selectedLogPoint)" title="当前选中日志位置">L</div>
                    <div
                      v-for="(item, index) in confirmedInspectedPoints"
                      :key="`inspect-${item.id}`"
                      class="inspected-map-marker confirmed"
                      :style="waypointDisplayPosition(item.point)"
                    >
                      {{ index + 1 }}
                      <span class="inspected-heading-arrow" :style="waypointHeadingStyle(item.point)"></span>
                    </div>
                    <div v-if="inspectedMapPoint" class="inspected-map-marker draft" :style="waypointDisplayPosition(inspectedMapPoint.point)">
                      {{ confirmedInspectedPoints.length + 1 }}
                      <span v-if="inspectPoseStep === 'complete'" class="inspected-heading-arrow" :style="waypointHeadingStyle(inspectedMapPoint.point)"></span>
                    </div>
                  </div>

                  <svg v-if="initialPoseHeadingLinePoints()" class="initial-pose-heading-line">
                    <polyline :points="initialPoseHeadingLinePoints()" fill="none" stroke="#f97316" stroke-width="2.5" stroke-linecap="round" />
                  </svg>

                  <svg v-if="inspectedHeadingLinePoints()" class="inspection-heading-line">
                    <polyline :points="inspectedHeadingLinePoints()" fill="none" stroke="#dc2626" stroke-width="2.5" stroke-linecap="round" />
                  </svg>

                  <svg v-if="waypoints.length > 1" class="path-lines">
                    <polyline :points="pathPolylinePoints()" fill="none" stroke="#1976d2" stroke-width="2" />
                  </svg>
                </div>
                <div v-else class="map-placeholder">地图预览不可用</div>
              </div>
            </div>

          </div>

        </div>
        <section v-if="selectedMap?.thumbnail_url" class="keyframe-panel route-keyframe-row" :class="{ open: keyframePanelOpen }">
          <button type="button" class="keyframe-panel-toggle" @click="keyframePanelOpen = !keyframePanelOpen">
            <span>建图关键帧（{{ mappingTrace.length }}）</span>
            <strong>{{ keyframePanelOpen ? '收起' : '展开' }}</strong>
          </button>
          <div v-if="keyframePanelOpen" class="keyframe-panel-body">
            <div v-if="mappingTraceLoading" class="keyframe-empty">正在加载关键帧</div>
            <div v-else-if="!mappingTrace.length" class="keyframe-empty">当前地图没有关键帧定位记录</div>
            <template v-else>
              <div class="keyframe-table-scroll">
                <table class="keyframe-table">
                  <thead>
                    <tr>
                      <th>序号</th>
                      <th>采样时间</th>
                      <th>NDT x / y / yaw</th>
                      <th>RTK x / y / yaw / 状态</th>
                    </tr>
                  </thead>
                  <tbody>
                    <tr
                      v-for="(sample, rowIndex) in keyframePageData.rows"
                      :key="sample.index ?? keyframePageData.start + rowIndex"
                      :class="{ selected: selectedKeyframeIndex === (sample.index ?? keyframePageData.start + rowIndex) }"
                      @click="selectKeyframe(sample, rowIndex)"
                    >
                      <td>{{ sample.index ?? keyframePageData.start + rowIndex + 1 }}</td>
                      <td>{{ mappingSampleTime(sample) }}</td>
                      <td>{{ poseText(sample.slam) }}</td>
                      <td>{{ rtkPoseText(sample.rtk) }}</td>
                    </tr>
                  </tbody>
                </table>
              </div>
              <div class="keyframe-pagination">
                <button type="button" class="btn btn-sm" :disabled="keyframePageData.page <= 1" @click="changeKeyframePage(-1)">上一页</button>
                <span>{{ keyframePageData.page }} / {{ keyframePageData.pageCount }}</span>
                <button type="button" class="btn btn-sm" :disabled="keyframePageData.page >= keyframePageData.pageCount" @click="changeKeyframePage(1)">下一页</button>
              </div>
            </template>
          </div>
        </section>
      </div>
    </section>
    <SystemLogPanel :robot-id="selectedRobot?.id" :map-id="selectedMap?.id" @locate="locateSystemLog" />
  </section>
</template>

<style scoped>
.route-header-actions {
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

.btn.drill-btn {
  min-width: 118px;
  border: 2px solid #15803d;
  color: #fff;
  background: #16a34a;
  box-shadow: 0 8px 22px rgba(22, 163, 74, 0.28);
  font-weight: 900;
  letter-spacing: 0;
  animation: drill-button-pulse 1.8s ease-in-out infinite;
}

.btn.drill-btn:hover:not(:disabled) {
  border-color: #166534;
  color: #fff;
  background: #15803d;
  transform: translateY(-1px);
  box-shadow: 0 10px 26px rgba(22, 163, 74, 0.38);
}

.btn.drill-btn.running {
  border-color: #991b1b;
  color: #fff;
  background: #991b1b;
  animation: none;
}

.btn.drill-btn:disabled {
  color: #fff;
  background: #16a34a;
  box-shadow: none;
  animation: none;
}

@keyframes drill-button-pulse {
  0%, 100% { box-shadow: 0 8px 22px rgba(22, 163, 74, 0.24); }
  50% { box-shadow: 0 8px 28px rgba(22, 163, 74, 0.48); }
}

.route-planner-layout {
  --route-main-action-height: 38px;
  display: grid;
  grid-template-columns: minmax(0, 1.15fr) minmax(0, 1.15fr) minmax(190px, 0.6fr) minmax(300px, 0.9fr);
  grid-template-rows: auto minmax(620px, max-content) auto auto;
  column-gap: 1rem;
  row-gap: 0.45rem;
  height: auto;
  min-height: calc(100vh - 200px);
}

.route-planner-panel {
  min-width: 0;
  min-height: 0;
  overflow: visible;
}

.side-panel {
  display: contents;
}

.route-waypoint-column {
  display: flex;
  grid-column: 4;
  grid-row: 2 / span 2;
  min-width: 0;
  min-height: 0;
  flex-direction: column;
  gap: 0.45rem;
}

.route-config-panel {
  grid-column: 2 / span 2;
  grid-row: 1;
  align-self: stretch;
  min-width: 0;
}

.route-config-grid {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 0.65rem;
}

.route-config-grid label {
  display: grid;
  min-width: 0;
  gap: 0.3rem;
  color: var(--muted);
  font-size: 0.75rem;
  font-weight: 700;
}

.route-config-grid input,
.route-config-grid select,
.route-config-grid textarea {
  width: 100%;
  min-width: 0;
  max-width: 100%;
  padding: 0.5rem;
  border: 1px solid var(--line);
  border-radius: 6px;
  color: var(--text);
  background: var(--input-bg);
  font: inherit;
  font-weight: 400;
}

.route-description-field {
  grid-column: 1 / -1;
}

.route-description-under-waypoints {
  display: grid;
  gap: 0.3rem;
  margin-top: 0.65rem;
  color: var(--muted);
  font-size: 0.75rem;
  font-weight: 700;
}

.route-description-under-waypoints textarea {
  width: 100%;
  min-width: 0;
  max-width: 100%;
  min-height: 56px;
  padding: 0.5rem;
  border: 1px solid var(--line);
  border-radius: 6px;
  color: var(--text);
  background: var(--input-bg);
  font: inherit;
  font-weight: 400;
  resize: vertical;
}

.route-config-panel .empty-hint {
  margin: 0.5rem 0 0;
  padding: 0;
  text-align: left;
}

.route-drill-panel {
  display: flex;
  grid-column: 4;
  grid-row: 1;
  min-width: 0;
  align-self: stretch;
  align-items: stretch;
  justify-content: stretch;
  gap: 0.65rem;
  flex-direction: column;
  text-align: center;
}

.route-drill-actions {
  display: flex;
  width: 100%;
  min-height: 0;
  flex: 0 0 auto;
  flex-direction: column;
  gap: 0.55rem;
}

.route-drill-actions > .drill-btn {
  height: var(--route-main-action-height);
  min-height: var(--route-main-action-height);
  flex: 0 0 var(--route-main-action-height);
}

.route-drill-actions .route-preview-actions {
  min-height: 0;
  flex: 0 0 auto;
}

.route-drill-panel .drill-btn {
  width: 100%;
  height: var(--route-main-action-height);
  min-height: var(--route-main-action-height);
}

.route-drill-panel p {
  margin: 0;
  color: var(--muted);
  font-size: 0.72rem;
  line-height: 1.4;
}

.route-step-panel {
  display: flex;
  align-self: stretch;
  flex-direction: column;
  overflow: hidden;
  min-width: 0;
  min-height: 0;
}

.route-step-3 {
  grid-column: auto;
  grid-row: auto;
  flex: 0 0 auto;
  align-self: stretch;
  min-height: 0;
  height: auto;
  max-height: none;
  overflow: hidden;
}

.route-step-4 {
  grid-column: 1;
  grid-row: 1;
  align-self: stretch;
}

.route-step-5 {
  grid-column: 1 / -1;
  grid-row: 4;
  align-self: start;
  overflow: visible;
}

.route-step-heading {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 1rem;
}

.route-step-heading h3 {
  margin-bottom: 0;
}

.route-step-heading > div {
  min-width: 0;
}

.route-step-heading small {
  display: block;
  margin-top: 0.25rem;
  color: var(--muted);
  font-size: 0.7rem;
}

.route-step-toggle {
  flex: 0 0 auto;
  min-height: 30px;
  padding: 0 10px;
  border: 1px solid #99d5ce;
  border-radius: 5px;
  color: #0f766e;
  background: #f0fdfa;
  cursor: pointer;
  font: inherit;
  font-size: 0.72rem;
  font-weight: 800;
}

.route-step-toggle:hover {
  color: #fff;
  background: #0f766e;
}

.route-step-content {
  min-height: 0;
  overflow: auto;
  margin-top: 0.75rem;
}

.route-step-3 .route-step-content {
  display: flex;
  flex: 0 0 auto;
  flex-direction: column;
  min-height: 0;
  overflow: visible;
}

.route-step-3 .waypoint-list {
  height: 280px;
  flex: 0 0 280px;
  min-height: 280px;
  max-height: 280px;
  overflow-y: auto;
}

.route-step-3 .route-description-under-waypoints {
  margin-top: 0.65rem;
  margin-bottom: 0;
}

.route-select-content {
  overflow: visible;
}

.route-step-content select,
.route-step-content input,
.route-step-content textarea {
  min-width: 0;
  max-width: 100%;
}

.route-step-1 .route-step-content > select {
  width: 100%;
}

.panel-section {
  width: 100%;
  min-width: 0;
  color: var(--text);
  background: var(--panel-soft);
  padding: 1rem;
  border-radius: 4px;
}

.panel-section h3 {
  margin: 0 0 0.75rem 0;
  font-size: 0.875rem;
  color: var(--text);
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

.waypoint-load-error {
  color: #b42318;
  background: #fef3f2;
  border-radius: 6px;
}

.waypoint-list {
  height: 280px;
  min-height: 0;
  max-height: none;
  overflow-y: auto;
}

.waypoint-item {
  display: block;
  padding: 0.5rem;
  background: #fff;
  border-radius: 4px;
  margin-bottom: 0.5rem;
  font-size: 0.875rem;
}

.waypoint-main {
  display: grid;
  min-width: 0;
  flex: 1;
  gap: 0.45rem;
}

.waypoint-title-row {
  display: flex;
  align-items: center;
  justify-content: flex-start;
  gap: 0.5rem;
}

.waypoint-expand-toggle {
  flex: 0 0 auto;
  min-width: 42px;
  padding: 0.25rem 0.4rem;
  border: 1px solid #99d5ce;
  border-radius: 5px;
  color: #0f766e;
  background: #f0fdfa;
  cursor: pointer;
  font: inherit;
  font-size: 0.68rem;
  font-weight: 800;
}

.waypoint-title-row > span {
  min-width: 0;
  flex: 1;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.waypoint-delete-btn {
  flex: 0 0 auto;
}

.waypoint-main label {
  display: grid;
  grid-template-columns: 7rem minmax(0, 1fr);
  align-items: center;
  gap: 0.5rem;
  color: #667085;
  font-size: 0.78rem;
}

.waypoint-pose-grid {
  display: grid;
  gap: 2px;
  padding: 0.4rem 0.5rem;
  border-left: 3px solid #0f766e;
  background: #f0fdfa;
}

.waypoint-pose-grid small {
  color: #475467;
  font-size: 0.68rem;
  line-height: 1.35;
}

.waypoint-heading-input {
  display: grid;
  grid-template-columns: minmax(96px, 1fr) 18px auto;
  align-items: center;
  gap: 0.3rem;
  width: 100%;
}

.waypoint-heading-input input {
  width: 100%;
  min-width: 96px;
  padding: 0.35rem 0.45rem;
  border: 1px solid #d0d5dd;
  border-radius: 4px;
}

.waypoint-main label.waypoint-heading-row {
  grid-template-columns: 44px minmax(0, 1fr);
  align-items: center;
  gap: 0.4rem;
}

.heading-unit {
  color: #475467;
  font-weight: 700;
  text-align: center;
}

.heading-confirm-btn {
  min-width: 54px;
  white-space: nowrap;
}

.heading-confirm-btn.confirmed {
  border-color: #86d5ad;
  color: #027a48;
  background: #ecfdf3;
}

.waypoint-field-error {
  color: #b42318 !important;
  white-space: normal !important;
}

.waypoint-check {
  grid-template-columns: 18px minmax(0, 1fr) !important;
}

.waypoint-check input {
  width: 16px;
  height: 16px;
  margin: 0;
}

.waypoint-main select {
  min-width: 0;
  min-height: 34px;
  padding: 0 0.5rem;
  border: 1px solid #d0d5dd;
  border-radius: 6px;
  background: #fff;
}

.waypoint-main > small {
  overflow: hidden;
  color: #667085;
  line-height: 1.45;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.waypoint-speech-empty {
  color: #b54708 !important;
}

.waypoint-actions {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 0.5rem;
  margin-top: 0.5rem;
}

.waypoint-actions .btn {
  width: 100%;
}

.waypoint-actions .route-save-action {
  min-width: 0;
  padding-inline: 0.35rem;
  font-size: 0.75rem;
  white-space: nowrap;
}

.status-grid {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 0.5rem;
}

.status-grid div {
  display: grid;
  grid-template-columns: 72px minmax(0, 1fr);
  gap: 0.5rem;
  align-items: center;
  min-width: 0;
  padding: 0.45rem 0.55rem;
  border: 1px solid #e5e7eb;
  border-radius: 4px;
  background: #f9fafb;
  font-size: 0.75rem;
}

.status-grid span {
  color: #667085;
}

.status-grid strong {
  color: #1f2937;
  font-size: 0.78rem;
  font-weight: 700;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.nav-actions {
  display: flex;
  flex-wrap: wrap;
  gap: 0.5rem;
  margin-top: 0.75rem;
}

.localization-algorithm-note {
  display: grid;
  gap: 0.25rem;
  margin-top: 0.6rem;
  padding: 0.55rem 0.65rem;
  border: 1px solid #ddd6fe;
  border-radius: 4px;
  color: #4c1d95;
  background: #f5f3ff;
  font-size: 0.72rem;
}

.legend-relocalization-dot {
  display: inline-block;
  width: 9px;
  height: 9px;
  margin-right: 0.25rem;
  border-radius: 50%;
  background: #7c3aed;
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
  grid-template-columns: repeat(3, minmax(0, 1fr));
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
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
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

.sensor-state-row.has-action {
  grid-template-columns: minmax(0, 1fr) auto;
}

.sensor-restart-btn {
  align-self: center;
  justify-self: end;
  min-width: 52px;
  padding: 0.3rem 0.55rem;
  white-space: nowrap;
}

.sensor-state-head {
  grid-column: 1 / -1;
  background: #f8fafc;
}

.sensor-state-head strong {
  color: #101828;
  font-size: 0.76rem;
}

.sensor-state-row small {
  grid-column: 1 / -1;
}

.sensor-state-row.has-action small {
  grid-column: 1;
  grid-row: 2;
  min-width: 0;
}

.sensor-state-row.has-action .sensor-restart-btn {
  grid-column: 2;
  grid-row: 2;
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
  white-space: nowrap;
}

.debug-header span {
  font-size: 0.7rem;
  color: #667085;
  white-space: nowrap;
}

.debug-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
}

.debug-row {
  display: grid;
  grid-template-columns: minmax(160px, 190px) minmax(0, 1fr);
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
  white-space: nowrap;
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
  max-height: 260px;
  overflow-y: auto;
}

.route-selector,
.route-list-toggle {
  width: 100%;
  min-height: 36px;
  padding: 0 0.6rem;
  border: 1px solid var(--line);
  border-radius: 6px;
  color: var(--text);
  background: var(--input-bg);
  font: inherit;
}

.route-last-execution {
  display: block;
  margin-top: 0.45rem;
  color: var(--muted);
  font-size: 0.72rem;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.route-record-toggle {
  display: flex;
  align-items: center;
  gap: 0.4rem;
  width: 100%;
  margin-top: 0.55rem;
  color: #667085;
  font-size: 0.78rem;
  line-height: 1.2;
  font-weight: 400;
}

.route-record-toggle input {
  width: 16px;
  height: 16px;
  flex: 0 0 auto;
  margin: 0;
}

.route-list-toggle {
  margin-top: 0.65rem;
  color: var(--cyan);
  background: var(--ghost-bg);
  cursor: pointer;
  font-size: 0.75rem;
  font-weight: 800;
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

.route-preview-btn {
  margin-top: 0;
}

.route-preview-actions {
  display: flex;
  min-height: 0;
  align-items: stretch;
  gap: 0.65rem;
  flex-direction: column;
  margin-top: 0;
}

.route-preview-actions .route-preview-btn {
  height: var(--route-main-action-height);
  min-height: var(--route-main-action-height);
  flex: 0 0 var(--route-main-action-height);
}

.route-main-action {
  box-sizing: border-box;
  height: var(--route-main-action-height);
  min-height: var(--route-main-action-height);
}

.route-preview-note {
  color: #667085;
  line-height: 1.4;
}

.route-save-hints {
  display: grid;
  gap: 0.35rem;
  margin-top: 0.5rem;
  padding: 0.6rem 0.7rem;
  border: 1px solid #d0d5dd;
  border-radius: 6px;
  background: #f8fafc;
}

.route-save-hints .drill-status {
  margin-top: 0;
}

.map-preview-area {
  grid-column: 1 / span 3;
  grid-row: 2;
  min-width: 0;
  min-height: calc(100vh - 320px);
  height: auto;
  background: var(--panel-soft);
  border-radius: 4px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  align-items: stretch;
  justify-content: flex-start;
  position: relative;
  overflow: visible;
}

.map-stage-layout {
  display: grid;
  grid-template-columns: minmax(0, 1fr);
  min-height: 0;
  flex: 0 0 auto;
  gap: 0.85rem;
}

.map-workspace {
  display: flex;
  min-width: 0;
  flex-direction: column;
  gap: 0.65rem;
}

.map-container {
  position: relative;
  width: 100%;
  height: min(68vh, 760px);
  min-height: 420px;
  overflow: auto;
  padding: 1rem;
  background:
    linear-gradient(45deg, #eef1f6 25%, transparent 25%),
    linear-gradient(-45deg, #eef1f6 25%, transparent 25%),
    linear-gradient(45deg, transparent 75%, #eef1f6 75%),
    linear-gradient(-45deg, transparent 75%, #eef1f6 75%);
  background-size: 24px 24px;
  background-position: 0 0, 0 12px, 12px -12px, -12px 0;
}

.map-toolbar {
  display: flex;
  height: 40px;
  min-height: 40px;
  box-sizing: border-box;
  align-items: center;
  justify-content: flex-start;
  flex-wrap: nowrap;
  gap: 8px;
  overflow-x: auto;
  overflow-y: hidden;
  padding: 4px;
  border: 1px solid #d0d5dd;
  border-radius: 6px;
  background: #fff;
  scrollbar-width: thin;
  white-space: nowrap;
}

.map-toolbar button {
  min-width: 34px;
  height: 30px;
  padding: 0 9px;
  border: 1px solid transparent;
  border-radius: 4px;
  color: #344054;
  background: transparent;
  cursor: pointer;
  font-weight: 700;
}

.map-toolbar button:disabled {
  cursor: not-allowed;
  opacity: 0.45;
}

.map-toolbar button:hover,
.map-toolbar button.active {
  border-color: #99d5ce;
  color: #0f766e;
  background: #f0fdfa;
}

.map-toolbar .map-zoom-value {
  min-width: 54px;
  font-size: 0.72rem;
}

.map-mode-hint {
  display: flex;
  min-height: 34px;
  width: 100%;
  box-sizing: border-box;
  align-items: center;
  gap: 0.45rem;
  overflow: hidden;
  margin-top: 0.55rem;
  padding: 0.35rem 0.55rem;
  border: 1px solid #bcd7ff;
  border-radius: 5px;
  color: #344054;
  background: #f5f9ff;
  font-size: 0.72rem;
}

.map-mode-hint strong {
  flex: 0 0 auto;
  color: #175cd3;
}

.map-mode-hint-text {
  flex: 1 1 auto;
  min-width: 0;
  overflow: hidden;
  text-overflow: ellipsis;
}

.map-origin-legend {
  display: flex;
  flex: 0 0 auto;
  align-items: center;
  gap: 0.65rem;
  margin-left: auto;
  color: #475467;
  font-size: 0.66rem;
  white-space: nowrap;
}

.map-origin-legend-item {
  display: inline-flex;
  align-items: center;
  gap: 0.25rem;
}

.map-origin-legend-icon {
  display: inline-grid;
  width: 17px;
  height: 17px;
  place-items: center;
  border-radius: 3px;
  color: #fff;
  font-style: normal;
  transform: rotate(45deg);
}

.map-origin-legend-icon b {
  font-size: 0.6rem;
  line-height: 1;
  transform: rotate(-45deg);
}

.map-origin-legend-map {
  background: #2563eb;
}

.map-origin-legend-grid {
  background: #475569;
}

.map-origin-legend-rtk {
  background: #0891b2;
}

.map-mode-hint.error {
  border-color: #fda29b;
  color: #b42318;
  background: #fff5f5;
}

.map-click-mode,
.map-display-controls {
  display: flex;
  flex: 0 0 auto;
  align-items: center;
  gap: 4px;
}

.map-click-mode {
  height: 34px;
  box-sizing: border-box;
  padding: 1px;
  border: 1px solid #d0d5dd;
  border-radius: 5px;
  background: #f8fafc;
}

.map-click-mode button {
  min-width: 90px;
}

.drill-timeline-panel {
  display: flex;
  grid-column: auto;
  grid-row: auto;
  flex: 1 1 auto;
  min-width: 0;
  min-height: 0;
  max-height: none;
  box-sizing: border-box;
  align-self: stretch;
  padding: 0.85rem;
  border: 1px solid #fed7aa;
  border-radius: 12px;
  flex-direction: column;
  background: rgba(255, 255, 255, 0.96);
  box-shadow: 0 12px 30px rgba(124, 45, 18, 0.12);
}

.drill-timeline-panel.open {
  min-height: 280px;
  max-height: none;
}

.drill-timeline-actions {
  display: flex;
  flex-wrap: wrap;
  gap: 0.35rem;
}

.drill-timeline-reopen {
  align-self: flex-end;
  margin-top: 0.65rem;
}

.drill-timeline-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 0.6rem;
}

.drill-timeline-header > div {
  display: grid;
  gap: 1px;
}

.drill-timeline-header span {
  color: #b54708;
  font-size: 0.7rem;
  font-weight: 800;
  letter-spacing: 0.12em;
}

.drill-timeline-header strong {
  color: #7c2d12;
  font-size: 1.05rem;
}

.drill-timeline-summary {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 0.4rem;
  margin-top: 0.7rem;
}

.drill-timeline-summary div {
  display: grid;
  min-width: 0;
  gap: 2px;
  padding: 0.45rem;
  border-radius: 7px;
  background: #fff7ed;
}

.drill-timeline-summary span {
  color: #9a3412;
  font-size: 0.62rem;
}

.drill-timeline-summary strong {
  overflow: hidden;
  color: #7c2d12;
  font-size: 0.74rem;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.drill-timeline-empty {
  display: grid;
  min-height: 0;
  flex: 1 1 auto;
  padding: 1rem;
  place-items: center;
  color: #9a6b53;
  font-size: 0.78rem;
  line-height: 1.6;
  text-align: center;
}

.drill-timeline-list {
  min-height: 0;
  flex: 1 1 auto;
  margin-top: 0.75rem;
  padding: 0 0.2rem 0 0.1rem;
  overflow-y: auto;
}

.drill-timeline-item {
  position: relative;
  display: grid;
  grid-template-columns: 18px minmax(0, 1fr);
  gap: 0.45rem;
  padding-bottom: 0.85rem;
}

.drill-timeline-item:not(:last-child)::before {
  position: absolute;
  top: 13px;
  bottom: -2px;
  left: 5px;
  width: 2px;
  background: #fed7aa;
  content: '';
}

.timeline-node {
  position: relative;
  z-index: 1;
  width: 12px;
  height: 12px;
  margin-top: 4px;
  border: 3px solid #fff;
  border-radius: 50%;
  background: #f97316;
  box-shadow: 0 0 0 2px #fdba74;
}

.event-speech .timeline-node,
.event-speech-end .timeline-node {
  background: #2563eb;
  box-shadow: 0 0 0 2px #93c5fd;
}

.event-dispatch .timeline-node,
.event-created .timeline-node,
.event-accepted .timeline-node,
.event-target .timeline-node,
.event-start .timeline-node {
  background: #2563eb;
  box-shadow: 0 0 0 2px #93c5fd;
}

.event-pause .timeline-node,
.event-resume .timeline-node {
  background: #d97706;
  box-shadow: 0 0 0 2px #fcd34d;
}

.event-arrival .timeline-node,
.event-complete .timeline-node {
  background: #16a34a;
  box-shadow: 0 0 0 2px #86efac;
}

.event-stop .timeline-node,
.event-error .timeline-node {
  background: #dc2626;
  box-shadow: 0 0 0 2px #fca5a5;
}

.timeline-content {
  min-width: 0;
}

.timeline-time {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 0.5rem;
  color: #9a6b53;
  font-size: 0.65rem;
  font-variant-numeric: tabular-nums;
}

.timeline-time em {
  color: #c2410c;
  font-style: normal;
  font-weight: 700;
}

.timeline-content > strong {
  display: block;
  margin-top: 2px;
  color: #431407;
  font-size: 0.8rem;
}

.timeline-content p {
  margin: 0.22rem 0 0;
  color: #6b4d3e;
  font-size: 0.7rem;
  line-height: 1.45;
  overflow-wrap: anywhere;
}

.timeline-meta {
  display: flex;
  flex-wrap: wrap;
  gap: 0.35rem;
  margin-top: 0.3rem;
}

.timeline-meta span {
  padding: 2px 5px;
  border-radius: 999px;
  color: #9a3412;
  background: #ffedd5;
  font-size: 0.62rem;
}

.map-image-layer {
  position: relative;
  margin: 0 auto;
  background: #fff;
  box-shadow: 0 12px 30px rgba(15, 23, 42, 0.14);
}

.boundary-editor {
  margin: 0.65rem 0;
  padding: 0.7rem;
  border: 1px solid #bfdbfe;
  border-radius: 10px;
  background: #eff6ff;
}

.boundary-policy-status {
  display: grid;
  grid-template-columns: auto minmax(220px, 1fr) auto;
  align-items: center;
  gap: 0.55rem 0.8rem;
  margin: 0.55rem 0;
  padding: 0.55rem 0.7rem;
  border-left: 4px solid #f59e0b;
  border-radius: 7px;
  color: #78350f;
  background: #fffbeb;
  font-size: 0.76rem;
}

.boundary-policy-status.active { border-left-color: #16a34a; color: #14532d; background: #f0fdf4; }
.boundary-policy-status.applying { border-left-color: #2563eb; color: #1e3a8a; background: #eff6ff; }
.boundary-policy-status.error { border-left-color: #dc2626; color: #7f1d1d; background: #fef2f2; }

.boundary-legend {
  display: flex;
  align-items: center;
  gap: 0.28rem;
  white-space: nowrap;
}

.boundary-legend i {
  width: 12px;
  height: 12px;
  margin-left: 0.25rem;
  border: 2px solid #2563eb;
  background: rgba(37, 99, 235, 0.08);
}

.boundary-legend i.forbidden { border-color: #dc2626; background: rgba(220, 38, 38, 0.28); }
.boundary-legend i.restricted { border-color: #7c3aed; background: rgba(124, 58, 237, 0.2); }
.boundary-legend i.warning { border-color: #d97706; background: rgba(245, 158, 11, 0.2); }

.boundary-editor-actions,
.boundary-zone-list > div {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: 0.45rem;
}

.boundary-editor-actions label,
.boundary-zone-list label {
  display: inline-flex;
  align-items: center;
  gap: 0.3rem;
  color: #334155;
  font-size: 0.76rem;
}

.boundary-editor input {
  width: 92px;
  padding: 0.3rem 0.4rem;
  border: 1px solid #94a3b8;
  border-radius: 5px;
}

.boundary-editor input[type="checkbox"] {
  width: auto;
}

.boundary-revision {
  color: #475569;
  font-size: 0.72rem;
}

.boundary-zone-list {
  display: grid;
  gap: 0.4rem;
  margin-top: 0.6rem;
}

.boundary-zone-list > div {
  padding: 0.45rem;
  border-left: 4px solid #f59e0b;
  border-radius: 6px;
  background: #fff;
}

.boundary-zone-list > div.zone-forbidden { border-left-color: #dc2626; }
.boundary-zone-list > div.zone-restricted { border-left-color: #7c3aed; }
.boundary-zone-list > div.zone-warning { border-left-color: #f59e0b; }
.boundary-zone-list > div > input:first-child { width: 150px; }

.navigation-boundary-layer {
  position: absolute;
  inset: 0;
  z-index: 1;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.boundary-outer {
  fill: rgba(37, 99, 235, 0.06);
  stroke: #2563eb;
  stroke-width: 3;
  stroke-dasharray: 9 5;
}

.boundary-zone { stroke-width: 2.5; }
.boundary-forbidden { fill: rgba(220, 38, 38, 0.28); stroke: #dc2626; }
.boundary-restricted { fill: rgba(124, 58, 237, 0.2); stroke: #7c3aed; }
.boundary-warning { fill: rgba(245, 158, 11, 0.2); stroke: #d97706; }
.boundary-draft { fill: rgba(14, 165, 233, 0.12); stroke: #0891b2; stroke-width: 2.5; stroke-dasharray: 5 4; }

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

.system-log-map-marker {
  position: absolute;
  z-index: 18;
  display: grid;
  place-items: center;
  width: 25px;
  height: 25px;
  border: 3px solid #fff;
  border-radius: 50%;
  color: #fff;
  background: #0f172a;
  box-shadow: 0 0 0 3px #38bdf8, 0 4px 10px rgba(15, 23, 42, 0.35);
  font-size: 11px;
  font-weight: 900;
  transform: translate(-50%, -50%);
}

.waypoint-heading-arrow {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 20px;
  height: 2px;
  background: #dc2626;
  transform-origin: 0 50%;
}

.waypoint-heading-arrow::after {
  position: absolute;
  top: -4px;
  right: -1px;
  width: 0;
  height: 0;
  border-top: 5px solid transparent;
  border-bottom: 5px solid transparent;
  border-left: 7px solid #dc2626;
  content: '';
}

.mapping-trace-lines,
.global-plan-lines,
.pose-trail-lines {
  position: absolute;
  inset: 0;
  z-index: 2;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.global-plan-lines {
  z-index: 3;
}

.inspected-map-marker {
  position: absolute;
  z-index: 15;
  display: grid;
  place-items: center;
  width: 22px;
  height: 22px;
  border: 3px solid #fff;
  border-radius: 50%;
  color: #fff;
  background: #dc2626;
  box-shadow: 0 0 0 2px #dc2626;
  font-size: 10px;
  font-weight: 900;
  transform: translate(-50%, -50%);
}

.inspected-map-marker.draft {
  background: #f97316;
  box-shadow: 0 0 0 2px #f97316;
}

.inspected-heading-arrow {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 26px;
  height: 3px;
  background: #b42318;
  transform-origin: 0 50%;
}

.inspected-heading-arrow::after {
  position: absolute;
  top: -5px;
  right: -1px;
  width: 0;
  height: 0;
  border-top: 6px solid transparent;
  border-bottom: 6px solid transparent;
  border-left: 8px solid #b42318;
  content: '';
}

.map-inspection-list {
  position: absolute;
  top: 24px;
  left: 24px;
  z-index: 30;
  display: grid;
  width: 50%;
  max-width: 50%;
  max-height: min(42%, 300px);
  gap: 6px;
  overflow: auto;
  scrollbar-gutter: stable;
  pointer-events: auto;
}

.map-inspection-row {
  display: grid;
  grid-template-columns: repeat(7, max-content);
  align-items: center;
  width: max-content;
  min-width: 100%;
  gap: 0.5ch;
  padding: 7px 8px;
  border: 1px solid #fecaca;
  border-radius: 6px;
  color: #344054;
  background: rgba(255, 255, 255, 0.97);
  box-shadow: 0 6px 20px rgba(15, 23, 42, 0.18);
  font-size: 0.72rem;
}

.map-inspection-row.draft { border-color: #fdba74; }
.map-inspection-row strong { color: #991b1b; white-space: nowrap; }
.map-inspection-row.draft strong { color: #c2410c; }
.map-inspection-row span,
.map-inspection-row small {
  white-space: nowrap;
}
.map-inspection-row small { color: #667085; }

.inspection-row-actions {
  position: sticky;
  left: 0;
  z-index: 2;
  display: flex;
  align-items: center;
  gap: 5px;
  padding-right: 0.5ch;
  background: inherit;
}

.inspection-reset-btn {
  white-space: nowrap;
}

.keyframe-panel {
  overflow: hidden;
  border: 1px solid #d0d5dd;
  border-radius: 6px;
  background: #fff;
}

.route-keyframe-row {
  grid-column: 1 / span 3;
  grid-row: 3;
  min-width: 0;
  align-self: start;
}

.keyframe-panel-toggle {
  display: flex;
  width: 100%;
  min-height: 42px;
  align-items: center;
  justify-content: space-between;
  padding: 0.55rem 0.75rem;
  border: 0;
  color: #344054;
  background: #f8fafc;
  cursor: pointer;
  font-weight: 700;
}

.keyframe-panel-toggle strong {
  color: #0f766e;
  font-size: 0.75rem;
}

.keyframe-panel-body {
  border-top: 1px solid #e4e7ec;
}

.keyframe-table-scroll {
  max-height: 360px;
  overflow: auto;
}

.keyframe-table {
  width: 100%;
  min-width: 780px;
  border-collapse: collapse;
  font-size: 0.72rem;
}

.keyframe-table th,
.keyframe-table td {
  padding: 0.5rem 0.65rem;
  border-bottom: 1px solid #eaecf0;
  text-align: left;
  white-space: nowrap;
}

.keyframe-table th {
  position: sticky;
  top: 0;
  z-index: 1;
  color: #475467;
  background: #f9fafb;
}

.keyframe-table tbody tr {
  cursor: pointer;
}

.keyframe-table tbody tr:hover,
.keyframe-table tbody tr.selected {
  background: #ecfdf3;
}

.keyframe-empty {
  padding: 1.2rem;
  color: #667085;
  text-align: center;
  font-size: 0.78rem;
}

.keyframe-pagination {
  display: flex;
  align-items: center;
  justify-content: flex-end;
  gap: 0.65rem;
  padding: 0.55rem 0.75rem;
  color: #475467;
  font-size: 0.75rem;
}

.waypoint-marker.drill-arrived {
  border-color: #fff;
  background: #f97316;
  box-shadow: 0 0 0 5px rgba(249, 115, 22, 0.28);
  transform: translate(-50%, -50%) scale(1.18);
}

.drill-robot-marker {
  position: absolute;
  z-index: 14;
  display: grid;
  place-items: center;
  width: 42px;
  height: 42px;
  border: 3px solid #fff;
  border-radius: 50%;
  background: linear-gradient(145deg, #fb923c, #dc2626);
  box-shadow: 0 6px 18px rgba(127, 29, 29, 0.45);
  transform: translate(-50%, -50%);
  pointer-events: none;
}

.drill-dog-icon {
  position: relative;
  z-index: 2;
  width: 24px;
  height: 24px;
  filter: drop-shadow(0 1px 1px rgba(0, 0, 0, 0.25));
}

.drill-robot-marker strong {
  position: absolute;
  top: 43px;
  padding: 2px 7px;
  border-radius: 999px;
  color: #fff;
  background: #b42318;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.24);
  font-size: 10px;
  white-space: nowrap;
}

.drill-status {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  width: 100%;
  margin-top: 0.65rem;
  padding: 0.65rem 0.85rem;
  border: 1px solid #fed7aa;
  border-radius: 8px;
  color: #9a3412;
  background: #fff7ed;
  font-size: 0.82rem;
  font-weight: 700;
}

.drill-status-dot {
  width: 9px;
  height: 9px;
  flex: 0 0 auto;
  border-radius: 50%;
  background: #9ca3af;
}

.drill-status.active .drill-status-dot {
  background: #f97316;
  box-shadow: 0 0 0 4px rgba(249, 115, 22, 0.18);
  animation: drill-dot-pulse 1s ease-in-out infinite;
}

@keyframes drill-dot-pulse {
  50% { opacity: 0.42; }
}

.localization-attempt-card {
  margin: 0.6rem 0;
  border: 1px solid #e5e7eb;
  border-radius: 10px;
  background: #f8fafc;
}
.localization-attempt-toggle {
  width: 100%;
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.55rem 0.75rem;
  border: 0;
  background: transparent;
  cursor: pointer;
}
.localization-attempt-toggle span,
.localization-attempt-toggle small { color: #64748b; font-size: 0.78rem; }
.localization-attempt-toggle small { margin-left: auto; }
.localization-attempt-body { padding: 0 0.75rem 0.75rem; font-size: 0.78rem; color: #334155; }
.localization-attempt-list { margin: 0.4rem 0 0; padding: 0; list-style: none; display: grid; gap: 0.35rem; }
.localization-attempt-list li {
  display: grid;
  grid-template-columns: 1.6rem 4rem 1fr;
  gap: 0.25rem 0.5rem;
  align-items: start;
  padding: 0.35rem 0.45rem;
  border-radius: 8px;
  background: #fff;
}
.localization-attempt-list li small { grid-column: 2 / -1; color: #64748b; }
.localization-attempt-list li.accepted { background: #ecfdf5; }
.localization-attempt-list li.active { background: #fff7ed; }
.localization-attempt-list li.failed { background: #fef2f2; }
.localization-attempt-timeline {
  position: relative;
  display: grid;
  gap: 0.5rem;
  margin: 0.55rem 0 0;
  padding: 0 0 0 1.7rem;
  list-style: none;
}
.localization-attempt-timeline::before {
  content: '';
  position: absolute;
  top: 0.7rem;
  bottom: 0.7rem;
  left: 0.55rem;
  width: 2px;
  background: #cbd5e1;
}
.localization-timeline-step {
  position: relative;
  min-width: 0;
  padding: 0.5rem 0.6rem;
  border: 1px solid #e2e8f0;
  border-radius: 8px;
  background: #fff;
}
.localization-timeline-step.active { border-color: #fbbf24; background: #fffbeb; }
.localization-timeline-step.done { border-color: #86efac; background: #f0fdf4; }
.localization-timeline-step.failed { border-color: #fca5a5; background: #fef2f2; }
.localization-timeline-step.skipped { opacity: 0.72; }
.localization-timeline-node {
  position: absolute;
  top: 0.55rem;
  left: -1.7rem;
  z-index: 1;
  display: grid;
  width: 1.15rem;
  height: 1.15rem;
  place-items: center;
  border: 2px solid #fff;
  border-radius: 50%;
  color: #fff;
  background: #94a3b8;
  box-shadow: 0 0 0 1px #cbd5e1;
  font-size: 0.65rem;
  font-weight: 800;
}
.localization-timeline-step.active .localization-timeline-node { background: #f59e0b; }
.localization-timeline-step.done .localization-timeline-node { background: #16a34a; }
.localization-timeline-step.failed .localization-timeline-node { background: #dc2626; }
.localization-timeline-content { min-width: 0; }
.localization-timeline-heading {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}
.localization-timeline-heading strong { color: #0f172a; }
.localization-timeline-heading span {
  padding: 0.08rem 0.35rem;
  border-radius: 999px;
  color: #475569;
  background: #e2e8f0;
  font-size: 0.68rem;
  white-space: nowrap;
}
.localization-timeline-step.active .localization-timeline-heading span { color: #92400e; background: #fde68a; }
.localization-timeline-step.done .localization-timeline-heading span { color: #166534; background: #bbf7d0; }
.localization-timeline-step.failed .localization-timeline-heading span { color: #991b1b; background: #fecaca; }
.localization-timeline-content > small { display: block; margin-top: 0.18rem; color: #64748b; line-height: 1.35; }
.localization-timeline-time { margin-top: 0.16rem; color: #475569; font-size: 0.68rem; font-variant-numeric: tabular-nums; }
.localization-timeline-attempts {
  display: grid;
  gap: 0.25rem;
  margin-top: 0.4rem;
  padding-left: 0.45rem;
  border-left: 2px solid #e2e8f0;
}
.localization-timeline-attempt {
  display: flex;
  flex-wrap: wrap;
  align-items: baseline;
  gap: 0.25rem 0.5rem;
  padding: 0.25rem 0.4rem;
  border-radius: 5px;
  background: #f8fafc;
  font-size: 0.7rem;
}
.localization-timeline-attempt.active { background: #fff7ed; }
.localization-timeline-attempt.qualified { background: #f0f9ff; }
.localization-timeline-attempt.accepted { background: #ecfdf5; }
.localization-timeline-attempt.failed { background: #fef2f2; }
.localization-timeline-attempt small { color: #64748b; }
.localization-attempt-marker {
  position: absolute;
  width: 22px;
  height: 22px;
  transform: translate(-50%, -50%);
  border-radius: 999px;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 0.68rem;
  font-weight: 700;
  color: #fff;
  background: #94a3b8;
  border: 2px solid #fff;
  z-index: 2;
  pointer-events: none;
}
.localization-attempt-marker.active { background: #f59e0b; box-shadow: 0 0 0 3px rgba(245, 158, 11, 0.28); animation: localization-attempt-pulse 1s ease-in-out infinite; }
.localization-attempt-marker.qualified { background: #0ea5e9; }
.localization-attempt-marker.failed { background: #ef4444; }
.localization-attempt-marker.accepted { background: #10b981; box-shadow: 0 0 0 3px rgba(16,185,129,0.28); }
@keyframes localization-attempt-pulse {
  50% { transform: translate(-50%, -50%) scale(1.18); }
}
@media (prefers-reduced-motion: reduce) {
  .localization-attempt-marker.active { animation: none; }
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

.robot-marker > .robot-dog-icon {
  position: absolute;
  inset: 1px;
  z-index: 4;
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
  z-index: 5;
}

.robot-marker > small {
  position: absolute;
  top: 30px;
  left: 50%;
  min-width: max-content;
  padding: 2px 5px;
  color: #fff;
  background: #065f46;
  font-size: 10px;
  transform: translateX(-50%);
}

.robot-marker.untrusted::before {
  border-style: dashed;
  background: #f59e0b;
  box-shadow: 0 0 0 4px rgba(220, 38, 38, .28);
}

.robot-marker.untrusted span { border-bottom-color: #b45309; }
.robot-marker.untrusted > small { background: #b45309; }

.planner-localization-loss {
  position: absolute;
  z-index: 4;
  width: 28px;
  height: 28px;
  transform: translate(-50%, -50%);
}

.planner-localization-loss::before {
  content: '';
  position: absolute;
  inset: 6px;
  border: 3px solid #fff;
  border-radius: 50%;
  background: #dc2626;
  box-shadow: 0 0 0 3px rgba(220, 38, 38, .28);
}

.planner-localization-loss i {
  position: absolute;
  left: 50%;
  top: 50%;
  z-index: 2;
  width: 0;
  height: 0;
  border-right: 5px solid transparent;
  border-bottom: 15px solid #7f1d1d;
  border-left: 5px solid transparent;
  transform-origin: 50% 100%;
}

.planner-localization-loss small {
  position: absolute;
  top: 26px;
  left: 50%;
  min-width: 16px;
  padding: 1px 3px;
  color: #fff;
  background: #991b1b;
  font-size: 9px;
  text-align: center;
  transform: translateX(-50%);
}

.planner-relocalization-marker {
  position: absolute;
  z-index: 4;
  width: 28px;
  height: 28px;
  transform: translate(-50%, -50%);
}

.planner-map-origin-marker,
.planner-grid-origin-marker,
.planner-rtk-origin-marker {
  position: absolute;
  z-index: 3;
  display: grid;
  width: 22px;
  height: 22px;
  place-items: center;
  border: 2px solid #fff;
  border-radius: 3px;
  color: #fff;
  font-size: 10px;
  font-weight: 800;
  transform: translate(-50%, -50%) rotate(45deg);
  box-shadow: 0 0 0 2px rgba(15, 23, 42, .2);
}

.planner-map-origin-marker {
  background: #2563eb;
}

.planner-grid-origin-marker {
  background: #475569;
}

.planner-rtk-origin-marker {
  background: #0891b2;
}

.planner-map-origin-marker,
.planner-grid-origin-marker,
.planner-rtk-origin-marker {
  line-height: 1;
  text-shadow: 0 1px rgba(0, 0, 0, .25);
}

.planner-relocalization-marker::before {
  content: '';
  position: absolute;
  inset: 6px;
  border: 3px solid #fff;
  border-radius: 50%;
  background: #7c3aed;
  box-shadow: 0 0 0 3px rgba(124, 58, 237, .28);
}

.planner-relocalization-marker i {
  position: absolute;
  left: 50%;
  top: 50%;
  z-index: 2;
  width: 0;
  height: 0;
  border-right: 5px solid transparent;
  border-bottom: 15px solid #5b21b6;
  border-left: 5px solid transparent;
  transform-origin: 50% 100%;
}

.planner-relocalization-marker small {
  position: absolute;
  top: 26px;
  left: 50%;
  min-width: 16px;
  padding: 1px 3px;
  color: #fff;
  background: #6d28d9;
  font-size: 9px;
  text-align: center;
  transform: translateX(-50%);
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

.global-plan-lines,
.pose-trail-lines,
.path-lines,
.initial-pose-heading-line,
.inspection-heading-line {
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

.inspection-heading-line {
  z-index: 14;
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
    grid-template-rows: none;
    height: auto;
    min-height: 0;
  }

  .route-step-panel {
    height: auto;
    overflow: visible;
  }

  .route-waypoint-column {
    display: contents;
  }

  .route-waypoint-column .route-step-3 {
    flex: 0 0 auto;
  }

  .route-step-content {
    overflow: visible;
  }

  .route-step-panel,
  .route-config-panel,
    .route-drill-panel,
    .route-step-5,
    .map-preview-area,
    .route-timeline-column,
    .route-keyframe-row {
    grid-column: 1;
    grid-row: auto;
  }

  .route-step-4 { grid-row: 1; }
  .route-config-panel { grid-row: 2; }
  .route-drill-panel { grid-row: 3; }
  .map-preview-area { grid-row: 4; }
  .route-step-3 { grid-row: 5; }
  .route-keyframe-row { grid-row: 6; }
  .route-timeline-column { grid-row: 7; }
  .route-step-5 { grid-row: 8; }

  .route-step-3 {
    min-height: 0;
    max-height: none;
  }

  .route-waypoint-column .route-timeline-column {
    flex: 0 0 auto;
    align-self: start;
    max-height: 58px;
  }

  .route-waypoint-column .route-timeline-column.open {
    max-height: 420px;
  }

  .route-step-3 .route-step-content {
    overflow: visible;
  }

  .route-step-3 .waypoint-list {
    flex: 0 0 280px;
    height: 280px;
    min-height: 280px;
    max-height: 280px;
  }

  .status-grid,
  .state-machine-grid,
  .sensor-state-list,
  .debug-grid {
    grid-template-columns: 1fr;
  }

  .map-container {
    min-height: 420px;
  }

  .map-stage-layout {
    grid-template-columns: 1fr;
  }

  .drill-timeline-panel {
    max-height: 58px;
  }

  .drill-timeline-panel.open {
    max-height: 420px;
  }

}

/* RoutePlannerPage uses a legacy, compact card palette in several detail blocks.
   Keep those blocks readable when the shared dashboard theme switches to dark. */
[data-theme="dark"] .route-step-panel,
[data-theme="dark"] .panel-section,
[data-theme="dark"] .map-preview-area {
  color: var(--text);
  border-color: var(--line);
  background: var(--panel-soft);
}

[data-theme="dark"] .route-step-toggle,
[data-theme="dark"] .waypoint-expand-toggle,
[data-theme="dark"] .route-list-toggle,
[data-theme="dark"] .route-selector,
[data-theme="dark"] .map-toolbar,
[data-theme="dark"] .map-click-mode,
[data-theme="dark"] .state-machine-panel,
[data-theme="dark"] .localization-debug-panel,
[data-theme="dark"] .waypoint-item,
[data-theme="dark"] .route-item,
[data-theme="dark"] .map-mode-hint,
[data-theme="dark"] .initial-pose-panel,
[data-theme="dark"] .route-save-hints,
[data-theme="dark"] .map-inspection-row {
  color: var(--text);
  border-color: var(--line);
  background: var(--panel);
}

[data-theme="dark"] .route-step-toggle:hover,
[data-theme="dark"] .waypoint-expand-toggle:hover,
[data-theme="dark"] .map-toolbar button:hover,
[data-theme="dark"] .map-toolbar button.active {
  color: #06111f;
  background: var(--cyan);
}

[data-theme="dark"] .form-group label,
[data-theme="dark"] .empty-hint,
[data-theme="dark"] .waypoint-main label,
[data-theme="dark"] .waypoint-main > small,
[data-theme="dark"] .status-grid span,
[data-theme="dark"] .command-note,
[data-theme="dark"] .route-item small,
[data-theme="dark"] .debug-header span,
[data-theme="dark"] .debug-row span,
[data-theme="dark"] .state-card span,
[data-theme="dark"] .state-card small,
[data-theme="dark"] .sensor-state-row span,
[data-theme="dark"] .sensor-state-row small {
  color: var(--muted);
}

[data-theme="dark"] .form-group input,
[data-theme="dark"] .form-group textarea,
[data-theme="dark"] .form-group select,
[data-theme="dark"] .waypoint-main input,
[data-theme="dark"] .waypoint-main select,
[data-theme="dark"] .initial-pose-panel input {
  color: var(--text);
  border-color: var(--line);
  background: var(--input-bg);
}

[data-theme="dark"] .status-grid strong,
[data-theme="dark"] .state-card strong,
[data-theme="dark"] .sensor-state-row strong,
[data-theme="dark"] .debug-row strong,
[data-theme="dark"] .debug-header strong,
[data-theme="dark"] .route-item strong {
  color: var(--text);
}

[data-theme="dark"] .status-grid div,
[data-theme="dark"] .state-card,
[data-theme="dark"] .sensor-state-head,
[data-theme="dark"] .debug-header,
[data-theme="dark"] .waypoint-pose-grid,
[data-theme="dark"] .route-item:hover {
  border-color: var(--line);
  background: var(--panel-soft);
}

[data-theme="dark"] .waypoint-pose-grid {
  border-left-color: var(--cyan);
}

[data-theme="dark"] .route-item.active {
  border-color: var(--cyan);
  background: rgba(67, 213, 255, 0.12);
}

[data-theme="dark"] .map-mode-hint {
  color: var(--text);
  background: rgba(67, 213, 255, 0.1);
}

[data-theme="dark"] .map-origin-legend {
  color: var(--muted);
}

[data-theme="dark"] .map-inspection-row span,
[data-theme="dark"] .map-inspection-row small {
  color: var(--muted);
}

[data-theme="dark"] .drill-timeline-panel {
  color: var(--text);
  border-color: rgba(255, 196, 92, 0.42);
  background: var(--panel);
  box-shadow: 0 12px 30px rgba(0, 0, 0, 0.28);
}

[data-theme="dark"] .drill-timeline-header span,
[data-theme="dark"] .drill-timeline-summary span {
  color: #ffd58a;
}

[data-theme="dark"] .drill-timeline-header strong,
[data-theme="dark"] .drill-timeline-summary strong {
  color: #ffe7bd;
}

[data-theme="dark"] .drill-timeline-summary div {
  background: rgba(255, 196, 92, 0.12);
}

[data-theme="dark"] .drill-timeline-empty,
[data-theme="dark"] .timeline-time,
[data-theme="dark"] .timeline-content p {
  color: #d8c4ac;
}

[data-theme="dark"] .timeline-time em,
[data-theme="dark"] .timeline-meta span {
  color: #ffd58a;
}

[data-theme="dark"] .timeline-meta span {
  background: rgba(255, 196, 92, 0.14);
}

@media (max-width: 640px) {
  .route-planner-layout { gap: 0.75rem; }
  .panel-section { padding: 0.75rem; }
  .route-config-grid { grid-template-columns: 1fr; }
  .route-description-field { grid-column: auto; }
  .route-header-actions { width: 100%; flex-wrap: wrap; }
  .route-header-actions .btn { flex: 1 1 140px; }
  .map-container {
    height: min(58vh, 460px);
    min-height: 320px;
    padding: 0.65rem;
  }
  .map-inspection-list { top: 18px; left: 18px; max-width: calc(100% - 36px); }
  .map-toolbar { align-items: center; }
  .map-display-controls,
  .map-click-mode { flex: 0 0 auto; justify-content: flex-start; }
  .map-click-mode button { flex: 0 0 auto; min-width: 82px; }
  .map-mode-hint { min-height: 34px; }
  .map-origin-legend { gap: 0.35rem; }
  .map-origin-legend-item { font-size: 0; }
  .waypoint-main label { grid-template-columns: 1fr; gap: 0.3rem; }
  .waypoint-heading-input { grid-template-columns: minmax(0, 1fr) 18px auto; }
  .waypoint-heading-input input { min-width: 0; }
  .waypoint-list { height: auto; }
  .drill-timeline-panel { max-height: 54px; padding: 0.7rem; }
  .drill-timeline-panel.open { max-height: 360px; }
  .drill-timeline-summary { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  .keyframe-pagination { justify-content: space-between; gap: 0.4rem; }
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
