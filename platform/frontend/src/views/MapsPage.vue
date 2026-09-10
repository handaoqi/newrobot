<script setup>
import { nextTick, onMounted, ref, computed, watch } from 'vue'
import { useAsyncPoller } from '../composables/useAsyncPoller'
import {
  canRetryFailedMappingSave,
  hasActiveMappingWorkflow,
  isActiveMappingState,
} from '../utils/mappingWorkflowState'
import { hasRescueMetadata, parseMapDescription } from '../utils/mapDescription'
import {
  fetchMapSummaries,
  fetchMapSetSummaries,
  fetchMapDetail,
  fetchMapMappingTrace,
  fetchRobots,
  deleteMap,
  downloadMap,
  setActiveMap,
  manuallyCleanMap,
  fetchMapLoopReview,
  optimizeMapLoops,
  fetchRobotCommand,
  createMap,
  fetchRobotMappingStatus,
  startRobotMappingOrigin,
  cancelRobotMappingOrigin,
  extractRobotMappingGlobalEnu,
  startRobotMappingSlam,
  beginRobotMapping,
  saveRobotMapping,
  cancelRobotMapping,
  syncRobotMapping,
} from '../services/api'

const maps = ref([])
const mapSets = ref([])
const robots = ref([])
const loading = ref(false)
const uploading = ref(false)
const showUploadDialog = ref(false)
const mappingBusy = ref(false)
const mappingCancelInFlight = ref(false)
const mappingStepFeedback = ref(null)
const mappingStatus = ref(null)
const mappingResult = ref({})
const mappingResultMapId = ref('')
const mappingResultRobotId = ref('')
const selectedMapId = ref(null)
const mapImageError = ref({})
const syncing = ref(false)
const activatingMapId = ref(null)
const collapsedMapGroups = ref(new Set())
const showCleaner = ref(false)
const cleanerCanvas = ref(null)
const cleanerViewport = ref(null)
const cleanerTool = ref('erase')
const cleanerBrushM = ref(0.5)
const cleanerZoom = ref(1)
const cleanerStrokes = ref([])
const cleanerRedoStrokes = ref([])
const cleanerSaving = ref(false)
const cleanerShowTrace = ref(true)
const cleanerImageSize = ref({ width: 0, height: 0 })
const mappingTrace = ref(null)
const mappingTraceLoading = ref(false)
const showMappingTrace = ref(true)
const globalEnuBusy = ref(false)
const showLoopReview = ref(false)
const loopReviewLoading = ref(false)
const loopOptimizeBusy = ref(false)
const loopReview = ref(null)
const selectedLoopCandidates = ref(new Set())
const loopOptimizeStatus = ref('')
const loopThresholds = ref({
  max_rank: 1,
  descriptor_distance_max: 0.30,
  geometric_translation_max_m: 2.50,
  yaw_consistency_max_deg: 34.38,
  slam_distance_min_m: 2.0,
  slam_distance_max_m: 20.0,
  max_selected_loops: 30,
  max_pose_correction_m: 5.0,
  max_yaw_correction_deg: 20.0,
})
let cleanerImage = null
let activeCleanerStroke = null
let cleanerPanStart = null
let mappingStatusRefreshing = false

const mappingForm = ref({
  robot: '',
  map_name: '太阳宫园区 V1',
  route_hint: '南门 → 主步道 → 牡丹园 → 活动广场',
  record_rosbag: true,
  scene_scope: 'indoor',
  mapping_type: 'indoor',
})

function setMappingStepFeedback(step, success, message) {
  const outdoor = mappingForm.value.mapping_type === 'outdoor'
  const stepNumbers = outdoor
    ? {
        '启动并检查': 2,
        '启动 SLAM 并检查航向': 2,
        '锁定 ENU 原点': 3,
        '航向复核': 4,
        '停止并保存地图': 5,
      }
    : {
        '启动并检查': 2,
        '检查确认': 3,
        '停止并保存地图': 4,
      }
  const normalizedStep = stepNumbers[step]
    ? `第${stepNumbers[step]}步：${step}命令`
    : step === '取消建图'
      ? '清理命令：取消建图'
      : step
  mappingStepFeedback.value = { step: normalizedStep, success, message, at: Date.now() }
}
let lastSlamAlert = ''

watch(() => mappingForm.value.mapping_type, (mappingType) => {
  if (mappingType === 'indoor') mappingForm.value.scene_scope = 'indoor'
  else if (mappingForm.value.scene_scope === 'indoor') mappingForm.value.scene_scope = 'outdoor'
})

const uploadForm = ref({
  name: '',
  pgm_file: null,
  yaml_file: null,
  thumbnail: null,
  resolution: 0.05,
  description: '',
})

const mappingStatusPoller = useAsyncPoller(async (signal) => {
  if (mappingForm.value.robot) await refreshMappingStatus({ signal })
}, { intervalMs: 1_000 })

onMounted(async () => {
  await loadMaps()
  await loadRobots()
  restorePendingMappingResult(mappingForm.value.robot)
  if (mappingForm.value.robot) await refreshMappingStatus()
})

// 预览 URL（用绝对 URL 避免相对路径问题）
const fullPreviewUrl = (relativeUrl) => {
  if (!relativeUrl) return ''
  if (relativeUrl.startsWith('http')) return relativeUrl
  return window.location.origin + relativeUrl
}

const selectedRobot = computed(() => {
  const robot = robots.value.find(item => String(item.id) === String(mappingForm.value.robot))
  if (robot) return robot
  if (!mappingForm.value.robot) return null
  return {
    id: mappingForm.value.robot,
    name: mappingStatus.value?.robot_code || selectedMap.value?.robot_name || '机器狗',
    code: mappingStatus.value?.robot_code || selectedMap.value?.robot_code || `#${mappingForm.value.robot}`,
  }
})

function mappingResultStorageKey(robotId) {
  return `roamerx.mapping-result:${String(robotId || '')}`
}

function restorePendingMappingResult(robotId) {
  mappingResultRobotId.value = String(robotId || '')
  mappingResultMapId.value = ''
  mappingResult.value = {}
  if (!robotId || typeof window === 'undefined') return
  try {
    const saved = JSON.parse(window.localStorage.getItem(mappingResultStorageKey(robotId)) || 'null')
    if (!saved || typeof saved !== 'object' || !saved.metrics || typeof saved.metrics !== 'object') return
    mappingResultMapId.value = String(saved.map_id || '')
    mappingResult.value = {
      ...saved.metrics,
      completion_step: Number(saved.completion_step || 0),
      mapping_type: saved.mapping_type || '',
    }
  } catch (error) {
    console.warn('恢复本次建图结果失败:', error)
  }
}

function publishPendingMappingResult(mapId, metrics) {
  if (!metrics || typeof metrics !== 'object' || !Object.keys(metrics).length) return
  const robotId = mappingForm.value.robot
  const completionStep = mappingForm.value.mapping_type === 'outdoor' ? 15 : 12
  mappingResultRobotId.value = String(robotId || '')
  mappingResultMapId.value = String(mapId || '')
  mappingResult.value = {
    ...metrics,
    completion_step: completionStep,
    mapping_type: mappingForm.value.mapping_type,
  }
  if (!robotId || typeof window === 'undefined') return
  try {
    window.localStorage.setItem(mappingResultStorageKey(robotId), JSON.stringify({
      map_id: mappingResultMapId.value,
      metrics,
      completion_step: completionStep,
      mapping_type: mappingForm.value.mapping_type,
      created_at: new Date().toISOString(),
    }))
  } catch (error) {
    console.warn('保存本次建图结果失败:', error)
  }
}

function clearPendingMappingResult() {
  const robotId = mappingResultRobotId.value || mappingForm.value.robot
  if (robotId && typeof window !== 'undefined') {
    try {
      window.localStorage.removeItem(mappingResultStorageKey(robotId))
    } catch (error) {
      console.warn('清除本次建图结果失败:', error)
    }
  }
  mappingResultMapId.value = ''
  mappingResult.value = {}
}

watch(() => mappingForm.value.robot, (robot, previousRobot) => {
  if (String(robot || '') !== String(previousRobot || '')) restorePendingMappingResult(robot)
})

const selectedMapDetail = ref(null)
const selectedMap = computed(() => {
  const summary = maps.value.find(m => m.id === selectedMapId.value)
  if (!summary) return null
  return selectedMapDetail.value?.id === summary.id
    ? { ...summary, ...selectedMapDetail.value }
    : summary
})
const mapGroups = computed(() => {
  const groups = new Map()
  for (const map of maps.value) {
    const key = mapDateKey(map)
    if (!groups.has(key)) {
      groups.set(key, { key, label: mapDateLabel(key), maps: [], activeCount: 0 })
    }
    const group = groups.get(key)
    group.maps.push(map)
    if (map.active) group.activeCount += 1
  }
  return Array.from(groups.values())
})

function mapDateKey(map) {
  const raw = map.created_at || map.createdAt || map.uploaded_at || map.name || ''
  const timestampMatch = String(raw).match(/(\d{4})(\d{2})(\d{2})/)
  if (timestampMatch) return `${timestampMatch[1]}-${timestampMatch[2]}-${timestampMatch[3]}`
  const parsed = new Date(raw)
  if (!Number.isNaN(parsed.getTime())) return parsed.toISOString().slice(0, 10)
  return 'unknown'
}

function mapDateLabel(key) {
  if (key === 'unknown') return '未标注日期'
  const [year, month, day] = key.split('-')
  return `${year}年${month}月${day}日`
}

function isMapGroupExpanded(key) {
  return !collapsedMapGroups.value.has(key)
}

function toggleMapGroup(key) {
  const next = new Set(collapsedMapGroups.value)
  if (next.has(key)) next.delete(key)
  else next.add(key)
  collapsedMapGroups.value = next
}

function expandAllMapGroups() {
  collapsedMapGroups.value = new Set()
}

function collapseAllMapGroups() {
  collapsedMapGroups.value = new Set(mapGroups.value.map(group => group.key))
}

function resetMapGroupState() {
  const visibleKeys = new Set()
  if (mapGroups.value[0]) visibleKeys.add(mapGroups.value[0].key)
  for (const group of mapGroups.value) {
    if (group.activeCount) visibleKeys.add(group.key)
  }
  collapsedMapGroups.value = new Set(
    mapGroups.value.filter(group => !visibleKeys.has(group.key)).map(group => group.key),
  )
}

function ensureMapGroupExpanded(mapId) {
  const group = mapGroups.value.find(item => item.maps.some(map => map.id === mapId))
  if (!group) return
  if (collapsedMapGroups.value.has(group.key)) {
    const next = new Set(collapsedMapGroups.value)
    next.delete(group.key)
    collapsedMapGroups.value = next
  }
}

watch(selectedMapId, (mapId) => {
  ensureMapGroupExpanded(mapId)
  void loadSelectedMapDetail(mapId)
})
watch(selectedMapId, (mapId) => loadMappingTrace(mapId), { immediate: true })
const robotCurrentMap = computed(() => mappingStatus.value?.current_map || {})
const robotCurrentMapId = computed(() => String(robotCurrentMap.value.map_id || mappingStatus.value?.current_map_id || ''))
const robotCurrentMapVersion = computed(() => String(robotCurrentMap.value.map_version || mappingStatus.value?.current_map_version || ''))
const selectedMapVersion = computed(() => selectedMap.value ? `legacy-mapdata-${selectedMap.value.id}` : '')
const activeMapSync = computed(() => {
  if (!selectedMap.value) return { state: 'unknown', label: '未选择地图', className: 'status-unknown' }
  if (!selectedMap.value.robot) return { state: 'unbound', label: '未绑定机器狗', className: 'status-unknown' }
  if (connectionStatus.value !== 'online') return { state: 'offline', label: '机器狗离线', className: 'status-offline' }
  const localState = robotCurrentMap.value.local_state
  if (localState && localState !== 'applied') {
    return { state: 'error', label: robotCurrentMap.value.last_activation_error || localState, className: 'status-offline' }
  }
  if (robotCurrentMapId.value === String(selectedMap.value.id) && robotCurrentMapVersion.value === selectedMapVersion.value) {
    return { state: 'synced', label: '机器狗端已应用', className: 'status-online' }
  }
  return { state: 'mismatch', label: '平台与机器狗端不一致', className: 'status-offline' }
})

const selectedMapDescription = computed(() => parseMapDescription(selectedMap.value?.description))
const selectedMapIsRescue = computed(() => hasRescueMetadata(selectedMap.value?.description))
const selectedMapMetrics = computed(() => (
  selectedMap.value?.mapping_metrics && Object.keys(selectedMap.value.mapping_metrics).length
    ? selectedMap.value.mapping_metrics
    : selectedMapDescription.value.mapping_metrics || {}
))
const selectedMapPackageFiles = computed(() => {
  const files = selectedMapDescription.value.package_files || selectedMapDescription.value.files || []
  return Array.isArray(files) ? files : []
})
const selectedMapOptimization = computed(() => {
  const explicit = selectedMap.value?.optimization_summary
  if (explicit && Object.keys(explicit).length) return explicit
  const traceValue = mappingTrace.value?.optimization
  if (traceValue && Object.keys(traceValue).length) return traceValue
  const descriptionValue = selectedMapDescription.value.optimization
  if (descriptionValue && typeof descriptionValue === 'object') return descriptionValue
  return selectedMapDescription.value.map_manifest?.optimization || {}
})
const runtimeOptimization = computed(() => mappingStatus.value?.result?.optimization || {})
const globalEnu = computed(() => mappingStatus.value?.result?.global_enu || {})
const currentMapGlobalEnu = computed(() => (
  selectedMap.value && String(globalEnu.value.source_map_id || '') === String(selectedMap.value.id)
    ? globalEnu.value
    : {}
))
const currentMapConfirmedHeading = computed(() => {
  const confirmedRaw = currentMapGlobalEnu.value.confirmed_heading_deg
  const confirmed = Number(confirmedRaw)
  if (confirmedRaw !== null && confirmedRaw !== undefined && confirmedRaw !== '' && Number.isFinite(confirmed)) {
    return { value: confirmed, confirmed: true }
  }
  const lockSnapshotRaw = currentMapGlobalEnu.value.heading_deg
  const lockSnapshot = Number(lockSnapshotRaw)
  if (lockSnapshotRaw !== null && lockSnapshotRaw !== undefined && lockSnapshotRaw !== '' && Number.isFinite(lockSnapshot)) {
    return { value: lockSnapshot, confirmed: false }
  }
  return null
})
const currentMapEnuToMapYawDeg = computed(() => {
  const yawRaw = currentMapGlobalEnu.value.enu_to_map_yaw
  const yaw = Number(yawRaw)
  return yawRaw !== null && yawRaw !== undefined && yawRaw !== '' && Number.isFinite(yaw)
    ? yaw * 180 / Math.PI
    : null
})
const mapArtifactFiles = computed(() => [
  { key: 'map.pcd', label: '点云地图', match: (name) => name === 'map.pcd' },
  { key: 'keyframes/keyframes.csv', label: '关键帧位置', match: (name) => name === 'keyframes/keyframes.csv' || name === 'trajectory_optimized.csv' || name === 'trajectory_raw.csv' },
  { key: 'scan_context/index.json', label: '指纹库', match: (name) => name === 'scan_context/index.json' || name === 'scan_context/loop_candidates.csv' },
  { key: 'optimization_summary.json', label: '回环优化摘要', match: (name) => name === 'optimization_summary.json' },
])

function tracePoints(poseNames) {
  const map = selectedMap.value
  const samples = mappingTrace.value?.samples || []
  if (!map || !samples.length || !Number(map.resolution) || !Number(map.width) || !Number(map.height)) return []
  const origin = Array.isArray(map.origin) ? map.origin : [0, 0, 0]
  return samples.map((sample) => {
    const pose = poseNames.map(name => sample[name]).find(value => value && typeof value === 'object') || {}
    return [
      (Number(pose.x || 0) - Number(origin[0] || 0)) / Number(map.resolution),
      Number(map.height) - (Number(pose.y || 0) - Number(origin[1] || 0)) / Number(map.resolution),
    ]
  }).filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y))
}
const mappingTraceRawPoints = computed(() => tracePoints(['raw', 'slam', 'pose']))
const mappingTraceOptimizedPoints = computed(() => tracePoints(['optimized', 'slam', 'pose']))
const mappingTracePoints = computed(() => mappingTraceOptimizedPoints.value)
const mappingTraceRawSvgPoints = computed(() => mappingTraceRawPoints.value.map(([x, y]) => `${x.toFixed(2)},${y.toFixed(2)}`).join(' '))
const mappingTraceSvgPoints = computed(() => mappingTracePoints.value.map(([x, y]) => `${x.toFixed(2)},${y.toFixed(2)}`).join(' '))
const mappingCorrectionArrows = computed(() => {
  const map = selectedMap.value
  if (!map || !Number(map.resolution) || !Number(map.height)) return []
  const origin = Array.isArray(map.origin) ? map.origin : [0, 0, 0]
  const toPoint = (pose) => ({
    x: (Number(pose?.x || 0) - Number(origin[0] || 0)) / Number(map.resolution),
    y: Number(map.height) - (Number(pose?.y || 0) - Number(origin[1] || 0)) / Number(map.resolution),
  })
  return (mappingTrace.value?.samples || [])
    .filter(sample => Number(sample.correction?.position_m || 0) >= 0.30 || Math.abs(Number(sample.correction?.yaw_deg || 0)) >= 3)
    .map(sample => ({
      index: sample.index,
      from: toPoint(sample.raw),
      to: toPoint(sample.optimized),
      yawSignificant: Math.abs(Number(sample.correction?.yaw_deg || 0)) >= 3,
    }))
    .filter(item => [item.from.x, item.from.y, item.to.x, item.to.y].every(Number.isFinite))
})

async function loadMappingTrace(mapId) {
  mappingTrace.value = null
  if (!mapId) return
  mappingTraceLoading.value = true
  try {
    mappingTrace.value = await fetchMapMappingTrace(mapId)
  } catch (error) {
    console.warn('加载建图轨迹失败:', error)
  } finally {
    mappingTraceLoading.value = false
  }
}

async function handleExtractGlobalEnu() {
  if (!selectedMap.value?.robot || globalEnuBusy.value) return
  if (!confirm(`确认将地图“${selectedMap.value.name}”的锁定原点提取为机器狗全局 ENU 配置吗？`)) return
  globalEnuBusy.value = true
  try {
    await extractRobotMappingGlobalEnu(selectedMap.value.robot, { map_id: selectedMap.value.id })
    const deadline = Date.now() + 30 * 1000
    while (Date.now() < deadline) {
      await new Promise(resolve => setTimeout(resolve, 800))
      await refreshMappingStatus()
      if (Object.keys(currentMapGlobalEnu.value).length) break
      if (['failed', 'command_failed', 'command_rejected', 'command_timed_out'].includes(mappingStatus.value?.mapping_state)) {
        throw new Error(mappingStatus.value?.error_message || 'Edge Agent 提取全局 ENU 失败')
      }
    }
    if (!Object.keys(currentMapGlobalEnu.value).length) throw new Error('Edge Agent 尚未返回当前地图的全局 ENU 配置，请刷新状态')
  } catch (error) {
    alert(`提取全局 ENU 失败: ${error.message}`)
  } finally {
    globalEnuBusy.value = false
  }
}

// 连接状态
const connectionStatus = computed(() => mappingStatus.value?.connection_status || 'unknown')
const connectionLabel = computed(() => {
  const labels = { online: '已连接', offline: '已断线', unknown: '未知' }
  return labels[connectionStatus.value] || connectionStatus.value
})
const connectionClass = computed(() => {
  if (connectionStatus.value === 'online') return 'status-online'
  if (connectionStatus.value === 'offline') return 'status-offline'
  return 'status-unknown'
})

// 建图状态机
const commandStatus = computed(() => mappingStatus.value?.command_status || 'idle')
const completedUploadMapId = computed(() => (
  mappingStatus.value?.result?.upload_result?.id
  || (mappingStatus.value?.command_type === 'mapping.save' && commandStatus.value === 'succeeded'
    ? mappingStatus.value?.latest_map?.id
    : '')
))
const mappingState = computed(() => (
  mappingStatus.value?.command_type === 'mapping.save'
  && commandStatus.value === 'succeeded'
  && completedUploadMapId.value
    ? 'exited'
    : (mappingStatus.value?.mapping_state || 'idle')
))
const mappingCommandInFlight = computed(() => ['created', 'published', 'accepted', 'executing'].includes(commandStatus.value))
const saveProgress = computed(() => mappingStatus.value?.result?.save_progress || {})
const postSaveValidation = computed(() => mappingStatus.value?.result?.post_save_validation || {})
const indoorValidation = computed(() => postSaveValidation.value.indoor || { required: 10, success: 0, attempts: 0 })
const outdoorValidation = computed(() => postSaveValidation.value.outdoor || { required: 5, success: 0, attempts: 0 })
const validationPercent = bucket => Math.min(100, Number(bucket?.success || 0) / Math.max(1, Number(bucket?.required || 1)) * 100)
const postSaveValidationResult = computed(() => postSaveValidation.value.result || {})
const validationStateLabel = computed(() => {
  if (postSaveValidation.value.state === 'passed') {
    return postSaveValidationResult.value.accurate === true ? '本次定位准确' : '本轮通过'
  }
  return ({ queued: '排队中', running: '正在加载本次地图并定位', failed: '本次定位不通过', unavailable: '待接入验证器', idle: '未开始' }[postSaveValidation.value.state] || postSaveValidation.value.state || '未开始')
})
const validationMappingTypeLabel = computed(() => (
  (postSaveValidationResult.value.mapping_type || postSaveValidation.value.mapping_type) === 'outdoor'
    ? '室外'
    : '室内'
))
const validationCoordinateLabel = computed(() => {
  const mode = postSaveValidationResult.value.coordinate_mode
  if (mode === 'global_enu') return '全局 ENU / map'
  if (mode === 'local_only') return '本地 map'
  return 'map'
})
const finiteValidationNumber = value => {
  if (value === null || value === undefined || value === '') return null
  const number = Number(value)
  return Number.isFinite(number) ? number : null
}
const formatValidationPose = pose => {
  const x = finiteValidationNumber(pose?.x)
  const y = finiteValidationNumber(pose?.y)
  const z = finiteValidationNumber(pose?.z)
  const yaw = finiteValidationNumber(pose?.yaw_deg)
  if (x === null || y === null || yaw === null) return '未取得定位坐标'
  return `X ${x.toFixed(2)} m · Y ${y.toFixed(2)} m${z === null ? '' : ` · Z ${z.toFixed(2)} m`} · 航向 ${yaw.toFixed(1)}°`
}
const validationPoseLabel = computed(() => formatValidationPose(postSaveValidationResult.value.pose))
const validationSavedPoseLabel = computed(() => formatValidationPose(postSaveValidationResult.value.saved_terminal_pose))
const validationDiagnostics = computed(() => postSaveValidationResult.value.diagnostics || {})
const validationSeedLabel = computed(() => {
  const source = validationDiagnostics.value.seed_source
  if (source === 'saved_terminal_pose') {
    return validationDiagnostics.value.seed_published ? '保存终点初始位姿已发布' : '保存终点初始位姿未发布'
  }
  if (source === 'rtk_map_initialization') return '候选地图 RTK/ENU 初始化'
  return '初始化来源未知'
})
const validationSampleLabel = computed(() => (
  `定位 ${Number(validationDiagnostics.value.fresh_localization_samples || 0)} 帧`
  + ` · 匹配 ${Number(validationDiagnostics.value.fresh_scan_match_samples || 0)} 帧`
  + ` · 姿态 ${Number(validationDiagnostics.value.fresh_pose_samples || 0)} 帧`
))
const formatValidationMetric = (value, digits, suffix) => {
  const number = finiteValidationNumber(value)
  return number === null ? '无数据' : `${number.toFixed(digits)}${suffix}`
}
const validationResultTime = computed(() => {
  const value = postSaveValidationResult.value.sampled_at || postSaveValidation.value.updated_at
  if (!value) return ''
  const date = new Date(value)
  return Number.isNaN(date.getTime()) ? value : date.toLocaleString('zh-CN', { hour12: false })
})
let lastValidationNotice = ''
watch(
  () => `${postSaveValidation.value.map_dir || ''}:${postSaveValidation.value.state || ''}:${postSaveValidation.value.updated_at || ''}`,
  (signature) => {
    const state = postSaveValidation.value.state
    if (!['passed', 'failed'].includes(state) || signature === lastValidationNotice) return
    lastValidationNotice = signature
    const result = postSaveValidationResult.value
    setMappingStepFeedback(
      '保存后静止定位自检',
      state === 'passed' && result.accurate === true,
      `${result.message || validationStateLabel.value}；${formatValidationPose(result.pose)}`,
    )
  },
)
// Runtime result is intentionally independent from the selected map. It is
// published only after the package has been uploaded, and is cleared by the
// next successful startup/check command.
const mappingMetrics = computed(() => mappingResult.value)
const rosbagStatus = computed(() => mappingStatus.value?.result?.rosbag || {})
const mappingOdometry = computed(() => mappingStatus.value?.result?.odometry || {})
const mappingOdometrySourceLabel = computed(() => {
  if (mappingOdometry.value.source === 'slam') return 'FAST-LIO-SAM'
  if (mappingOdometry.value.source === 'localization') return 'Localization'
  return '未知来源'
})
const originStatus = computed(() => mappingStatus.value?.result?.origin || {})
const originState = computed(() => originStatus.value.origin_status || 'idle')
const isOutdoorMapping = computed(() => mappingForm.value.mapping_type === 'outdoor')
const originLocked = computed(() => originState.value === 'locked')
const rtkAlignment = computed(() => (
  mappingStatus.value?.result?.rtk_alignment
  || saveProgress.value.rtk_alignment
  || {}
))
const rtkAlignmentLabel = computed(() => {
  if (!isOutdoorMapping.value) return ''
  if (rtkAlignment.value.locked) {
    if (rtkAlignment.value.source === 'heading') return 'ENU-地图航向 双天线已锁'
    if (rtkAlignment.value.source === 'trajectory') return 'ENU-地图航向 轨迹已锁'
    return 'ENU-地图航向 已锁定'
  }
  if (mappingStatus.value?.result?.mapping_capture_enabled) return 'ENU-地图航向 待锁定'
  if (originLocked.value) return 'ENU-地图航向 确认后锁定'
  return ''
})
const headingReviewStatus = computed(() => originStatus.value.heading_review_status || 'idle')
const originQualityRequired = computed(() => Number(originStatus.value.origin_quality_required_seconds || 10))
const originQualityElapsed = computed(() => Number(originStatus.value.continuous_seconds || 0))
const originQualityPercent = computed(() => {
  const required = originQualityRequired.value
  return required > 0 ? Math.min(100, originQualityElapsed.value / required * 100) : 0
})
const headingBaselineReady = computed(() => (
  Number(originStatus.value.baseline_m || 0) >= Number(originStatus.value.heading_min_baseline_m || 0.2)
))
const headingAccuracyReady = computed(() => (
  Number(originStatus.value.heading_std_deg ?? Infinity) <= Number(originStatus.value.heading_max_std_deg || 5)
  && Number(originStatus.value.data_age_seconds ?? originStatus.value.age_seconds ?? Infinity) <= Number(originStatus.value.heading_max_age_seconds || 1.5)
))
const originQualityLabel = computed(() => {
  const labels = {
    rtk_fixed: 'RTK FIX',
    fixed: 'RTK FIX',
    rtk_float: 'RTK FLOAT',
    float: 'RTK FLOAT',
    standalone: '单点解',
    invalid: '无效解',
  }
  const quality = String(originStatus.value.ntrip_quality || '').toLowerCase()
  return labels[quality] || (quality ? quality.toUpperCase() : '等待数据')
})
const originDataStale = computed(() => Number(originStatus.value.data_age_seconds ?? originStatus.value.age_seconds ?? 0) > 1.5)
const rtkTelemetryOnline = computed(() => (
  Number.isFinite(Number(originStatus.value.data_age_seconds ?? originStatus.value.age_seconds))
  && !originDataStale.value
))

function formatOriginNumber(value, digits = 3, suffix = '') {
  if (value === null || value === undefined || value === '' || !Number.isFinite(Number(value))) return '—'
  return `${Number(value).toFixed(digits)}${suffix}`
}

function formatOriginStamp(value) {
  const stamp = Number(value)
  if (!Number.isFinite(stamp) || stamp <= 0) return '—'
  return new Date(stamp * 1000).toLocaleString('zh-CN', { hour12: false })
}

const originDiagnosticRows = computed(() => [
  { label: '解类型', value: originQualityLabel.value, tone: originStatus.value.position_fixed ? 'ok' : 'warn' },
  { label: 'NTRIP 状态', value: originStatus.value.ntrip_state || '未上报' },
  { label: '解状态 / 类型', value: `${originStatus.value.solution_status ?? '—'} / ${originStatus.value.position_type ?? '—'}` },
  { label: '跟踪 / 解算卫星', value: `${originStatus.value.tracking_satellites ?? '—'} / ${originStatus.value.solution_satellites ?? '—'}` },
  { label: '水平 / 垂直误差', value: `${formatOriginNumber(originStatus.value.horizontal_std_m, 3, ' m')} / ${formatOriginNumber(originStatus.value.vertical_std_m, 3, ' m')}` },
  { label: '经度', value: formatOriginNumber(originStatus.value.longitude, 10) },
  { label: '纬度', value: formatOriginNumber(originStatus.value.latitude, 10) },
  { label: '高程', value: formatOriginNumber(originStatus.value.altitude, 3, ' m') },
  { label: '航向状态 / 类型', value: `${originStatus.value.heading_status ?? '—'} / ${originStatus.value.heading_type ?? '—'}`, tone: originStatus.value.heading_fixed ? 'ok' : 'warn' },
  { label: '接收机航向 / 标准差', value: `${formatOriginNumber(originStatus.value.heading_deg, 2, '°')} / ${formatOriginNumber(originStatus.value.heading_std_deg, 2, '°')}` },
  { label: '航向安装补偿', value: formatOriginNumber(originStatus.value.heading_offset_deg, 1, '°') },
  { label: '人工确认原点航向', value: formatOriginNumber(originStatus.value.confirmed_heading_deg, 2, '°'), tone: originStatus.value.heading_confirmed ? 'ok' : '' },
  { label: '航向确认时间', value: formatOriginStamp(originStatus.value.heading_confirmed_at_unix) },
  { label: '俯仰 / 标准差', value: `${formatOriginNumber(originStatus.value.pitch_deg, 2, '°')} / ${formatOriginNumber(originStatus.value.pitch_std_deg, 2, '°')}` },
  { label: '双天线基线', value: formatOriginNumber(originStatus.value.baseline_m, 3, ' m') },
  { label: '航向跟踪 / 解算卫星', value: `${originStatus.value.heading_satellites ?? '—'} / ${originStatus.value.heading_solution_satellites ?? '—'}` },
  { label: '差分龄期', value: formatOriginNumber(originStatus.value.differential_age_seconds, 2, ' s') },
  { label: '消息时间偏差', value: formatOriginNumber(originStatus.value.message_time_offset_seconds, 3, ' s'), tone: originDataStale.value ? 'bad' : 'ok' },
  { label: '数据延迟', value: formatOriginNumber(originStatus.value.data_age_seconds ?? originStatus.value.age_seconds, 3, ' s'), tone: originDataStale.value ? 'bad' : 'ok' },
  { label: '测量时间源', value: originStatus.value.measurement_time_source || '未上报' },
  { label: 'RTK 测量时间', value: formatOriginStamp(originStatus.value.measurement_stamp) },
])
const originDiagnosticTableRows = computed(() => {
  const rows = []
  for (let index = 0; index < originDiagnosticRows.value.length; index += 2) {
    rows.push(originDiagnosticRows.value.slice(index, index + 2))
  }
  return rows
})
const originStatusMessage = computed(() => {
  if (originStatus.value.message) return originStatus.value.message
  if (!originStatus.value.position_fixed) return '等待 RTK 位置 FIX，暂不能锁定 ENU 原点'
  if (!originLocked.value) return '点击锁定后，将连续检查位置 FIX、双天线航向 FIX 和坐标波动＜2 cm 达到 10 秒'
  if (headingReviewStatus.value === 'manual_confirmation') return '请原地小范围转动，确认航向稳定后点击“确认航向稳定，开始建图”'
  return 'ENU 原点已锁定，等待启动 SLAM 和航向复核'
})
const slamHealth = computed(() => saveProgress.value.slam_health || {})
const slamHealthState = computed(() => slamHealth.value.state || 'unknown')
const slamDiverged = computed(() => (
  saveProgress.value.error_code === 'SLAM_DIVERGED' || slamHealthState.value === 'diverged'
))
const slamDegraded = computed(() => slamHealthState.value === 'degraded')
const slamHealthIssue = computed(() => slamDiverged.value || slamDegraded.value)
const slamHealthMessage = computed(() => (
  saveProgress.value.error || slamHealth.value.warning || ''
))
const slamHealthLabel = computed(() => {
  if (slamDiverged.value) return 'SLAM 已发散'
  if (slamDegraded.value) return '定位质量正在恶化'
  if (slamHealthState.value === 'healthy') return 'SLAM 正常'
  if (slamHealthState.value === 'initializing') return 'SLAM 初始化中'
  return 'SLAM 状态未知'
})
const mappingReadiness = computed(() => {
  if (['command_created', 'command_published', 'command_accepted', 'starting'].includes(mappingState.value)) {
    return {
      state: 'starting',
      message: '正在启动 SLAM，等待本次建图会话的雷达和 IMU 数据',
      ready_for_motion: false,
      ready_for_save: false,
      imu_initialized: false,
      keyframe_count: 0,
    }
  }
  const reported = mappingStatus.value?.result?.readiness
  if (reported?.state) return reported
  const processAlive = Boolean(mappingStatus.value?.result?.process_alive)
  const imuInitialized = Boolean(slamHealth.value.imu_initialized)
  const keyframeCount = Number(saveProgress.value.written_keyframes || saveProgress.value.keyframe_count || 0)
  if (!processAlive) return { state: 'offline', message: '建图进程未运行', ready_for_motion: false }
  if (!imuInitialized) {
    return {
      state: 'imu_initializing',
      message: `IMU 初始化中（已采样 ${Number(slamHealth.value.imu_samples || 0)}），请保持机器狗静止`,
      ready_for_motion: false,
    }
  }
  if (keyframeCount < 1) {
    return {
      state: 'waiting_first_keyframe',
      message: 'IMU 已初始化，正在建立首个有效关键帧，请继续保持静止',
      ready_for_motion: false,
    }
  }
  return { state: 'ready', message: '传感器和首个关键帧正常，可以开始移动建图', ready_for_motion: true }
})
const readinessState = computed(() => mappingReadiness.value.state || 'offline')
const readyForMotion = computed(() => Boolean(
  mappingStatus.value?.result?.ready_for_motion ?? mappingReadiness.value.ready_for_motion
))
const readyForSave = computed(() => Boolean(
  mappingStatus.value?.result?.ready_for_save
  ?? (Number(saveProgress.value.written_keyframes || saveProgress.value.keyframe_count || 0) > 0)
))
const readinessLabel = computed(() => {
  const labels = {
    starting: '正在启动建图',
    imu_initializing: '请保持静止',
    waiting_first_keyframe: '正在确认首帧',
    ready: '可以开始移动',
    telemetry_stale: '传感器状态超时',
    diverged: 'SLAM 已发散',
    offline: '建图未启动',
  }
  return labels[readinessState.value] || '建图状态未知'
})
const readinessClass = computed(() => ({
  'is-ready': readyForMotion.value,
  'is-error': ['telemetry_stale', 'diverged', 'offline'].includes(readinessState.value),
}))
const readinessSampleAge = computed(() => {
  const age = mappingReadiness.value.sample_age_seconds
  return age === null || age === undefined ? null : Number(age)
})
const mappingProcessAlive = computed(() => Boolean(mappingStatus.value?.result?.process_alive))
const mappingTelemetryFresh = computed(() => (
  mappingProcessAlive.value
  && readinessSampleAge.value !== null
  && readinessSampleAge.value <= 5
))
const mappingStartupChecks = computed(() => {
  const imuSamples = Number(mappingReadiness.value.imu_samples || slamHealth.value.imu_samples || 0)
  return [
    { key: 'edge', label: 'Edge Agent', detail: connectionStatus.value === 'online' ? '已连接' : '未连接', ok: connectionStatus.value === 'online' },
    { key: 'slam', label: 'SLAM 进程', detail: mappingProcessAlive.value ? '运行中' : '等待启动', ok: mappingProcessAlive.value },
    { key: 'lidar', label: '激光雷达', detail: mappingTelemetryFresh.value ? '数据正常' : '等待实时数据', ok: mappingTelemetryFresh.value },
    { key: 'imu_stream', label: 'IMU 数据', detail: mappingTelemetryFresh.value && imuSamples > 0 ? `${imuSamples} 帧` : '等待实时数据', ok: mappingTelemetryFresh.value && imuSamples > 0 },
    { key: 'imu_init', label: 'IMU 初始化', detail: mappingReadiness.value.imu_initialized ? '已完成' : '请保持静止', ok: Boolean(mappingReadiness.value.imu_initialized) },
    { key: 'pose', label: '有效 SLAM 位姿', detail: mappingReadiness.value.slam_pose_ready ? '已确认（未采集正式关键帧）' : '等待位姿稳定', ok: Boolean(mappingReadiness.value.slam_pose_ready) },
  ]
})
const mappingStartupReady = computed(() => mappingStartupChecks.value.every(item => item.ok))
const saveStageLabels = {
  initializing_imu: 'IMU 初始化',
  waiting_first_keyframe: '建立有效 SLAM 位姿',
  ready_to_map: '等待人工确认正式采集',
  mapping: '采集关键帧',
  recovering: '恢复落盘关键帧',
  flushing_keyframes: '刷新关键帧',
  filtering: '过滤动态点',
  partitioning_filter: '分片统计动态点',
  writing_pcd: '写入点云',
  building_grid: '生成栅格图',
  writing_metadata: '写入地图信息',
  completed: '保存完成',
  failed: '保存失败',
  cancelled: '已取消建图',
}
const saveStageLabel = computed(() => saveStageLabels[saveProgress.value.stage] || saveProgress.value.stage || '')
const formatBytes = (value) => {
  const bytes = Number(value || 0)
  if (!bytes) return '0 B'
  const units = ['B', 'KB', 'MB', 'GB', 'TB']
  const index = Math.min(Math.floor(Math.log(bytes) / Math.log(1024)), units.length - 1)
  return `${(bytes / (1024 ** index)).toFixed(index > 1 ? 1 : 0)} ${units[index]}`
}
const formatDuration = (value) => {
  const seconds = Math.max(0, Math.round(Number(value || 0)))
  if (seconds < 60) return `${seconds} 秒`
  const minutes = Math.floor(seconds / 60)
  const remainder = seconds % 60
  if (minutes < 60) return `${minutes} 分 ${remainder} 秒`
  const hours = Math.floor(minutes / 60)
  return `${hours} 小时 ${minutes % 60} 分`
}
const optimizationStageLabels = {
  waiting: '等待触发',
  detecting: '回环检测中',
  optimizing: '全局优化中',
  trajectory_optimized: '轨迹优化完成',
  rebuilding_map: '重建地图中',
  completed: '回环优化完成',
  no_valid_loop: '无有效回环，保持原始轨迹',
  fallback: '优化异常，已回退原始地图',
  failed: '优化失败',
}
const formatAgo = (unix) => {
  const seconds = Math.max(0, Math.round(Date.now() / 1000 - Number(unix || 0)))
  if (!unix) return ''
  if (seconds < 60) return `${seconds}秒前`
  if (seconds < 3600) return `${Math.floor(seconds / 60)}分钟前`
  return `${Math.floor(seconds / 3600)}小时前`
}
function optimizationLine(value) {
  if (!value || !Object.keys(value).length) return ''
  const stage = optimizationStageLabels[value.stage] || value.stage || '等待触发'
  const accepted = Number(value.accepted_loop_count || 0)
  const candidates = Number(value.candidate_count || 0)
  const position = value.correction?.position_m || {}
  const yaw = value.correction?.yaw_deg || {}
  const reduction = value.graph_error?.reduction_percent
  const parts = [stage]
  if (candidates || accepted) parts.push(`接受 ${accepted}/${candidates} 个回环`)
  if (value.stage === 'completed') {
    parts.push(`平均位置 ${Number(position.mean || 0).toFixed(2)}m`)
    parts.push(`最大 ${Number(position.max || 0).toFixed(2)}m`)
    parts.push(`平均航向 ${Number(yaw.mean || 0).toFixed(1)}°`)
    parts.push(`最大 ${Number(yaw.max || 0).toFixed(1)}°`)
    if (reduction !== null && reduction !== undefined) parts.push(`图误差下降 ${Number(reduction).toFixed(1)}%`)
  }
  const age = formatAgo(value.updated_at_unix || value.completed_at_unix)
  if (age) parts.push(age)
  return parts.join(' · ')
}

const stateSteps = computed(() => [
  { key: 'idle', label: '配置' },
  { key: 'command_created', label: '已创建' },
  { key: 'command_published', label: '已下发' },
  { key: 'command_accepted', label: 'Edge确认' },
  ...(isOutdoorMapping.value ? [
    { key: 'origin_starting', label: '启动RTK' },
    { key: 'origin_waiting', label: '3秒锁定原点' },
    { key: 'origin_locked', label: '原点已锁' },
  ] : []),
  { key: 'slam_starting', label: '启动SLAM' },
  { key: 'slam_warmup', label: 'SLAM预热' },
  { key: 'imu_initializing', label: 'IMU初始化' },
  { key: 'waiting_first_keyframe', label: '位姿确认' },
  { key: 'ready_to_map', label: isOutdoorMapping.value ? '航向复核' : '等待确认' },
  { key: 'mapping', label: '正式采集' },
  { key: 'saving', label: '保存中' },
  { key: 'optimizing', label: '回环优化' },
  { key: 'packaging', label: '打包中' },
  { key: 'uploading', label: '上传中' },
  { key: 'stopping', label: '退出建图' },
  { key: 'exited', label: '已退出建图' },
])

const terminalStates = ['command_timed_out', 'command_failed', 'command_rejected', 'cancelled', 'completed', 'exited']
const isTerminal = computed(() => terminalStates.includes(mappingState.value))
const isError = computed(() => (
  slamDiverged.value
  || mappingState.value === 'failed'
  || originState.value === 'failed'
  || (mappingProcessAlive.value && readinessState.value === 'telemetry_stale')
  || ['command_timed_out', 'command_failed', 'command_rejected'].includes(mappingState.value)
))
const isActiveMapping = computed(() => isActiveMappingState(mappingState.value))
const displayMappingState = computed(() => {
  if (['slam_warmup', 'ready_to_map'].includes(mappingState.value)) {
    if (['imu_initializing', 'waiting_first_keyframe'].includes(readinessState.value)) return readinessState.value
    if (readinessState.value === 'ready') return 'ready_to_map'
  }
  if (mappingState.value !== 'mapping') return mappingState.value
  if (mappingStatus.value?.result?.mapping_capture_enabled) return 'mapping'
  if (readinessState.value === 'telemetry_stale') {
    if (!mappingReadiness.value.imu_initialized) return 'imu_initializing'
    return Number(mappingReadiness.value.keyframe_count || 0) > 0 ? 'mapping' : 'waiting_first_keyframe'
  }
  if (['starting', 'imu_initializing', 'waiting_first_keyframe'].includes(readinessState.value)) {
    return readinessState.value
  }
  return mappingState.value
})
const canRetrySave = computed(() => canRetryFailedMappingSave({
  commandType: mappingStatus.value?.command_type,
  commandStatus: commandStatus.value,
  uploadedMapId: completedUploadMapId.value,
  failureStepKey: isError.value ? failureStepKey.value : '',
}))
const canSaveMapping = computed(() => (
  !mappingCommandInFlight.value && (
    slamDiverged.value
    || (mappingState.value === 'mapping' && readyForSave.value)
    || canRetrySave.value
  )
))
const canCancelMapping = computed(() => (
  Boolean(selectedRobot.value)
  && hasActiveMappingWorkflow({
    mappingState: mappingState.value,
    commandStatus: commandStatus.value,
    processAlive: mappingProcessAlive.value,
    originState: originState.value,
  })
))
const showMappingReadiness = computed(() => (
  Boolean(mappingStatus.value?.result?.process_alive)
  || ['slam_starting', 'slam_warmup', 'ready_to_map', 'mapping', 'saving', 'optimizing', 'packaging', 'uploading', 'stopping'].includes(mappingState.value)
))
const workflowSessionId = computed(() => mappingStatus.value?.result?.mapping_session_id || '')
const mappingModeSwitchDisabled = computed(() => (
  mappingBusy.value
  || mappingCommandInFlight.value
  || mappingProcessAlive.value
  || ['slam_starting', 'slam_warmup', 'ready_to_map', 'mapping', 'saving', 'optimizing', 'packaging', 'uploading', 'stopping'].includes(mappingState.value)
))
const canLockOrigin = computed(() => (
  isOutdoorMapping.value
  && connectionStatus.value === 'online'
  && (mappingState.value === 'origin_waiting' || (isError.value && failureStepKey.value === 'origin_waiting'))
  && !mappingProcessAlive.value
  && !originLocked.value
  && !mappingCommandInFlight.value
  && !['waiting_fix', 'quality_holding'].includes(originState.value)
))
const canStartSlam = computed(() => (
  connectionStatus.value === 'online'
  && (!isError.value || ['slam_starting', 'slam_warmup', 'imu_initializing', 'waiting_first_keyframe'].includes(failureStepKey.value))
  && !mappingProcessAlive.value
  && !mappingCommandInFlight.value
  && (!isOutdoorMapping.value || originLocked.value || ['idle', 'cancelled', 'failed'].includes(originState.value))
))
const startSlamButtonLabel = computed(() => {
  if (mappingBusy.value) return '正在下发...'
  if (isError.value && ['slam_starting', 'slam_warmup', 'imu_initializing', 'waiting_first_keyframe'].includes(failureStepKey.value)) {
    return '重试启动并检查'
  }
  if (isOutdoorMapping.value && originLocked.value) return '启动 SLAM 并检查航向'
  if (isOutdoorMapping.value) return '启动传感器并检查'
  return '启动并检查'
})
const canBeginMapping = computed(() => (
  mappingProcessAlive.value
  && !mappingCommandInFlight.value
  && Boolean(mappingStatus.value?.result?.ready_for_mapping ?? mappingReadiness.value.ready_for_mapping)
  && (!isOutdoorMapping.value || originLocked.value)
  && !mappingStatus.value?.result?.mapping_capture_enabled
  && (!isError.value || failureStepKey.value === 'ready_to_map')
))

const activeStepIndex = computed(() => {
  if (isError.value) {
    const failedIndex = stateSteps.value.findIndex(step => step.key === failureStepKey.value)
    if (failedIndex >= 0) return failedIndex
  }
  if (slamDiverged.value) return stateSteps.value.findIndex(s => s.key === 'mapping')
  if (['command_created', 'command_published', 'command_accepted'].includes(mappingState.value)) {
    const targetByCommand = {
      'mapping.slam_start': 'slam_starting',
      'mapping.begin': 'ready_to_map',
      'mapping.save': 'saving',
      'mapping.cancel': 'stopping',
      'mapping.origin_cancel': 'origin_waiting',
    }
    const target = targetByCommand[mappingStatus.value?.command_type]
    if (target) return stateSteps.value.findIndex(step => step.key === target)
  }
  if (
    mappingStatus.value?.command_type === 'mapping.save' &&
    ['command_created', 'command_published', 'command_accepted', 'command_failed', 'command_timed_out', 'command_rejected'].includes(mappingState.value)
  ) {
    return stateSteps.value.findIndex(s => s.key === 'saving')
  }
  const idx = stateSteps.value.findIndex(s => s.key === displayMappingState.value)
  return idx >= 0 ? idx : -1
})

const isPastStep = (stepIdx) => {
  if (activeStepIndex.value < 0) return false
  if (isTerminal.value && !isError.value && stepIdx <= stateSteps.value.length - 1) return true
  if (isError.value) return stepIdx < activeStepIndex.value
  return stepIdx < activeStepIndex.value
}

const isCurrentStep = (stepIdx) => stepIdx === activeStepIndex.value && activeStepIndex.value >= 0

const errorLabel = computed(() => {
  const labels = {
    command_timed_out: '命令超时',
    command_failed: '命令失败',
    command_rejected: '命令被拒',
    failed: '状态机停止',
    cancelled: '已取消',
  }
  if (readinessState.value === 'telemetry_stale') return '传感器状态超时'
  return slamDiverged.value ? 'SLAM 发散' : (labels[mappingState.value] || '错误')
})

const failureStepKey = computed(() => {
  if (originState.value === 'failed' || originStatus.value.error_code === 'RTK_SIGNAL_TIMEOUT') return 'origin_waiting'
  const commandStepKeys = {
    'mapping.origin_start': 'origin_waiting',
    'mapping.slam_start': 'slam_starting',
    'mapping.begin': 'ready_to_map',
    'mapping.save': 'saving',
    'mapping.cancel': 'stopping',
  }
  if (commandStepKeys[mappingStatus.value?.command_type]) return commandStepKeys[mappingStatus.value.command_type]
  if (slamDiverged.value) return 'mapping'
  if (['telemetry_stale', 'diverged'].includes(readinessState.value)) return displayMappingState.value
  if (stateSteps.value.some(step => step.key === mappingState.value)) return mappingState.value
  return 'idle'
})

const failureStepIndex = computed(() => stateSteps.value.findIndex(step => step.key === failureStepKey.value))
const failureStepLabel = computed(() => (
  failureStepIndex.value >= 0 ? stateSteps.value[failureStepIndex.value].label : failureStepKey.value
))
const failureMessage = computed(() => (
  mappingStatus.value?.error_message
  || mappingStatus.value?.last_command_error_message
  || (originState.value === 'failed' ? originStatus.value.message : '')
  || saveProgress.value.error
  || slamHealthMessage.value
  || '状态机在该步骤停止，未返回具体错误信息'
))
const failureCode = computed(() => (
  mappingStatus.value?.error_code
  || mappingStatus.value?.last_command_error_code
  || originStatus.value.error_code
  || saveProgress.value.error_code
  || ''
))

const slamAlertSignature = computed(() => (
  slamDiverged.value
    ? `${mappingStatus.value?.result?.save_progress?.error_code || 'SLAM_DIVERGED'}:${slamHealthMessage.value}`
    : ''
))

watch(slamAlertSignature, (signature) => {
  if (!signature || signature === lastSlamAlert) return
  lastSlamAlert = signature
  window.alert(`SLAM 已发散，地图已停止记录。请停止并保存以生成救援地图。\n${slamHealthMessage.value}`)
})

const mappingStateLabel = computed(() => {
  const step = stateSteps.value.find(s => s.key === displayMappingState.value)
  if (step) return step.label
  if (mappingState.value === 'command_issued') return '等待Edge'
  if (mappingState.value === 'completed') return '已完成'
  return mappingState.value
})

async function loadRobots() {
  try {
    robots.value = await fetchRobots()
    if (!mappingForm.value.robot && robots.value.length) {
      mappingForm.value.robot = robots.value[0].id
    }
  } catch (error) {
    console.error('加载机器人失败:', error)
    seedRobotsFromMaps()
  }
}

async function loadMaps() {
  loading.value = true
  try {
    const [loadedMaps, loadedMapSets] = await Promise.all([fetchMapSummaries(), fetchMapSetSummaries()])
    maps.value = loadedMaps
    mapSets.value = loadedMapSets
    if (maps.value.length && !selectedMapId.value) {
      selectedMapId.value = maps.value[0].id
    }
    resetMapGroupState()
    seedRobotsFromMaps()
  } catch (error) {
    console.error('加载地图失败:', error)
  } finally {
    loading.value = false
  }
}

async function loadSelectedMapDetail(mapId) {
  if (!mapId) return
  try {
    selectedMapDetail.value = await fetchMapDetail(mapId)
  } catch (error) {
    console.warn('加载地图详情失败:', error)
  }
}

function seedRobotsFromMaps() {
  if (!maps.value.length) return
  const known = new Map(robots.value.map(robot => [String(robot.id), robot]))
  for (const map of maps.value) {
    if (!map.robot || known.has(String(map.robot))) continue
    known.set(String(map.robot), {
      id: map.robot,
      name: map.robot_name || map.robot_code || `机器狗 ${map.robot}`,
      code: map.robot_code || String(map.robot),
    })
  }
  robots.value = Array.from(known.values())
  if (!mappingForm.value.robot && robots.value.length) {
    const activeMap = maps.value.find(map => map.active && map.robot) || maps.value.find(map => map.robot)
    mappingForm.value.robot = activeMap?.robot || robots.value[0].id
  }
}

async function refreshMappingStatus({ signal } = {}) {
  if (!mappingForm.value.robot || mappingStatusRefreshing) return
  mappingStatusRefreshing = true
  try {
    mappingStatus.value = await fetchRobotMappingStatus(mappingForm.value.robot, { signal })
    const reportedType = mappingStatus.value?.result?.mapping_type
    const reportedState = mappingStatus.value?.mapping_state || mappingStatus.value?.result?.state || 'idle'
    const activeReportedWorkflow = [
      'command_created', 'command_published', 'command_accepted', 'starting',
    'origin_starting', 'origin_waiting', 'origin_locked', 'slam_starting',
      'slam_warmup', 'ready_to_map', 'mapping', 'saving', 'optimizing', 'packaging',
      'uploading', 'stopping',
    ].includes(reportedState)
    // An idle Edge Agent reports its default indoor mapping type even when
    // the operator has just selected outdoor mode. Keep that explicit user
    // choice so a missing RTK fix is shown in the origin panel instead of
    // silently switching the form back to indoor.
    const shouldAdoptReportedType = activeReportedWorkflow
    if (['indoor', 'outdoor'].includes(reportedType)
      && reportedType !== mappingForm.value.mapping_type
      && shouldAdoptReportedType) {
      mappingForm.value.mapping_type = reportedType
    }
    const completedMapId = mappingStatus.value?.result?.upload_result?.id
      || (mappingStatus.value?.command_type === 'mapping.save'
        && mappingStatus.value?.command_status === 'succeeded'
        ? mappingStatus.value?.latest_map?.id
        : '')
    const completedMetrics = mappingStatus.value?.result?.mapping_metrics
      || mappingStatus.value?.latest_map?.mapping_metrics
      || {}
    if (completedMapId && Object.keys(completedMetrics).length
      && String(mappingResultMapId.value || '') !== String(completedMapId)) {
      publishPendingMappingResult(completedMapId, completedMetrics)
    }
  } catch (error) {
    if (error?.name === 'AbortError') return
    console.error('获取建图状态失败:', error)
  } finally {
    mappingStatusRefreshing = false
  }
}

async function selectMap(map) {
  selectedMapId.value = map.id
  mapImageError.value = {}
  if (map.robot && String(mappingForm.value.robot) !== String(map.robot)) {
    mappingForm.value.robot = map.robot
  }
  await refreshMappingStatus()
}

async function handleStartMapping() {
  if (!mappingForm.value.robot) {
    alert('请先选择机器狗')
    return
  }
  if (connectionStatus.value !== 'online') {
    alert('机器狗 Edge Agent 未连接，请先启动 NX 板 edge_agent')
    return
  }
  mappingBusy.value = true
  const prepareOrigin = isOutdoorMapping.value && !originLocked.value
  const startStep = prepareOrigin
    ? '启动并检查'
    : (isOutdoorMapping.value ? '启动 SLAM 并检查航向' : '启动并检查')
  setMappingStepFeedback(startStep, true, '正在下发命令，请等待 Edge Agent 响应…')
  try {
    const payload = {
      map_name: mappingForm.value.map_name,
      route_hint: mappingForm.value.route_hint,
      record_rosbag: mappingForm.value.record_rosbag,
      scene_scope: mappingForm.value.scene_scope,
      mapping_type: mappingForm.value.mapping_type,
      mapping_session_id: workflowSessionId.value,
    }
    if (isOutdoorMapping.value && !originLocked.value) {
      await startRobotMappingOrigin(mappingForm.value.robot, { ...payload, prepare_only: true })
    } else {
      await startRobotMappingSlam(mappingForm.value.robot, payload)
    }
    await refreshMappingStatus()
    // Keep the previous package result visible while the new command is
    // being accepted; hide it only after the next startup/check succeeds.
    clearPendingMappingResult()
    setMappingStepFeedback(startStep, true, prepareOrigin
      ? '传感器检查完成，已停在“锁定原点”，等待点击锁定按钮'
      : '已进入下一检查步骤，等待人工确认后继续')
  } catch (error) {
    setMappingStepFeedback(startStep, false, error.message)
    alert(`开始建图失败: ${error.message}`)
  } finally {
    mappingBusy.value = false
  }
}

async function handleMappingTypeChange(mappingType) {
  if (mappingType === mappingForm.value.mapping_type) return
  const originWorkflowActive = isOutdoorMapping.value
    && ['origin_starting', 'origin_waiting', 'origin_locked'].includes(mappingState.value)
  if (originWorkflowActive) {
    if (!confirm('切换建图模式将取消当前室外原点流程，是否继续？')) return
    mappingBusy.value = true
    try {
      await cancelRobotMappingOrigin(mappingForm.value.robot, { mapping_session_id: workflowSessionId.value })
    } catch (error) {
      alert(`取消室外原点流程失败: ${error.message}`)
      return
    } finally {
      mappingBusy.value = false
    }
  }
  mappingForm.value.mapping_type = mappingType
  mappingStepFeedback.value = null
  await refreshMappingStatus()
}

async function handleLockOrigin() {
  if (!mappingForm.value.robot || connectionStatus.value !== 'online') {
    alert('请先选择已连接 Edge Agent 的机器狗')
    return
  }
  mappingBusy.value = true
  setMappingStepFeedback('锁定 ENU 原点', true, '正在等待 RTK 位置固定解，最多 3 秒…')
  try {
    await startRobotMappingOrigin(mappingForm.value.robot, {
      map_name: mappingForm.value.map_name,
      route_hint: mappingForm.value.route_hint,
      scene_scope: mappingForm.value.scene_scope,
      mapping_type: 'outdoor',
      mapping_session_id: workflowSessionId.value,
    })
    await refreshMappingStatus()
    setMappingStepFeedback('锁定 ENU 原点', true, '3 秒固定解检查已启动；成功后原点立即锁定')
  } catch (error) {
    setMappingStepFeedback('锁定 ENU 原点', false, error.message)
    alert(`锁定原点启动失败: ${error.message}`)
  } finally {
    mappingBusy.value = false
  }
}

async function handleBeginMapping() {
  if (!mappingForm.value.robot) return
  const prompt = isOutdoorMapping.value
    ? '确认已原地小范围转动，双天线航向输出稳定，并开始正式采集数据和关键帧？'
    : '确认 IMU 与 SLAM 位姿检查通过，并开始正式采集数据和关键帧？'
  if (!confirm(prompt)) return
  mappingBusy.value = true
  const beginStep = isOutdoorMapping.value ? '航向复核' : '检查确认'
  setMappingStepFeedback(beginStep, true, '正在下发开始建图命令，请等待 Edge Agent 响应…')
  try {
    await beginRobotMapping(mappingForm.value.robot, {
      mapping_session_id: workflowSessionId.value,
      heading_check_confirmed: isOutdoorMapping.value,
    })
    await refreshMappingStatus()
    const confirmedHeadingRaw = originStatus.value.confirmed_heading_deg
    const confirmedHeading = Number(confirmedHeadingRaw)
    setMappingStepFeedback(
      beginStep,
      true,
      isOutdoorMapping.value
        && confirmedHeadingRaw !== null
        && confirmedHeadingRaw !== undefined
        && confirmedHeadingRaw !== ''
        && Number.isFinite(confirmedHeading)
        ? `已记录人工确认原点航向 ${confirmedHeading.toFixed(2)}°，正式关键帧采集已开始`
        : '人工确认已完成，正式关键帧采集已开始',
    )
  } catch (error) {
    setMappingStepFeedback(beginStep, false, error.message)
    alert(`开始正式建图失败: ${error.message}`)
  } finally {
    mappingBusy.value = false
  }
}

async function handleSaveMapping() {
  if (!mappingForm.value.robot) return
  mappingBusy.value = true
  setMappingStepFeedback('停止并保存地图', true, '正在下发停止并保存命令，请等待地图打包完成…')
  try {
    await saveRobotMapping(mappingForm.value.robot, {
      map_name: mappingForm.value.map_name,
    })
    // The save endpoint only queues the command. Keep polling through
    // packaging/upload so the newly uploaded map is actually visible before
    // returning the workflow to the configuration step.
    const deadline = Date.now() + 30 * 60 * 1000
    while (Date.now() < deadline) {
      await refreshMappingStatus()
      if (['failed', 'command_failed', 'command_rejected', 'command_timed_out'].includes(mappingState.value)) {
        throw new Error(failureMessage.value)
      }
      if (!mappingCommandInFlight.value && ['exited', 'completed', 'cancelled', 'idle'].includes(mappingState.value)) break
      await new Promise(resolve => setTimeout(resolve, 1000))
    }
    if (mappingCommandInFlight.value || ['saving', 'optimizing', 'packaging', 'uploading', 'stopping'].includes(mappingState.value)) {
      throw new Error('地图仍在上传，请稍后点击“刷新状态”查看结果')
    }
    const uploadedMapId = mappingStatus.value?.result?.upload_result?.id || mappingStatus.value?.latest_map?.id
    const commandMetrics = mappingStatus.value?.result?.mapping_metrics || {}
    await loadMaps()
    if (uploadedMapId) {
      selectedMapId.value = uploadedMapId
      const syncDeadline = Date.now() + 60 * 1000
      while (Date.now() < syncDeadline) {
        await refreshMappingStatus()
        if (robotCurrentMapId.value === String(uploadedMapId)) break
        await new Promise(resolve => setTimeout(resolve, 1000))
      }
    }
    const uploadedMap = maps.value.find(map => String(map.id) === String(uploadedMapId))
    const completedMetrics = uploadedMap?.mapping_metrics && Object.keys(uploadedMap.mapping_metrics).length
      ? uploadedMap.mapping_metrics
      : commandMetrics
    // A map row with metrics is created only after the complete package has
    // been built and accepted by the cloud upload endpoint. This is the
    // indoor step 12 / outdoor step 15 completion boundary.
    if (uploadedMapId && Object.keys(completedMetrics).length) {
      publishPendingMappingResult(uploadedMapId, completedMetrics)
    }
    const synced = uploadedMapId && robotCurrentMapId.value === String(uploadedMapId)
    setMappingStepFeedback(
      '停止并保存地图',
      true,
      synced
        ? `地图已上传平台并自动下发机器狗，平台/机器狗已同步（地图 #${uploadedMapId}）`
        : '地图已上传平台，机器狗地图切换命令已下发；请刷新状态确认应用完成',
    )
  } catch (error) {
    setMappingStepFeedback('停止并保存地图', false, error.message)
    alert(`停止并保存失败: ${error.message}`)
  } finally {
    mappingBusy.value = false
  }
}

async function handleCancelMapping() {
  if (!mappingForm.value.robot) {
    alert('请先选择机器狗')
    return
  }
  if (mappingCancelInFlight.value) {
    setMappingStepFeedback('取消建图', true, '取消命令正在处理中，请等待 Edge Agent 响应…')
    return
  }
  if (!confirm('确定要取消本次建图吗？')) return
  mappingCancelInFlight.value = true
  mappingBusy.value = true
  setMappingStepFeedback('取消建图', true, '正在下发取消建图命令，清理当前状态…')
  try {
    if (!mappingProcessAlive.value && ['ready', 'waiting_fix', 'locked'].includes(originState.value)) {
      await cancelRobotMappingOrigin(mappingForm.value.robot, { mapping_session_id: workflowSessionId.value })
    } else {
      await cancelRobotMapping(mappingForm.value.robot, { reason: 'operator_cancel', mapping_session_id: workflowSessionId.value })
    }
    await refreshMappingStatus()
    // Cancellation is the only recovery path from a failed workflow. Clear
    // the terminal snapshot so the operator is visibly returned to step one;
    // the next status poll will repopulate the fresh idle state from Edge.
    mappingStatus.value = null
    setMappingStepFeedback('取消建图', true, '建图流程已取消并停止')
  } catch (error) {
    setMappingStepFeedback('取消建图', false, error.message)
    alert(`取消建图失败: ${error.message}`)
  } finally {
    mappingCancelInFlight.value = false
    mappingBusy.value = false
  }
}

async function handleSyncMaps() {
  if (!mappingForm.value.robot) {
    alert('请先选择机器狗')
    return
  }
  if (connectionStatus.value !== 'online') {
    alert('机器狗 Edge Agent 未连接')
    return
  }
  syncing.value = true
  try {
    const result = await syncRobotMapping(mappingForm.value.robot, {
      map_name: mappingForm.value.map_name + ' (同步)',
    })
    await refreshMappingStatus()
    await loadMaps()
    alert('同步命令已下发，请等待 Edge Agent 处理完成')
  } catch (error) {
    alert(`同步失败: ${error.message}`)
  } finally {
    syncing.value = false
  }
}

function handleImageError(event, map) {
  console.warn('地图预览加载失败:', map.name, map.thumbnail_url)
  mapImageError.value = { ...mapImageError.value, [map.id]: true }
}

function formatSize(bytes) {
  if (!bytes) return '0 B'
  if (bytes < 1024) return bytes + ' B'
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB'
  return (bytes / (1024 * 1024)).toFixed(1) + ' MB'
}

async function handleDownload(map) {
  try {
    const blob = await downloadMap(map.id)
    const url = window.URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `${map.name}.zip`
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    window.URL.revokeObjectURL(url)
  } catch (error) {
    console.error('下载失败:', error)
    alert('下载失败')
  }
}

async function handleSetActive(map) {
  try {
    activatingMapId.value = map.id
    if (map.robot) mappingForm.value.robot = map.robot
    const result = await setActiveMap(map.id)
    await loadMaps()
    await refreshMappingStatus()
    const command = result.activation_command
    alert(command ? '已设为平台活动地图，已下发机器狗端切换命令，请等待状态变为“机器狗端已应用”。' : '已设为活动地图。')
  } catch (error) {
    console.error('设置活动地图失败:', error)
    alert(error.message || '设置失败')
  } finally {
    activatingMapId.value = null
  }
}

async function loadLoopReview() {
  if (!selectedMap.value) return
  loopReviewLoading.value = true
  try {
    loopReview.value = await fetchMapLoopReview(selectedMap.value.id, loopThresholds.value)
    // Candidate application is intentionally opt-in: threshold passage is an
    // audit result, not operator confirmation.
    selectedLoopCandidates.value = new Set()
  } catch (error) {
    loopReview.value = null
    alert(error.message || '回环审核加载失败')
  } finally {
    loopReviewLoading.value = false
  }
}

async function openLoopReview() {
  showLoopReview.value = true
  loopOptimizeStatus.value = ''
  await loadLoopReview()
}

function toggleLoopCandidate(candidateId) {
  const next = new Set(selectedLoopCandidates.value)
  if (next.has(candidateId)) next.delete(candidateId)
  else next.add(candidateId)
  selectedLoopCandidates.value = next
}

async function confirmLoopOptimization() {
  if (!selectedMap.value || !selectedLoopCandidates.value.size || loopOptimizeBusy.value) return
  const map = selectedMap.value
  if (!confirm(`确认使用 ${selectedLoopCandidates.value.size} 个已审核回环优化“${map.name}”吗？\n源地图不会覆盖，结果会创建为新版本且默认不激活。`)) return
  loopOptimizeBusy.value = true
  loopOptimizeStatus.value = '命令已下发，正在设备端优化、重建和上传…'
  try {
    const queued = await optimizeMapLoops(map.id, {
      thresholds: loopThresholds.value,
      selected_candidate_ids: [...selectedLoopCandidates.value],
      activate_after_upload: false,
    })
    const command = queued.command
    const deadline = Date.now() + 30 * 60 * 1000
    let latest = command
    while (Date.now() < deadline && !['succeeded', 'failed', 'rejected', 'timed_out', 'expired'].includes(latest.status)) {
      await new Promise(resolve => setTimeout(resolve, 2000))
      latest = await fetchRobotCommand(map.robot, command.id)
      loopOptimizeStatus.value = `设备执行状态：${latest.status}`
    }
    if (latest.status !== 'succeeded') throw new Error(latest.error_message || `优化未完成：${latest.status}`)
    const uploadedId = latest.result_payload?.upload_result?.id
    await loadMaps()
    if (uploadedId) selectedMapId.value = uploadedId
    showLoopReview.value = false
    alert('离线回环优化完成，新地图版本已上传；请核对轨迹与纠正量后再设为活动地图。')
  } catch (error) {
    loopOptimizeStatus.value = error.message || '离线优化失败'
    alert(loopOptimizeStatus.value)
  } finally {
    loopOptimizeBusy.value = false
  }
}

async function handleDelete(map) {
  if (!confirm(`确定要删除地图 "${map.name}" 吗？`)) return
  try {
    await deleteMap(map.id)
    if (selectedMapId.value === map.id) selectedMapId.value = null
    await loadMaps()
  } catch (error) {
    console.error('删除失败:', error)
    const refs = formatReferences(error.payload?.references)
    if (confirm(`${error.message || '删除失败'}${refs ? `\n\n关联数据：${refs}` : ''}\n\n是否强制删除地图及全部关联数据？`)) {
      await handleForceDelete(map)
    }
  }
}

async function handleForceDelete(map) {
  const message = `强制删除会同时删除地图 "${map.name}" 关联的路线、禁区、巡检任务、日历计划、执行记录、轨迹和告警事件，且不可恢复。确定继续吗？`
  if (!confirm(message)) return
  try {
    await deleteMap(map.id, { force: true })
    if (selectedMapId.value === map.id) selectedMapId.value = null
    await loadMaps()
  } catch (error) {
    console.error('强制删除失败:', error)
    alert(error.message || '强制删除失败')
  }
}

function formatReferences(references = {}) {
  return Object.entries(references)
    .filter(([, count]) => Number(count) > 0)
    .map(([name, count]) => `${name} ${count} 个`)
    .join('，')
}

function handleFileChange(event, field) {
  uploadForm.value[field] = event.target.files[0]
}

async function handleUpload() {
  uploading.value = true
  try {
    await createMap(uploadForm.value)
    showUploadDialog.value = false
    uploadForm.value = {
      name: '',
      pgm_file: null,
      yaml_file: null,
      thumbnail: null,
      resolution: 0.05,
      description: '',
    }
    await loadMaps()
  } catch (error) {
    console.error('上传失败:', error)
    alert('上传失败')
  } finally {
    uploading.value = false
  }
}

const cleanerCanvasStyle = computed(() => ({
  width: `${Math.max(1, cleanerImageSize.value.width * cleanerZoom.value)}px`,
  height: `${Math.max(1, cleanerImageSize.value.height * cleanerZoom.value)}px`,
}))

async function openCleaner() {
  if (!selectedMap.value?.pgm_url || !selectedMap.value?.robot) {
    alert('该地图缺少可编辑文件或未关联机器狗')
    return
  }
  showCleaner.value = true
  cleanerTool.value = 'erase'
  cleanerShowTrace.value = showMappingTrace.value
  cleanerBrushM.value = 0.5
  cleanerStrokes.value = []
  cleanerRedoStrokes.value = []
  await nextTick()
  const image = new Image()
  image.onload = async () => {
    cleanerImage = image
    cleanerImageSize.value = { width: image.naturalWidth, height: image.naturalHeight }
    await nextTick()
    const canvas = cleanerCanvas.value
    if (!canvas) return
    canvas.width = image.naturalWidth
    canvas.height = image.naturalHeight
    const viewportWidth = cleanerViewport.value?.clientWidth || image.naturalWidth
    const viewportHeight = cleanerViewport.value?.clientHeight || image.naturalHeight
    cleanerZoom.value = Math.min(1, Math.max(0.2, Math.min((viewportWidth - 24) / image.naturalWidth, (viewportHeight - 24) / image.naturalHeight)))
    redrawCleaner()
  }
  image.onerror = () => {
    alert('原始地图加载失败')
    closeCleaner()
  }
  image.src = fullPreviewUrl(`/api/maps/${selectedMap.value.id}/preview/?raw=1&t=${Date.now()}`)
}

function closeCleaner() {
  if (cleanerSaving.value) return
  showCleaner.value = false
  activeCleanerStroke = null
  cleanerPanStart = null
  cleanerImage = null
}

function redrawCleaner() {
  const canvas = cleanerCanvas.value
  if (!canvas || !cleanerImage) return
  const context = canvas.getContext('2d')
  context.clearRect(0, 0, canvas.width, canvas.height)
  context.drawImage(cleanerImage, 0, 0)
  if (cleanerShowTrace.value && mappingTracePoints.value.length > 1) {
    context.save()
    context.strokeStyle = 'rgba(22, 120, 255, 0.9)'
    context.lineWidth = Math.max(2, 2 / cleanerZoom.value)
    context.lineCap = 'round'
    context.lineJoin = 'round'
    context.beginPath()
    mappingTracePoints.value.forEach(([x, y], index) => {
      if (index === 0) context.moveTo(x, y)
      else context.lineTo(x, y)
    })
    context.stroke()
    context.restore()
  }
  context.save()
  context.strokeStyle = 'rgba(220, 38, 38, 0.72)'
  context.fillStyle = 'rgba(220, 38, 38, 0.72)'
  context.lineCap = 'round'
  context.lineJoin = 'round'
  for (const stroke of cleanerStrokes.value) {
    const points = stroke.points
    if (!points.length) continue
    const brushPixels = Math.max(1, stroke.diameter_m / Number(selectedMap.value?.resolution || 0.05))
    context.lineWidth = brushPixels
    if (points.length === 1) {
      context.beginPath()
      context.arc(points[0][0], points[0][1], brushPixels / 2, 0, Math.PI * 2)
      context.fill()
      continue
    }
    context.beginPath()
    context.moveTo(points[0][0], points[0][1])
    for (let index = 1; index < points.length; index += 1) {
      context.lineTo(points[index][0], points[index][1])
    }
    context.stroke()
  }
  context.restore()
}

function cleanerPoint(event) {
  const rect = cleanerCanvas.value.getBoundingClientRect()
  return [
    (event.clientX - rect.left) * (cleanerCanvas.value.width / rect.width),
    (event.clientY - rect.top) * (cleanerCanvas.value.height / rect.height),
  ]
}

function startCleanerPointer(event) {
  event.currentTarget.setPointerCapture?.(event.pointerId)
  if (cleanerTool.value === 'move') {
    cleanerPanStart = {
      x: event.clientX,
      y: event.clientY,
      left: cleanerViewport.value.scrollLeft,
      top: cleanerViewport.value.scrollTop,
    }
    return
  }
  activeCleanerStroke = { diameter_m: cleanerBrushM.value, points: [cleanerPoint(event)] }
  cleanerStrokes.value = [...cleanerStrokes.value, activeCleanerStroke]
  cleanerRedoStrokes.value = []
  redrawCleaner()
}

function moveCleanerPointer(event) {
  if (cleanerPanStart) {
    cleanerViewport.value.scrollLeft = cleanerPanStart.left - (event.clientX - cleanerPanStart.x)
    cleanerViewport.value.scrollTop = cleanerPanStart.top - (event.clientY - cleanerPanStart.y)
    return
  }
  if (!activeCleanerStroke) return
  const point = cleanerPoint(event)
  const previous = activeCleanerStroke.points[activeCleanerStroke.points.length - 1]
  if (Math.hypot(point[0] - previous[0], point[1] - previous[1]) < 2) return
  activeCleanerStroke.points.push(point)
  redrawCleaner()
}

function stopCleanerPointer() {
  activeCleanerStroke = null
  cleanerPanStart = null
}

function undoCleaner() {
  if (!cleanerStrokes.value.length) return
  const next = [...cleanerStrokes.value]
  const removed = next.pop()
  cleanerStrokes.value = next
  cleanerRedoStrokes.value = [...cleanerRedoStrokes.value, removed]
  redrawCleaner()
}

function redoCleaner() {
  if (!cleanerRedoStrokes.value.length) return
  const redo = [...cleanerRedoStrokes.value]
  const restored = redo.pop()
  cleanerRedoStrokes.value = redo
  cleanerStrokes.value = [...cleanerStrokes.value, restored]
  redrawCleaner()
}

function resetCleaner() {
  cleanerStrokes.value = []
  cleanerRedoStrokes.value = []
  redrawCleaner()
}

function changeCleanerZoom(delta) {
  cleanerZoom.value = Math.min(3, Math.max(0.2, Number((cleanerZoom.value + delta).toFixed(2))))
}

function toggleCleanerTrace() {
  cleanerShowTrace.value = !cleanerShowTrace.value
  redrawCleaner()
}

async function saveCleaner() {
  if (!cleanerStrokes.value.length || !selectedMap.value) return
  if (!confirm('保存清理版后将立即设为活动地图并下发到机器狗，确定继续吗？')) return
  cleanerSaving.value = true
  try {
    const result = await manuallyCleanMap(selectedMap.value.id, { strokes: cleanerStrokes.value })
    showCleaner.value = false
    await loadMaps()
    selectedMapId.value = result.id
    await refreshMappingStatus()
    alert('清理版已保存并下发，机器狗正在同步清理定位点云。')
  } catch (error) {
    alert(`保存清理版失败: ${error.message}`)
  } finally {
    cleanerSaving.value = false
  }
}
</script>

<template>
  <section class="page-section">
    <section class="panel detail-panel">
      <div class="panel-header">
        <h2>地图管理</h2>
        <div class="header-actions">
          <button class="btn btn-primary" @click="handleSyncMaps" :disabled="syncing || !mappingForm.robot || connectionStatus !== 'online'">
            {{ syncing ? '同步中...' : '从机器人同步' }}
          </button>
          <button class="btn btn-primary" @click="showUploadDialog = true">
            上传地图
          </button>
        </div>
      </div>

      <!-- 地图选择器 + 大图预览 -->
      <div class="map-full-preview">
        <div class="map-selector-row">
          <label>
            <span>选择地图</span>
            <select v-model="selectedMapId" @change="mapImageError = {}">
              <option v-for="map in maps" :key="map.id" :value="map.id">
                {{ map.name }} ({{ map.robot_name || map.robot_code }})
              </option>
            </select>
          </label>
        </div>

        <div v-if="selectedMap" class="map-preview-content">
          <div class="map-preview-image">
            <div v-if="!selectedMap.thumbnail_url" class="no-preview">
              该地图无预览数据，请从机器人建图后同步
            </div>
            <div v-else-if="mapImageError[selectedMap.id]" class="no-preview">
              预览加载失败
            </div>
            <div v-else class="map-image-stage">
              <img
                :src="fullPreviewUrl(selectedMap.thumbnail_url)"
                :alt="selectedMap.name"
                loading="lazy"
                decoding="async"
                @error="handleImageError($event, selectedMap)"
              />
              <svg
                v-if="showMappingTrace && mappingTracePoints.length > 1"
                class="map-trace-overlay"
                :viewBox="`0 0 ${selectedMap.width || 1} ${selectedMap.height || 1}`"
                preserveAspectRatio="none"
                aria-label="建图轨迹"
              >
                <defs>
                  <marker id="correction-arrow" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">
                    <path d="M0,0 L6,3 L0,6 Z" />
                  </marker>
                </defs>
                <polyline v-if="mappingTraceRawPoints.length > 1" class="trace-raw" :points="mappingTraceRawSvgPoints" />
                <polyline class="trace-optimized" :points="mappingTraceSvgPoints" />
                <g class="trace-corrections">
                  <line
                    v-for="item in mappingCorrectionArrows"
                    :key="item.index"
                    :x1="item.from.x" :y1="item.from.y" :x2="item.to.x" :y2="item.to.y"
                    marker-end="url(#correction-arrow)"
                  />
                  <circle
                    v-for="item in mappingCorrectionArrows.filter(value => value.yawSignificant)"
                    :key="`yaw-${item.index}`"
                    :cx="item.to.x" :cy="item.to.y" r="3"
                  />
                </g>
              </svg>
            </div>
          </div>
          <div class="map-preview-info">
            <h3>{{ selectedMap.name }}</h3>
            <div class="map-details">
              <div><strong>机器人:</strong> {{ selectedMap.robot_name }} ({{ selectedMap.robot_code }})</div>
              <div><strong>分辨率:</strong> {{ selectedMap.resolution }} m/像素</div>
              <div><strong>大小:</strong> {{ formatSize(selectedMap.file_size) }}</div>
              <div v-if="selectedMap.width"><strong>尺寸:</strong> {{ selectedMap.width }} × {{ selectedMap.height }}</div>
              <div v-if="selectedMap.coordinate_mode || selectedMapDescription.coordinate_mode">
                <strong>坐标:</strong>
                {{ (selectedMap.coordinate_mode || selectedMapDescription.coordinate_mode) === 'local_only' ? '无 RTK 原点 / 仅室内 NDT' : 'RTK 原点' }}
              </div>
              <div v-if="selectedMap.scene_scope || selectedMapDescription.scene_scope">
                <strong>场景:</strong> {{ selectedMap.scene_scope || selectedMapDescription.scene_scope }}
              </div>
              <div v-if="selectedMapIsRescue" class="rescue-map-label">
                <strong>质量:</strong> 发散救援地图，启用前必须现场核对
              </div>
            </div>
            <div v-if="Object.keys(selectedMapMetrics).length" class="mapping-metrics-panel">
              <div class="map-artifact-head">
                <strong>建图统计</strong>
                <span>{{ selectedMapMetrics.schema || 'mapping-metrics.v1' }}</span>
              </div>
              <div class="mapping-metrics-grid">
                <span>ZIP 包大小<strong>{{ formatBytes(selectedMapMetrics.package_size_bytes) }}</strong></span>
                <span>机器狗目录<strong>{{ formatBytes(selectedMapMetrics.robot_directory_size_bytes) }}</strong></span>
                <span>关键帧数量<strong>{{ selectedMapMetrics.keyframe_count || 0 }}</strong></span>
                <span>里程<strong>{{ Number(selectedMapMetrics.trajectory_m || 0).toFixed(1) }} m</strong></span>
                <span>建图耗时<strong>{{ formatDuration(selectedMapMetrics.mapping_duration_seconds) }}</strong></span>
                <span>诊断数据<strong>{{ formatBytes(selectedMapMetrics.diagnostic_data_size_bytes) }}</strong></span>
              </div>
            </div>
            <div v-if="Object.keys(selectedMapOptimization).length" class="optimization-panel" :class="`optimization-${selectedMapOptimization.stage || 'waiting'}`">
              <div class="optimization-line" :title="optimizationLine(selectedMapOptimization)">
                {{ optimizationLine(selectedMapOptimization) }}
              </div>
              <details>
                <summary>优化详情</summary>
                <div class="optimization-grid">
                  <span>触发来源<strong>{{ selectedMapOptimization.trigger_source || 'automatic_save' }}</strong></span>
                  <span>候选/接受/拒绝<strong>{{ selectedMapOptimization.candidate_count || 0 }}/{{ selectedMapOptimization.accepted_loop_count || 0 }}/{{ selectedMapOptimization.rejected_loop_count || 0 }}</strong></span>
                  <span>位置 RMS / P95<strong>{{ Number(selectedMapOptimization.correction?.position_m?.rms || 0).toFixed(3) }} / {{ Number(selectedMapOptimization.correction?.position_m?.p95 || 0).toFixed(3) }} m</strong></span>
                  <span>航向 RMS / P95<strong>{{ Number(selectedMapOptimization.correction?.yaw_deg?.rms || 0).toFixed(2) }} / {{ Number(selectedMapOptimization.correction?.yaw_deg?.p95 || 0).toFixed(2) }}°</strong></span>
                  <span>激光 / IMU / GPS / 航向 / 回环因子<strong>{{ selectedMapOptimization.factors?.ndt || 0 }}/{{ selectedMapOptimization.factors?.imu || 0 }}/{{ selectedMapOptimization.factors?.rtk_position || 0 }}/{{ selectedMapOptimization.factors?.rtk_heading || 0 }}/{{ selectedMapOptimization.factors?.loop_closure || 0 }}</strong></span>
                  <span>检测 / 优化重建耗时<strong>{{ formatDuration(selectedMapOptimization.timing?.loop_detection_seconds) }} / {{ formatDuration(selectedMapOptimization.timing?.optimization_and_rebuild_seconds) }}</strong></span>
                </div>
                <div v-if="selectedMapOptimization.mapping_type === 'outdoor'" class="enu-guard" :class="{ danger: !selectedMapOptimization.enu_guard?.passed }">
                  ENU 原点保护：{{ selectedMapOptimization.enu_guard?.passed ? '通过' : '未通过，已禁止自动激活' }}
                  · 起点 {{ Number(selectedMapOptimization.enu_guard?.start_translation_m || 0).toFixed(3) }} m
                  · 航向 {{ Number(selectedMapOptimization.enu_guard?.global_yaw_deg || 0).toFixed(2) }}°
                </div>
                <div v-if="selectedMapOptimization.fallback_error" class="mapping-progress-error">{{ selectedMapOptimization.fallback_error }}</div>
              </details>
              <button class="btn btn-sm loop-review-button" type="button" @click="openLoopReview">
                阈值审核与手工优化
              </button>
            </div>
            <div class="map-artifact-panel">
              <div class="map-artifact-head">
                <strong>地图数据文件</strong>
                <span>{{ selectedMapPackageFiles.length }} 项已上传</span>
              </div>
              <div v-for="artifact in mapArtifactFiles" :key="artifact.key" class="map-artifact-row">
                <span>{{ artifact.label }}</span>
                <strong :class="{ missing: !selectedMapPackageFiles.some(artifact.match) }">
                  {{ selectedMapPackageFiles.some(artifact.match) ? '已上传' : '缺失' }}
                </strong>
              </div>
              <small v-if="selectedMap.package_url">三类文件随完整地图包上传，可通过“下载”获取。</small>
            </div>
            <div class="global-enu-panel">
              <div class="map-artifact-head">
                <strong>当前全局 ENU</strong>
                <button
                  class="btn btn-sm"
                  type="button"
                  :disabled="globalEnuBusy || !selectedMap.robot"
                  @click="handleExtractGlobalEnu"
                >{{ globalEnuBusy ? '提取中...' : '提取全局 ENU' }}</button>
              </div>
              <div v-if="Object.keys(currentMapGlobalEnu).length" class="global-enu-values">
                <span>LAT {{ Number(currentMapGlobalEnu.origin_latitude || 0).toFixed(10) }}</span>
                <span>LON {{ Number(currentMapGlobalEnu.origin_longitude || 0).toFixed(10) }}</span>
                <span>ALT {{ Number(currentMapGlobalEnu.origin_altitude || 0).toFixed(3) }} m</span>
                <span v-if="currentMapConfirmedHeading">
                  {{ currentMapConfirmedHeading.confirmed ? '人工确认原点航向' : '锁定时航向' }}
                  {{ currentMapConfirmedHeading.value.toFixed(2) }}°
                </span>
                <span v-if="currentMapEnuToMapYawDeg !== null">ENU→地图旋转 {{ currentMapEnuToMapYawDeg.toFixed(2) }}°</span>
                <span>来源地图 {{ currentMapGlobalEnu.source_map_name || selectedMap.name }}</span>
              </div>
              <small v-else>尚未从当前地图提取全局 ENU。</small>
            </div>
            <div class="map-sync-panel">
              <div class="sync-row">
                <span>平台活动地图</span>
                <strong>{{ selectedMap.active ? `${selectedMap.id} / ${selectedMapVersion}` : '非活动' }}</strong>
              </div>
              <div class="sync-row">
                <span>机器狗端地图</span>
                <strong>{{ robotCurrentMapId || '—' }} / {{ robotCurrentMapVersion || '—' }}</strong>
              </div>
              <div class="sync-row">
                <span>本地目录</span>
                <strong>{{ robotCurrentMap.source_dir || robotCurrentMap.local_map_dir || '—' }}</strong>
              </div>
              <div class="sync-row">
                <span>同步状态</span>
                <strong class="sync-status" :class="activeMapSync.className">{{ activeMapSync.label }}</strong>
              </div>
            </div>
            <div v-if="selectedMap.description" class="map-description">
              <template v-if="selectedMapDescription.source">
                <div><strong>来源:</strong> {{ selectedMapDescription.source === 'edge_mapping' ? 'Edge Agent 建图' : selectedMapDescription.source }}</div>
                <div v-if="selectedMapDescription.map_version"><strong>版本:</strong> {{ selectedMapDescription.map_version }}</div>
              </template>
              <template v-else>
                {{ selectedMap.description }}
              </template>
            </div>
            <div class="map-preview-actions">
              <span v-if="selectedMap.active" class="badge badge-success">活动地图</span>
              <button
                v-if="mappingTracePoints.length > 1"
                class="btn btn-sm"
                type="button"
                @click="showMappingTrace = !showMappingTrace"
              >{{ showMappingTrace ? '隐藏轨迹' : '显示轨迹' }}</button>
              <button class="btn btn-sm btn-primary" @click="openCleaner">擦除障碍</button>
              <button class="btn btn-sm" @click="handleDownload(selectedMap)">下载</button>
              <button
                v-if="!selectedMap.active || activeMapSync.state !== 'synced'"
                class="btn btn-sm"
                :disabled="activatingMapId === selectedMap.id"
                @click="handleSetActive(selectedMap)"
              >
                {{ activatingMapId === selectedMap.id ? '切换中...' : (selectedMap.active ? '重新下发到机器狗' : '设为活动') }}
              </button>
              <button class="btn btn-sm btn-danger" @click="handleDelete(selectedMap)">删除</button>
              <button class="btn btn-sm btn-danger" @click="handleForceDelete(selectedMap)">强制删除</button>
            </div>
          </div>
        </div>
        <div v-else-if="maps.length === 0 && !loading" class="no-preview">
          暂无地图，请通过建图或上传添加
        </div>
      </div>

      <!-- 地图列表 -->
      <div v-if="mapSets.length" class="map-set-list">
        <h3 class="section-subtitle">跑道地图集</h3>
        <article v-for="mapSet in mapSets" :key="mapSet.id" class="map-set-card">
          <div>
            <h4>{{ mapSet.name }}</h4>
            <span>{{ mapSet.member_count }} 个子图 · {{ mapSet.manifest?.total_distance_m || '—' }} m · 重叠 {{ mapSet.manifest?.overlap_m || '—' }} m</span>
          </div>
          <div class="map-set-members">
            <span v-for="submapId in mapSet.submap_ids" :key="submapId" class="badge badge-sm">{{ submapId }}</span>
          </div>
        </article>
      </div>
      <div v-if="loading" class="loading">加载中...</div>
      <div v-else class="map-list-section">
        <div class="map-list-heading">
          <h3 class="section-subtitle">所有地图 ({{ maps.length }})</h3>
          <div class="map-list-actions" v-if="mapGroups.length > 1">
            <button class="btn btn-sm" type="button" @click="expandAllMapGroups">全部展开</button>
            <button class="btn btn-sm" type="button" @click="collapseAllMapGroups">全部折叠</button>
          </div>
        </div>
        <div v-if="maps.length > 0" class="map-groups">
          <section v-for="group in mapGroups" :key="group.key" class="map-group">
            <button
              class="map-group-header"
              type="button"
              :aria-expanded="isMapGroupExpanded(group.key)"
              @click="toggleMapGroup(group.key)"
            >
              <span class="map-group-chevron" aria-hidden="true">{{ isMapGroupExpanded(group.key) ? '▾' : '▸' }}</span>
              <span class="map-group-title">{{ group.label }}</span>
              <span class="map-group-count">{{ group.maps.length }} 张</span>
              <span v-if="group.activeCount" class="badge badge-success badge-sm">活动 {{ group.activeCount }}</span>
            </button>
            <div v-if="isMapGroupExpanded(group.key)" class="map-list">
              <article
                v-for="map in group.maps"
                :key="map.id"
                class="map-card"
                :class="{ 'map-card-selected': map.id === selectedMapId }"
                @click="selectMap(map)"
              >
                <div class="map-thumbnail">
                  <template v-if="map.thumbnail_url && !mapImageError[map.id]">
                    <img
                      :src="fullPreviewUrl(map.thumbnail_url)"
                      :alt="map.name"
                      loading="lazy"
                      decoding="async"
                      @error.stop="handleImageError($event, map)"
                    />
                  </template>
                  <div v-else class="no-thumbnail">无预览</div>
                </div>
                <div class="map-card-info">
                  <h4>{{ map.name }}</h4>
                  <span class="map-card-size">{{ formatSize(map.file_size) }}</span>
                </div>
                <span v-if="map.active" class="badge badge-success badge-sm">活动</span>
              </article>
            </div>
          </section>
        </div>
        <div v-else class="empty-state">
          暂无地图，点击"上传地图"或"从机器人同步"
        </div>
      </div>

      <!-- 建图面板 -->
      <div class="mapping-card">
        <div class="mapping-head">
          <div>
            <h3>现场建图</h3>
            <p>通过云端远程控制 NX 板 Edge Agent 执行建图</p>
          </div>
        </div>

        <div class="mapping-mode-switch" role="radiogroup" aria-label="建图模式">
          <button
            type="button"
            :class="{ active: mappingForm.mapping_type === 'indoor' }"
            :disabled="mappingModeSwitchDisabled"
            @click="handleMappingTypeChange('indoor')"
          >
            <strong>室内建图</strong>
            <span>跳过 RTK 原点，完成 IMU 与位姿预热后开始</span>
          </button>
          <button
            type="button"
            :class="{ active: mappingForm.mapping_type === 'outdoor' }"
            :disabled="mappingModeSwitchDisabled"
            @click="handleMappingTypeChange('outdoor')"
          >
            <strong>室外建图</strong>
            <span>先锁定 ENU 原点，再复核双天线航向</span>
          </button>
        </div>

        <!-- 连接状态栏 -->
        <div class="connection-bar">
          <div class="connection-row">
            <span class="connection-dot" :class="connectionClass"></span>
            <span class="connection-text">
              Edge Agent: <strong>{{ connectionLabel }}</strong>
            </span>
            <span v-if="mappingStatus?.robot_code" class="connection-detail">
              | {{ mappingStatus.robot_code }}
              <template v-if="mappingStatus?.agent_version">v{{ mappingStatus.agent_version }}</template>
            </span>
          </div>
          <div v-if="connectionStatus !== 'online'" class="connection-hint">
            请确保 NX 板 edge_agent 已启动并连接到 MQTT Broker
          </div>
        </div>

        <section v-if="isOutdoorMapping" class="origin-quality-card" :class="`origin-${originState}`">
          <div class="origin-quality-head">
            <div>
              <span class="origin-kicker">室外 ENU 锚点</span>
              <strong>RTK 传感器实时信息</strong>
            </div>
            <span class="origin-state-chip" :class="{ locked: originLocked, failed: originState === 'failed' }">
              {{ originLocked
                ? 'ENU 原点已锁定'
                : (originState === 'quality_holding'
                  ? `质量稳定 ${originQualityElapsed.toFixed(1)} / ${originQualityRequired.toFixed(0)}s`
                  : (originState === 'waiting_fix'
                    ? `等待位置 FIX ${Number(originStatus.lock_wait_seconds || 0).toFixed(1)} / ${Number(originStatus.lock_timeout_seconds || 3).toFixed(0)}s`
                    : originState)) }}
            </span>
          </div>
          <p>{{ originStatusMessage }}</p>
          <progress v-if="['waiting_fix', 'quality_holding'].includes(originState)" :value="originQualityPercent" max="100"></progress>
          <div class="origin-lock-check-grid">
            <div :class="{ ok: originStatus.origin_position_ready }">
              <span>{{ originStatus.origin_position_ready ? '✓' : '…' }}</span>
              <div>
                <strong>位置 FIX</strong>
                <small>{{ originQualityLabel }} · 状态 / 类型 {{ originStatus.solution_status ?? '—' }} / {{ originStatus.position_type ?? '—' }}</small>
              </div>
            </div>
            <div :class="{ ok: originStatus.origin_heading_ready }">
              <span>{{ originStatus.origin_heading_ready ? '✓' : '…' }}</span>
              <div>
                <strong>双天线航向基线 FIX</strong>
                <small>{{ originStatus.heading_fixed ? '航向 FIX' : '等待航向 FIX' }} · 基线 {{ formatOriginNumber(originStatus.baseline_m, 3, ' m') }}</small>
              </div>
            </div>
            <div :class="{ ok: originStatus.origin_spread_ready }">
              <span>{{ originStatus.origin_spread_ready ? '✓' : '…' }}</span>
              <div>
                <strong>lat / lon 数值波动＜2 cm</strong>
                <small>当前波动 {{ formatOriginNumber(Number(originStatus.position_spread_m) * 100, 2, ' cm') }} · 连续 {{ originQualityElapsed.toFixed(1) }}s</small>
              </div>
            </div>
          </div>
          <div v-if="originLocked" class="origin-coordinate">
            <span>LAT {{ Number(originStatus.origin?.origin_latitude || originStatus.latitude).toFixed(10) }}</span>
            <span>LON {{ Number(originStatus.origin?.origin_longitude || originStatus.longitude).toFixed(10) }}</span>
            <span>ALT {{ Number(originStatus.origin?.origin_altitude || originStatus.altitude).toFixed(3) }} m</span>
          </div>
          <div class="origin-diagnostic-table-wrap" :class="{ stale: originDataStale }">
            <table class="origin-diagnostic-table">
              <tbody>
                <tr v-for="(rowPair, rowIndex) in originDiagnosticTableRows" :key="rowIndex">
                  <template v-for="row in rowPair" :key="row.label">
                    <th>{{ row.label }}</th>
                    <td :class="row.tone || ''">{{ row.value }}</td>
                  </template>
                  <template v-if="rowPair.length < 2"><th></th><td></td></template>
                </tr>
              </tbody>
            </table>
          </div>
          <div v-if="originLocked && mappingState === 'ready_to_map'" class="heading-review-panel" :class="`review-${headingReviewStatus}`">
            <div class="heading-review-head">
              <div>
                <strong>双天线航向复核</strong>
                <small>请原地小范围转动，观察下列实时数据；确认航向稳定后点击“确认航向稳定，开始建图”</small>
              </div>
              <span>人工确认</span>
            </div>
            <div class="heading-quality-grid">
              <div :class="{ ok: originStatus.heading_fixed }">
                <span>{{ originStatus.heading_fixed ? '✓' : '…' }}</span>
                <div>
                  <strong>双天线航向 FIX</strong>
                  <small>状态 / 类型 {{ originStatus.heading_status ?? '—' }} / {{ originStatus.heading_type ?? '—' }}</small>
                </div>
              </div>
              <div :class="{ ok: headingBaselineReady }">
                <span>{{ headingBaselineReady ? '✓' : '…' }}</span>
                <div>
                  <strong>双天线基线达标</strong>
                  <small>{{ formatOriginNumber(originStatus.baseline_m, 3, ' m') }} / ≥ {{ formatOriginNumber(originStatus.heading_min_baseline_m || 0.2, 2, ' m') }}</small>
                </div>
              </div>
              <div :class="{ ok: headingAccuracyReady }">
                <span>{{ headingAccuracyReady ? '✓' : '…' }}</span>
                <div>
                  <strong>航向精度与时效达标</strong>
                  <small>σ {{ formatOriginNumber(originStatus.heading_std_deg, 2, '°') }} · 延迟 {{ formatOriginNumber(originStatus.data_age_seconds ?? originStatus.age_seconds, 2, ' s') }}</small>
                </div>
              </div>
            </div>
            <div class="heading-review-summary">
              <span>航向数据实时刷新，仅供人工复核，不再等待倒计时</span>
              <span>确认按钮已激活后可随时继续</span>
            </div>
            <p>{{ originStatus.message }}</p>
          </div>
          <div class="mapping-odom-strip" :class="{ online: mappingOdometry.online, offline: !mappingOdometry.online }">
            <strong>/odom/localization_odom</strong>
            <span>{{ mappingOdometry.online ? '实时' : '无数据' }}</span>
            <span>来源 {{ mappingOdometrySourceLabel }}</span>
            <span>{{ formatOriginNumber(mappingOdometry.frequency_hz, 1, ' Hz') }}</span>
            <span>接收延迟 {{ formatOriginNumber(mappingOdometry.sample_age_seconds, 3, ' s') }}</span>
            <span>时间偏差 {{ formatOriginNumber(mappingOdometry.message_time_offset_seconds, 3, ' s') }}</span>
            <span>{{ mappingOdometry.frame_id || '—' }} → {{ mappingOdometry.child_frame_id || '—' }}</span>
          </div>
          <div class="mapping-odom-strip mapping-rtk-strip" :class="{ online: rtkTelemetryOnline, offline: !rtkTelemetryOnline }">
            <strong>/rtk_pvh</strong>
            <span>{{ rtkTelemetryOnline ? '实时' : '无数据或已过期' }}</span>
            <span>X {{ formatOriginNumber(originStatus.rtk_enu_x_m, 3, ' m') }}</span>
            <span>Y {{ formatOriginNumber(originStatus.rtk_enu_y_m, 3, ' m') }}</span>
            <span>Yaw(ENU/机身) {{ formatOriginNumber(originStatus.rtk_yaw_deg, 2, '°') }}</span>
            <span>解 {{ originQualityLabel }} · {{ originStatus.solution_status ?? '—' }} / {{ originStatus.position_type ?? '—' }}</span>
            <span>数据年龄 {{ formatOriginNumber(originStatus.data_age_seconds ?? originStatus.age_seconds, 3, ' s') }}</span>
          </div>
        </section>

        <!-- 建图状态机 -->
        <div v-if="showMappingReadiness" class="mapping-readiness" :class="readinessClass" role="status">
          <div class="mapping-readiness-main">
            <strong>{{ readinessLabel }}</strong>
            <span>{{ mappingReadiness.message }}</span>
          </div>
          <div class="mapping-readiness-sensors">
            <span :class="{ ok: readinessState !== 'telemetry_stale' }">
              雷达 {{ readinessState === 'telemetry_stale' ? '无新数据' : '有数据' }}
            </span>
            <span :class="{ ok: mappingReadiness.imu_initialized }">
              IMU {{ mappingReadiness.imu_initialized ? '已初始化' : '初始化中' }}
              <template v-if="mappingReadiness.imu_required_samples">
                {{ mappingReadiness.imu_samples || 0 }}/{{ mappingReadiness.imu_required_samples }}
              </template>
            </span>
            <span :class="{ ok: mappingStatus?.result?.mapping_capture_enabled }">
              正式采集 {{ mappingStatus?.result?.mapping_capture_enabled ? '已开启' : '门控关闭' }}
            </span>
            <span v-if="isOutdoorMapping && rtkAlignmentLabel" :class="{ ok: rtkAlignment.locked }">
              {{ rtkAlignmentLabel }}
            </span>
            <span v-if="mappingStatus?.result?.mapping_capture_enabled" :class="{ ok: Number(mappingReadiness.keyframe_count || 0) > 0 }">
              关键帧 {{ mappingReadiness.keyframe_count || 0 }}
            </span>
            <span v-if="readinessSampleAge !== null">状态延迟 {{ readinessSampleAge.toFixed(1) }}s</span>
          </div>
        </div>
        <div v-if="showMappingReadiness" class="mapping-startup-checks" :class="{ 'is-ready': mappingStartupReady }">
          <div class="mapping-startup-checks-head">
            <strong>建图前置检查</strong>
            <span>{{ mappingStartupReady ? '全部通过，请人工确认后再移动' : '检查中，请保持机器狗静止' }}</span>
          </div>
          <div class="mapping-startup-check-grid">
            <div v-for="item in mappingStartupChecks" :key="item.key" class="mapping-startup-check" :class="{ ok: item.ok }">
              <span class="mapping-startup-check-icon" aria-hidden="true">{{ item.ok ? '✓' : '…' }}</span>
              <span>
                <strong>{{ item.label }}</strong>
                <small>{{ item.detail }}</small>
              </span>
            </div>
          </div>
        </div>
        <div v-if="showMappingReadiness" class="slam-health-alert" :class="{ 'is-diverged': slamDiverged, 'is-healthy': !slamHealthIssue }" role="status">
          <div>
            <strong>{{ slamHealthLabel }}</strong>
            <span>{{ slamHealthMessage }}</span>
          </div>
          <div class="slam-health-metrics">
            <span>速度 {{ Number(slamHealth.speed_mps || 0).toFixed(2) }} m/s</span>
            <span>Z 漂移 {{ Number(slamHealth.pose_z_m || 0).toFixed(2) }} m</span>
            <span>帧跳变 {{ Number(slamHealth.frame_delta_m || 0).toFixed(2) }} m</span>
            <span>连续无匹配点 {{ slamHealth.no_effective_points_streak || 0 }} 帧</span>
            <span>连续异常 {{ slamHealth.pose_anomaly_streak || 0 }}</span>
          </div>
        </div>
        <div v-if="showMappingReadiness || rosbagStatus.bag_dir" class="rosbag-status" :class="{ 'is-recording': rosbagStatus.running }">
          <div>
            <strong>诊断数据录制</strong>
            <span>{{ rosbagStatus.running ? '录制中' : (rosbagStatus.bag_dir ? '已停止并保存' : '未录制') }}</span>
          </div>
          <div class="rosbag-status-metrics">
            <span v-if="rosbagStatus.duration_seconds !== undefined">时长 {{ Math.floor(Number(rosbagStatus.duration_seconds || 0) / 60) }}分{{ Number(rosbagStatus.duration_seconds || 0) % 60 }}秒</span>
            <span v-if="rosbagStatus.size_bytes !== undefined">大小 {{ formatBytes(rosbagStatus.size_bytes) }}</span>
            <span v-if="rosbagStatus.bag_dir" :title="rosbagStatus.bag_dir">{{ rosbagStatus.bag_dir }}</span>
            <span v-if="rosbagStatus.error" class="text-danger">{{ rosbagStatus.error }}</span>
          </div>
        </div>
        <div class="state-machine">
          <div class="state-header">
            <span class="state-label">建图状态机</span>
            <span v-if="isError" class="state-badge badge-error">{{ errorLabel }}</span>
            <span v-else-if="isActiveMapping" class="state-badge badge-active">进行中</span>
            <span v-else class="state-badge">{{ mappingStateLabel }}</span>
          </div>
          <div v-if="mappingStatus?.result" class="state-runtime">
            <span>建图进程: {{ mappingStatus.result.process_alive ? '运行中' : '已退出' }}</span>
            <span v-if="mappingStatus.result.slam_pids?.length">PID {{ mappingStatus.result.slam_pids.join(', ') }}</span>
          </div>
          <div v-if="saveProgress.stage" class="mapping-progress">
            <div class="mapping-progress-head">
              <strong>{{ saveStageLabel }}</strong>
              <span>{{ Number(saveProgress.progress_percent || 0).toFixed(0) }}%</span>
            </div>
            <progress :value="saveProgress.progress_percent || 0" max="100"></progress>
            <div class="mapping-progress-stats">
              <span>里程 {{ Number(saveProgress.trajectory_m || 0).toFixed(1) }} m</span>
              <span>关键帧 {{ saveProgress.written_keyframes || 0 }}/{{ saveProgress.keyframe_count || 0 }}</span>
              <span>队列 {{ saveProgress.queued_keyframes || 0 }}</span>
              <span>内存 {{ formatBytes(saveProgress.rss_bytes) }}</span>
              <span>预计点云 {{ formatBytes(saveProgress.estimated_output_bytes) }}</span>
              <span>磁盘可用 {{ formatBytes(saveProgress.disk_free_bytes) }}</span>
              <span>
                RTK {{ saveProgress.rtk_quality?.valid ? '可融合' : '激光兜底' }}
                · {{ Number(saveProgress.rtk_quality?.horizontal_std ?? -1) >= 0 ? `${Number(saveProgress.rtk_quality.horizontal_std).toFixed(2)} m` : '无精度' }}
              </span>
              <span :class="{ 'text-danger': slamHealthIssue }">{{ slamHealthLabel }}</span>
            </div>
            <div v-if="saveProgress.error" class="mapping-progress-error">{{ saveProgress.error }}</div>
          </div>
          <div v-if="Object.keys(runtimeOptimization).length" class="optimization-panel optimization-runtime" :class="`optimization-${runtimeOptimization.stage || 'waiting'}`">
            <div class="optimization-line" :title="optimizationLine(runtimeOptimization)">
              {{ optimizationLine(runtimeOptimization) }}
            </div>
          </div>
          <div v-if="Object.keys(mappingMetrics).length" class="mapping-metrics-panel mapping-metrics-runtime">
            <div class="map-artifact-head">
              <strong>本次建图结果</strong>
              <span>第{{ mappingMetrics.completion_step || (mappingMetrics.mapping_type === 'outdoor' ? 15 : 12) }}步打包完成</span>
            </div>
            <div class="mapping-metrics-grid">
              <span>ZIP 包大小<strong>{{ formatBytes(mappingMetrics.package_size_bytes) }}</strong></span>
              <span>机器狗目录<strong>{{ formatBytes(mappingMetrics.robot_directory_size_bytes) }}</strong></span>
              <span>关键帧数量<strong>{{ mappingMetrics.keyframe_count || 0 }}</strong></span>
              <span>里程<strong>{{ Number(mappingMetrics.trajectory_m || 0).toFixed(1) }} m</strong></span>
              <span>建图耗时<strong>{{ formatDuration(mappingMetrics.mapping_duration_seconds) }}</strong></span>
              <span>诊断数据<strong>{{ formatBytes(mappingMetrics.diagnostic_data_size_bytes) }}</strong></span>
            </div>
          </div>
          <div class="post-save-validation-panel">
            <div class="map-artifact-head">
              <strong>保存后静止定位自检</strong>
              <span>{{ validationStateLabel }}</span>
            </div>
            <div
              v-if="Object.keys(postSaveValidationResult).length"
              class="validation-current-result"
              :class="postSaveValidation.state === 'passed' && postSaveValidationResult.accurate ? 'is-accurate' : 'is-inaccurate'"
            >
              <div class="validation-result-title">
                <strong>本次{{ validationMappingTypeLabel }}定位</strong>
                <span>{{ validationStateLabel }}</span>
              </div>
              <div class="validation-position">
                <span>{{ validationCoordinateLabel }} 定位位置</span>
                <strong>{{ validationPoseLabel }}</strong>
              </div>
              <div v-if="postSaveValidationResult.saved_terminal_pose" class="validation-position validation-reference">
                <span>保存结束参考位置</span>
                <strong>{{ validationSavedPoseLabel }}</strong>
              </div>
              <div class="validation-quality-grid">
                <span>位置偏差<strong>{{ formatValidationMetric(postSaveValidationResult.position_error_m, 2, ' m') }}</strong></span>
                <span>航向偏差<strong>{{ formatValidationMetric(postSaveValidationResult.yaw_error_deg, 1, '°') }}</strong></span>
                <span>NDT 匹配误差<strong>{{ formatValidationMetric(postSaveValidationResult.quality?.matching_error, 3, '') }}</strong></span>
                <span>内点率<strong>{{ formatValidationMetric(Number(postSaveValidationResult.quality?.inlier_fraction) * 100, 1, '%') }}</strong></span>
                <span>最大位置跳变<strong>{{ formatValidationMetric(validationDiagnostics.max_pose_step_m, 3, ' m') }}</strong></span>
                <span>最大航向跳变<strong>{{ formatValidationMetric(validationDiagnostics.max_yaw_step_deg, 1, '°') }}</strong></span>
              </div>
              <small>{{ validationSeedLabel }} · {{ validationSampleLabel }}</small>
              <small v-if="postSaveValidationResult.reason_code">判定代码 {{ postSaveValidationResult.reason_code }}</small>
              <small v-if="postSaveValidationResult.map_dir" :title="postSaveValidationResult.map_dir">候选地图 {{ postSaveValidationResult.map_dir }}</small>
              <small v-if="validationResultTime">检测时间 {{ validationResultTime }}</small>
              <small v-if="postSaveValidationResult.message">{{ postSaveValidationResult.message }}</small>
            </div>
            <div v-else-if="['queued', 'running'].includes(postSaveValidation.state)" class="validation-waiting">
              正在启动候选地图定位、加载本次保存地图并核对保存结束位置；不会启动导航或下发速度指令。
            </div>
            <div class="validation-counters">
              <div><span>室内累计</span><strong>{{ indoorValidation.success }}/{{ indoorValidation.required }}</strong><progress :value="validationPercent(indoorValidation)" max="100"></progress><small>尝试 {{ indoorValidation.attempts || 0 }} 次</small></div>
              <div><span>室外累计</span><strong>{{ outdoorValidation.success }}/{{ outdoorValidation.required }}</strong><progress :value="validationPercent(outdoorValidation)" max="100"></progress><small>尝试 {{ outdoorValidation.attempts || 0 }} 次</small></div>
            </div>
            <div v-if="postSaveValidation.detail && postSaveValidation.state !== 'passed'" class="mapping-progress-error">{{ postSaveValidation.detail }}</div>
          </div>
          <div class="state-steps">
            <div
              v-for="(step, index) in stateSteps"
              :key="step.key"
              class="state-step"
              :class="{
                'step-past': isPastStep(index),
                'step-current': isCurrentStep(index) && !isError,
                'step-error': isCurrentStep(index) && isError,
                'step-future': !isPastStep(index) && !isCurrentStep(index),
              }"
            >
              <div class="step-dot">
                <span v-if="isError && isCurrentStep(index)" class="dot-icon">✕</span>
                <span v-else-if="isPastStep(index)" class="dot-icon">✓</span>
                <span v-else-if="isActiveMapping && isCurrentStep(index)" class="dot-icon dot-spin">◌</span>
                <span v-else>{{ index + 1 }}</span>
              </div>
              <span class="step-label">{{ step.label }}</span>
            </div>
          </div>
        </div>

        <div class="mapping-grid">
          <label>
            <span>机器狗</span>
            <select v-model="mappingForm.robot" @change="refreshMappingStatus">
              <option v-for="robot in robots" :key="robot.id" :value="robot.id">
                {{ robot.name }} / {{ robot.code }}
              </option>
            </select>
          </label>
          <label>
            <span>地图名称</span>
            <input v-model="mappingForm.map_name" type="text" placeholder="太阳宫园区 V1" />
          </label>
          <label class="mapping-route">
            <span>演示路线</span>
            <input v-model="mappingForm.route_hint" type="text" />
          </label>
          <label class="mapping-scene-option">
            <span>场景范围</span>
            <select v-model="mappingForm.scene_scope" :disabled="isActiveMapping">
              <option v-if="!isOutdoorMapping" value="indoor">室内</option>
              <option v-if="isOutdoorMapping" value="transition">室内外过渡（按室外流程）</option>
              <option v-if="isOutdoorMapping" value="outdoor">室外</option>
            </select>
          </label>
          <label class="mapping-record-option">
            <input v-model="mappingForm.record_rosbag" type="checkbox" :disabled="isActiveMapping" />
            <span>
              <strong>同步录制诊断数据</strong>
              <small>记录雷达、IMU、里程计、TF和RTK，便于离线复现漂移；诊断包留在机器上，不随地图上传</small>
            </span>
          </label>
        </div>

        <div class="mapping-actions">
          <button class="btn btn-primary" :disabled="mappingBusy || !selectedRobot || !canStartSlam" @click="handleStartMapping">
            {{ startSlamButtonLabel }}
          </button>
          <button v-if="isOutdoorMapping" class="btn btn-origin" :disabled="mappingBusy || !selectedRobot || !canLockOrigin" @click="handleLockOrigin">
            {{ originState === 'waiting_fix'
              ? '等待位置 FIX（最多3秒）…'
              : (originState === 'quality_holding'
                ? `检查三项质量（${originQualityElapsed.toFixed(1)} / ${originQualityRequired.toFixed(0)}s）…`
                : (originLocked ? 'ENU 原点已锁定' : (isError && failureStepKey === 'origin_waiting' ? '重试锁定 ENU 原点' : '锁定 ENU 原点'))) }}
          </button>
          <button class="btn btn-confirm" :disabled="mappingBusy || !selectedRobot || !canBeginMapping" @click="handleBeginMapping">
            {{ isError && failureStepKey === 'ready_to_map' ? '重试确认并开始建图' : (isOutdoorMapping ? '确认航向稳定，开始建图' : '确认检查通过，开始建图') }}
          </button>
          <button class="btn btn-primary" :disabled="mappingBusy || !selectedRobot || !canSaveMapping" @click="handleSaveMapping">
            {{ slamDiverged ? '停止并生成救援地图' : (canRetrySave ? '重试停止并保存地图' : '停止并保存地图') }}
          </button>
          <button class="btn btn-sm" :disabled="!canCancelMapping" @click="handleCancelMapping">
            取消建图
          </button>
          <button class="btn btn-sm" :disabled="mappingBusy || !selectedRobot" @click="refreshMappingStatus">
            刷新状态
          </button>
        </div>

        <div v-if="mappingStepFeedback" class="mapping-step-feedback" :class="mappingStepFeedback.success ? 'is-success' : 'is-error'">
          <strong>{{ mappingStepFeedback.success ? '✓' : '✕' }} {{ mappingStepFeedback.step }}</strong>
          <span>{{ mappingStepFeedback.message }}</span>
        </div>

        <div v-if="isError && isOutdoorMapping" class="mapping-error mapping-error-outdoor">
          <strong>室外建图状态机停止：第 {{ failureStepIndex >= 0 ? failureStepIndex + 1 : '?' }} 步「{{ failureStepLabel }}」失败</strong>
          <span v-if="failureCode">错误码：{{ failureCode }}</span>
          <span>失败信息：{{ failureMessage }}</span>
          <span v-if="canRetrySave">本地地图已经导出，请点「重试停止并保存地图」，不要重新走场。也可以点页面上方「从机器人同步」。</span>
        </div>
        <div v-else-if="isError" class="mapping-error mapping-error-indoor">
          <strong>室内建图状态机停止：第 {{ failureStepIndex >= 0 ? failureStepIndex + 1 : '?' }} 步「{{ failureStepLabel }}」失败</strong>
          <span v-if="failureCode">错误码：{{ failureCode }}</span>
          <span>失败信息：{{ failureMessage }}</span>
        </div>
        <div v-else-if="mappingStatus?.error_message" class="mapping-error">
          <strong>{{ isOutdoorMapping ? '室外建图错误' : '室内建图错误' }}</strong>
          <span>{{ mappingStatus.error_message }}</span>
        </div>
        <div
          v-else-if="mappingStatus?.last_command_error_message && ['failed', 'rejected', 'timed_out'].includes(commandStatus)"
          class="mapping-error"
        >
          上次命令失败: {{ mappingStatus.last_command_error_message }}
          <span v-if="canRetrySave">本地地图已经导出，请点「重试停止并保存地图」，不要重新走场。也可以点页面上方「从机器人同步」。</span>
        </div>

        <div class="mapping-guide">
          <strong>操作步骤：</strong>
          <template v-if="isOutdoorMapping">
            <span>1. 将机器人开到预选开阔锚点，点击“启动传感器并检查”，完成传感器检查后停在锁定原点步骤</span>
            <span>2. 点击“锁定 ENU 原点”，3 秒内须获得位置 FIX；位置 FIX、双天线航向基线 FIX、坐标波动＜2 cm 连续稳定 10 秒后锁定经纬高</span>
            <span>3. 原点锁定后点击“启动 SLAM 并检查航向”，完成 IMU/位姿预热。预热和原地转动期间 GNSS 不会拉雷达</span>
            <span>4. 状态机停在“航向复核”，请原地小范围转动；观察航向稳定后点击已激活的“确认航向稳定，开始建图”</span>
            <span>5. 确认后用当前雷达航向和双天线航向锁定 ENU→地图航向，再开始采关键帧和 GNSS 融合；若航向锁失败，先直线走约 15 米用轨迹拟合</span>
          </template>
          <template v-else>
            <span>1. 点击“启动并检查”，自动启动雷达内置 IMU 和 SLAM 预热</span>
            <span>2. 保持静止，等待 IMU 初始化与有效 SLAM 位姿全部通过</span>
            <span>3. 点击“确认检查通过，开始建图”，此时才正式采集数据和关键帧</span>
            <span>4. 用 Orche APP / 遥控器走场，结束后停止并保存地图</span>
          </template>
        </div>
      </div>
    </section>

    <!-- 上传对话框 -->
    <div v-if="showUploadDialog" class="modal-overlay" @click.self="showUploadDialog = false">
      <div class="modal">
        <div class="modal-header">
          <h3>上传地图</h3>
          <button class="btn-close" @click="showUploadDialog = false">×</button>
        </div>
        <div class="modal-body">
          <div class="form-group">
            <label>地图名称</label>
            <input v-model="uploadForm.name" type="text" placeholder="输入地图名称" />
          </div>
          <div class="form-group">
            <label>PGM文件</label>
            <input type="file" accept=".pgm" @change="handleFileChange($event, 'pgm_file')" />
          </div>
          <div class="form-group">
            <label>YAML文件</label>
            <input type="file" accept=".yaml,.yml" @change="handleFileChange($event, 'yaml_file')" />
          </div>
          <div class="form-group">
            <label>缩略图（可选）</label>
            <input type="file" accept="image/*" @change="handleFileChange($event, 'thumbnail')" />
          </div>
          <div class="form-group">
            <label>分辨率（m/像素）</label>
            <input v-model.number="uploadForm.resolution" type="number" step="0.01" />
          </div>
          <div class="form-group">
            <label>描述</label>
            <textarea v-model="uploadForm.description" rows="3" placeholder="输入地图描述"></textarea>
          </div>
        </div>
        <div class="modal-footer">
          <button class="btn" @click="showUploadDialog = false">取消</button>
          <button class="btn btn-primary" @click="handleUpload" :disabled="uploading">
            {{ uploading ? '上传中...' : '上传' }}
          </button>
        </div>
      </div>
    </div>

    <div v-if="showCleaner" class="cleaner-overlay">
      <section class="cleaner-dialog" aria-modal="true" role="dialog">
        <header class="cleaner-header">
          <div>
            <h3>擦除动态障碍</h3>
            <span>{{ selectedMap?.name }} · {{ cleanerImageSize.width }} × {{ cleanerImageSize.height }}</span>
          </div>
          <button class="btn-close" type="button" aria-label="关闭" @click="closeCleaner">×</button>
        </header>
        <div class="cleaner-toolbar">
          <div class="cleaner-segmented">
            <button type="button" :class="{ active: cleanerTool === 'erase' }" @click="cleanerTool = 'erase'">擦除</button>
            <button type="button" :class="{ active: cleanerTool === 'move' }" @click="cleanerTool = 'move'">移动</button>
          </div>
          <label class="cleaner-brush">
            <span>画笔</span>
            <select v-model.number="cleanerBrushM">
              <option :value="0.2">0.2 m</option>
              <option :value="0.5">0.5 m</option>
              <option :value="1">1.0 m</option>
            </select>
          </label>
          <div class="cleaner-icon-actions">
            <button type="button" title="撤销" :disabled="!cleanerStrokes.length" @click="undoCleaner">↶</button>
            <button type="button" title="重做" :disabled="!cleanerRedoStrokes.length" @click="redoCleaner">↷</button>
            <button type="button" title="缩小" @click="changeCleanerZoom(-0.2)">−</button>
            <span>{{ Math.round(cleanerZoom * 100) }}%</span>
            <button type="button" title="放大" @click="changeCleanerZoom(0.2)">+</button>
            <button type="button" title="重置擦除" :disabled="!cleanerStrokes.length" @click="resetCleaner">重置</button>
            <button type="button" :class="{ active: cleanerShowTrace }" @click="toggleCleanerTrace">
              {{ cleanerShowTrace ? '隐藏轨迹' : '显示轨迹' }}
            </button>
          </div>
        </div>
        <div ref="cleanerViewport" class="cleaner-viewport" :class="`tool-${cleanerTool}`">
          <canvas
            ref="cleanerCanvas"
            :style="cleanerCanvasStyle"
            @pointerdown.prevent="startCleanerPointer"
            @pointermove.prevent="moveCleanerPointer"
            @pointerup.prevent="stopCleanerPointer"
            @pointercancel.prevent="stopCleanerPointer"
          ></canvas>
        </div>
        <footer class="cleaner-footer">
          <span>已擦除 {{ cleanerStrokes.length }} 笔</span>
          <div>
            <button class="btn" type="button" :disabled="cleanerSaving" @click="closeCleaner">取消</button>
            <button class="btn btn-primary" type="button" :disabled="cleanerSaving || !cleanerStrokes.length" @click="saveCleaner">
              {{ cleanerSaving ? '保存中...' : '保存并启用' }}
            </button>
          </div>
        </footer>
      </section>
    </div>

    <div v-if="showLoopReview" class="cleaner-overlay">
      <section class="loop-review-dialog" aria-modal="true" role="dialog">
        <header class="cleaner-header">
          <div>
            <h3>离线回环阈值审核</h3>
            <span>{{ selectedMap?.name }} · 源地图只读，优化结果另存新版本</span>
          </div>
          <button class="btn-close" type="button" :disabled="loopOptimizeBusy" @click="showLoopReview = false">×</button>
        </header>
        <div class="loop-threshold-grid">
          <label>候选排名 ≤<input v-model.number="loopThresholds.max_rank" type="number" min="1" max="5"></label>
          <label>指纹距离 ≤<input v-model.number="loopThresholds.descriptor_distance_max" type="number" min="0" step="0.01"></label>
          <label>几何平移 ≤ m<input v-model.number="loopThresholds.geometric_translation_max_m" type="number" min="0" step="0.1"></label>
          <label>航向差 ≤ °<input v-model.number="loopThresholds.yaw_consistency_max_deg" type="number" min="0" step="1"></label>
          <label>SLAM 间距 ≥ m<input v-model.number="loopThresholds.slam_distance_min_m" type="number" min="0" step="0.5"></label>
          <label>SLAM 间距 ≤ m<input v-model.number="loopThresholds.slam_distance_max_m" type="number" min="0" step="0.5"></label>
          <button class="btn btn-sm" type="button" :disabled="loopReviewLoading || loopOptimizeBusy" @click="loadLoopReview">
            {{ loopReviewLoading ? '审核中…' : '重新审核' }}
          </button>
        </div>
        <div v-if="loopReview" class="loop-review-summary">
          关键帧 {{ loopReview.keyframe_count }} · 候选 {{ loopReview.candidate_count }} · 通过 {{ loopReview.eligible_count }} · 已选 {{ selectedLoopCandidates.size }}
        </div>
        <div class="loop-candidate-table-wrap">
          <table v-if="loopReview?.candidates?.length" class="loop-candidate-table">
            <thead><tr><th>选</th><th>关键帧</th><th>排名</th><th>指纹距离</th><th>几何平移</th><th>SLAM 间距</th><th>航向差</th><th>审核</th></tr></thead>
            <tbody>
              <tr v-for="item in loopReview.candidates" :key="item.candidate_id" :class="{ eligible: item.eligible }">
                <td><input type="checkbox" :checked="selectedLoopCandidates.has(item.candidate_id)" :disabled="!item.eligible || loopOptimizeBusy" @change="toggleLoopCandidate(item.candidate_id)"></td>
                <td>{{ item.candidate_id }}</td><td>{{ item.rank }}</td>
                <td>{{ Number(item.descriptor_distance).toFixed(3) }}</td>
                <td>{{ Number(item.geometric_translation_m).toFixed(2) }} m</td>
                <td>{{ Number(item.slam_distance_m).toFixed(2) }} m</td>
                <td>{{ Number(item.yaw_consistency_deg).toFixed(1) }}°</td>
                <td :title="item.failed_checks.join(', ')">{{ item.eligible ? '通过' : item.failed_checks.join(' / ') }}</td>
              </tr>
            </tbody>
          </table>
          <div v-else-if="!loopReviewLoading" class="empty-state">当前地图包没有回环候选</div>
        </div>
        <footer class="cleaner-footer">
          <span>{{ loopOptimizeStatus || '执行后会重建 PCD、PGM、轨迹和指纹定位验证，默认不自动启用。' }}</span>
          <div>
            <button class="btn" type="button" :disabled="loopOptimizeBusy" @click="showLoopReview = false">取消</button>
            <button class="btn btn-primary" type="button" :disabled="loopOptimizeBusy || !selectedLoopCandidates.size" @click="confirmLoopOptimization">
              {{ loopOptimizeBusy ? '优化并上传中…' : '确认优化' }}
            </button>
          </div>
        </footer>
      </section>
    </div>
  </section>
</template>

<style scoped>
.mapping-mode-switch {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 0.75rem;
  margin: 1rem 0;
}

.mapping-mode-switch button {
  display: grid;
  gap: 0.3rem;
  padding: 0.9rem 1rem;
  border: 1px solid #d6dee9;
  border-radius: 10px;
  background: #fff;
  color: #344054;
  text-align: left;
  transition: border-color 0.18s, box-shadow 0.18s, transform 0.18s;
}

.mapping-mode-switch button:not(:disabled):hover {
  border-color: #66a6df;
  transform: translateY(-1px);
}

.mapping-mode-switch button.active {
  border-color: #1976d2;
  background: linear-gradient(135deg, #eef7ff, #fff);
  box-shadow: 0 0 0 2px rgba(25, 118, 210, 0.12);
}

.mapping-mode-switch strong { font-size: 0.95rem; }
.mapping-mode-switch span { color: #667085; font-size: 0.78rem; line-height: 1.45; }

.origin-quality-card {
  display: grid;
  gap: 0.85rem;
  margin: 1rem 0;
  padding: 1rem;
  border: 1px solid #f1c77b;
  border-radius: 12px;
  background: linear-gradient(145deg, #fffbeb, #fff);
}

.origin-quality-card.origin-locked {
  border-color: #6fcf97;
  background: linear-gradient(145deg, #ecfdf3, #fff);
}

.origin-quality-head,
.origin-coordinate {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 0.75rem;
  flex-wrap: wrap;
}

.origin-quality-head > div { display: grid; gap: 0.15rem; }
.origin-kicker { color: #8a5a00; font-size: 0.7rem; font-weight: 700; letter-spacing: 0.08em; text-transform: uppercase; }
.origin-state-chip { padding: 0.35rem 0.65rem; border-radius: 999px; background: #fff1c7; color: #8a5a00; font-size: 0.72rem; font-weight: 800; font-variant-numeric: tabular-nums; }
.origin-state-chip.locked { background: #dff7e8; color: #176b3a; }
.origin-state-chip.failed { background: #fee4e2; color: #b42318; }
.origin-quality-card progress { width: 100%; height: 0.55rem; accent-color: #e4a11b; }
.origin-locked progress { accent-color: #198754; }
.origin-quality-card p { margin: 0; color: #5f4b20; font-size: 0.8rem; }
.origin-coordinate { padding-top: 0.65rem; border-top: 1px dashed #9ed6b5; color: #176b3a; font: 600 0.75rem ui-monospace, SFMono-Regular, Menlo, monospace; }
.origin-diagnostic-table-wrap { overflow-x: auto; border: 1px solid #eadfba; border-radius: 8px; background: rgba(255,255,255,0.82); }
.origin-diagnostic-table { width: 100%; min-width: 760px; border-collapse: collapse; table-layout: fixed; }
.origin-diagnostic-table th,
.origin-diagnostic-table td { padding: 0.55rem 0.65rem; border-right: 1px solid #eee5ca; border-bottom: 1px solid #eee5ca; text-align: left; }
.origin-diagnostic-table th { width: 17%; color: #8a6b24; background: rgba(255,248,230,0.65); font-size: 0.68rem; font-weight: 600; }
.origin-diagnostic-table td { width: 33%; color: #344054; font: 700 0.72rem ui-monospace, SFMono-Regular, Menlo, monospace; }
.origin-diagnostic-table tr:last-child th,
.origin-diagnostic-table tr:last-child td { border-bottom: 0; }
.origin-diagnostic-table td.ok { color: #16794a; }
.origin-diagnostic-table td.warn { color: #a15c00; }
.origin-diagnostic-table td.bad { color: #c43232; }
.origin-diagnostic-table-wrap.stale { border-color: #e58b8b; }
.heading-review-panel { display: grid; gap: 0.65rem; padding: 0.8rem; border: 1px solid #d6dee9; border-radius: 9px; background: #f8fbff; }
.heading-review-panel.review-ready { border-color: #6fcf97; background: #f1fff6; }
.heading-review-head { display: flex; align-items: center; justify-content: space-between; gap: 0.8rem; }
.heading-review-head > div { display: grid; gap: 0.15rem; }
.heading-review-head small { color: #667085; font-size: 0.7rem; }
.heading-review-head > span { color: #176b3a; font-size: 1.05rem; font-weight: 800; font-variant-numeric: tabular-nums; }
.origin-lock-check-grid,
.heading-quality-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 0.6rem; }
.origin-lock-check-grid > div,
.heading-quality-grid > div { display: flex; gap: 0.55rem; min-width: 0; padding: 0.65rem; border: 1px solid #e6e9ef; border-radius: 8px; background: #fff; }
.origin-lock-check-grid > div > span,
.heading-quality-grid > div > span { color: #98a2b3; font-size: 1rem; font-weight: 800; }
.origin-lock-check-grid > div.ok,
.heading-quality-grid > div.ok { border-color: #9ed6b5; background: #f1fff6; }
.origin-lock-check-grid > div.ok > span,
.heading-quality-grid > div.ok > span { color: #198754; }
.origin-lock-check-grid > div > div,
.heading-quality-grid > div > div { display: grid; min-width: 0; gap: 0.15rem; }
.origin-lock-check-grid strong,
.heading-quality-grid strong { color: #344054; font-size: 0.78rem; }
.origin-lock-check-grid small,
.heading-quality-grid small { overflow: hidden; color: #667085; font-size: 0.7rem; text-overflow: ellipsis; white-space: nowrap; }
.heading-review-summary { display: flex; justify-content: space-between; gap: 0.8rem; color: #667085; font-size: 0.7rem; }
.mapping-odom-strip { display: flex; flex-wrap: wrap; align-items: center; gap: 0.45rem 0.85rem; padding: 0.65rem 0.75rem; border: 1px solid #e6e9ef; border-radius: 8px; color: #667085; background: rgba(255,255,255,0.85); font-size: 0.72rem; }
.mapping-odom-strip strong { color: #344054; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
.mapping-odom-strip.online { border-color: #9ed6b5; color: #176b3a; }
.mapping-odom-strip.offline { border-color: #f1b4b4; color: #b42318; }

.mapping-actions .btn-origin { border-color: #d99a19; color: #7a5100; background: #fff8e6; }
.mapping-actions .btn-confirm { border-color: #198754; color: #fff; background: #198754; }
.mapping-actions .btn-confirm:disabled { border-color: #b9c4cf; background: #b9c4cf; }

@media (max-width: 900px) {
  .origin-diagnostic-table { min-width: 680px; }
  .origin-lock-check-grid,
  .heading-quality-grid { grid-template-columns: 1fr; }
}

@media (max-width: 620px) {
  .mapping-mode-switch { grid-template-columns: 1fr; }
  .heading-review-head { align-items: flex-start; }
}

.map-full-preview {
  background: #f9f9f9;
  border-radius: 8px;
  padding: 1rem;
  margin-bottom: 1.5rem;
  border: 1px solid #e0e0e0;
}

.map-set-list {
  margin: 0 0 1.25rem;
}

.map-set-card {
  display: flex;
  justify-content: space-between;
  align-items: center;
  gap: 1rem;
  padding: .75rem;
  border: 1px solid #d7dde5;
  border-radius: 6px;
  background: #fff;
}

.map-set-card h4 { margin: 0 0 .25rem; font-size: .95rem; }
.map-set-card span { color: #667085; font-size: .8rem; }
.map-set-members { display: flex; flex-wrap: wrap; justify-content: flex-end; gap: .35rem; }

.map-selector-row {
  margin-bottom: 1rem;
}

.map-selector-row label {
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

.map-selector-row span {
  font-weight: 500;
  white-space: nowrap;
}

.map-selector-row select {
  flex: 1;
  padding: 0.5rem;
  border: 1px solid #e0e0e0;
  border-radius: 4px;
  font-size: 0.875rem;
  max-width: 400px;
}

.map-preview-content {
  display: flex;
  gap: 1.5rem;
  min-height: 200px;
}

.map-preview-image {
  flex: 0 0 320px;
  background: #fff;
  border: 1px solid #e0e0e0;
  border-radius: 6px;
  display: flex;
  align-items: center;
  justify-content: center;
  overflow: hidden;
  min-height: 220px;
}

.map-preview-image img {
  max-width: 100%;
  max-height: 100%;
  object-fit: contain;
}

.map-image-stage { position: relative; display: inline-flex; max-width: 100%; max-height: 100%; }
.map-image-stage img { display: block; }
.map-trace-overlay { position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; }
.map-trace-overlay polyline { fill: none; stroke-width: 2.5; stroke-linecap: round; stroke-linejoin: round; vector-effect: non-scaling-stroke; }
.map-trace-overlay .trace-raw { stroke: #f59e0b; stroke-dasharray: 7 5; opacity: 0.85; }
.map-trace-overlay .trace-optimized { stroke: #1678ff; }
.map-trace-overlay .trace-corrections line { stroke: #dc2626; stroke-width: 1.4; vector-effect: non-scaling-stroke; }
.map-trace-overlay .trace-corrections path { fill: #dc2626; }
.map-trace-overlay .trace-corrections circle { fill: none; stroke: #7c3aed; stroke-width: 1.5; vector-effect: non-scaling-stroke; }

.no-preview {
  color: #999;
  text-align: center;
  padding: 2rem;
  font-size: 0.875rem;
}

.preview-debug {
  margin-top: 0.5rem;
  font-size: 0.7rem;
  color: #ccc;
  word-break: break-all;
}

.map-preview-info {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.map-preview-info h3 {
  margin: 0;
  font-size: 1.15rem;
}

.map-details {
  display: flex;
  flex-direction: column;
  gap: 0.35rem;
  font-size: 0.875rem;
  color: #555;
}

.map-artifact-panel { display: grid; gap: 0.35rem; padding: 0.7rem 0.75rem; border: 1px solid #e5e7eb; border-radius: 6px; background: #fff; font-size: 0.78rem; }
.map-artifact-head, .map-artifact-row { display: flex; align-items: center; justify-content: space-between; gap: 0.75rem; }
.map-artifact-head { padding-bottom: 0.25rem; border-bottom: 1px solid #eef0f3; }
.map-artifact-head span { color: #667085; font-size: 0.7rem; }
.map-artifact-row strong { color: #137333; }
.map-artifact-row strong.missing { color: #b42318; }
.map-artifact-panel small { color: #667085; }
.mapping-metrics-panel { display: grid; gap: 0.55rem; padding: 0.7rem 0.75rem; border: 1px solid #b8c7e8; border-radius: 6px; background: #f7f9ff; font-size: 0.78rem; }
.mapping-metrics-runtime { margin-top: 0.7rem; }
.mapping-metrics-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 0.45rem 0.8rem; }
.mapping-metrics-grid span { display: flex; justify-content: space-between; gap: 0.6rem; color: #667085; }
.mapping-metrics-grid strong { color: #1f2937; white-space: nowrap; }
.post-save-validation-panel { display: grid; gap: 0.6rem; margin-top: 0.7rem; padding: 0.7rem 0.75rem; border: 1px solid #d0d5dd; border-radius: 6px; background: #fff; font-size: 0.78rem; }
.validation-current-result { display: grid; gap: 0.5rem; padding: 0.65rem; border: 1px solid #f0a69a; border-radius: 6px; background: #fff4f2; }
.validation-current-result.is-accurate { border-color: #8bd3a8; background: #f0fdf4; }
.validation-result-title, .validation-position, .validation-quality-grid span { display: flex; justify-content: space-between; gap: 0.75rem; }
.validation-result-title span { color: #b42318; font-weight: 650; }
.validation-current-result.is-accurate .validation-result-title span { color: #067647; }
.validation-position { align-items: baseline; color: #667085; }
.validation-position strong { color: #101828; text-align: right; font: 650 0.75rem ui-monospace, SFMono-Regular, Menlo, monospace; }
.validation-reference { padding-top: 0.35rem; border-top: 1px dashed #d0d5dd; }
.validation-quality-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 0.35rem 0.8rem; }
.validation-quality-grid span { color: #667085; }
.validation-quality-grid strong { color: #344054; }
.validation-current-result small, .validation-waiting { color: #667085; }
.validation-counters { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 0.6rem; }
.validation-counters > div { display: grid; grid-template-columns: 1fr auto; gap: 0.25rem 0.5rem; align-items: center; }
.validation-counters progress { grid-column: 1 / -1; width: 100%; }
.validation-counters small { grid-column: 1 / -1; color: #667085; }
.optimization-panel { display: grid; gap: 0.45rem; padding: 0.65rem 0.75rem; border: 1px solid #93c5fd; border-radius: 6px; background: #eff6ff; font-size: 0.78rem; }
.optimization-runtime { margin-top: 0.7rem; }
.optimization-line { overflow: hidden; color: #174ea6; font-weight: 650; text-overflow: ellipsis; white-space: nowrap; }
.optimization-no_valid_loop { border-color: #cbd5e1; background: #f8fafc; }
.optimization-fallback, .optimization-failed { border-color: #f0a69a; background: #fff4f2; }
.optimization-fallback .optimization-line, .optimization-failed .optimization-line { color: #b42318; }
.optimization-panel details summary { cursor: pointer; color: #475467; }
.loop-review-button { justify-self: start; }
.optimization-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 0.4rem 0.8rem; padding-top: 0.5rem; }
.optimization-grid span { display: flex; justify-content: space-between; gap: 0.6rem; color: #667085; }
.optimization-grid strong { color: #1f2937; text-align: right; }
.enu-guard { margin-top: 0.5rem; padding: 0.4rem 0.5rem; border-radius: 4px; background: #ecfdf3; color: #067647; }
.enu-guard.danger { background: #fef3f2; color: #b42318; font-weight: 650; }
.global-enu-panel { display: grid; gap: 0.45rem; padding: 0.7rem 0.75rem; border: 1px solid #9ed6b5; border-radius: 6px; background: #f3fff7; font-size: 0.78rem; }
.global-enu-values { display: flex; flex-wrap: wrap; gap: 0.45rem 0.8rem; color: #176b3a; font: 600 0.72rem ui-monospace, SFMono-Regular, Menlo, monospace; }
.global-enu-panel small { color: #667085; }

.map-sync-panel {
  display: grid;
  gap: 0.4rem;
  padding: 0.75rem;
  background: #fff;
  border: 1px solid #e5e7eb;
  border-radius: 6px;
  font-size: 0.8rem;
}

.sync-row {
  display: grid;
  grid-template-columns: 7rem minmax(0, 1fr);
  gap: 0.75rem;
  align-items: center;
}

.sync-row span {
  color: #667085;
}

.sync-row strong {
  min-width: 0;
  overflow-wrap: anywhere;
  color: #1f2937;
}

.sync-status.status-online {
  color: #137333;
}

.sync-status.status-offline {
  color: #b42318;
}

.sync-status.status-unknown {
  color: #667085;
}

.map-description {
  background: #fff;
  padding: 0.75rem;
  border-radius: 4px;
  font-size: 0.8rem;
  color: #666;
  border: 1px solid #eee;
}

.map-preview-actions {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-top: auto;
  padding-top: 0.75rem;
}

.section-subtitle {
  font-size: 0.95rem;
  color: #666;
  margin: 0;
  padding-bottom: 0.5rem;
  border-bottom: 1px solid #eee;
}

.map-list-section {
  margin-bottom: 1.5rem;
}

.map-list-heading {
  display: flex;
  align-items: flex-end;
  justify-content: space-between;
  gap: 1rem;
  margin-bottom: 0.75rem;
}

.map-list-actions {
  display: flex;
  gap: 0.5rem;
}

.map-groups {
  display: grid;
  gap: 0.65rem;
}

.map-group {
  border: 1px solid #e5e7eb;
  border-radius: 6px;
  overflow: hidden;
  background: rgba(255, 255, 255, 0.46);
}

.map-group-header {
  width: 100%;
  display: flex;
  align-items: center;
  gap: 0.55rem;
  padding: 0.7rem 0.85rem;
  border: 0;
  background: rgba(248, 250, 252, 0.86);
  color: inherit;
  text-align: left;
}

.map-group-header:hover {
  background: rgba(226, 242, 253, 0.86);
}

.map-group-chevron {
  width: 1rem;
  color: #1976d2;
  font-size: 1rem;
}

.map-group-title {
  font-weight: 700;
  color: #334155;
}

.map-group-count {
  color: #64748b;
  font-size: 0.78rem;
}

.map-group-header .badge {
  margin-left: auto;
}

.map-group .map-list {
  padding: 0.65rem;
  border-top: 1px solid #e5e7eb;
}

.map-card {
  cursor: pointer;
  border: 2px solid transparent;
  transition: border-color 0.2s, background 0.2s;
}

.map-card:hover {
  border-color: #90caf9;
}

.map-card-selected {
  border-color: #1976d2 !important;
  background: #e3f2fd !important;
}

.map-card-info {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.map-card-info h4 {
  margin: 0;
  font-size: 0.9rem;
}

.map-card-size {
  font-size: 0.75rem;
  color: #999;
}

.badge-sm {
  padding: 0.15rem 0.35rem;
  font-size: 0.65rem;
}

.cleaner-overlay {
  position: fixed;
  inset: 0;
  z-index: 1200;
  display: grid;
  place-items: center;
  padding: 1rem;
  background: rgba(15, 23, 42, 0.72);
}

.cleaner-dialog {
  width: min(1180px, 100%);
  height: min(860px, calc(100vh - 2rem));
  display: grid;
  grid-template-rows: auto auto minmax(0, 1fr) auto;
  overflow: hidden;
  background: #fff;
  border-radius: 6px;
}

.loop-review-dialog {
  width: min(1100px, 100%);
  max-height: min(860px, calc(100vh - 2rem));
  display: grid;
  grid-template-rows: auto auto auto minmax(180px, 1fr) auto;
  overflow: hidden;
  background: #fff;
  border-radius: 6px;
}

.loop-threshold-grid {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 0.55rem;
  padding: 0.75rem 1rem;
  background: #f8fafc;
  border-bottom: 1px solid #d9dee7;
}
.loop-threshold-grid label { display: flex; align-items: center; justify-content: space-between; gap: 0.5rem; font-size: 0.75rem; color: #475467; }
.loop-threshold-grid input { width: 6rem; padding: 0.35rem; border: 1px solid #cfd6e1; border-radius: 4px; }
.loop-review-summary { padding: 0.55rem 1rem; color: #174ea6; font-size: 0.8rem; font-weight: 650; border-bottom: 1px solid #e5e7eb; }
.loop-candidate-table-wrap { min-height: 0; overflow: auto; padding: 0.5rem 1rem; }
.loop-candidate-table { width: 100%; border-collapse: collapse; font-size: 0.73rem; }
.loop-candidate-table th, .loop-candidate-table td { padding: 0.42rem; border-bottom: 1px solid #e5e7eb; text-align: right; white-space: nowrap; }
.loop-candidate-table th:nth-child(2), .loop-candidate-table td:nth-child(2), .loop-candidate-table th:last-child, .loop-candidate-table td:last-child { text-align: left; }
.loop-candidate-table tr:not(.eligible) { color: #98a2b3; }
.loop-candidate-table tr.eligible { background: #f0fdf4; color: #166534; }

.cleaner-header,
.cleaner-toolbar,
.cleaner-footer {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 1rem;
  padding: 0.75rem 1rem;
  border-bottom: 1px solid #d9dee7;
}

.cleaner-header h3 { margin: 0; font-size: 1rem; }
.cleaner-header span { color: #667085; font-size: 0.75rem; }
.cleaner-toolbar { justify-content: flex-start; flex-wrap: wrap; background: #f8fafc; }
.cleaner-footer { border-top: 1px solid #d9dee7; border-bottom: 0; }
.cleaner-footer > div { display: flex; gap: 0.5rem; }
.cleaner-footer > span { color: #667085; font-size: 0.8rem; }

.cleaner-segmented,
.cleaner-icon-actions {
  display: flex;
  align-items: center;
  gap: 0.25rem;
}

.cleaner-segmented button,
.cleaner-icon-actions button {
  min-width: 2.25rem;
  min-height: 2.25rem;
  border: 1px solid #cfd6e1;
  border-radius: 4px;
  background: #fff;
  color: #344054;
}

.cleaner-segmented button.active {
  border-color: #1976d2;
  background: #e8f1fb;
  color: #0b5cad;
  font-weight: 700;
}

.cleaner-brush { display: flex; align-items: center; gap: 0.5rem; font-size: 0.8rem; }
.cleaner-brush select { min-height: 2.25rem; border: 1px solid #cfd6e1; border-radius: 4px; }
.cleaner-icon-actions span { width: 3.25rem; text-align: center; font-size: 0.75rem; }

.cleaner-viewport {
  min-height: 0;
  overflow: auto;
  padding: 12px;
  background: #27313f;
  overscroll-behavior: contain;
  user-select: none;
  -webkit-user-select: none;
  touch-action: none;
}

.cleaner-viewport canvas {
  display: block;
  max-width: none;
  background: #fff;
  image-rendering: pixelated;
  touch-action: none;
}

.cleaner-viewport.tool-erase canvas { cursor: crosshair; }
.cleaner-viewport.tool-move canvas { cursor: grab; }

@media (max-width: 720px) {
  .cleaner-overlay { padding: 0; }
  .cleaner-dialog { width: 100%; height: 100vh; border-radius: 0; }
  .loop-review-dialog { width: 100%; max-height: 100vh; height: 100vh; border-radius: 0; }
  .loop-threshold-grid { grid-template-columns: 1fr; }
  .cleaner-header, .cleaner-toolbar, .cleaner-footer { padding: 0.6rem; }
  .cleaner-toolbar { gap: 0.5rem; }
}
</style>
