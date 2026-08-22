<script setup>
import { nextTick, onBeforeUnmount, onMounted, ref, computed, watch } from 'vue'
import {
  fetchMaps,
  fetchMapSets,
  fetchRobots,
  deleteMap,
  downloadMap,
  setActiveMap,
  manuallyCleanMap,
  createMap,
  fetchRobotMappingStatus,
  startRobotMappingOrigin,
  cancelRobotMappingOrigin,
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
const mappingStatus = ref(null)
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
const cleanerImageSize = ref({ width: 0, height: 0 })
let cleanerImage = null
let activeCleanerStroke = null
let cleanerPanStart = null

const mappingForm = ref({
  robot: '',
  map_name: '太阳宫园区 V1',
  route_hint: '南门 → 主步道 → 牡丹园 → 活动广场',
  record_rosbag: true,
  scene_scope: 'indoor',
  mapping_type: 'indoor',
})
let statusTimer = null
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

onMounted(async () => {
  await loadMaps()
  await loadRobots()
  if (mappingForm.value.robot) await refreshMappingStatus()
  statusTimer = setInterval(() => {
    if (mappingForm.value.robot) refreshMappingStatus()
  }, 1000)
})

onBeforeUnmount(() => {
  if (statusTimer) clearInterval(statusTimer)
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
const selectedMap = computed(() => maps.value.find(m => m.id === selectedMapId.value))
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

watch(selectedMapId, (mapId) => ensureMapGroupExpanded(mapId))
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
const mappingState = computed(() => mappingStatus.value?.mapping_state || 'idle')
const commandStatus = computed(() => mappingStatus.value?.command_status || 'idle')
const mappingCommandInFlight = computed(() => ['created', 'published', 'accepted', 'executing'].includes(commandStatus.value))
const saveProgress = computed(() => mappingStatus.value?.result?.save_progress || {})
const rosbagStatus = computed(() => mappingStatus.value?.result?.rosbag || {})
const originStatus = computed(() => mappingStatus.value?.result?.origin || {})
const originState = computed(() => originStatus.value.origin_status || 'idle')
const isOutdoorMapping = computed(() => mappingForm.value.mapping_type === 'outdoor')
const originLocked = computed(() => originState.value === 'locked')
const originLockPercent = computed(() => {
  const required = Number(originStatus.value.required_seconds || 60)
  return required > 0 ? Math.min(100, Number(originStatus.value.continuous_seconds || 0) / required * 100) : 0
})
const originStatusMessage = computed(() => {
  if (originStatus.value.message) return originStatus.value.message
  if (!originStatus.value.position_fixed) return '等待 RTK 位置 FIX，暂不能锁定 ENU 原点'
  if (!originStatus.value.heading_fixed) return '位置已 FIX，等待双天线航向 FIX'
  if (!originStatus.value.heading_stable) return '位置和航向已接入，等待连续稳定质量窗'
  return '质量条件满足后即可锁定 ENU 原点'
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

const stateSteps = computed(() => [
  { key: 'idle', label: '配置' },
  { key: 'command_created', label: '已创建' },
  { key: 'command_published', label: '已下发' },
  { key: 'command_accepted', label: 'Edge确认' },
  ...(isOutdoorMapping.value ? [
    { key: 'origin_starting', label: '启动RTK' },
    { key: 'origin_waiting', label: '锁定原点' },
    { key: 'origin_locked', label: '原点已锁' },
  ] : []),
  { key: 'slam_starting', label: '启动SLAM' },
  { key: 'slam_warmup', label: 'SLAM预热' },
  { key: 'imu_initializing', label: 'IMU初始化' },
  { key: 'waiting_first_keyframe', label: '位姿确认' },
  { key: 'ready_to_map', label: isOutdoorMapping.value ? '航向复核' : '等待确认' },
  { key: 'mapping', label: '正式采集' },
  { key: 'saving', label: '保存中' },
  { key: 'packaging', label: '打包中' },
  { key: 'uploading', label: '上传中' },
  { key: 'stopping', label: '退出建图' },
  { key: 'exited', label: '已退出建图' },
])

const terminalStates = ['command_timed_out', 'command_failed', 'command_rejected', 'cancelled', 'completed', 'exited']
const isTerminal = computed(() => terminalStates.includes(mappingState.value))
const isError = computed(() => (
  slamDiverged.value
  || (mappingProcessAlive.value && readinessState.value === 'telemetry_stale')
  || ['command_timed_out', 'command_failed', 'command_rejected'].includes(mappingState.value)
))
const isActiveMapping = computed(() => [
  'command_created', 'command_published', 'command_accepted', 'starting',
  'origin_starting', 'origin_waiting', 'origin_locked', 'slam_starting', 'slam_warmup',
  'ready_to_map', 'mapping', 'saving', 'packaging', 'uploading', 'stopping',
].includes(mappingState.value))
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
const canSaveMapping = computed(() => (
  !mappingCommandInFlight.value && (slamDiverged.value || (mappingState.value === 'mapping' && readyForSave.value))
))
const canCancelMapping = computed(() => (
  isActiveMapping.value || Boolean(mappingStatus.value?.result?.process_alive) || ['waiting_quality', 'quality_holding'].includes(originState.value)
))
const showMappingReadiness = computed(() => (
  Boolean(mappingStatus.value?.result?.process_alive)
  || ['slam_starting', 'slam_warmup', 'ready_to_map', 'mapping', 'saving', 'packaging', 'uploading', 'stopping'].includes(mappingState.value)
))
const workflowSessionId = computed(() => mappingStatus.value?.result?.mapping_session_id || '')
const mappingModeSwitchDisabled = computed(() => (
  mappingBusy.value
  || mappingCommandInFlight.value
  || mappingProcessAlive.value
  || ['slam_starting', 'slam_warmup', 'ready_to_map', 'mapping', 'saving', 'packaging', 'uploading', 'stopping'].includes(mappingState.value)
))
const canLockOrigin = computed(() => (
  isOutdoorMapping.value
  && connectionStatus.value === 'online'
  && !mappingProcessAlive.value
  && !originLocked.value
  && !mappingCommandInFlight.value
  && !['waiting_quality', 'quality_holding'].includes(originState.value)
))
const canStartSlam = computed(() => (
  connectionStatus.value === 'online'
  && !mappingProcessAlive.value
  && !mappingCommandInFlight.value
  && (!isOutdoorMapping.value || originLocked.value || ['idle', 'cancelled', 'failed'].includes(originState.value))
))
const canBeginMapping = computed(() => (
  mappingProcessAlive.value
  && !mappingCommandInFlight.value
  && Boolean(mappingStatus.value?.result?.ready_for_mapping ?? mappingReadiness.value.ready_for_mapping)
  && (!isOutdoorMapping.value || (originLocked.value && originStatus.value.heading_stable))
  && !mappingStatus.value?.result?.mapping_capture_enabled
))

const activeStepIndex = computed(() => {
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
    cancelled: '已取消',
  }
  if (readinessState.value === 'telemetry_stale') return '传感器状态超时'
  return slamDiverged.value ? 'SLAM 发散' : (labels[mappingState.value] || '错误')
})

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
    const [loadedMaps, loadedMapSets] = await Promise.all([fetchMaps(), fetchMapSets()])
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

async function refreshMappingStatus() {
  if (!mappingForm.value.robot) return
  try {
    mappingStatus.value = await fetchRobotMappingStatus(mappingForm.value.robot)
    const reportedType = mappingStatus.value?.result?.mapping_type
    const reportedState = mappingStatus.value?.mapping_state || mappingStatus.value?.result?.state || 'idle'
    const activeReportedWorkflow = [
      'command_created', 'command_published', 'command_accepted', 'starting',
    'origin_starting', 'origin_waiting', 'origin_locked', 'slam_starting',
      'slam_warmup', 'ready_to_map', 'mapping', 'saving', 'packaging',
      'uploading', 'stopping',
    ].includes(reportedState)
    // An idle Edge Agent reports its default indoor mapping type even when
    // the operator has just selected outdoor mode. Keep that explicit user
    // choice so a missing RTK fix is shown in the origin panel instead of
    // silently switching the form back to indoor.
    const shouldAdoptReportedType = (
      reportedType === 'outdoor'
      || mappingForm.value.mapping_type === 'indoor'
      || activeReportedWorkflow
    )
    if (['indoor', 'outdoor'].includes(reportedType)
      && reportedType !== mappingForm.value.mapping_type
      && shouldAdoptReportedType) {
      mappingForm.value.mapping_type = reportedType
    }
  } catch (error) {
    console.error('获取建图状态失败:', error)
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
  } catch (error) {
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
  await refreshMappingStatus()
}

async function handleLockOrigin() {
  if (!mappingForm.value.robot || connectionStatus.value !== 'online') {
    alert('请先选择已连接 Edge Agent 的机器狗')
    return
  }
  mappingBusy.value = true
  try {
    await startRobotMappingOrigin(mappingForm.value.robot, {
      map_name: mappingForm.value.map_name,
      route_hint: mappingForm.value.route_hint,
      scene_scope: mappingForm.value.scene_scope,
      mapping_type: 'outdoor',
      mapping_session_id: workflowSessionId.value,
    })
    await refreshMappingStatus()
  } catch (error) {
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
  try {
    await beginRobotMapping(mappingForm.value.robot, {
      mapping_session_id: workflowSessionId.value,
      heading_check_confirmed: isOutdoorMapping.value,
    })
    await refreshMappingStatus()
  } catch (error) {
    alert(`开始正式建图失败: ${error.message}`)
  } finally {
    mappingBusy.value = false
  }
}

async function handleSaveMapping() {
  if (!mappingForm.value.robot) return
  mappingBusy.value = true
  try {
    await saveRobotMapping(mappingForm.value.robot, {
      map_name: mappingForm.value.map_name,
    })
    await refreshMappingStatus()
    await loadMaps()
  } catch (error) {
    alert(`停止并保存失败: ${error.message}`)
  } finally {
    mappingBusy.value = false
  }
}

async function handleCancelMapping() {
  if (!mappingForm.value.robot) return
  if (!confirm('确定要取消本次建图吗？')) return
  mappingBusy.value = true
  try {
    if (!mappingProcessAlive.value && ['ready', 'waiting_quality', 'quality_holding', 'locked'].includes(originState.value)) {
      await cancelRobotMappingOrigin(mappingForm.value.robot, { mapping_session_id: workflowSessionId.value })
    } else {
      await cancelRobotMapping(mappingForm.value.robot, { reason: 'operator_cancel', mapping_session_id: workflowSessionId.value })
    }
    await refreshMappingStatus()
  } catch (error) {
    alert(`取消建图失败: ${error.message}`)
  } finally {
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

function parseDescription(desc) {
  try {
    if (desc && (desc.startsWith('{') || desc.startsWith('['))) {
      return JSON.parse(desc)
    }
  } catch {}
  return { raw: desc || '' }
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
            <img
              v-else
              :src="fullPreviewUrl(selectedMap.thumbnail_url)"
              :alt="selectedMap.name"
              @error="handleImageError($event, selectedMap)"
            />
          </div>
          <div class="map-preview-info">
            <h3>{{ selectedMap.name }}</h3>
            <div class="map-details">
              <div><strong>机器人:</strong> {{ selectedMap.robot_name }} ({{ selectedMap.robot_code }})</div>
              <div><strong>分辨率:</strong> {{ selectedMap.resolution }} m/像素</div>
              <div><strong>大小:</strong> {{ formatSize(selectedMap.file_size) }}</div>
              <div v-if="selectedMap.width"><strong>尺寸:</strong> {{ selectedMap.width }} × {{ selectedMap.height }}</div>
              <div v-if="selectedMap.coordinate_mode || parseDescription(selectedMap.description).coordinate_mode">
                <strong>坐标:</strong>
                {{ (selectedMap.coordinate_mode || parseDescription(selectedMap.description).coordinate_mode) === 'local_only' ? '无 RTK 原点 / 仅室内 NDT' : 'RTK 原点' }}
              </div>
              <div v-if="selectedMap.scene_scope || parseDescription(selectedMap.description).scene_scope">
                <strong>场景:</strong> {{ selectedMap.scene_scope || parseDescription(selectedMap.description).scene_scope }}
              </div>
              <div v-if="parseDescription(selectedMap.description).rescue" class="rescue-map-label">
                <strong>质量:</strong> 发散救援地图，启用前必须现场核对
              </div>
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
              <template v-if="parseDescription(selectedMap.description).source">
                <div><strong>来源:</strong> {{ parseDescription(selectedMap.description).source === 'edge_mapping' ? 'Edge Agent 建图' : parseDescription(selectedMap.description).source }}</div>
                <div v-if="parseDescription(selectedMap.description).map_version"><strong>版本:</strong> {{ parseDescription(selectedMap.description).map_version }}</div>
              </template>
              <template v-else>
                {{ selectedMap.description }}
              </template>
            </div>
            <div class="map-preview-actions">
              <span v-if="selectedMap.active" class="badge badge-success">活动地图</span>
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
            <span>{{ mapSet.members.length }} 个子图 · {{ mapSet.manifest?.total_distance_m || '—' }} m · 重叠 {{ mapSet.manifest?.overlap_m || '—' }} m</span>
          </div>
          <div class="map-set-members">
            <span v-for="member in mapSet.members" :key="member.submap_id" class="badge badge-sm">{{ member.submap_id }}</span>
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
              <strong>{{ originLocked ? '原点已锁定' : '60 秒连续质量窗' }}</strong>
            </div>
            <span class="origin-countdown">
              {{ Math.floor(Number(originStatus.continuous_seconds || 0)) }} / {{ Number(originStatus.required_seconds || 60) }}s
            </span>
          </div>
          <progress :value="originLockPercent" max="100"></progress>
          <div class="origin-quality-grid">
            <div :class="{ ok: originStatus.position_fixed }">
              <span>{{ originStatus.position_fixed ? '✓' : '…' }}</span>
              <div><strong>位置 FIX</strong><small>{{ originStatus.ntrip_quality || '等待 RTK FIX' }}</small></div>
            </div>
            <div :class="{ ok: originStatus.heading_fixed }">
              <span>{{ originStatus.heading_fixed ? '✓' : '…' }}</span>
              <div><strong>双天线航向 FIX</strong><small>基线 {{ Number(originStatus.baseline_m || 0).toFixed(2) }} m</small></div>
            </div>
            <div :class="{ ok: Number(originStatus.position_spread_m ?? 1) <= 0.02 }">
              <span>{{ Number(originStatus.position_spread_m ?? 1) <= 0.02 ? '✓' : '…' }}</span>
              <div><strong>位置波动 &lt; 2 cm</strong><small>{{ originStatus.position_spread_m == null ? '等待稳定窗口' : `${(Number(originStatus.position_spread_m) * 100).toFixed(1)} cm` }}</small></div>
            </div>
            <div :class="{ ok: originStatus.heading_stable }">
              <span>{{ originStatus.heading_stable ? '✓' : '…' }}</span>
              <div><strong>航向质量</strong><small>σ {{ Number(originStatus.heading_std_deg || 0).toFixed(2) }}° · 延迟 {{ Number(originStatus.age_seconds || 0).toFixed(1) }}s</small></div>
            </div>
          </div>
          <p>{{ originStatusMessage }}</p>
          <div v-if="originLocked" class="origin-coordinate">
            <span>LAT {{ Number(originStatus.origin?.origin_latitude || originStatus.latitude).toFixed(10) }}</span>
            <span>LON {{ Number(originStatus.origin?.origin_longitude || originStatus.longitude).toFixed(10) }}</span>
            <span>航向 {{ Number(originStatus.heading_deg || 0).toFixed(2) }}°</span>
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
              <small>记录雷达、IMU、里程计、TF和RTK，便于离线复现漂移</small>
            </span>
          </label>
        </div>

        <div class="mapping-actions">
          <button v-if="isOutdoorMapping" class="btn btn-origin" :disabled="mappingBusy || !selectedRobot || !canLockOrigin" @click="handleLockOrigin">
            {{ ['waiting_quality', 'quality_holding'].includes(originState) ? '原点锁定中…' : (originLocked ? 'ENU 原点已锁定' : '锁定 ENU 原点') }}
          </button>
          <button class="btn btn-primary" :disabled="mappingBusy || !selectedRobot || !canStartSlam" @click="handleStartMapping">
            {{ mappingBusy ? '正在下发...' : '启动并检查' }}
          </button>
          <button class="btn btn-confirm" :disabled="mappingBusy || !selectedRobot || !canBeginMapping" @click="handleBeginMapping">
            {{ isOutdoorMapping ? '确认航向稳定，开始建图' : '确认检查通过，开始建图' }}
          </button>
          <button class="btn btn-primary" :disabled="mappingBusy || !selectedRobot || !canSaveMapping" @click="handleSaveMapping">
            {{ slamDiverged ? '停止并生成救援地图' : '停止并保存地图' }}
          </button>
          <button class="btn btn-sm" :disabled="mappingBusy || !selectedRobot || !canCancelMapping" @click="handleCancelMapping">
            取消建图
          </button>
          <button class="btn btn-sm" :disabled="mappingBusy || !selectedRobot" @click="refreshMappingStatus">
            刷新状态
          </button>
        </div>

        <div v-if="mappingStatus?.error_message" class="mapping-error">
          错误: {{ mappingStatus.error_message }}
        </div>
        <div
          v-else-if="mappingStatus?.last_command_error_message && ['failed', 'rejected', 'timed_out'].includes(commandStatus)"
          class="mapping-error"
        >
          上次命令失败: {{ mappingStatus.last_command_error_message }}
        </div>

        <div class="mapping-guide">
          <strong>操作步骤：</strong>
          <template v-if="isOutdoorMapping">
            <span>1. 将机器人开到预选开阔锚点，点击“锁定 ENU 原点”，随后保持静止</span>
            <span>2. 等待位置 FIX、双天线航向 FIX、位置波动小于 2 cm 连续满足 60 秒</span>
            <span>3. 原点锁定后点击“启动 SLAM 并检查航向”，IMU 使用 NX 板雷达内置 IMU 完成预热</span>
            <span>4. 原地小范围转动，确认航向稳定后点击“确认航向稳定，开始建图”</span>
            <span>5. 此时才正式采集数据和关键帧；走场结束后停止并保存地图</span>
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
.origin-countdown { font-variant-numeric: tabular-nums; font-size: 1.15rem; font-weight: 800; color: #8a5a00; }
.origin-quality-card progress { width: 100%; height: 0.55rem; accent-color: #e4a11b; }
.origin-locked progress { accent-color: #198754; }

.origin-quality-grid {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 0.6rem;
}

.origin-quality-grid > div {
  display: flex;
  gap: 0.55rem;
  min-width: 0;
  padding: 0.65rem;
  border: 1px solid #e6e9ef;
  border-radius: 8px;
  background: rgba(255,255,255,0.82);
}

.origin-quality-grid > div > span { color: #98a2b3; font-weight: 800; }
.origin-quality-grid > div.ok > span { color: #198754; }
.origin-quality-grid div div { display: grid; min-width: 0; gap: 0.15rem; }
.origin-quality-grid strong { font-size: 0.78rem; }
.origin-quality-grid small { overflow: hidden; color: #667085; font-size: 0.7rem; text-overflow: ellipsis; white-space: nowrap; }
.origin-quality-card p { margin: 0; color: #5f4b20; font-size: 0.8rem; }
.origin-coordinate { padding-top: 0.65rem; border-top: 1px dashed #9ed6b5; color: #176b3a; font: 600 0.75rem ui-monospace, SFMono-Regular, Menlo, monospace; }

.mapping-actions .btn-origin { border-color: #d99a19; color: #7a5100; background: #fff8e6; }
.mapping-actions .btn-confirm { border-color: #198754; color: #fff; background: #198754; }
.mapping-actions .btn-confirm:disabled { border-color: #b9c4cf; background: #b9c4cf; }

@media (max-width: 900px) {
  .origin-quality-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
}

@media (max-width: 620px) {
  .mapping-mode-switch, .origin-quality-grid { grid-template-columns: 1fr; }
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
  .cleaner-header, .cleaner-toolbar, .cleaner-footer { padding: 0.6rem; }
  .cleaner-toolbar { gap: 0.5rem; }
}
</style>
