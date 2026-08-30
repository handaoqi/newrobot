<script setup>
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'
import { useRoute } from 'vue-router'

import {
  fetchMapDetail,
  fetchRobotStatus,
  fetchTaskExecution,
  fetchTaskTrajectory,
  sendTaskExecutionAction,
} from '../services/api'
import { API_BASE } from '../services/api'
import RobotDogIcon from '../components/RobotDogIcon.vue'
import { executionActions, powerLabel } from '../services/executionState'
import {
  buildLocalizationLossMarkers,
  currentRobotMapPose,
  localizationRecoveryLabel,
} from '../services/taskMapState'

const route = useRoute()
const execution = ref(null)
const mapData = ref(null)
const robotStatus = ref(null)
const trajectory = ref([])
const error = ref('')
const mapImageRef = ref(null)
const imageReadyTick = ref(0)
let timer

const actions = computed(() => executionActions(execution.value?.state))
const waypoints = computed(() => execution.value?.route_snapshot?.waypoints || [])
const currentIndex = computed(() => execution.value?.current_waypoint_index ?? 0)
const currentTarget = computed(() => waypoints.value[currentIndex.value] || waypoints.value[0] || null)
const latestCommand = computed(() => {
  const commands = execution.value?.commands || []
  return commands[commands.length - 1] || null
})
const failureInfo = computed(() => buildFailureInfo())
const failedWaypointIndexes = computed(() => failureInfo.value.waypointIndexes)
const localizationLossMarkers = computed(() => buildLocalizationLossMarkers(
  execution.value,
  trajectory.value,
  mapData.value?.id,
))
const progress = computed(() => {
  if (!execution.value?.total_waypoints) return 0
  return Math.round(execution.value.completed_waypoints / execution.value.total_waypoints * 100)
})
const isActive = computed(() => ['created', 'dispatching', 'accepted', 'running', 'pausing', 'paused', 'resuming', 'cancelling', 'interrupted'].includes(execution.value?.state))
const rosbagStatus = computed(() => {
  const events = [...(execution.value?.events || [])].reverse()
  for (const event of events) {
    const status = event?.payload?.rosbag
    if (status && typeof status === 'object') return status
  }
  return null
})

function fullUrl(relativeUrl) {
  if (!relativeUrl) return ''
  if (relativeUrl.startsWith('http')) return relativeUrl
  return `${API_BASE.replace('/api', '')}${relativeUrl}`
}

async function refresh() {
  const data = await fetchTaskExecution(route.params.executionId)
  execution.value = data
  const [status, track] = await Promise.all([
    fetchRobotStatus(data.robot),
    fetchTaskTrajectory(data.id),
  ])
  robotStatus.value = status
  trajectory.value = track.points || []
  if ((!mapData.value || mapData.value.id !== data.map_data) && data.map_data) {
    mapData.value = await fetchMapDetail(data.map_data)
  }
  refreshImageGeometry()
}

async function act(action) {
  if (action === 'force-exit' && !window.confirm('强制退出会停止当前导航，并清理该机器人的全部未结束任务。确认继续？')) return
  error.value = ''
  try {
    execution.value = await sendTaskExecutionAction(execution.value.id, action)
    await refresh()
  } catch (exc) {
    error.value = exc.message
  }
}

function controlTask() {
  if (actions.value.control.action) act(actions.value.control.action)
}

function refreshImageGeometry() {
  imageReadyTick.value += 1
}

function mapGeometry() {
  imageReadyTick.value
  const map = mapData.value
  if (!map) return null
  const image = mapImageRef.value
  const mapWidth = Number(map.width || image?.naturalWidth || 0)
  const mapHeight = Number(map.height || image?.naturalHeight || 0)
  const resolution = Number(map.resolution || 0.05)
  const origin = Array.isArray(map.origin) && map.origin.length >= 2
    ? [Number(map.origin[0]), Number(map.origin[1]), Number(map.origin[2] || 0)]
    : [0, 0, 0]
  const rect = image?.getBoundingClientRect() || { width: mapWidth, height: mapHeight }
  if (!mapWidth || !mapHeight || !resolution || !rect.width || !rect.height) return null
  return { mapWidth, mapHeight, resolution, origin, rect }
}

function mapPointToImagePoint(x, y, geometry = mapGeometry()) {
  if (!geometry) return null
  return {
    imageX: (Number(x) - geometry.origin[0]) / geometry.resolution,
    imageY: geometry.mapHeight - ((Number(y) - geometry.origin[1]) / geometry.resolution),
  }
}

function displayPosition(point) {
  const geometry = mapGeometry()
  if (!geometry || !point) return null
  const imagePoint = mapPointToImagePoint(point.x, point.y, geometry)
  if (!imagePoint) return null
  return {
    left: `${imagePoint.imageX * (geometry.rect.width / geometry.mapWidth)}px`,
    top: `${imagePoint.imageY * (geometry.rect.height / geometry.mapHeight)}px`,
  }
}

function polylinePoints(points) {
  return points
    .map(point => displayPosition(point))
    .filter(Boolean)
    .map(pos => `${Number.parseFloat(pos.left)},${Number.parseFloat(pos.top)}`)
    .join(' ')
}

function robotPoint() {
  return currentRobotMapPose(robotStatus.value, mapData.value?.id, localizationLossMarkers.value)
}

function robotDisplayPosition() {
  return displayPosition(robotPoint())
}

function robotHeadingStyle() {
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - Number(robotPoint()?.yaw || 0)}rad)` }
}

function robotMarkerTitle() {
  const point = robotPoint()
  if (!point) return '暂无定位'
  return `${point.trusted ? '机器狗当前定位' : '机器狗最新上报位置（定位不可信）'}\nx=${Number(point.x).toFixed(2)}, y=${Number(point.y).toFixed(2)}, yaw=${Number(point.yaw || 0).toFixed(3)}`
}

function lossHeadingStyle(point) {
  return { transform: `translate(-50%, -50%) rotate(${Math.PI / 2 - Number(point?.yaw || 0)}rad)` }
}

function lossMarkerTitle(point) {
  const time = point.occurredAt ? new Date(point.occurredAt).toLocaleString('zh-CN', { hour12: false }) : '—'
  const score = Number(point.quality?.matching_error)
  const scoreText = Number.isFinite(score) ? score.toFixed(3) : '—'
  const target = point.waypoint?.map_point_number || (Number.isFinite(Number(point.waypointIndex)) ? Number(point.waypointIndex) + 1 : '—')
  return `第 ${point.sequence} 次定位丢失\n最后可信位置 x=${Number(point.x).toFixed(2)}, y=${Number(point.y).toFixed(2)}\n航向 ${Number(point.yaw || 0).toFixed(3)} rad\n目标 ${target}号点 · NDT ${scoreText}\n${localizationRecoveryLabel(point.recoveryState)}\n${time}`
}

function lossMarkerTime(point) {
  if (!point.occurredAt) return '—'
  return new Date(point.occurredAt).toLocaleTimeString('zh-CN', { hour12: false })
}

function sampleIsStale(value, thresholdMs = 10000) {
  if (!value) return false
  const sampledTime = new Date(value).getTime()
  if (Number.isNaN(sampledTime)) return false
  return Date.now() - sampledTime > thresholdMs
}

function ndtQualityValid(quality) {
  if (!quality) return false
  const error = Number(quality.matching_error)
  const inlier = Number(quality.inlier_fraction)
  const translation = Number(quality.relative_translation_m)
  return Number.isFinite(error)
    && error < 0.5
    && (!Number.isFinite(inlier) || inlier >= 0.05)
    && (!Number.isFinite(translation) || translation < 20)
}

function commandResult(command = latestCommand.value) {
  const raw = command?.result
  if (!raw) return {}
  if (typeof raw === 'object') return raw
  try {
    return JSON.parse(raw)
  } catch {
    return {}
  }
}

function parseWaypointIndexes(text = '') {
  const match = String(text).match(/\[([^\]]+)\]/)
  if (!match) return []
  return match[1]
    .split(',')
    .map(item => Number.parseInt(item.trim(), 10))
    .filter(Number.isFinite)
}

function lastTrajectoryPoint() {
  return trajectory.value.length ? trajectory.value[trajectory.value.length - 1] : null
}

function distanceText(pointA, pointB) {
  if (!pointA || !pointB) return '—'
  const distance = Math.hypot(Number(pointA.x) - Number(pointB.x), Number(pointA.y) - Number(pointB.y))
  return Number.isFinite(distance) ? `${distance.toFixed(2)} m` : '—'
}

function buildFailureInfo() {
  const command = latestCommand.value
  const code = command?.error_code || ''
  const message = command?.error_message || ''
  const result = commandResult(command)
  const indexes = code === 'NAVIGATION_MISSED_WAYPOINTS'
    ? parseWaypointIndexes(message)
    : []

  if (['pausing', 'paused'].includes(execution.value?.state)) {
    return {
      severity: 'idle',
      title: '任务暂停中',
      summary: '当前导航已暂停，点击“继续”可从当前航点恢复执行。',
      detail: '',
      waypointIndexes: [],
      suggestion: '',
    }
  }

  if (execution.value?.state === 'cancelled') {
    return {
      severity: 'idle',
      title: '任务结束',
      summary: '任务已退出，机器人任务状态已清理。',
      detail: '',
      waypointIndexes: [],
      suggestion: '',
    }
  }

  if (['failed', 'timed_out', 'rejected'].includes(execution.value?.state)) {
    return {
      severity: 'bad',
      title: '任务结束',
      summary: execution.value.failure_code || '任务未能继续执行。',
      detail: execution.value.failure_message || '',
      waypointIndexes: [],
      suggestion: '可查看命令生命周期确认结束原因，清理状态后重新执行任务。',
    }
  }

  if (code === 'NAVIGATION_MISSED_WAYPOINTS') {
    return {
      severity: 'bad',
      title: '有航点未到达',
      summary: `${indexes.map(index => `第 ${index + 1} 个点`).join('、') || '部分航点'} 没有完成。通常是目标点在障碍区、膨胀区，或到目标点的路径被实时障碍堵住。`,
      detail: message || 'Nav2 reported missed waypoints',
      waypointIndexes: indexes,
      suggestion: '检查失败点是否落在可通行区域，观察地图上失败点、机器人最终位置和绿色轨迹之间的距离。',
    }
  }

  if (code === 'FINAL_POSE_OUT_OF_TOLERANCE') {
    return {
      severity: 'bad',
      title: '最终位置未到最后点',
      summary: '导航返回结束后，机器人最终位置仍离最后一个点过远。',
      detail: message,
      waypointIndexes: waypoints.value.length ? [waypoints.value.length - 1] : [],
      suggestion: '保留这个报错，不应简单放大容差。应继续查 Nav2 到点阈值、控制器是否提前判到点，以及最终点附近是否被障碍物影响。',
    }
  }

  if (code) {
    return {
      severity: 'warn',
      title: '命令执行失败',
      summary: code,
      detail: message || result.final_task_state || '',
      waypointIndexes: [],
      suggestion: '查看命令生命周期和地图上的最终位置，确认是平台命令失败还是导航栈失败。',
    }
  }

  if (execution.value?.state === 'completed') {
    return {
      severity: 'ok',
      title: '任务完成',
      summary: '所有航点已完成。',
      detail: '',
      waypointIndexes: [],
      suggestion: '',
    }
  }

  return {
    severity: 'idle',
    title: '任务执行中',
    summary: '地图会持续刷新机器人位置、目标点和轨迹。',
    detail: '',
    waypointIndexes: [],
    suggestion: '',
  }
}

function waypointClass(index) {
  if (failedWaypointIndexes.value.includes(index)) return 'failed'
  if (index < currentIndex.value) return 'done'
  if (index === currentIndex.value && isActive.value) return 'current'
  return ''
}

function statusText() {
  const status = robotStatus.value?.status
  if (!status) return '暂无定位'
  return `定位 ${status.localization_status || 'unknown'} · Nav2 ${status.nav_ready ? 'ready' : 'not ready'}`
}

function targetText() {
  if (!currentTarget.value) return '暂无目标点'
  return `${currentTarget.value.name || `点${currentIndex.value + 1}`} (${Number(currentTarget.value.x).toFixed(2)}, ${Number(currentTarget.value.y).toFixed(2)})`
}

function formatBytes(value) {
  const bytes = Number(value || 0)
  if (!Number.isFinite(bytes) || bytes <= 0) return '0 B'
  const units = ['B', 'KB', 'MB', 'GB']
  const unitIndex = Math.min(Math.floor(Math.log(bytes) / Math.log(1024)), units.length - 1)
  return `${(bytes / (1024 ** unitIndex)).toFixed(unitIndex ? 1 : 0)} ${units[unitIndex]}`
}

function rosbagDuration() {
  const status = rosbagStatus.value
  if (!status) return 0
  if (status.running && status.started_at_unix) {
    return Math.max(0, Math.floor(Date.now() / 1000) - Number(status.started_at_unix))
  }
  return Math.max(0, Number(status.duration_seconds || 0))
}

function durationText(seconds) {
  const value = Math.max(0, Number(seconds || 0))
  return `${Math.floor(value / 60)}分${Math.floor(value % 60)}秒`
}

function localizationSourceLabel(source) {
  const labels = {
    lio_imu: 'LIO + IMU',
    ndt_imu: 'NDT + IMU',
    rtk_imu: 'RTK + IMU',
    imu_odom_bridge: 'IMU + 里程计桥接',
    unavailable: '无可用定位源',
  }
  return labels[source] || source || '等待决策'
}

function localizationDecisionBasis(quality = {}) {
  const decision = quality.decision || {}
  const source = decision.active_source || ''
  const preferred = String(decision.preferred_source || 'ndt').toLowerCase()
  const correctionPolicy = String(decision.correction_policy || preferred).toUpperCase()
  const rtkUsable = decision.rtk_usable === true
  const rtkQuality = decision.rtk_quality || '无数据'
  const decisionScore = Number(decision.ndt_score)
  const qualityScore = Number(quality.matching_error)
  const score = Number.isFinite(decisionScore) ? decisionScore : qualityScore
  const ndtHealthy = typeof decision.ndt_healthy === 'boolean'
    ? decision.ndt_healthy
    : Number.isFinite(score) && score < 0.5 && quality.has_converged !== false
  const ndtDetail = Number.isFinite(score)
    ? `NDT${ndtHealthy ? '健康' : '不健康'}（分数 ${score.toFixed(3)}）`
    : 'NDT质量未上报'
  const rtkDetail = `RTK ${rtkQuality}${rtkUsable ? '，可用' : '，不可用'}`

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
  return source ? 'warn' : 'idle'
}

function localizationDebugItems() {
  const status = robotStatus.value?.status || {}
  const quality = status.localization_quality || {}
  const decision = quality.decision || {}
  const mapMatch = mapData.value?.id && status.map_id
    ? String(mapData.value.id) === String(status.map_id)
    : true
  return [
    ['地图一致', mapMatch ? '是' : `否：页面 ${mapData.value?.id || '—'} / 机器人 ${status.map_id || '—'}`, mapMatch ? 'ok' : 'bad'],
    ['定位状态', status.localization_status || 'unknown', status.localization_status === 'normal' ? 'ok' : 'bad'],
    ['当前定位决策', localizationSourceLabel(decision.active_source), localizationDecisionTone(decision)],
    ['当前决策依据', localizationDecisionBasis(quality), localizationDecisionTone(decision)],
    ['绝对定位确认', decision.absolute_stable ? `稳定（${Number(decision.absolute_stable_samples || 0)}帧）` : `等待（${Number(decision.absolute_stable_samples || 0)}帧）`, decision.absolute_stable ? 'ok' : 'warn'],
    ['航点定位校正方式', String(decision.correction_policy || decision.preferred_source || 'ndt').toUpperCase(), 'idle'],
    ['校正源就绪', decision.policy_source_ready === true ? '是' : '否', decision.policy_source_ready === true ? 'ok' : 'warn'],
    ['本次校正候选', String(decision.correction_candidate_source || 'none').toUpperCase(), 'idle'],
    ['RTK质量', decision.rtk_quality || '—', decision.rtk_usable ? 'ok' : 'warn'],
    ['RTK地图坐标', Number.isFinite(Number(decision.rtk_x)) ? `${Number(decision.rtk_x).toFixed(2)}, ${Number(decision.rtk_y).toFixed(2)}` : '—', decision.rtk_usable ? 'ok' : 'warn'],
    ['RTK航向', Number.isFinite(Number(decision.rtk_yaw)) ? `${(Number(decision.rtk_yaw) * 180 / Math.PI).toFixed(1)}°` : '—', decision.rtk_usable ? 'ok' : 'warn'],
    ['桥接余量', decision.active_source === 'imu_odom_bridge' ? `${Number(decision.bridge_distance_m || 0).toFixed(2)}m / ${Number(decision.bridge_elapsed_s || 0).toFixed(1)}s` : '—', decision.active_source === 'imu_odom_bridge' ? 'warn' : 'idle'],
    ['桥接拒绝原因', decision.bridge_rejection_reason || '—', decision.bridge_rejection_reason ? 'bad' : 'idle'],
    ['NDT分数', Number.isFinite(Number(quality.matching_error)) ? Number(quality.matching_error).toFixed(3) : '—', ndtQualityValid(quality) ? 'ok' : 'warn'],
    ['内点率', Number.isFinite(Number(quality.inlier_fraction)) ? Number(quality.inlier_fraction).toFixed(3) : '—', ndtQualityValid(quality) ? 'ok' : 'warn'],
    ['最终点距离', distanceText(lastTrajectoryPoint() || robotPoint(), waypoints.value[waypoints.value.length - 1]), execution.value?.state === 'failed' ? 'warn' : 'idle'],
  ]
}

onMounted(async () => {
  await refresh()
  timer = window.setInterval(refresh, 2000)
  window.addEventListener('resize', refreshImageGeometry)
})
onBeforeUnmount(() => {
  if (timer) window.clearInterval(timer)
  window.removeEventListener('resize', refreshImageGeometry)
})
</script>

<template>
  <section v-if="execution" class="execution-page">
    <section class="execution-sidebar">
      <div class="panel detail-panel">
        <div class="panel-head">
          <div>
            <h3>{{ execution.task_name }}</h3>
            <p>{{ execution.robot_name }} / {{ execution.route_name }}</p>
          </div>
          <span class="panel-badge">{{ actions.statusLabel }}</span>
        </div>
        <div class="metrics-grid">
          <div class="metric-card"><strong>{{ currentIndex + 1 }} / {{ execution.total_waypoints || waypoints.length }}</strong><span>当前航点</span></div>
          <div class="metric-card"><strong>{{ progress }}%</strong><span>任务进度</span></div>
          <div class="metric-card"><strong>{{ robotStatus?.connection_status || 'unknown' }}</strong><span>设备连接</span></div>
          <div class="metric-card"><strong>{{ powerLabel(robotStatus?.status) }}</strong><span>真实电量</span></div>
        </div>
        <div class="detail-card">
          <strong>当前目标</strong>
          <p>{{ targetText() }}</p>
          <small>{{ statusText() }}</small>
        </div>
        <div class="failure-card" :class="failureInfo.severity">
          <strong>{{ failureInfo.title }}</strong>
          <p>{{ failureInfo.summary }}</p>
          <small v-if="failureInfo.detail">{{ failureInfo.detail }}</small>
          <small v-if="failureInfo.suggestion">{{ failureInfo.suggestion }}</small>
        </div>
        <div class="debug-list">
          <div v-for="[label, value, state] in localizationDebugItems()" :key="label" class="debug-row" :class="state">
            <span>{{ label }}</span>
            <strong>{{ value }}</strong>
          </div>
        </div>
        <div v-if="rosbagStatus" class="detail-card rosbag-card" :class="{ recording: rosbagStatus.running, failed: rosbagStatus.error }">
          <strong>{{ rosbagStatus.running ? '导航诊断包录制中' : (rosbagStatus.error ? '导航诊断包异常' : '导航诊断包已保存') }}</strong>
          <p>{{ durationText(rosbagDuration()) }} · {{ formatBytes(rosbagStatus.size_bytes) }}</p>
          <small v-if="rosbagStatus.bag_dir" :title="rosbagStatus.bag_dir">{{ rosbagStatus.bag_dir }}</small>
          <small v-if="rosbagStatus.error">{{ rosbagStatus.error }}</small>
        </div>
        <div class="action-row">
          <button class="primary-btn" :disabled="!actions.control.enabled" @click="controlTask">{{ actions.control.label }}</button>
          <button class="danger-btn" :disabled="!actions.forceExit" @click="act('force-exit')">强制退出</button>
        </div>
        <p v-if="error" class="form-error">{{ error }}</p>
      </div>

      <div class="panel detail-panel">
        <h3>航点状态</h3>
        <div class="execution-waypoints">
          <div v-for="(point, index) in waypoints" :key="point.waypoint_id || index" class="execution-waypoint" :class="waypointClass(index)">
            <span>{{ point.map_point_number ?? point.sequence + 1 }}</span>
            <div>
              <strong>{{ point.name || `点${index + 1}` }}</strong>
              <small>x {{ Number(point.x).toFixed(2) }} / y {{ Number(point.y).toFixed(2) }}</small>
            </div>
          </div>
        </div>
      </div>

      <div class="panel detail-panel">
        <h3>命令生命周期</h3>
        <article v-for="command in execution.commands" :key="command.id" class="task-card">
          <strong>{{ command.command_type }}</strong>
          <span>{{ command.status }} · {{ command.error_code || command.ack_reason_code || 'OK' }}</span>
          <small v-if="command.error_message">{{ command.error_message }}</small>
        </article>
      </div>
    </section>

    <section class="execution-map-panel">
      <div class="execution-map-head">
        <div>
          <h3>{{ mapData?.name || execution.map_name || '执行地图' }}</h3>
          <p>{{ robotStatus?.robot_code || execution.robot_code }} · 轨迹 {{ trajectory.length }} 点 · 定位丢失 {{ localizationLossMarkers.length }} 次</p>
        </div>
        <span class="panel-badge">{{ robotPoint() ? `x ${robotPoint().x.toFixed(2)} / y ${robotPoint().y.toFixed(2)}` : '暂无定位' }}</span>
      </div>

      <div class="execution-map-stage">
        <div v-if="!mapData?.thumbnail_url" class="map-placeholder">地图预览不可用</div>
        <div v-else class="execution-map-layer">
          <img ref="mapImageRef" :src="fullUrl(mapData.thumbnail_url)" :alt="mapData.name" @load="refreshImageGeometry" />
          <svg class="execution-lines">
            <polyline v-if="waypoints.length > 1" :points="polylinePoints(waypoints)" fill="none" stroke="#2563eb" stroke-width="3" />
            <polyline v-if="trajectory.length > 1" :points="polylinePoints(trajectory)" fill="none" stroke="#10b981" stroke-width="2" stroke-dasharray="6 5" />
          </svg>
          <div class="execution-markers">
            <div
              v-for="(point, index) in waypoints"
              :key="point.waypoint_id || index"
              class="execution-marker waypoint"
              :class="waypointClass(index)"
              :style="displayPosition(point)"
            >
              {{ point.map_point_number ?? point.sequence + 1 }}
            </div>
            <div v-if="currentTarget" class="execution-marker target" :style="displayPosition(currentTarget)">目标</div>
            <div v-if="lastTrajectoryPoint()" class="execution-marker final" :style="displayPosition(lastTrajectoryPoint())">终点</div>
            <div
              v-for="point in localizationLossMarkers"
              :key="`localization-loss-${point.eventId}`"
              class="localization-loss-marker"
              :style="displayPosition(point)"
              :title="lossMarkerTitle(point)"
            >
              <span class="localization-loss-heading" :style="lossHeadingStyle(point)"></span>
              <small>{{ point.sequence }}</small>
            </div>
            <div v-if="robotDisplayPosition()" class="execution-robot" :class="{ untrusted: !robotPoint()?.trusted }" :style="robotDisplayPosition()" :title="robotMarkerTitle()">
              <RobotDogIcon :size="28" />
              <span :style="robotHeadingStyle()"></span>
              <small>{{ robotPoint()?.trusted ? '机器狗' : '定位不可信' }}</small>
            </div>
          </div>
          <div v-if="localizationLossMarkers.length" class="localization-loss-list">
            <strong>定位丢失位置</strong>
            <div v-for="point in localizationLossMarkers" :key="`localization-loss-detail-${point.eventId}`">
              <i></i>
              <span>#{{ point.sequence }} x {{ Number(point.x).toFixed(2) }} / y {{ Number(point.y).toFixed(2) }}</span>
              <small>yaw {{ Number(point.yaw || 0).toFixed(3) }} rad · {{ lossMarkerTime(point) }} · {{ localizationRecoveryLabel(point.recoveryState) }}</small>
            </div>
          </div>
        </div>
      </div>
    </section>
  </section>
</template>

<style scoped>
.execution-page {
  display: grid;
  grid-template-columns: 360px 1fr;
  align-items: start;
  gap: 20px;
  min-height: calc(100vh - 136px);
}

.execution-sidebar {
  display: grid;
  gap: 16px;
  align-content: start;
  overflow: visible;
}

.execution-map-panel {
  position: sticky;
  top: 24px;
  min-width: 0;
  height: calc(100vh - 136px);
  min-height: 680px;
  display: grid;
  grid-template-rows: auto 1fr;
  gap: 12px;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--panel);
  padding: 16px;
}

.execution-map-head {
  display: flex;
  justify-content: space-between;
  gap: 16px;
  align-items: center;
}

.execution-map-head h3,
.execution-map-head p {
  margin: 0;
}

.execution-map-stage {
  position: relative;
  overflow: auto;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: #f4f6f8;
  padding: 16px;
}

.execution-map-layer {
  position: relative;
  width: min(100%, 1280px);
  min-width: 760px;
  margin: 0 auto;
  background: #fff;
  box-shadow: 0 12px 30px rgba(15, 23, 42, 0.16);
}

.execution-map-layer img {
  display: block;
  width: 100%;
  height: auto;
  user-select: none;
}

.execution-lines,
.execution-markers {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.execution-marker {
  position: absolute;
  transform: translate(-50%, -50%);
  display: inline-flex;
  align-items: center;
  justify-content: center;
  font-weight: 800;
  box-shadow: 0 3px 8px rgba(15, 23, 42, 0.25);
}

.execution-marker.waypoint {
  width: 26px;
  height: 26px;
  border-radius: 50%;
  background: #2563eb;
  color: #fff;
}

.execution-marker.waypoint.done {
  background: #10b981;
}

.execution-marker.waypoint.current {
  background: #f59e0b;
}

.execution-marker.waypoint.failed {
  background: #dc2626;
  outline: 3px solid rgba(220, 38, 38, 0.28);
}

.execution-marker.target {
  min-width: 42px;
  height: 24px;
  padding: 0 8px;
  border-radius: 999px;
  background: #ef4444;
  color: #fff;
  transform: translate(-50%, calc(-100% - 18px));
  font-size: 12px;
}

.execution-marker.final {
  min-width: 42px;
  height: 24px;
  padding: 0 8px;
  border-radius: 999px;
  background: #111827;
  color: #fff;
  transform: translate(-50%, 18px);
  font-size: 12px;
}

.localization-loss-marker {
  position: absolute;
  width: 30px;
  height: 30px;
  transform: translate(-50%, -50%);
  z-index: 9;
  pointer-events: auto;
}

.localization-loss-marker::before {
  content: "";
  position: absolute;
  inset: 6px;
  border-radius: 50%;
  background: #dc2626;
  border: 3px solid #fff;
  box-shadow: 0 0 0 3px rgba(220, 38, 38, 0.3), 0 3px 10px rgba(127, 29, 29, 0.45);
}

.localization-loss-marker small {
  position: absolute;
  left: 50%;
  top: 100%;
  transform: translate(-50%, 2px);
  min-width: 18px;
  padding: 1px 4px;
  border-radius: 3px;
  background: #991b1b;
  color: #fff;
  font-size: 10px;
  line-height: 14px;
  text-align: center;
}

.localization-loss-heading {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 0;
  height: 0;
  border-left: 5px solid transparent;
  border-right: 5px solid transparent;
  border-bottom: 15px solid #7f1d1d;
  transform-origin: 50% 100%;
  z-index: 2;
}

.localization-loss-list {
  position: absolute;
  top: 12px;
  right: 12px;
  z-index: 12;
  display: grid;
  gap: 6px;
  width: min(310px, calc(100% - 24px));
  max-height: 180px;
  overflow: auto;
  padding: 10px;
  border: 1px solid rgba(220, 38, 38, 0.32);
  border-radius: 6px;
  background: rgba(255, 255, 255, 0.94);
  box-shadow: 0 4px 14px rgba(15, 23, 42, 0.18);
}

.localization-loss-list > div {
  display: grid;
  grid-template-columns: 10px 1fr;
  column-gap: 7px;
  font-size: 12px;
}

.localization-loss-list i {
  width: 9px;
  height: 9px;
  margin-top: 4px;
  border-radius: 50%;
  background: #dc2626;
}

.localization-loss-list small {
  grid-column: 2;
  color: #64748b;
}

.execution-robot {
  position: absolute;
  width: 34px;
  height: 34px;
  transform: translate(-50%, -50%);
  z-index: 8;
}

.execution-robot::before {
  content: "";
  position: absolute;
  inset: 4px;
  border-radius: 50%;
  background: #14b8a6;
  border: 3px solid #fff;
  box-shadow: 0 3px 10px rgba(20, 184, 166, 0.42);
}
.execution-robot > .robot-dog-icon { position: absolute; inset: 1px; z-index: 5; }

.execution-robot span {
  position: absolute;
  left: 50%;
  top: 50%;
  width: 0;
  height: 0;
  border-left: 7px solid transparent;
  border-right: 7px solid transparent;
  border-bottom: 19px solid #0f766e;
  transform-origin: 50% 72%;
  z-index: 9;
}

.execution-robot small {
  position: absolute;
  top: 34px;
  left: 50%;
  min-width: max-content;
  padding: 2px 5px;
  border-radius: 3px;
  color: #fff;
  background: #0f766e;
  font-size: 10px;
  transform: translateX(-50%);
}

.execution-robot.untrusted::before {
  border-style: dashed;
  background: #f59e0b;
  box-shadow: 0 0 0 4px rgba(220, 38, 38, .28);
}

.execution-robot.untrusted span { border-bottom-color: #b45309; }
.execution-robot.untrusted small { background: #b45309; }

.execution-waypoints {
  display: grid;
  gap: 8px;
}

.rosbag-card {
  border-left: 3px solid #64748b;
}

.rosbag-card.recording {
  border-left-color: #dc2626;
}

.rosbag-card.failed {
  border-left-color: #d97706;
}

.rosbag-card small {
  display: block;
  overflow-wrap: anywhere;
}

.execution-waypoint {
  display: grid;
  grid-template-columns: 28px 1fr;
  gap: 10px;
  align-items: center;
  padding: 10px;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--table-bg);
}

.execution-waypoint > span {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 28px;
  height: 28px;
  border-radius: 50%;
  background: #2563eb;
  color: #fff;
  font-size: 13px;
  font-weight: 800;
}

.execution-waypoint.done > span {
  background: #10b981;
}

.execution-waypoint.current > span {
  background: #f59e0b;
}

.execution-waypoint.failed {
  border-color: rgba(220, 38, 38, 0.45);
  background: rgba(220, 38, 38, 0.08);
}

.execution-waypoint.failed > span {
  background: #dc2626;
}

.execution-waypoint strong,
.execution-waypoint small {
  display: block;
}

.failure-card {
  display: grid;
  gap: 6px;
  padding: 12px;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--table-bg);
}

.failure-card strong,
.failure-card p,
.failure-card small {
  margin: 0;
}

.failure-card small {
  color: var(--muted);
  line-height: 1.45;
}

.failure-card.bad {
  border-color: rgba(220, 38, 38, 0.45);
  background: rgba(220, 38, 38, 0.08);
}

.failure-card.warn {
  border-color: rgba(245, 158, 11, 0.5);
  background: rgba(245, 158, 11, 0.10);
}

.failure-card.ok {
  border-color: rgba(16, 185, 129, 0.45);
  background: rgba(16, 185, 129, 0.08);
}

.debug-list {
  display: grid;
  gap: 6px;
}

.debug-row {
  display: flex;
  justify-content: space-between;
  gap: 12px;
  padding: 8px 10px;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--table-bg);
  font-size: 13px;
}

.debug-row span {
  color: var(--muted);
}

.debug-row strong {
  text-align: right;
}

.debug-row.bad strong {
  color: #dc2626;
}

.debug-row.warn strong {
  color: #b45309;
}

.task-card small {
  display: block;
  margin-top: 4px;
  color: var(--muted);
  line-height: 1.45;
}

.map-placeholder {
  display: grid;
  min-height: 420px;
  place-items: center;
  color: var(--muted);
}

@media (max-width: 1100px) {
  .execution-page {
    grid-template-columns: 1fr;
    height: auto;
  }

  .execution-map-layer {
    min-width: 680px;
  }

  .execution-map-panel {
    position: static;
    height: auto;
    min-height: 680px;
  }
}

@media (max-width: 680px) {
  .execution-page { gap: 14px; min-height: 0; }
  .execution-sidebar { gap: 12px; }
  .execution-map-panel { padding: 12px; }
  .execution-map-head { align-items: flex-start; flex-direction: column; gap: 6px; }
  .execution-map-stage { min-height: 380px; padding: 10px; }
  .execution-map-layer { min-width: 600px; }
  .localization-loss-list { top: 8px; right: 8px; width: min(280px, calc(100% - 16px)); }
  .debug-row { align-items: flex-start; flex-direction: column; gap: 3px; }
  .debug-row strong { text-align: left; }
}
</style>
