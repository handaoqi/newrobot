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
import { executionActions, powerLabel } from '../services/executionState'

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
const progress = computed(() => {
  if (!execution.value?.total_waypoints) return 0
  return Math.round(execution.value.completed_waypoints / execution.value.total_waypoints * 100)
})
const isActive = computed(() => ['created', 'dispatching', 'accepted', 'running', 'pausing', 'paused', 'resuming', 'cancelling', 'interrupted'].includes(execution.value?.state))

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
  error.value = ''
  try {
    execution.value = await sendTaskExecutionAction(execution.value.id, action)
    await refresh()
  } catch (exc) {
    error.value = exc.message
  }
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
  const status = robotStatus.value?.status
  if (!status || status.x === null || status.y === null) return null
  return { x: Number(status.x), y: Number(status.y), yaw: Number(status.yaw || 0) }
}

function robotDisplayPosition() {
  return displayPosition(robotPoint())
}

function robotHeadingStyle() {
  return { transform: `translate(-50%, -50%) rotate(${Number(robotPoint()?.yaw || 0)}rad)` }
}

function waypointClass(index) {
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
          <span class="panel-badge">{{ execution.state }}</span>
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
        <div class="action-row">
          <button class="ghost-btn" :disabled="!actions.pause" @click="act('pause')">暂停</button>
          <button class="primary-btn" :disabled="!actions.resume" @click="act('resume')">继续</button>
          <button class="danger-btn" :disabled="!actions.cancel" @click="act('cancel')">终止</button>
        </div>
        <p v-if="error" class="form-error">{{ error }}</p>
      </div>

      <div class="panel detail-panel">
        <h3>航点状态</h3>
        <div class="execution-waypoints">
          <div v-for="(point, index) in waypoints" :key="point.waypoint_id || index" class="execution-waypoint" :class="waypointClass(index)">
            <span>{{ index + 1 }}</span>
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
        </article>
      </div>
    </section>

    <section class="execution-map-panel">
      <div class="execution-map-head">
        <div>
          <h3>{{ mapData?.name || execution.map_name || '执行地图' }}</h3>
          <p>{{ robotStatus?.robot_code || execution.robot_code }} · 轨迹 {{ trajectory.length }} 点</p>
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
              {{ index + 1 }}
            </div>
            <div v-if="currentTarget" class="execution-marker target" :style="displayPosition(currentTarget)">目标</div>
            <div v-if="robotDisplayPosition()" class="execution-robot" :style="robotDisplayPosition()">
              <span :style="robotHeadingStyle()"></span>
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
  gap: 20px;
  height: calc(100vh - 136px);
  min-height: 680px;
}

.execution-sidebar {
  display: grid;
  gap: 16px;
  align-content: start;
  overflow: auto;
}

.execution-map-panel {
  min-width: 0;
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

.execution-waypoints {
  display: grid;
  gap: 8px;
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

.execution-waypoint strong,
.execution-waypoint small {
  display: block;
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
}
</style>
