<script setup>
import { onMounted, ref, computed } from 'vue'
import { fetchMaps, fetchRoutes, createRoute, updateRoute, deleteRoute } from '../services/api'
import { API_BASE } from '../services/api'

const maps = ref([])

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

const routeForm = ref({
  name: '',
  map_data: null,
  robot: null,
  description: '',
})

onMounted(async () => {
  await loadData()
})

async function loadData() {
  loading.value = true
  try {
    maps.value = await fetchMaps()
    routes.value = await fetchRoutes()
  } catch (error) {
    console.error('加载数据失败:', error)
  } finally {
    loading.value = false
  }
}

function handleMapSelect(map) {
  selectedMap.value = map
  selectedRoute.value = null
  waypoints.value = []
  waypointNames.value = []
}

function handleMapClick(event) {
  if (!selectedMap.value) return

  const rect = event.target.getBoundingClientRect()
  const x = event.clientX - rect.left
  const y = event.clientY - rect.top

  waypoints.value.push([x, y])
  waypointNames.value.push(`点${waypoints.value.length}`)
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
    robot: routeForm.value.robot || 1,
    waypoints: waypoints.value,
    waypoint_names: waypointNames.value,
    description: routeForm.value.description,
  }

  try {
    await createRoute(payload)
    await loadData()
    clearWaypoints()
    routeForm.value = {
      name: '',
      map_data: null,
      robot: null,
      description: '',
    }
    alert('保存成功')
  } catch (error) {
    console.error('保存失败:', error)
    alert('保存失败')
  }
}

async function handleLoadRoute(route) {
  selectedRoute.value = route
  selectedMap.value = maps.value.find(m => m.id === route.map_data)
  waypoints.value = [...route.waypoints]
  waypointNames.value = [...route.waypoint_names]
  routeForm.value.name = route.name
  routeForm.value.description = route.description
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
                <span>{{ waypointNames[index] }}: ({{ point[0].toFixed(1) }}, {{ point[1].toFixed(1) }})</span>
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
        </div>

        <!-- 右侧地图预览区 -->
        <div class="map-preview-area">
          <div v-if="!selectedMap" class="map-placeholder">
            请先选择地图
          </div>
          <div v-else class="map-container">
            <img v-if="selectedMap.thumbnail_url" :src="getFullUrl(selectedMap.thumbnail_url)" alt="地图预览" @click="handleMapClick" />
            <div v-else class="map-placeholder">地图预览不可用</div>

            <!-- 途经点标记 -->
            <div v-if="selectedMap.thumbnail_url" class="waypoint-markers">
              <div v-for="(point, index) in waypoints" :key="index" class="waypoint-marker" :style="{ left: point[0] + 'px', top: point[1] + 'px' }">
                {{ index + 1 }}
              </div>
            </div>

            <!-- 路径连线 -->
            <svg v-if="selectedMap.thumbnail_url && waypoints.length > 1" class="path-lines">
              <polyline :points="waypoints.map(p => p.join(',')).join(' ')" fill="none" stroke="#1976d2" stroke-width="2" />
            </svg>
          </div>

          <div class="map-hint" v-if="selectedMap">
            💡 点击地图添加途经点
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
  align-items: center;
  justify-content: center;
  position: relative;
}

.map-container {
  position: relative;
  max-width: 100%;
  max-height: calc(100% - 40px);
}

.map-container img {
  max-width: 100%;
  max-height: 100%;
  cursor: crosshair;
  display: block;
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

.path-lines {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  pointer-events: none;
}

.map-hint {
  margin-top: 0.5rem;
  font-size: 0.875rem;
  color: #666;
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