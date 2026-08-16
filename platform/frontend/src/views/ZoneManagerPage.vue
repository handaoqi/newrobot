<script setup>
import { onMounted, ref } from 'vue'
import { fetchMaps, fetchZones, createZone, updateZone, deleteZone } from '../services/api'

const maps = ref([])
const zones = ref([])
const selectedMap = ref(null)
const showZoneDialog = ref(false)
const editingZone = ref(null)

const zoneForm = ref({
  name: '',
  map_data: null,
  zone_type: 'forbidden',
  polygon: [],
  description: '',
  active: true,
})

const zoneTypes = [
  { value: 'forbidden', label: '禁入区', color: '#d32f2f' },
  { value: 'warning', label: '警告区', color: '#ff9800' },
  { value: 'restricted', label: '限行区', color: '#1976d2' },
]

onMounted(async () => {
  await loadData()
})

async function loadData() {
  try {
    maps.value = await fetchMaps()
    zones.value = await fetchZones()
  } catch (error) {
    console.error('加载数据失败:', error)
  }
}

function getZoneTypeInfo(type) {
  return zoneTypes.find(t => t.value === type) || zoneTypes[0]
}

async function handleOpenDialog(zone = null) {
  if (zone) {
    editingZone.value = zone
    zoneForm.value = {
      name: zone.name,
      map_data: zone.map_data,
      zone_type: zone.zone_type,
      polygon: [...zone.polygon],
      description: zone.description,
      active: zone.active,
    }
  } else {
    editingZone.value = null
    zoneForm.value = {
      name: '',
      map_data: selectedMap.value?.id || null,
      zone_type: 'forbidden',
      polygon: [],
      description: '',
      active: true,
    }
  }
  showZoneDialog.value = true
}

async function handleSaveZone() {
  if (!zoneForm.value.name || !zoneForm.value.map_data) {
    alert('请填写禁区名称并选择地图')
    return
  }

  const payload = {
    name: zoneForm.value.name,
    map_data: zoneForm.value.map_data,
    zone_type: zoneForm.value.zone_type,
    polygon: zoneForm.value.polygon,
    description: zoneForm.value.description,
    active: zoneForm.value.active,
  }

  try {
    if (editingZone.value) {
      await updateZone(editingZone.value.id, payload)
    } else {
      await createZone(payload)
    }
    showZoneDialog.value = false
    await loadData()
  } catch (error) {
    console.error('保存失败:', error)
    alert('保存失败')
  }
}

async function handleDeleteZone(zone) {
  if (!confirm(`确定要删除禁区 "${zone.name}" 吗？`)) return
  try {
    await deleteZone(zone.id)
    await loadData()
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
        <h2>禁区管理</h2>
        <button class="btn btn-primary" @click="handleOpenDialog()">
          添加禁区
        </button>
      </div>

      <!-- 筛选区域 -->
      <div class="filter-section">
        <label>选择地图:</label>
        <select v-model="selectedMap">
          <option :value="null">全部地图</option>
          <option v-for="map in maps" :key="map.id" :value="map">
            {{ map.name }}
          </option>
        </select>
      </div>

      <!-- 禁区列表 -->
      <div class="zone-list">
        <article v-for="zone in zones" :key="zone.id" class="zone-card">
          <div class="zone-header">
            <div class="zone-type-badge" :style="{ background: getZoneTypeInfo(zone.zone_type).color }">
              {{ getZoneTypeInfo(zone.zone_type).label }}
            </div>
            <h3>{{ zone.name }}</h3>
          </div>
          <div class="zone-info">
            <p><strong>关联地图:</strong> {{ zone.map_name || '未关联' }}</p>
            <p><strong>顶点数:</strong> {{ zone.polygon.length }} 个</p>
            <p v-if="zone.description"><strong>描述:</strong> {{ zone.description }}</p>
          </div>
          <div class="zone-status">
            <span :class="['badge', zone.active ? 'badge-success' : 'badge-warning']">
              {{ zone.active ? '启用' : '禁用' }}
            </span>
          </div>
          <div class="zone-actions">
            <button class="btn btn-sm" @click="handleOpenDialog(zone)">编辑</button>
            <button class="btn btn-sm btn-danger" @click="handleDeleteZone(zone)">删除</button>
          </div>
        </article>

        <div v-if="zones.length === 0" class="empty-state">
          暂无禁区，点击"添加禁区"创建
        </div>
      </div>
    </section>

    <!-- 添加/编辑对话框 -->
    <div v-if="showZoneDialog" class="modal-overlay" @click.self="showZoneDialog = false">
      <div class="modal">
        <div class="modal-header">
          <h3>{{ editingZone ? '编辑禁区' : '添加禁区' }}</h3>
          <button class="btn-close" @click="showZoneDialog = false">×</button>
        </div>
        <div class="modal-body">
          <div class="form-group">
            <label>禁区名称</label>
            <input v-model="zoneForm.name" type="text" placeholder="输入禁区名称" />
          </div>
          <div class="form-group">
            <label>关联地图</label>
            <select v-model="zoneForm.map_data">
              <option :value="null">请选择地图</option>
              <option v-for="map in maps" :key="map.id" :value="map.id">
                {{ map.name }}
              </option>
            </select>
          </div>
          <div class="form-group">
            <label>禁区类型</label>
            <select v-model="zoneForm.zone_type">
              <option v-for="type in zoneTypes" :key="type.value" :value="type.value">
                {{ type.label }}
              </option>
            </select>
          </div>
          <div class="form-group">
            <label>多边形顶点（JSON格式）</label>
            <textarea v-model="zoneForm.polygon" rows="4" placeholder="[[x1,y1],[x2,y2],...]"></textarea>
          </div>
          <div class="form-group">
            <label>描述</label>
            <textarea v-model="zoneForm.description" rows="2" placeholder="输入禁区描述"></textarea>
          </div>
          <div class="form-group">
            <label>
              <input v-model="zoneForm.active" type="checkbox" />
              启用禁区
            </label>
          </div>
        </div>
        <div class="modal-footer">
          <button class="btn" @click="showZoneDialog = false">取消</button>
          <button class="btn btn-primary" @click="handleSaveZone">
            {{ editingZone ? '保存修改' : '添加禁区' }}
          </button>
        </div>
      </div>
    </div>
  </section>
</template>

<style scoped>
.filter-section {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  margin-bottom: 1rem;
}

.filter-section label {
  font-weight: 500;
}

.filter-section select {
  padding: 0.5rem;
  border: 1px solid #e0e0e0;
  border-radius: 4px;
}

.zone-list {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
  gap: 1rem;
}

.zone-card {
  border: 1px solid #e0e0e0;
  border-radius: 8px;
  padding: 1rem;
  background: #fff;
}

.zone-header {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  margin-bottom: 0.75rem;
}

.zone-type-badge {
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  color: #fff;
  font-size: 0.75rem;
  font-weight: 500;
}

.zone-header h3 {
  margin: 0;
  font-size: 1rem;
}

.zone-info {
  margin-bottom: 0.75rem;
}

.zone-info p {
  margin: 0.25rem 0;
  font-size: 0.875rem;
  color: #666;
}

.zone-status {
  margin-bottom: 0.75rem;
}

.badge {
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  font-size: 0.75rem;
}

.badge-success {
  background: #e8f5e9;
  color: #2e7d32;
}

.badge-warning {
  background: #fff3e0;
  color: #e65100;
}

.zone-actions {
  display: flex;
  gap: 0.5rem;
}

.empty-state {
  grid-column: 1 / -1;
  text-align: center;
  padding: 3rem;
  color: #999;
}

.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
}

.modal {
  background: #fff;
  border-radius: 8px;
  width: 90%;
  max-width: 500px;
  max-height: 90vh;
  overflow-y: auto;
}

.modal-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
  border-bottom: 1px solid #e0e0e0;
}

.modal-header h3 {
  margin: 0;
}

.btn-close {
  background: none;
  border: none;
  font-size: 1.5rem;
  cursor: pointer;
}

.modal-body {
  padding: 1rem;
}

.form-group {
  margin-bottom: 1rem;
}

.form-group label {
  display: block;
  margin-bottom: 0.5rem;
  font-weight: 500;
}

.form-group input,
.form-group select,
.form-group textarea {
  width: 100%;
  padding: 0.5rem;
  border: 1px solid #e0e0e0;
  border-radius: 4px;
  font-size: 0.875rem;
}

.modal-footer {
  display: flex;
  justify-content: flex-end;
  gap: 0.5rem;
  padding: 1rem;
  border-top: 1px solid #e0e0e0;
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
</style>