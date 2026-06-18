<script setup>
import { onMounted, ref, computed } from 'vue'
import { fetchMaps, deleteMap, downloadMap, setActiveMap, connectRobot, downloadFromRobot } from '../services/api'
import { API_BASE } from '../services/api'

const maps = ref([])

const getFullUrl = (relativeUrl) => {
  if (!relativeUrl) return null
  if (relativeUrl.startsWith('http')) return relativeUrl
  return `${API_BASE.replace('/api', '')}${relativeUrl}`
}
const loading = ref(false)
const uploading = ref(false)
const showUploadDialog = ref(false)

// 连接机器狗相关
const showRobotDialog = ref(false)
const robotConnecting = ref(false)
const robotConnected = ref(false)
const robotMaps = ref([])
const downloadingMap = ref(null)
const robotForm = ref({
  ip: '192.168.234.234',
  username: 'robot',
  password: ''
})
const connectionError = ref('')

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
})

async function loadMaps() {
  loading.value = true
  try {
    maps.value = await fetchMaps()
  } catch (error) {
    console.error('加载地图失败:', error)
  } finally {
    loading.value = false
  }
}

function handleImageError(event, map) {
  console.error(`地图 ${map.name} 预览加载失败:`, event)
  map.imageError = true
}

function formatSize(bytes) {
  if (!bytes) return '0 B'
  if (bytes < 1024) return bytes + ' B'
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB'
  return (bytes / (1024 * 1024)).toFixed(1) + ' MB'
}

// 连接机器狗
async function handleConnectRobot() {
  robotConnecting.value = true
  connectionError.value = ''
  robotMaps.value = []
  robotConnected.value = false

  try {
    const result = await connectRobot(robotForm.value)
    robotConnected.value = result.connected
    robotMaps.value = result.maps || []
  } catch (error) {
    connectionError.value = error.message
    console.error('连接失败:', error)
  } finally {
    robotConnecting.value = false
  }
}

// 下载地图
async function handleDownloadFromRobot(map) {
  downloadingMap.value = map.name

  try {
    const result = await downloadFromRobot({
      ip: robotForm.value.ip,
      username: robotForm.value.username,
      password: robotForm.value.password,
      map_path: map.path,
      map_name: map.name
    })

    if (result.success) {
      alert(`地图 "${map.name}" 下载成功！`)
      await loadMaps() // 刷新地图列表
    }
  } catch (error) {
    alert(`下载失败: ${error.message}`)
    console.error('下载失败:', error)
  } finally {
    downloadingMap.value = null
  }
}

// 关闭对话框
function closeRobotDialog() {
  showRobotDialog.value = false
  robotConnected.value = false
  robotMaps.value = []
  connectionError.value = ''
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
    await setActiveMap(map.id)
    await loadMaps()
  } catch (error) {
    console.error('设置活动地图失败:', error)
    alert('设置失败')
  }
}

async function handleDelete(map) {
  if (!confirm(`确定要删除地图 "${map.name}" 吗？`)) return
  try {
    await deleteMap(map.id)
    await loadMaps()
  } catch (error) {
    console.error('删除失败:', error)
    alert('删除失败')
  }
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
</script>

<template>
  <section class="page-section">
    <section class="panel detail-panel">
      <div class="panel-header">
        <h2>地图管理</h2>
        <div class="header-actions">
          <button class="btn btn-primary" @click="showRobotDialog = true">
            连接机器狗
          </button>
          <button class="btn btn-primary" @click="showUploadDialog = true">
            上传地图
          </button>
        </div>
      </div>

      <div v-if="loading" class="loading">加载中...</div>

      <div v-else class="map-list">
        <article v-for="map in maps" :key="map.id" class="map-card">
          <div class="map-thumbnail">
            <template v-if="map.thumbnail_url">
              <img 
                v-show="!map.imageError"
                :src="getFullUrl(map.thumbnail_url)" 
                :alt="map.name"
                @error="handleImageError($event, map)"
              />
              <div v-show="map.imageError" class="no-thumbnail">预览加载失败</div>
            </template>
            <div v-else class="no-thumbnail">无预览</div>
          </div>
          <div class="map-info">
            <h3>{{ map.name }}</h3>
            <div class="map-details">
              <span>分辨率: {{ map.resolution }}m/像素</span>
              <span>大小: {{ formatSize(map.file_size) }}</span>
              <span v-if="map.robot_name">机器人: {{ map.robot_name }}</span>
            </div>
            <p v-if="map.description" class="map-description">{{ map.description }}</p>
          </div>
          <div class="map-actions">
            <span v-if="map.active" class="badge badge-success">活动地图</span>
            <button class="btn btn-sm" @click="handleDownload(map)">下载</button>
            <button v-if="!map.active" class="btn btn-sm" @click="handleSetActive(map)">设为活动</button>
            <button class="btn btn-sm btn-danger" @click="handleDelete(map)">删除</button>
          </div>
        </article>

        <div v-if="maps.length === 0" class="empty-state">
          暂无地图，点击"上传地图"添加
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

    <!-- 连接机器狗对话框 -->
    <div v-if="showRobotDialog" class="modal-overlay" @click.self="closeRobotDialog">
      <div class="modal modal-lg">
        <div class="modal-header">
          <h3>连接机器狗</h3>
          <button class="btn-close" @click="closeRobotDialog">×</button>
        </div>
        <div class="modal-body">
          <!-- 连接表单 -->
          <div v-if="!robotConnected" class="connection-form">
            <div class="form-group">
              <label>机器狗IP</label>
              <input v-model="robotForm.ip" type="text" placeholder="192.168.234.234" />
            </div>
            <div class="form-group">
              <label>用户名</label>
              <input v-model="robotForm.username" type="text" placeholder="robot" />
            </div>
            <div class="form-group">
              <label>密码</label>
              <input v-model="robotForm.password" type="password" placeholder="输入密码" />
            </div>
            <div v-if="connectionError" class="error-message">
              {{ connectionError }}
            </div>
          </div>

          <!-- 已连接，显示地图列表 -->
          <div v-else class="robot-maps">
            <div class="success-message">
              已成功连接到机器狗 {{ robotForm.ip }}，找到 {{ robotMaps.length }} 个地图
            </div>
            <div v-if="robotMaps.length > 0" class="map-list-robot">
              <div v-for="map in robotMaps" :key="map.path" class="robot-map-item">
                <div class="robot-map-info">
                  <h4>{{ map.name }}</h4>
                  <div class="robot-map-files">
                    <span v-for="file in map.files" :key="file.name" class="file-tag">
                      {{ file.name }} ({{ file.size }})
                    </span>
                  </div>
                </div>
                <button 
                  class="btn btn-primary btn-sm"
                  @click="handleDownloadFromRobot(map)"
                  :disabled="downloadingMap === map.name"
                >
                  {{ downloadingMap === map.name ? '下载中...' : '下载' }}
                </button>
              </div>
            </div>
            <div v-else class="empty-state">
              未找到任何地图
            </div>
          </div>
        </div>
        <div class="modal-footer">
          <button class="btn" @click="closeRobotDialog">关闭</button>
          <button 
            v-if="!robotConnected" 
            class="btn btn-primary" 
            @click="handleConnectRobot"
            :disabled="robotConnecting || !robotForm.ip || !robotForm.username || !robotForm.password"
          >
            {{ robotConnecting ? '连接中...' : '连接' }}
          </button>
          <button 
            v-else 
            class="btn btn-primary" 
            @click="handleConnectRobot"
          >
            重新连接
          </button>
        </div>
      </div>
    </div>
  </section>
</template>

<style scoped>
.map-list {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
  gap: 1rem;
}

.map-card {
  border: 1px solid #e0e0e0;
  border-radius: 8px;
  padding: 1rem;
  background: #fff;
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.map-thumbnail {
  width: 100%;
  height: 150px;
  background: #f5f5f5;
  border-radius: 4px;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
}

.map-thumbnail img {
  max-width: 100%;
  max-height: 100%;
  object-fit: contain;
}

.no-thumbnail {
  color: #999;
  font-size: 0.875rem;
}

.map-info h3 {
  margin: 0 0 0.5rem 0;
  font-size: 1rem;
}

.map-details {
  display: flex;
  flex-wrap: wrap;
  gap: 0.75rem;
  font-size: 0.875rem;
  color: #666;
}

.map-description {
  margin: 0;
  font-size: 0.875rem;
  color: #999;
}

.map-actions {
  display: flex;
  gap: 0.5rem;
  flex-wrap: wrap;
  align-items: center;
}

.badge {
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  font-size: 0.75rem;
  font-weight: 500;
}

.badge-success {
  background: #e8f5e9;
  color: #2e7d32;
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
  padding: 0;
  width: 2rem;
  height: 2rem;
  display: flex;
  align-items: center;
  justify-content: center;
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

.btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.loading {
  text-align: center;
  padding: 2rem;
  color: #999;
}

.modal-lg {
  max-width: 700px;
}

.header-actions {
  display: flex;
  gap: 0.5rem;
}

.connection-form {
  padding: 0.5rem 0;
}

.error-message {
  padding: 0.75rem;
  background: #ffebee;
  color: #c62828;
  border-radius: 4px;
  margin-top: 1rem;
}

.success-message {
  padding: 0.75rem;
  background: #e8f5e9;
  color: #2e7d32;
  border-radius: 4px;
  margin-bottom: 1rem;
}

.robot-maps {
  padding: 0.5rem 0;
}

.map-list-robot {
  display: flex;
  flex-direction: column;
  gap: 1rem;
}

.robot-map-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
  background: #f5f5f5;
  border-radius: 4px;
}

.robot-map-info h4 {
  margin: 0 0 0.5rem 0;
  font-size: 1rem;
}

.robot-map-files {
  display: flex;
  flex-wrap: wrap;
  gap: 0.5rem;
}

.file-tag {
  padding: 0.25rem 0.5rem;
  background: #e0e0e0;
  border-radius: 4px;
  font-size: 0.75rem;
  color: #666;
}
</style>