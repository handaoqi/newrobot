<script setup>import { onBeforeUnmount, onMounted, ref, computed } from 'vue';
import { fetchMapSummaries, fetchTrackDetail, fetchTrackSummaries, fetchRouteSummaries, deleteTrack } from '../services/api';
const maps = ref([]);
const tracks = ref([]);
const routes = ref([]);
const selectedTrack = ref(null);
const playing = ref(false);
const playbackProgress = ref(0);
const currentPositionIndex = ref(0);
let playbackInterval = null;
let pageController = null;
let trackController = null;
const routeMap = computed(() => {
 const map = {};
 routes.value.forEach(r => {
 map[r.id] = r;
 });
 return map;
});
onMounted(async () => {
 pageController = new AbortController();
 await loadData(pageController.signal);
});
async function loadData(signal) {
 try {
 [maps.value, tracks.value, routes.value] = await Promise.all([
 fetchMapSummaries({ signal }),
 fetchTrackSummaries({ signal }),
 fetchRouteSummaries({ signal }),
 ]);
 }
 catch (error) {
 if (error?.name !== 'AbortError') console.error('加载数据失败:', error);
 }
}
function getMapName(mapId) {
 return maps.value.find(m => m.id === mapId)?.name || '未知地图';
}
function getRouteName(routeId) {
 return routeMap.value[routeId]?.name || '未知路线';
}
function formatDuration(seconds) {
 const mins = Math.floor(seconds / 60);
 const secs = seconds % 60;
 return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
}
function formatDistance(meters) {
 return meters.toFixed(2) + ' 米';
}
async function selectTrack(track) {
 stopPlayback();
 trackController?.abort();
 trackController = new AbortController();
 selectedTrack.value = { ...track, path: null };
 try {
 const detail = await fetchTrackDetail(track.id, { signal: trackController.signal });
 if (selectedTrack.value?.id === track.id) selectedTrack.value = detail;
 } catch (error) {
 if (error?.name !== 'AbortError') {
 console.error('加载轨迹详情失败:', error);
 if (selectedTrack.value?.id === track.id) selectedTrack.value = track;
 }
 }
 playbackProgress.value = 0;
 currentPositionIndex.value = 0;
}

onBeforeUnmount(() => {
 stopPlayback();
 pageController?.abort();
 trackController?.abort();
});
function startPlayback() {
 if (!selectedTrack.value || !Array.isArray(selectedTrack.value.path) || !selectedTrack.value.path.length)
 return;
 playing.value = true;
 const totalPoints = selectedTrack.value.path.length;
 playbackInterval = setInterval(() => {
 if (currentPositionIndex.value < totalPoints - 1) {
 currentPositionIndex.value++;
 playbackProgress.value = (currentPositionIndex.value / (totalPoints - 1)) * 100;
 }
 else {
 stopPlayback();
 }
 }, 200);
}
function stopPlayback() {
 playing.value = false;
 if (playbackInterval) {
 clearInterval(playbackInterval);
 playbackInterval = null;
 }
}
function resetPlayback() {
 stopPlayback();
 currentPositionIndex.value = 0;
 playbackProgress.value = 0;
}
async function handleDeleteTrack(track) {
 if (!confirm(`确定要删除轨迹记录吗？`))
 return;
 try {
 await deleteTrack(track.id);
 await loadData();
 if (selectedTrack.value?.id === track.id) {
 selectedTrack.value = null;
 }
 }
 catch (error) {
 console.error('删除失败:', error);
 alert('删除失败');
 }
}
</script>

<template>
  <section class="page-section">
    <section class="panel detail-panel">
      <div class="panel-header">
        <h2>轨迹记录</h2>
      </div>

      <div class="track-layout">
        <!-- 左侧轨迹列表 -->
        <div class="track-list-panel">
          <h3>轨迹列表</h3>
          <div class="track-list">
            <article
              v-for="track in tracks"
              :key="track.id"
              class="track-card"
              :class="{ active: selectedTrack?.id === track.id }"
              @click="selectTrack(track)"
            >
              <div class="track-header">
                <strong>{{ track.robot_name || track.robot_code }}</strong>
                <small>{{ track.start_time }}</small>
              </div>
              <div class="track-info">
                <span>地图: {{ getMapName(track.map_data) }}</span>
                <span>路线: {{ getRouteName(track.route) }}</span>
              </div>
              <div class="track-stats">
                <span>距离: {{ formatDistance(track.distance) }}</span>
                <span>时长: {{ formatDuration(track.duration) }}</span>
              </div>
              <button class="btn btn-sm btn-danger" @click.stop="handleDeleteTrack(track)">删除</button>
            </article>

            <div v-if="tracks.length === 0" class="empty-state">
              暂无轨迹记录
            </div>
          </div>
        </div>

        <!-- 右侧轨迹回放区 -->
        <div class="track-playback-panel">
          <div v-if="!selectedTrack" class="playback-placeholder">
            请选择一条轨迹查看详情
          </div>

          <div v-else class="playback-content">
            <!-- 轨迹信息 -->
            <div class="playback-info">
              <div class="info-row">
                <span class="label">机器人:</span>
                <span>{{ selectedTrack.robot_name || selectedTrack.robot_code }}</span>
              </div>
              <div class="info-row">
                <span class="label">地图:</span>
                <span>{{ getMapName(selectedTrack.map_data) }}</span>
              </div>
              <div class="info-row">
                <span class="label">路线:</span>
                <span>{{ getRouteName(selectedTrack.route) }}</span>
              </div>
              <div class="info-row">
                <span class="label">开始时间:</span>
                <span>{{ selectedTrack.start_time }}</span>
              </div>
              <div class="info-row">
                <span class="label">结束时间:</span>
                <span>{{ selectedTrack.end_time || '进行中' }}</span>
              </div>
              <div class="info-row">
                <span class="label">行驶距离:</span>
                <span>{{ formatDistance(selectedTrack.distance) }}</span>
              </div>
              <div class="info-row">
                <span class="label">持续时间:</span>
                <span>{{ formatDuration(selectedTrack.duration) }}</span>
              </div>
              <div class="info-row">
                <span class="label">路径点数:</span>
                <span>{{ selectedTrack.path?.length || 0 }}</span>
              </div>
            </div>

            <!-- 进度条 -->
            <div class="progress-container">
              <div class="progress-bar">
                <div class="progress-fill" :style="{ width: playbackProgress + '%' }"></div>
              </div>
              <span class="progress-text">{{ playbackProgress.toFixed(0) }}%</span>
            </div>

            <!-- 控制按钮 -->
            <div class="playback-controls">
              <button class="btn" @click="resetPlayback">
                ⏮ 重置
              </button>
              <button class="btn btn-primary" @click="playing ? stopPlayback() : startPlayback()" :disabled="!selectedTrack.path?.length">
                {{ playing ? '⏸ 暂停' : '▶ 播放' }}
              </button>
            </div>

            <!-- 路径可视化 -->
            <div class="path-visualization">
              <h4>路径预览</h4>
              <div class="path-canvas">
                <svg v-if="selectedTrack.path && selectedTrack.path.length > 1" viewBox="0 0 400 300" preserveAspectRatio="xMidYMid meet">
                  <!-- 网格背景 -->
                  <defs>
                    <pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse">
                      <path d="M 20 0 L 0 0 0 20" fill="none" stroke="#eee" stroke-width="0.5" />
                    </pattern>
                  </defs>
                  <rect width="100%" height="100%" fill="url(#grid)" />

                  <!-- 路径线 -->
                  <polyline
                    :points="selectedTrack.path.slice(0, currentPositionIndex + 1).map(p => `${(p[0] % 400 + 400) % 400},${(p[1] % 300 + 300) % 300}`).join(' ')"
                    fill="none"
                    stroke="#1976d2"
                    stroke-width="2"
                  />

                  <!-- 当前位置标记 -->
                  <circle
                    v-if="selectedTrack.path[currentPositionIndex]"
                    :cx="(selectedTrack.path[currentPositionIndex][0] % 400 + 400) % 400"
                    :cy="(selectedTrack.path[currentPositionIndex][1] % 300 + 300) % 300"
                    r="6"
                    fill="#d32f2f"
                  />
                </svg>
                <div v-else class="no-path-hint">
                  {{ selectedTrack.path?.length === 0 ? '该轨迹无路径数据' : '路径数据加载中...' }}
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  </section>
</template>

<style scoped>
.track-layout {
  display: grid;
  grid-template-columns: 350px 1fr;
  gap: 1rem;
  height: calc(100vh - 200px);
}

.track-list-panel {
  background: #f9f9f9;
  border-radius: 4px;
  padding: 1rem;
  overflow-y: auto;
}

.track-list-panel h3 {
  margin: 0 0 1rem 0;
  font-size: 0.875rem;
  color: #666;
}

.track-list {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.track-card {
  background: #fff;
  border: 1px solid #e0e0e0;
  border-radius: 4px;
  padding: 0.75rem;
  cursor: pointer;
  transition: all 0.2s;
}

.track-card:hover {
  border-color: #1976d2;
}

.track-card.active {
  border-color: #1976d2;
  background: #e3f2fd;
}

.track-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.5rem;
}

.track-header strong {
  font-size: 0.875rem;
}

.track-header small {
  font-size: 0.75rem;
  color: #999;
}

.track-info {
  display: flex;
  flex-wrap: wrap;
  gap: 0.5rem;
  font-size: 0.75rem;
  color: #666;
  margin-bottom: 0.5rem;
}

.track-stats {
  display: flex;
  gap: 0.75rem;
  font-size: 0.75rem;
  color: #666;
  margin-bottom: 0.5rem;
}

.track-playback-panel {
  background: #f9f9f9;
  border-radius: 4px;
  padding: 1rem;
  display: flex;
  align-items: center;
  justify-content: center;
}

.playback-placeholder {
  color: #999;
  font-size: 1rem;
}

.playback-content {
  width: 100%;
}

.playback-info {
  background: #fff;
  border-radius: 4px;
  padding: 1rem;
  margin-bottom: 1rem;
}

.info-row {
  display: flex;
  justify-content: space-between;
  padding: 0.5rem 0;
  border-bottom: 1px solid #f0f0f0;
}

.info-row:last-child {
  border-bottom: none;
}

.info-row .label {
  font-weight: 500;
  color: #666;
}

.progress-container {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  margin-bottom: 1rem;
}

.progress-bar {
  flex: 1;
  height: 8px;
  background: #e0e0e0;
  border-radius: 4px;
  overflow: hidden;
}

.progress-fill {
  height: 100%;
  background: #1976d2;
  border-radius: 4px;
  transition: width 0.2s;
}

.progress-text {
  font-size: 0.875rem;
  color: #666;
  min-width: 40px;
}

.playback-controls {
  display: flex;
  gap: 0.5rem;
  justify-content: center;
  margin-bottom: 1rem;
}

.path-visualization {
  background: #fff;
  border-radius: 4px;
  padding: 1rem;
}

.path-visualization h4 {
  margin: 0 0 1rem 0;
  font-size: 0.875rem;
  color: #666;
}

.path-canvas {
  width: 100%;
  height: 300px;
  background: #fafafa;
  border-radius: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.path-canvas svg {
  width: 100%;
  height: 100%;
}

.no-path-hint {
  color: #999;
  font-size: 0.875rem;
}

.empty-state {
  text-align: center;
  padding: 2rem;
  color: #999;
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
</style>
