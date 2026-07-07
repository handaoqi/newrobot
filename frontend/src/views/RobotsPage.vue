<script setup>
import { onBeforeUnmount, onMounted, ref } from 'vue'

import { fetchRobotDetail, fetchRobots, fetchRobotSessions, fetchRobotStatus } from '../services/api'

const robots = ref([])
const selectedRobot = ref(null)
const liveStatus = ref(null)
const sessions = ref([])
let refreshTimer = null

async function chooseRobot(robotId) {
  selectedRobot.value = await fetchRobotDetail(robotId)
  ;[liveStatus.value, sessions.value] = await Promise.all([
    fetchRobotStatus(robotId),
    fetchRobotSessions(robotId),
  ])
}

async function refreshRobots() {
  robots.value = await fetchRobots()
  const selectedRobotId = selectedRobot.value?.id || robots.value[0]?.id
  if (selectedRobotId) {
    await chooseRobot(selectedRobotId)
  }
}

onMounted(async () => {
  await refreshRobots()
  refreshTimer = window.setInterval(refreshRobots, 5000)
})

onBeforeUnmount(() => {
  if (refreshTimer) {
    window.clearInterval(refreshTimer)
  }
})
</script>

<template>
  <section class="page-section">
    <div class="data-grid">
      <section class="panel list-panel">
        <article
          v-for="robot in robots"
          :key="robot.id"
          class="table-card robot-table-card"
          :class="{ selected: selectedRobot?.id === robot.id }"
          @click="chooseRobot(robot.id)"
        >
          <div class="robot-card-main">
            <strong>{{ robot.name }}</strong>
            <span>{{ robot.code }}</span>
            <small>{{ robot.location }}</small>
          </div>
          <div class="table-side robot-card-side">
            <span :class="['robot-status', robot.status]">{{ robot.status_label }}</span>
            <small>{{ robot.battery_level }}%</small>
          </div>
        </article>
      </section>

      <section class="panel detail-panel" v-if="selectedRobot">
        <div class="panel-head">
          <div>
            <h3>{{ selectedRobot.name }}</h3>
            <p>{{ selectedRobot.code }} / {{ selectedRobot.area }}</p>
          </div>
          <span class="panel-badge">{{ selectedRobot.mode_label }}</span>
        </div>

        <div class="metrics-grid">
          <div class="metric-card">
            <strong>{{ liveStatus?.status?.power_available ? `${liveStatus.status.battery_percent}%` : '未知' }}</strong>
            <span>真实电量</span>
          </div>
          <div class="metric-card">
            <strong>{{ selectedRobot.network_strength }}%</strong>
            <span>网络强度</span>
          </div>
          <div class="metric-card">
            <strong>{{ selectedRobot.speaker_volume }}%</strong>
            <span>扬声器音量</span>
          </div>
          <div class="metric-card">
            <strong>{{ selectedRobot.firmware_version }}</strong>
            <span>固件版本</span>
          </div>
        </div>

        <div class="detail-card">
          <strong>当前执行任务</strong>
          <p>{{ selectedRobot.current_task_name || '暂无任务' }}</p>
        </div>

        <div class="detail-card">
          <strong>Edge Agent 状态</strong>
          <p>
            连接 {{ liveStatus?.connection_status || 'unknown' }} ·
            定位 {{ liveStatus?.status?.localization_status || 'unknown' }} ·
            Nav2 {{ liveStatus?.status?.nav_ready ? 'ready' : 'not ready' }} ·
            会话 {{ sessions.length }}
          </p>
        </div>

        <div class="detail-card">
          <strong>近期识别事件</strong>
          <ul class="simple-list">
            <li v-for="event in selectedRobot.recent_events" :key="event.id">
              {{ event.title }} - {{ event.status_label }}
            </li>
          </ul>
        </div>
      </section>
    </div>
  </section>
</template>
