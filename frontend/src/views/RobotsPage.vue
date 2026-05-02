<script setup>
import { onMounted, ref } from 'vue'

import { fetchRobotDetail, fetchRobots } from '../services/api'

const robots = ref([])
const selectedRobot = ref(null)

async function chooseRobot(robotId) {
  selectedRobot.value = await fetchRobotDetail(robotId)
}

onMounted(async () => {
  robots.value = await fetchRobots()
  if (robots.value.length > 0) {
    await chooseRobot(robots.value[0].id)
  }
})
</script>

<template>
  <section class="page-section">
    <div class="section-head">
      <div>
        <h3>机器人管理</h3>
        <p>查看设备在线状态、健康信息和近期事件</p>
      </div>
    </div>

    <div class="data-grid">
      <section class="panel list-panel">
        <article
          v-for="robot in robots"
          :key="robot.id"
          class="table-card"
          :class="{ selected: selectedRobot?.id === robot.id }"
          @click="chooseRobot(robot.id)"
        >
          <div>
            <strong>{{ robot.name }}</strong>
            <span>{{ robot.code }} / {{ robot.location }}</span>
          </div>
          <div class="table-side">
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
            <strong>{{ selectedRobot.battery_level }}%</strong>
            <span>设备电量</span>
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
          <strong>当前任务</strong>
          <p>{{ selectedRobot.current_task_name || '暂无任务' }}</p>
        </div>

        <div class="detail-card">
          <strong>近期事件</strong>
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
