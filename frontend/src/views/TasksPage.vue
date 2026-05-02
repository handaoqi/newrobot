<script setup>
import { onMounted, ref } from 'vue'

import { fetchTasks } from '../services/api'

const tasks = ref([])

onMounted(async () => {
  tasks.value = await fetchTasks()
})
</script>

<template>
  <section class="page-section">
    <div class="section-head">
      <div>
        <h3>巡检任务</h3>
        <p>统一查看任务编排、执行进度与路线覆盖情况。</p>
      </div>
    </div>

    <section class="panel detail-panel">
      <div class="task-list">
        <article v-for="task in tasks" :key="task.id" class="task-card">
          <div>
            <strong>{{ task.name }}</strong>
            <span>{{ task.robot_name }} / {{ task.route_name }}</span>
          </div>
          <div class="table-side">
            <span class="panel-badge">{{ task.status }}</span>
            <small>完成度 {{ task.completion_rate }}%</small>
          </div>
        </article>
      </div>
    </section>
  </section>
</template>
