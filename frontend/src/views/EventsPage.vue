<script setup>
import { onMounted, ref } from 'vue'

import { fetchEvents, handleEvent } from '../services/api'

const filters = [
  { label: '全部记录', value: '' },
  { label: '待处理', value: 'pending' },
  { label: '处理中', value: 'processing' },
  { label: '已完成', value: 'resolved' },
]

const activeFilter = ref('')
const events = ref([])
const selectedEvent = ref(null)

async function loadEvents() {
  events.value = await fetchEvents(activeFilter.value)
  if (!selectedEvent.value && events.value.length > 0) {
    selectedEvent.value = events.value[0]
  }
}

async function markResolved() {
  if (!selectedEvent.value) return
  selectedEvent.value = await handleEvent(selectedEvent.value.id, {
    status: 'resolved',
    handling_notes: '值班员已通过平台完成复核与处置。',
  })
  await loadEvents()
}

onMounted(loadEvents)
</script>

<template>
  <section class="page-section">
    <div class="section-head">
      <div>
        <h3>事件中心</h3>
        <p>查看告警记录、人工复核结果和事件闭环状态</p>
      </div>
    </div>

    <div class="filter-row">
      <button
        v-for="filter in filters"
        :key="filter.value || 'all'"
        class="chip"
        :class="{ active: activeFilter === filter.value }"
        @click="activeFilter = filter.value; loadEvents()"
      >
        {{ filter.label }}
      </button>
    </div>

    <div class="data-grid">
      <section class="panel list-panel">
        <article
          v-for="event in events"
          :key="event.id"
          class="table-card"
          :class="{ selected: selectedEvent?.id === event.id }"
          @click="selectedEvent = event"
        >
          <div>
            <strong>{{ event.title }}</strong>
            <span>{{ event.location }}</span>
          </div>
          <div class="table-side">
            <span :class="['risk-chip', event.status]">{{ event.status_label }}</span>
            <small>{{ event.detected_at }}</small>
          </div>
        </article>
      </section>

      <section class="panel detail-panel" v-if="selectedEvent">
        <div class="panel-head">
          <div>
            <h3>{{ selectedEvent.title }}</h3>
            <p>{{ selectedEvent.robot_name }} / {{ selectedEvent.location }}</p>
          </div>
          <span class="panel-badge">{{ selectedEvent.risk_label }}风险</span>
        </div>
        <div class="detail-stack">
          <div class="detail-card">
            <strong>事件描述</strong>
            <p>{{ selectedEvent.description || '暂无补充描述' }}</p>
          </div>
          <div class="detail-card">
            <strong>识别置信度</strong>
            <p>{{ selectedEvent.confidence }}%</p>
          </div>
          <div class="detail-card">
            <strong>处置备注</strong>
            <p>{{ selectedEvent.handling_notes || '尚未填写' }}</p>
          </div>
          <button class="primary-btn" @click="markResolved">标记为已完成</button>
        </div>
      </section>
    </div>
  </section>
</template>
