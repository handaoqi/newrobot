<script setup>
import { computed, onMounted, ref } from 'vue'

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
const loading = ref(true)
const eventImages = ['/images/event-1.jpg', '/images/event-2.jpg', '/images/event-3.jpg']

const activeFilterIndex = computed(() => {
  const index = filters.findIndex((filter) => filter.value === activeFilter.value)
  return index === -1 ? 0 : index
})

const segmentStyle = computed(() => ({
  width: `${100 / filters.length}%`,
  transform: `translateX(${activeFilterIndex.value * 100}%)`,
}))

const hasEvents = computed(() => events.value.length > 0)

function getEventImage(event) {
  const index = events.value.findIndex((item) => item.id === event?.id)
  return eventImages[(index === -1 ? 0 : index) % eventImages.length]
}

function eventThumbStyle(event) {
  return {
    backgroundImage: `linear-gradient(rgba(6, 16, 28, 0.08), rgba(6, 16, 28, 0.18)), url(${getEventImage(event)})`,
  }
}

function formatEventTime(value) {
  if (!value) return '--'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return value
  return date.toLocaleString('zh-CN', {
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
  })
}

function syncSelectedEvent() {
  if (!events.value.length) {
    selectedEvent.value = null
    return
  }

  const matched = selectedEvent.value
    ? events.value.find((event) => event.id === selectedEvent.value.id)
    : null

  selectedEvent.value = matched || events.value[0]
}

async function loadEvents() {
  loading.value = true
  try {
    events.value = await fetchEvents(activeFilter.value)
    syncSelectedEvent()
  } finally {
    loading.value = false
  }
}

function changeFilter(value) {
  activeFilter.value = value
  selectedEvent.value = null
  loadEvents()
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
        <p>集中查看告警记录、复核进度与事件闭环结果。</p>
      </div>
    </div>

    <div class="filter-row">
      <div class="segmented-control" role="tablist" aria-label="事件状态筛选">
        <div class="segmented-thumb" :style="segmentStyle"></div>
        <button
          v-for="filter in filters"
          :key="filter.value || 'all'"
          class="segment-btn"
          :class="{ active: activeFilter === filter.value }"
          :aria-pressed="activeFilter === filter.value"
          @click="changeFilter(filter.value)"
        >
          {{ filter.label }}
        </button>
      </div>
    </div>

    <div class="data-grid">
      <section class="panel list-panel">
        <div v-if="loading" class="empty-state">正在加载事件列表...</div>
        <template v-else-if="hasEvents">
        <article
          v-for="event in events"
          :key="event.id"
          class="table-card"
          :class="{ selected: selectedEvent?.id === event.id }"
          @click="selectedEvent = event"
        >
          <div class="event-thumb table-thumb" :style="eventThumbStyle(event)"></div>
          <div class="table-main">
            <strong>{{ event.title }}</strong>
            <span>{{ event.location }}</span>
          </div>
          <div class="table-side">
            <span :class="['risk-chip', event.status]">{{ event.status_label }}</span>
            <small>{{ formatEventTime(event.detected_at) }}</small>
          </div>
        </article>
        </template>
        <div v-else class="empty-state">当前筛选条件下暂无事件记录</div>
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
          <img class="event-detail-image" :src="getEventImage(selectedEvent)" :alt="selectedEvent.title" />
          <div class="detail-card">
            <strong>识别置信度</strong>
            <p>{{ selectedEvent.confidence }}%</p>
          </div>
          <div class="detail-card">
            <strong>处置备注</strong>
            <p>{{ selectedEvent.handling_notes || '当前尚未填写处置说明' }}</p>
          </div>
          <button class="primary-btn" @click="markResolved">完成复核并归档</button>
        </div>
      </section>

      <section v-else class="panel detail-panel detail-empty">
        <div class="panel-head">
          <div>
            <h3>事件详情</h3>
            <p>当前筛选结果为空，暂未选中任何事件</p>
          </div>
        </div>
        <div class="empty-state detail-empty-card">
          请选择一个事件开始处理。
        </div>
      </section>
    </div>
  </section>
</template>
