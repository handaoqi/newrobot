<script setup>
import { computed, onMounted, ref } from 'vue'

import { fetchEvents, handleEvent } from '../services/api'

const filters = [
  { label: '全部记录', value: '' },
  { label: '待处理', value: 'pending' },
  { label: '已处理', value: 'resolved' },
]

const reviewOptions = [
  { label: '确认违规', value: 'confirmed' },
  { label: '怀疑', value: 'suspected' },
  { label: '误报', value: 'false_alarm' },
]

const sortOptions = [
  { label: '最新优先', value: 'detected_desc' },
  { label: '最早优先', value: 'detected_asc' },
  { label: '高风险优先', value: 'risk_desc' },
  { label: '置信度优先', value: 'confidence_desc' },
]

const activeFilter = ref('pending')
const events = ref([])
const selectedEvent = ref(null)
const loading = ref(true)
const loadingMore = ref(false)
const archiving = ref(false)
const page = ref(1)
const hasNextPage = ref(false)
const archiveNotes = ref('')
const reviewResult = ref('confirmed')
const searchDraft = ref('')
const searchKeyword = ref('')
const ordering = ref('detected_desc')
const pageSize = 8
const eventImages = ['/images/event-1.jpg', '/images/event-2.jpg', '/images/event-3.jpg']

const activeFilterIndex = computed(() => {
  const index = filters.findIndex((filter) => filter.value === activeFilter.value)
  return index === -1 ? 0 : index
})

const segmentStyle = computed(() => ({
  width: `${100 / filters.length}%`,
  transform: `translateX(${activeFilterIndex.value * 100}%)`,
}))

const reviewResultIndex = computed(() => {
  const index = reviewOptions.findIndex((option) => option.value === reviewResult.value)
  return index === -1 ? 0 : index
})

const reviewSliderStyle = computed(() => ({
  width: `${100 / reviewOptions.length}%`,
  transform: `translateX(${reviewResultIndex.value * 100}%)`,
}))

const hasEvents = computed(() => events.value.length > 0)
const canArchiveSelectedEvent = computed(() => selectedEvent.value?.status === 'pending')

function getEventImage(event) {
  if (event?.annotated_snapshot_url) return event.annotated_snapshot_url
  if (event?.snapshot_url) return event.snapshot_url
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
    page.value = 1
    const payload = await fetchEvents({
      status: activeFilter.value,
      page: page.value,
      pageSize,
      search: searchKeyword.value,
      ordering: ordering.value,
    })
    events.value = payload.results || payload
    hasNextPage.value = Boolean(payload.has_next)
    syncSelectedEvent()
  } finally {
    loading.value = false
  }
}

async function loadMoreEvents() {
  if (loading.value || loadingMore.value || !hasNextPage.value) return
  loadingMore.value = true
  try {
    const nextPage = page.value + 1
    const payload = await fetchEvents({
      status: activeFilter.value,
      page: nextPage,
      pageSize,
      search: searchKeyword.value,
      ordering: ordering.value,
    })
    events.value = [...events.value, ...(payload.results || payload)]
    hasNextPage.value = Boolean(payload.has_next)
    page.value = nextPage
    syncSelectedEvent()
  } finally {
    loadingMore.value = false
  }
}

function handleListScroll(event) {
  const { scrollTop, clientHeight, scrollHeight } = event.target
  if (scrollTop + clientHeight >= scrollHeight - 24) {
    loadMoreEvents()
  }
}

function changeFilter(value) {
  activeFilter.value = value
  selectedEvent.value = null
  archiveNotes.value = ''
  reviewResult.value = 'confirmed'
  loadEvents()
}

function changeSearch() {
  searchKeyword.value = searchDraft.value.trim()
  selectedEvent.value = null
  loadEvents()
}

function handleSearchInput() {
  if (searchDraft.value.trim() || !searchKeyword.value) return
  searchKeyword.value = ''
  selectedEvent.value = null
  loadEvents()
}

function changeOrdering() {
  selectedEvent.value = null
  loadEvents()
}

function selectEvent(event) {
  selectedEvent.value = event
  archiveNotes.value = ''
  reviewResult.value = event.review_result || 'confirmed'
}

async function markResolved() {
  if (!selectedEvent.value || archiving.value) return
  archiving.value = true
  const notes = archiveNotes.value.trim() || '值班员已通过平台完成复核与处置。'
  try {
    selectedEvent.value = await handleEvent(selectedEvent.value.id, {
      status: 'resolved',
      handling_notes: notes,
      review_result: reviewResult.value,
    })
    archiveNotes.value = ''
    await loadEvents()
  } finally {
    archiving.value = false
  }
}

onMounted(loadEvents)
</script>

<template>
  <section class="page-section">
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

    <div class="event-tools" aria-label="事件搜索与排序">
      <div class="event-search">
        <span>搜索</span>
        <input
          v-model="searchDraft"
          type="search"
          placeholder="标题、地点、机器人"
          @input="handleSearchInput"
          @keyup.enter="changeSearch"
        />
        <button type="button" class="event-search-btn" @click="changeSearch">查询</button>
      </div>
      <label class="event-sort">
        <span>排序</span>
        <select v-model="ordering" @change="changeOrdering">
          <option v-for="option in sortOptions" :key="option.value" :value="option.value">
            {{ option.label }}
          </option>
        </select>
      </label>
    </div>

    <div class="data-grid">
      <section class="panel list-panel" @scroll.passive="handleListScroll">
        <div v-if="loading" class="empty-state">正在加载事件列表...</div>
        <template v-else-if="hasEvents">
          <article
            v-for="event in events"
            :key="event.id"
            class="table-card"
            :class="{ selected: selectedEvent?.id === event.id }"
            @click="selectEvent(event)"
          >
            <div class="event-thumb table-thumb" :style="eventThumbStyle(event)"></div>
            <div class="table-main">
              <strong>{{ event.title }}</strong>
              <span>{{ event.location }}</span>
            </div>
            <div class="table-side">
              <span :class="['risk-chip', event.status === 'pending' ? event.status : 'review-chip']">
                {{ event.status === 'pending' ? event.status_label : event.review_result_label || '确认违规' }}
              </span>
              <small>{{ formatEventTime(event.detected_at) }}</small>
            </div>
          </article>
          <button v-if="hasNextPage" class="load-more-btn" type="button" :disabled="loadingMore" @click="loadMoreEvents">
            {{ loadingMore ? '正在加载更多...' : '加载更多事件' }}
          </button>
          <div v-else class="list-end">已显示当前筛选下全部事件</div>
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
          <div class="event-image-frame">
            <img class="event-detail-image" :src="getEventImage(selectedEvent)" :alt="selectedEvent.title" />
          </div>
          <div class="detail-card">
            <strong>识别置信度</strong>
            <p>{{ selectedEvent.confidence }}%</p>
          </div>
          <div class="detail-card" v-if="selectedEvent.stream_id">
            <strong>关联视频流</strong>
            <p>{{ selectedEvent.stream_id }} / {{ selectedEvent.camera_id || 'front' }}</p>
          </div>
          <div class="detail-card" v-if="selectedEvent.bbox_width">
            <strong>检测框</strong>
            <p>
              x={{ selectedEvent.bbox_x }},
              y={{ selectedEvent.bbox_y }},
              w={{ selectedEvent.bbox_width }},
              h={{ selectedEvent.bbox_height }}
            </p>
          </div>
          <div class="detail-card">
            <strong>处置备注</strong>
            <p>{{ selectedEvent.handling_notes || '当前尚未填写处置说明' }}</p>
          </div>
          <div class="detail-card" v-if="selectedEvent.review_result_label && !canArchiveSelectedEvent">
            <strong>复核结论</strong>
            <p>{{ selectedEvent.review_result_label }}</p>
          </div>
          <label v-if="canArchiveSelectedEvent" class="archive-field">
            <span>复核结论</span>
            <div class="review-slider" role="radiogroup" aria-label="复核结论">
              <div class="review-slider-thumb" :style="reviewSliderStyle"></div>
              <button
                v-for="option in reviewOptions"
                :key="option.value"
                type="button"
                class="review-option"
                :class="{ active: reviewResult === option.value }"
                :aria-pressed="reviewResult === option.value"
                @click="reviewResult = option.value"
              >
                {{ option.label }}
              </button>
            </div>
            <span>归档评论</span>
            <textarea v-model="archiveNotes" placeholder="请输入复核意见或现场处置说明"></textarea>
          </label>
          <button
            class="primary-btn"
            :disabled="!canArchiveSelectedEvent || archiving"
            @click="markResolved"
          >
            {{ archiving ? '正在归档...' : canArchiveSelectedEvent ? '完成复核并归档' : '已处理' }}
          </button>
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
