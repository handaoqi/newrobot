<script setup>
import { computed, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useRoute } from 'vue-router'

import { fetchEvents, handleEvent } from '../services/api'

const route = useRoute()

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

const eventStatusValues = new Set(['', 'pending', 'resolved'])
const requestedStatus = String(route.query.status ?? 'pending')
const activeFilter = ref(eventStatusValues.has(requestedStatus) ? requestedStatus : 'pending')
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
const dateDraft = ref('')
const startTimeDraft = ref('00:00')
const endTimeDraft = ref('23:59')
const detectedFrom = ref('')
const detectedTo = ref('')
const timeFilterError = ref('')
const calendarMonth = ref(new Date(new Date().getFullYear(), new Date().getMonth(), 1))
const fullscreenImage = ref(null)
const pageSize = 8
const eventImages = ['/images/event-1.jpg', '/images/event-2.jpg', '/images/event-3.jpg']
let previousBodyOverflow = ''

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
const hasTimeFilter = computed(() => Boolean(detectedFrom.value || detectedTo.value))
const hasSecondaryFilter = computed(() => Boolean(hasTimeFilter.value || searchKeyword.value || searchDraft.value.trim()))
const timeRangeInvalid = computed(() => {
  if (!dateDraft.value || !startTimeDraft.value || !endTimeDraft.value) return false
  return startTimeDraft.value > endTimeDraft.value
})
const timeFilterLabel = computed(() => {
  if (!hasTimeFilter.value) return '未限定时间'
  const start = startTimeDraft.value || '00:00'
  const end = endTimeDraft.value || '23:59'
  return `${dateDraft.value} ${start}-${end}`
})
const filterSummary = computed(() => {
  const parts = []
  const statusLabel = filters.find((filter) => filter.value === activeFilter.value)?.label || '全部记录'
  parts.push(statusLabel)
  if (searchKeyword.value) parts.push(`关键词: ${searchKeyword.value}`)
  if (hasTimeFilter.value) parts.push(timeFilterLabel.value)
  return parts.join(' / ')
})
const calendarMonthTitle = computed(() => {
  const year = calendarMonth.value.getFullYear()
  const month = calendarMonth.value.getMonth() + 1
  return `${year}年${String(month).padStart(2, '0')}月`
})
const calendarDays = computed(() => {
  const year = calendarMonth.value.getFullYear()
  const month = calendarMonth.value.getMonth()
  const firstDay = new Date(year, month, 1)
  const startOffset = firstDay.getDay()
  const gridStart = new Date(year, month, 1 - startOffset)
  const todayValue = formatDateValue(new Date())

  return Array.from({ length: 42 }, (_, index) => {
    const day = new Date(gridStart)
    day.setDate(gridStart.getDate() + index)
    const value = formatDateValue(day)
    return {
      value,
      label: day.getDate(),
      inMonth: day.getMonth() === month,
      selected: value === dateDraft.value,
      today: value === todayValue,
    }
  })
})

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

function formatDateValue(date) {
  const year = date.getFullYear()
  const month = String(date.getMonth() + 1).padStart(2, '0')
  const day = String(date.getDate()).padStart(2, '0')
  return `${year}-${month}-${day}`
}

function toLocalIso(dateValue, timeValue, fallbackTime, includeFullMinute = false) {
  if (!dateValue) return ''
  const [year, month, day] = dateValue.split('-').map(Number)
  const [hour, minute] = (timeValue || fallbackTime).split(':').map(Number)
  const date = new Date(year, month - 1, day, hour, minute, includeFullMinute ? 59 : 0, includeFullMinute ? 999 : 0)
  return date.toISOString()
}

function shiftCalendarMonth(offset) {
  calendarMonth.value = new Date(
    calendarMonth.value.getFullYear(),
    calendarMonth.value.getMonth() + offset,
    1,
  )
}

function selectCalendarDate(value) {
  dateDraft.value = value
  timeFilterError.value = ''
}

function eventQueryParams(pageValue) {
  return {
    status: activeFilter.value,
    page: pageValue,
    pageSize,
    search: searchKeyword.value,
    ordering: ordering.value,
    detectedFrom: detectedFrom.value,
    detectedTo: detectedTo.value,
  }
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
    const payload = await fetchEvents(eventQueryParams(page.value))
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
    const payload = await fetchEvents(eventQueryParams(nextPage))
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

function applyFilters() {
  timeFilterError.value = ''
  if (dateDraft.value && timeRangeInvalid.value) {
    timeFilterError.value = '结束时间不能早于开始时间'
    return
  }

  searchKeyword.value = searchDraft.value.trim()
  if (dateDraft.value) {
    detectedFrom.value = toLocalIso(dateDraft.value, startTimeDraft.value, '00:00')
    detectedTo.value = toLocalIso(dateDraft.value, endTimeDraft.value, '23:59', true)
  }
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

function applyTimeFilter() {
  applyFilters()
}

function clearTimeFilter() {
  dateDraft.value = ''
  startTimeDraft.value = '00:00'
  endTimeDraft.value = '23:59'
  detectedFrom.value = ''
  detectedTo.value = ''
  searchDraft.value = ''
  searchKeyword.value = ''
  timeFilterError.value = ''
  selectedEvent.value = null
  loadEvents()
}

function selectEvent(event) {
  selectedEvent.value = event
  archiveNotes.value = ''
  reviewResult.value = event.review_result || 'confirmed'
}

function openFullscreenImage(event) {
  fullscreenImage.value = {
    src: getEventImage(event),
    alt: event?.title || '事件图片',
  }
  previousBodyOverflow = document.body.style.overflow
  document.body.style.overflow = 'hidden'
}

function closeFullscreenImage() {
  if (!fullscreenImage.value) return
  fullscreenImage.value = null
  document.body.style.overflow = previousBodyOverflow
}

function handleFullscreenKeydown(event) {
  if (event.key === 'Escape' && fullscreenImage.value) {
    closeFullscreenImage()
  }
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

watch(() => route.query.status, (value) => {
  const requested = String(value ?? 'pending')
  const nextFilter = eventStatusValues.has(requested) ? requested : 'pending'
  if (nextFilter === activeFilter.value) return
  activeFilter.value = nextFilter
  selectedEvent.value = null
  loadEvents()
})

onMounted(() => {
  document.addEventListener('keydown', handleFullscreenKeydown)
  loadEvents()
})

onBeforeUnmount(() => {
  document.removeEventListener('keydown', handleFullscreenKeydown)
  if (fullscreenImage.value) document.body.style.overflow = previousBodyOverflow
})
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

    <div class="event-time-filter" aria-label="事件时间筛选">
      <div class="time-filter-title">
        <span>时间筛选</span>
        <small :class="{ error: timeFilterError }">{{ timeFilterError || timeFilterLabel }}</small>
      </div>
      <div class="time-filter-controls">
        <div class="calendar-filter">
          <div class="calendar-head">
            <button type="button" aria-label="上个月" @click="shiftCalendarMonth(-1)">‹</button>
            <strong>{{ calendarMonthTitle }}</strong>
            <button type="button" aria-label="下个月" @click="shiftCalendarMonth(1)">›</button>
          </div>
          <div class="calendar-weekdays" aria-hidden="true">
            <span>日</span>
            <span>一</span>
            <span>二</span>
            <span>三</span>
            <span>四</span>
            <span>五</span>
            <span>六</span>
          </div>
          <div class="calendar-grid">
            <button
              v-for="day in calendarDays"
              :key="day.value"
              type="button"
              class="calendar-day"
              :class="{ muted: !day.inMonth, selected: day.selected, today: day.today }"
              @click="selectCalendarDate(day.value)"
            >
              {{ day.label }}
            </button>
          </div>
        </div>
        <div class="time-range-panel">
          <div class="selected-date-label">
            <span>当前日期</span>
            <strong>{{ dateDraft || '请选择日期' }}</strong>
          </div>
          <div class="time-range-fields">
            <label>
              <span>开始</span>
              <input v-model="startTimeDraft" type="time" @change="timeFilterError = ''" />
            </label>
            <label>
              <span>结束</span>
              <input v-model="endTimeDraft" type="time" @change="timeFilterError = ''" />
            </label>
          </div>
          <div class="filter-search-row" aria-label="事件搜索与排序">
            <label class="event-search">
              <span>关键词</span>
              <input
                v-model="searchDraft"
                type="search"
                placeholder="标题、地点、机器人"
                @input="handleSearchInput"
                @keyup.enter="applyFilters"
              />
            </label>
            <label class="event-sort">
              <span>排序</span>
              <select v-model="ordering" @change="changeOrdering">
                <option v-for="option in sortOptions" :key="option.value" :value="option.value">
                  {{ option.label }}
                </option>
              </select>
            </label>
            <div class="filter-summary">
              <span>范围</span>
              <strong>{{ filterSummary }}</strong>
            </div>
          </div>
          <div class="time-filter-actions">
            <button type="button" class="event-search-btn" :disabled="timeRangeInvalid" @click="applyTimeFilter">
              筛选
            </button>
            <button type="button" class="event-clear-btn" :disabled="!hasSecondaryFilter" @click="clearTimeFilter">
              清除
            </button>
          </div>
        </div>
      </div>
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
            <button
              type="button"
              class="event-thumb table-thumb event-image-trigger"
              :style="eventThumbStyle(event)"
              :aria-label="`全屏查看 ${event.title} 图片`"
              title="点击全屏查看"
              @click.stop="openFullscreenImage(event)"
            ></button>
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
          <button
            type="button"
            class="event-image-frame event-image-trigger"
            :aria-label="`全屏查看 ${selectedEvent.title} 图片`"
            title="点击全屏查看"
            @click="openFullscreenImage(selectedEvent)"
          >
            <img class="event-detail-image" :src="getEventImage(selectedEvent)" :alt="selectedEvent.title" />
            <span class="event-image-expand" aria-hidden="true">全屏查看</span>
          </button>
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

    <Teleport to="body">
      <Transition name="event-lightbox">
        <div
          v-if="fullscreenImage"
          class="event-image-lightbox"
          role="dialog"
          aria-modal="true"
          :aria-label="`${fullscreenImage.alt}全屏预览`"
          @click.self="closeFullscreenImage"
        >
          <button
            type="button"
            class="event-lightbox-close"
            aria-label="退出全屏查看"
            title="退出全屏 (Esc)"
            @click="closeFullscreenImage"
          >
            ×
          </button>
          <img :src="fullscreenImage.src" :alt="fullscreenImage.alt" />
          <span class="event-lightbox-hint">ESC 退出全屏</span>
        </div>
      </Transition>
    </Teleport>
  </section>
</template>
