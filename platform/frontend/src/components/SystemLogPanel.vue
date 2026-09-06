<script setup>
import { computed, onBeforeUnmount, ref, watch } from 'vue'
import {
  fetchRobotSystemLogs,
  startRobotDebugLogSession,
  stopRobotDebugLogSession,
} from '../services/api'

const props = defineProps({
  robotId: { type: [Number, String], default: null },
  mapId: { type: [Number, String], default: null },
})
const emit = defineEmits(['locate'])

const MODULES = [
  ['localization', '定位'], ['navigation', '导航'], ['avoidance', '避障'],
  ['relocalization', '主动重定位'], ['waypoint', '航点'], ['planner', '规划算法'],
  ['boundary', '边界'], ['system', '系统'],
]
const open = ref(false)
const logs = ref([])
const selected = ref(null)
const levels = ref(['INFO', 'WARNING', 'ERROR'])
const moduleFilter = ref('')
const keyword = ref('')
const paused = ref(false)
const loading = ref(false)
const error = ref('')
const cursor = ref('')
const debugSession = ref(null)
const debugMinutes = ref(15)
const nowTick = ref(Date.now())
const canDebug = computed(() => {
  try {
    return Boolean(JSON.parse(window.localStorage.getItem('inspection_user') || '{}').is_staff)
  } catch {
    return false
  }
})
let pollTimer = null
let clockTimer = window.setInterval(() => { nowTick.value = Date.now() }, 1000)

const warningCount = computed(() => logs.value.filter(item => item.level === 'WARNING').length)
const errorCount = computed(() => logs.value.filter(item => item.level === 'ERROR').length)
const debugRemaining = computed(() => {
  nowTick.value
  if (!debugSession.value?.expires_at) return ''
  const seconds = Math.max(0, Math.ceil((new Date(debugSession.value.expires_at).getTime() - Date.now()) / 1000))
  if (!seconds) return ''
  return `${Math.floor(seconds / 60)}:${String(seconds % 60).padStart(2, '0')}`
})

function params(incremental = true) {
  return {
    levels: levels.value.join(','),
    modules: moduleFilter.value,
    q: keyword.value.trim(),
    map_id: props.mapId || '',
    after: incremental ? cursor.value : '',
    limit: 200,
  }
}

async function refresh({ reset = false } = {}) {
  if (!props.robotId || loading.value || paused.value) return
  loading.value = true
  error.value = ''
  try {
    const response = await fetchRobotSystemLogs(props.robotId, params(!reset))
    const incoming = response.results || []
    if (reset) logs.value = incoming
    else {
      const known = new Set(logs.value.map(item => item.id))
      logs.value = [...incoming.filter(item => !known.has(item.id)), ...logs.value].slice(0, 500)
    }
    if (response.cursor) cursor.value = response.cursor
    debugSession.value = response.debug_session || null
  } catch (cause) {
    error.value = cause.message || '系统日志获取失败'
  } finally {
    loading.value = false
  }
}

function restartPolling() {
  if (pollTimer) window.clearInterval(pollTimer)
  cursor.value = ''
  logs.value = []
  if (props.robotId) {
    refresh({ reset: true })
    pollTimer = window.setInterval(() => refresh(), 2000)
  }
}

async function toggleDebug() {
  if (!props.robotId) return
  error.value = ''
  try {
    if (debugSession.value) {
      await stopRobotDebugLogSession(props.robotId, debugSession.value.id)
      debugSession.value = null
      levels.value = levels.value.filter(level => level !== 'DEBUG')
    } else {
      debugSession.value = await startRobotDebugLogSession(props.robotId, {
        duration_seconds: Number(debugMinutes.value) * 60,
        sample_hz: 1,
        modules: MODULES.map(item => item[0]),
      })
      if (!levels.value.includes('DEBUG')) levels.value = ['DEBUG', ...levels.value]
    }
    restartPolling()
  } catch (cause) {
    error.value = cause.message || 'DEBUG 会话操作失败'
  }
}

function toggleLevel(level) {
  levels.value = levels.value.includes(level)
    ? levels.value.filter(item => item !== level)
    : [...levels.value, level]
}

function inspect(item) {
  selected.value = selected.value?.id === item.id ? null : item
  if (item.x !== null && item.x !== undefined && item.y !== null && item.y !== undefined) emit('locate', item)
}

function clock(value) {
  if (!value) return '—'
  return new Date(value).toLocaleTimeString('zh-CN', { hour12: false, fractionalSecondDigits: 3 })
}

watch([() => props.robotId, () => props.mapId], restartPolling, { immediate: true })
watch([levels, moduleFilter], restartPolling, { deep: true })
let keywordTimer = null
watch(keyword, () => {
  if (keywordTimer) window.clearTimeout(keywordTimer)
  keywordTimer = window.setTimeout(restartPolling, 300)
})
onBeforeUnmount(() => {
  if (pollTimer) window.clearInterval(pollTimer)
  if (clockTimer) window.clearInterval(clockTimer)
  if (keywordTimer) window.clearTimeout(keywordTimer)
})
</script>

<template>
  <section class="system-log-panel" :class="{ open }">
    <button type="button" class="system-log-summary" @click="open = !open">
      <strong>系统日志</strong>
      <span>{{ loading ? '同步中' : `${logs.length} 条` }}</span>
      <span v-if="warningCount" class="count warning">WARNING {{ warningCount }}</span>
      <span v-if="errorCount" class="count error">ERROR {{ errorCount }}</span>
      <span v-if="debugRemaining" class="debug-time">DEBUG {{ debugRemaining }}</span>
      <em>{{ open ? '收起' : '展开' }}</em>
    </button>
    <div v-if="open" class="system-log-body">
      <div class="system-log-toolbar">
        <div class="level-filters">
          <button v-for="level in ['DEBUG', 'INFO', 'WARNING', 'ERROR']" :key="level" type="button" :class="['level-chip', level.toLowerCase(), { active: levels.includes(level) }]" @click="toggleLevel(level)">{{ level }}</button>
        </div>
        <select v-model="moduleFilter">
          <option value="">全部模块</option>
          <option v-for="item in MODULES" :key="item[0]" :value="item[0]">{{ item[1] }}</option>
        </select>
        <input v-model="keyword" placeholder="事件码 / 摘要 / 来源" />
        <button type="button" class="btn btn-sm" @click="paused = !paused">{{ paused ? '继续追踪' : '暂停滚动' }}</button>
        <select v-if="canDebug" v-model.number="debugMinutes" :disabled="!!debugSession">
          <option :value="5">5分钟</option><option :value="15">15分钟</option><option :value="30">30分钟</option>
        </select>
        <button v-if="canDebug" type="button" class="btn btn-sm" :class="{ 'btn-danger': debugSession }" @click="toggleDebug">{{ debugSession ? '关闭 DEBUG' : '开启 DEBUG' }}</button>
      </div>
      <p v-if="error" class="system-log-error">{{ error }}</p>
      <div class="system-log-scroll">
        <div class="system-log-head"><span>时间</span><span>级别</span><span>模块</span><span>事件与摘要</span></div>
        <button v-for="item in logs" :key="item.id" type="button" class="system-log-row" :class="[item.level.toLowerCase(), { selected: selected?.id === item.id }]" @click="inspect(item)">
          <time>{{ clock(item.occurred_at) }}</time><b>{{ item.level }}</b><span>{{ MODULES.find(row => row[0] === item.module)?.[1] || item.module }}</span>
          <div><code>{{ item.event_code }}</code><strong>{{ item.message }}</strong><i v-if="item.repeat_count > 1">×{{ item.repeat_count }}</i></div>
        </button>
        <div v-if="!logs.length && !loading" class="system-log-empty">当前筛选条件下暂无日志</div>
      </div>
      <aside v-if="selected" class="system-log-detail">
        <button type="button" aria-label="关闭详情" @click="selected = null">×</button>
        <strong>{{ selected.message }}</strong>
        <small>{{ selected.source }} · trace {{ selected.trace_id || '—' }} · task {{ selected.task_execution || '—' }} · command {{ selected.command || '—' }}</small>
        <pre>{{ JSON.stringify(selected.data || {}, null, 2) }}</pre>
      </aside>
    </div>
  </section>
</template>

<style scoped>
.system-log-panel { position: sticky; bottom: 0; z-index: 45; margin-top: .75rem; border: 1px solid #253449; border-radius: 10px; background: #07111f; color: #d7e3f4; box-shadow: 0 -8px 28px rgba(2, 8, 23, .28); }
.system-log-summary { width: 100%; min-height: 42px; display: flex; align-items: center; gap: .8rem; padding: .55rem .8rem; border: 0; color: inherit; background: transparent; cursor: pointer; }
.system-log-summary strong { color: #fff; }.system-log-summary em { margin-left: auto; font-style: normal; }.count,.debug-time { padding: .15rem .45rem; border-radius: 999px; font-size: 11px; }.warning { color: #ffd166; background: #4a3610; }.error { color: #ff9b9b; background: #4c1515; }.debug-time { color: #8ee6ff; background: #12364a; }
.system-log-body { position: relative; height: min(34vh, 360px); min-height: 240px; display: flex; flex-direction: column; border-top: 1px solid #253449; }
.system-log-toolbar { display: flex; flex-wrap: wrap; gap: .45rem; padding: .55rem; align-items: center; }.system-log-toolbar input { flex: 1; min-width: 180px; }.system-log-toolbar input,.system-log-toolbar select { border: 1px solid #38516f; border-radius: 6px; background: #0d1c2d; color: #e7eef8; padding: .4rem .5rem; }
.level-filters { display: flex; gap: .3rem; }.level-chip { opacity: .42; border: 1px solid currentColor; border-radius: 999px; background: transparent; padding: .25rem .45rem; font-size: 10px; }.level-chip.active { opacity: 1; }.debug { color: #6bdcff; }.info { color: #8fb9ff; }.warning { color: #ffd166; }.error { color: #ff8585; }
.system-log-scroll { min-height: 0; overflow: auto; }.system-log-head,.system-log-row { display: grid; grid-template-columns: 105px 72px 100px minmax(320px, 1fr); gap: .6rem; align-items: center; width: 100%; padding: .38rem .65rem; text-align: left; }.system-log-head { position: sticky; top: 0; z-index: 2; color: #8095ad; background: #0b1726; font-size: 11px; }.system-log-row { border: 0; border-left: 3px solid transparent; border-top: 1px solid #17283b; color: inherit; background: transparent; cursor: pointer; }.system-log-row:hover,.system-log-row.selected { background: #12243a; }.system-log-row.warning { border-left-color: #e5a82c; }.system-log-row.error { border-left-color: #e05252; background: #2a1419; }.system-log-row time,.system-log-row span { color: #98aac0; font-size: 11px; }.system-log-row div { display: flex; gap: .6rem; align-items: baseline; min-width: 0; }.system-log-row code { color: #7dd3fc; }.system-log-row strong { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }.system-log-row i { color: #9cafc4; }
.system-log-detail { position: absolute; right: 0; top: 48px; bottom: 0; width: min(46%, 560px); padding: .75rem; overflow: auto; border-left: 1px solid #38516f; background: #0c1929; box-shadow: -12px 0 30px rgba(0,0,0,.3); }.system-log-detail > button { float: right; border: 0; color: #fff; background: transparent; font-size: 20px; }.system-log-detail small { display: block; margin: .4rem 0; color: #8fa3ba; word-break: break-all; }.system-log-detail pre { white-space: pre-wrap; color: #b9e6ff; }.system-log-error { margin: 0; padding: .35rem .65rem; color: #ffaaaa; }.system-log-empty { padding: 2rem; text-align: center; color: #8095ad; }
@media (max-width: 900px) { .system-log-body { height: 42vh; }.system-log-head,.system-log-row { grid-template-columns: 88px 60px 80px minmax(220px, 1fr); }.system-log-detail { width: 70%; } }
</style>
