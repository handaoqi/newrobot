<script setup>
/**
 * 回放调试台 -- the four-zone debugging console from the design document.
 *
 * Zones 1, 3 and 4 are ours. Zone 2 is a Lichtblick instance embedded from our
 * own origin with its interface stripped off, driven entirely through
 * services/lichtblickBridge.js. Nothing on this page carries upstream branding,
 * and the operator never sees a second set of controls.
 *
 * Two sources, one page:
 *   - 实时: the dog's live data through the NX foxglove_bridge.
 *   - 本地 Bag: an .mcap the operator picks in the browser. It is handed straight
 *     to the host's own file input and never leaves the machine -- no upload, no
 *     server-side file API.
 */
import { computed, onBeforeUnmount, ref, watch } from 'vue'

import PlaybackBar from '../components/replay/PlaybackBar.vue'
import SceneFrame from '../components/replay/SceneFrame.vue'
import { createLichtblickBridge } from '../services/lichtblickBridge'
import { openBagSource, openLiveSource } from '../services/rosStream'

const LIVE_URL_KEY = 'replay_live_ws'
const defaultLiveUrl = import.meta.env.VITE_REPLAY_LIVE_WS?.trim()
  || `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.host}/foxglove/ws`
const legacyLiveUrl = `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.hostname}:8080/ws`
const storedLiveUrl = localStorage.getItem(LIVE_URL_KEY)?.trim()
const initialLiveUrl = storedLiveUrl && storedLiveUrl !== legacyLiveUrl ? storedLiveUrl : defaultLiveUrl

const scene = ref(null)
const filePicker = ref(null)
const bridge = createLichtblickBridge(() => scene.value?.frame)

const mode = ref('bag')
const liveUrl = ref(initialLiveUrl)
const bagName = ref('')
const busyLabel = ref('')
const attached = ref(false)
const error = ref('')
const clock = ref({ text: null, fraction: null })
const source = ref(null)
const sourceState = ref('idle')
const sourceProgress = ref(0)
const sourceElapsed = ref(0)
const sourceTick = ref(0)

let stopClock = null

const SIGNALS = [
  { key: 'localization-x', label: '定位 X', unit: 'm', topic: '/odom/localization_odom', pick: 'pose.pose.position.x', color: '#2563eb' },
  { key: 'localization-y', label: '定位 Y', unit: 'm', topic: '/odom/localization_odom', pick: 'pose.pose.position.y', color: '#16a34a' },
  { key: 'heading', label: '航向', unit: '°', topic: '/odom/localization_odom', pick: message => {
    const q = message?.pose?.pose?.orientation
    if (!q) return undefined
    const siny = 2 * (q.w * q.z + q.x * q.y)
    const cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    return Math.atan2(siny, cosy) * 180 / Math.PI
  }, color: '#9333ea' },
  { key: 'cmd-angular-z', label: 'cmd_vel 角速度', unit: 'rad/s', topic: '/cmd_vel', pick: 'angular.z', color: '#ea580c' },
  { key: 'raw-angular-z', label: 'raw 角速度', unit: 'rad/s', topic: '/cmd_vel_raw', pick: 'angular.z', color: '#dc2626' },
  { key: 'ndt-error', label: 'NDT 匹配误差', unit: '', topic: '/status', pick: 'matching_error', color: '#0891b2' },
  { key: 'ndt-inlier', label: 'NDT 内点率', unit: '%', topic: '/status', pick: message => Number(message?.inlier_fraction) * 100, color: '#0f766e' },
  { key: 'match-converged', label: '匹配收敛', unit: '', topic: '/status', pick: message => message?.has_converged ? 1 : 0, color: '#65a30d' },
  { key: 'localization-status', label: '定位状态', unit: '', topic: '/localization_info', pick: 'status', color: '#7c3aed' },
]

const STATUS_LABELS = {
  0: '初始化', 1: '重定位中', 2: '重定位成功', 3: '正常', 4: '丢失',
}

function closeSource() {
  source.value?.close?.()
  source.value = null
  sourceState.value = 'idle'
  sourceProgress.value = 0
  sourceElapsed.value = 0
  sourceTick.value++
}

function sourceError(nextError) {
  sourceState.value = 'error'
  error.value = `数据解码失败：${nextError?.message || '实时数据连接失败'}`
}

async function openBag(file) {
  closeSource()
  sourceState.value = 'loading'
  sourceProgress.value = 0
  try {
    source.value = await openBagSource(file, SIGNALS, {
      onProgress: progress => { sourceProgress.value = progress; sourceTick.value++ },
    })
    sourceState.value = 'ready'
    sourceTick.value++
  } catch (nextError) {
    sourceState.value = 'error'
    throw nextError
  }
}

function openLive() {
  closeSource()
  sourceState.value = 'connecting'
  source.value = openLiveSource(liveUrl.value, SIGNALS, {
    onUpdate: elapsed => {
      sourceElapsed.value = elapsed
      sourceState.value = 'ready'
      sourceTick.value++
    },
    onError: nextError => sourceError(nextError),
  })
}

/**
 * The host URL. In live mode the source is attached by query parameters, so the
 * host comes up already connected; in bag mode we open bare and hand it the file
 * afterwards. Changing this string reloads the frame, which is the only way to
 * swap the host's data source.
 */
const src = computed(() =>
  mode.value === 'live'
    ? `/foxglove/?ds=foxglove-websocket&ds.url=${encodeURIComponent(liveUrl.value)}`
    : '/foxglove/',
)

const sourceLabel = computed(() => {
  if (mode.value === 'live') return liveUrl.value.replace(/^wss?:\/\//, '')
  return bagName.value || '未选择文件'
})

function useMode(next) {
  if (mode.value === next) return
  closeSource()
  mode.value = next
  bagName.value = ''
  attached.value = false
  error.value = ''
  clock.value = { text: null, fraction: null }
}

function applyLiveUrl() {
  localStorage.setItem(LIVE_URL_KEY, liveUrl.value)
  // Re-attach by reloading the frame with the new address.
  const current = liveUrl.value
  liveUrl.value = ''
  requestAnimationFrame(() => {
    liveUrl.value = current
  })
}

async function onFilePicked(event) {
  const file = event.target.files?.[0]
  event.target.value = ''
  if (!file) return
  error.value = ''
  busyLabel.value = `正在打开 ${file.name}…`
  try {
    await bridge.openLocalFile(file)
    await openBag(file)
    bagName.value = file.name
  } catch (nextError) {
    sourceState.value = 'error'
    error.value = `打开 Bag 失败：${nextError.message}`
  } finally {
    busyLabel.value = ''
  }
}

function onSceneReady() {
  stopClock?.()
  stopClock = bridge.watchClock((next) => {
    clock.value = next
  })
}

function onSceneAttached() {
  attached.value = true
  if (mode.value === 'live') openLive()
}

function onSceneError(nextError) {
  closeSource()
  error.value = nextError?.message || '三维场景启动失败'
}

function sourceTime() {
  if (!source.value) return 0
  if (source.value.kind === 'bag') return (clock.value.fraction ?? 0) * source.value.duration
  return sourceElapsed.value
}

function seriesFor(signal) {
  // The live decoder appends into arrays held by a Map. The tick is the
  // reactive invalidation point for those external mutations.
  sourceTick.value
  return source.value?.series?.get(signal.key) || { t: [], v: [] }
}

function latestValue(key) {
  const signal = SIGNALS.find(item => item.key === key)
  const series = signal ? seriesFor(signal) : { t: [], v: [] }
  return series.v.length ? series.v[series.v.length - 1] : undefined
}

function formatValue(value, unit = '') {
  if (value == null || !Number.isFinite(Number(value))) return '—'
  return `${Number(value).toFixed(unit === '°' ? 1 : 3)} ${unit}`.trim()
}

function signalValue(signal) {
  return formatValue(latestValue(signal.key), signal.unit)
}

function signalPath(signal) {
  const series = seriesFor(signal)
  if (series.t.length < 2) return ''
  const end = Math.max(sourceTime(), series.t[series.t.length - 1], 1e-9)
  const start = source.value?.kind === 'live' ? Math.max(0, end - 60) : 0
  const values = series.v.filter(value => Number.isFinite(value))
  if (!values.length) return ''
  const min = Math.min(...values)
  const max = Math.max(...values)
  const span = Math.max(max - min, 1e-9)
  const points = []
  const step = Math.max(1, Math.ceil(series.t.length / 160))
  for (let i = 0; i < series.t.length; i += step) {
    const time = series.t[i]
    const value = series.v[i]
    if (time < start || time > end || !Number.isFinite(value)) continue
    points.push(`${((time - start) / Math.max(end - start, 1e-9) * 640).toFixed(1)},${(56 - (value - min) / span * 48).toFixed(1)}`)
  }
  return points.length > 1 ? `M ${points.join(' L ')}` : ''
}

const localizationStatus = computed(() => {
  const value = latestValue('localization-status')
  return value == null ? null : Number(value)
})
const localizationStatusLabel = computed(() => (
  localizationStatus.value == null
    ? '未知'
    : STATUS_LABELS[localizationStatus.value] || `状态 ${localizationStatus.value}`
))
const matchConverged = computed(() => latestValue('match-converged') === 1)
const missingTopics = computed(() => source.value?.missing || [])
const topicCount = new Set(SIGNALS.map(signal => signal.topic)).size
const availableTopics = computed(() => {
  if (source.value?.kind === 'live') return `${source.value.counts?.size || 0}/${topicCount}`
  return `${topicCount - missingTopics.value.length}/${topicCount}`
})

function statusClass(value) {
  return value === 3 || value === 'ok' ? 'status-ok' : value === 'warn' ? 'status-warn' : 'status-bad'
}

// A source swap reloads the frame, so the old clock watcher is watching a
// document that no longer exists.
watch(src, () => {
  stopClock?.()
  stopClock = null
  closeSource()
  attached.value = false
})

onBeforeUnmount(() => {
  stopClock?.()
  closeSource()
})
</script>

<template>
  <main class="replay-page">
    <header class="page-head">
      <div>
        <p class="eyebrow">REPLAY DEBUG</p>
        <h1>回放调试台</h1>
        <p>三维场景、信号曲线与定位状态共用同一条时间轴，用于判读蛇形与漂移故障。</p>
      </div>
      <div class="source-switch">
        <button type="button" :class="{ active: mode === 'live' }" @click="useMode('live')">实时数据</button>
        <button type="button" :class="{ active: mode === 'bag' }" @click="useMode('bag')">本地 Bag</button>
      </div>
    </header>

    <p v-if="error" class="error-banner">{{ error }}</p>

    <PlaybackBar
      :bridge="bridge"
      :clock="clock"
      :source-kind="mode"
      :source-label="sourceLabel"
      :ready="attached"
    />

    <section class="workspace">
      <SceneFrame
        ref="scene"
        class="zone-scene"
        :bridge="bridge"
        :src="src"
        :busy-label="busyLabel"
        @ready="onSceneReady"
        @attached="onSceneAttached"
        @error="onSceneError"
      >
        <template #waiting>
          <template v-if="mode === 'bag'">
            <h3>选择一个本地 MCAP 文件</h3>
            <p>文件只在本机解析，不会上传到平台。</p>
            <button type="button" class="primary" @click="filePicker.click()">选择 .mcap 文件</button>
          </template>
          <template v-else>
            <h3>正在连接现场机器狗</h3>
            <p>{{ liveUrl }}</p>
            <div class="live-url">
              <input v-model="liveUrl" spellcheck="false" aria-label="现场 bridge 地址" />
              <button type="button" class="primary" @click="applyLiveUrl">重新连接</button>
            </div>
          </template>
        </template>
      </SceneFrame>

      <div class="side">
        <article class="zone-card zone-charts">
          <div class="card-title">
            <h2>信号曲线</h2>
            <small>位置 / 航向 · cmd_vel · RTK · 配准得分</small>
          </div>
          <div v-if="sourceState === 'loading'" class="pending">正在解析本地 MCAP（{{ Math.round(sourceProgress * 100) }}%）</div>
          <div v-else-if="sourceState === 'connecting'" class="pending">正在连接实时数据并等待话题…</div>
          <div v-else-if="sourceState === 'error'" class="pending error-text">{{ error }}</div>
          <div v-else-if="!source" class="pending">选择 MCAP 或连接实时数据后显示曲线</div>
          <div v-else class="signal-list" :data-tick="sourceTick">
            <div v-for="signal in SIGNALS.filter(item => ['localization-x', 'localization-y', 'heading', 'cmd-angular-z', 'raw-angular-z', 'ndt-error'].includes(item.key))" :key="signal.key" class="signal-row">
              <div class="signal-label"><span><i :style="{ background: signal.color }" />{{ signal.label }}</span><strong>{{ signalValue(signal) }}</strong></div>
              <svg viewBox="0 0 640 64" preserveAspectRatio="none" role="img" :aria-label="signal.label">
                <path :d="signalPath(signal)" fill="none" :stroke="signal.color" stroke-width="2" />
              </svg>
            </div>
          </div>
        </article>

        <article class="zone-card zone-status">
          <div class="card-title">
            <h2>定位状态</h2>
            <small>TF 树 · 原始消息 · 室内/室外</small>
          </div>
          <div v-if="!source" class="pending">选择 MCAP 或连接实时数据后显示定位状态</div>
          <div v-else class="status-list" :data-tick="sourceTick">
            <div><span>定位状态</span><strong :class="statusClass(localizationStatus)">{{ localizationStatusLabel }}</strong></div>
            <div><span>匹配状态</span><strong :class="statusClass(matchConverged ? 'ok' : 'warn')">{{ matchConverged ? '已收敛' : '未收敛' }}</strong></div>
            <div><span>NDT 内点率</span><strong>{{ formatValue(latestValue('ndt-inlier'), '%') }}</strong></div>
            <div><span>数据话题</span><strong>{{ availableTopics }}</strong></div>
            <small v-if="missingTopics.length">缺少：{{ missingTopics.join('、') }}</small>
          </div>
        </article>
      </div>
    </section>

    <input
      ref="filePicker" class="file-picker" type="file" accept=".mcap"
      aria-label="选择本地 MCAP 文件" @change="onFilePicked"
    />
  </main>
</template>

<style scoped>
.replay-page { display: grid; gap: 16px; padding: 4px; }
.page-head { display: flex; align-items: center; justify-content: space-between; gap: 16px; }
.page-head h1 { margin: 0; }
.page-head p { margin: 7px 0 0; color: var(--muted); }
.eyebrow { margin: 0; }
.source-switch { display: flex; gap: 6px; padding: 5px; border: 1px solid var(--line); border-radius: 14px; background: var(--panel-soft); }
.source-switch button { padding: 9px 16px; border: 0; border-radius: 10px; background: transparent; color: var(--muted); cursor: pointer; font: inherit; }
.source-switch button.active { background: var(--menu-active-bg); color: var(--cyan); font-weight: 600; }
.workspace { display: grid; grid-template-columns: minmax(0, 1.6fr) minmax(300px, 1fr); gap: 16px; min-height: min(720px, calc(100vh - 260px)); }
.side { display: grid; grid-template-rows: minmax(0, 1.35fr) minmax(0, 1fr); gap: 16px; min-height: 0; }
.zone-card { display: grid; grid-template-rows: auto minmax(0, 1fr); gap: 12px; padding: 18px 20px; overflow: hidden; border: 1px solid var(--line); border-radius: var(--radius-lg); background: var(--panel); }
.card-title { display: flex; align-items: baseline; justify-content: space-between; gap: 12px; }
.card-title h2 { margin: 0; font-size: 16px; }
.card-title small, .pending { color: var(--muted); font-size: 12px; }
.pending { display: grid; place-content: center; margin: 0; text-align: center; }
.error-text { color: var(--danger); }
.signal-list, .status-list { display: grid; gap: 10px; align-content: start; overflow: auto; }
.signal-row { display: grid; gap: 3px; }
.signal-label, .status-list > div { display: flex; align-items: center; justify-content: space-between; gap: 12px; color: var(--muted); font-size: 12px; }
.signal-label span { display: inline-flex; align-items: center; gap: 7px; }
.signal-label i { width: 8px; height: 8px; border-radius: 50%; }
.signal-label strong, .status-list strong { color: var(--text); font-family: "SFMono-Regular", ui-monospace, monospace; font-size: 12px; }
.signal-row svg { width: 100%; height: 42px; overflow: visible; border-bottom: 1px solid var(--line); background: linear-gradient(to bottom, transparent 49%, var(--line) 50%, transparent 51%); }
.status-list > div { padding: 7px 0; border-bottom: 1px solid var(--line); }
.status-list small { color: var(--muted); line-height: 1.5; }
.status-ok { color: var(--green) !important; }
.status-warn { color: var(--warning, #d97706) !important; }
.status-bad { color: var(--danger) !important; }
.primary { padding: 10px 16px; border: 0; border-radius: 12px; background: var(--cyan); color: #fff; cursor: pointer; font: inherit; }
.live-url { display: flex; gap: 8px; }
.live-url input { min-width: 260px; padding: 9px 12px; border: 1px solid var(--line); border-radius: 12px; background: var(--input-bg); color: var(--text); }
.error-banner { margin: 0; padding: 12px 14px; border-radius: 12px; background: rgba(255, 93, 93, 0.12); color: var(--danger); }
.file-picker { position: absolute; width: 1px; height: 1px; opacity: 0; pointer-events: none; }
@media (max-width: 1100px) {
  .workspace { grid-template-columns: 1fr; }
  .zone-scene { min-height: 420px; }
}
</style>
