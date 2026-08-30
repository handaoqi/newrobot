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

const LIVE_URL_KEY = 'replay_live_ws'
const defaultLiveUrl = `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.hostname}:8080/ws`

const scene = ref(null)
const filePicker = ref(null)
const bridge = createLichtblickBridge(() => scene.value?.frame)

const mode = ref('bag')
const liveUrl = ref(localStorage.getItem(LIVE_URL_KEY) || defaultLiveUrl)
const bagName = ref('')
const busyLabel = ref('')
const attached = ref(false)
const error = ref('')
const clock = ref({ text: null, fraction: null })

let stopClock = null

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
    bagName.value = file.name
  } catch (nextError) {
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

function onSceneError(nextError) {
  error.value = nextError.message
}

// A source swap reloads the frame, so the old clock watcher is watching a
// document that no longer exists.
watch(src, () => {
  stopClock?.()
  stopClock = null
  attached.value = false
})

onBeforeUnmount(() => stopClock?.())
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
        @attached="attached = true"
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
          <p class="pending">曲线区待接入本地解码通路（`rosStream.js`）。</p>
        </article>

        <article class="zone-card zone-status">
          <div class="card-title">
            <h2>定位状态</h2>
            <small>TF 树 · 原始消息 · 室内/室外</small>
          </div>
          <p class="pending">状态区待接入本地解码通路（`rosStream.js`）。</p>
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
