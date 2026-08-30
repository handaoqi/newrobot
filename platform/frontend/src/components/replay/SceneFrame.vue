<script setup>
/**
 * Zone 2: the 3D scene.
 *
 * This is a Lichtblick instance served from our own origin at /foxglove/, with
 * its entire interface stripped away so only the WebGL canvas remains. It is a
 * rendering engine here, not an application: every control the operator touches
 * lives in our own Vue components and drives this frame through
 * services/lichtblickBridge.js.
 *
 * Two things the host shows that CSS cannot reach are covered by our own overlay
 * instead: the boot splash and the data-source dialog, both of which carry
 * upstream branding. Covering them in *our* DOM is simpler than fighting the
 * host's, and it is what the operator should see anyway -- a RoamerX panel
 * saying what the scene is waiting for.
 */
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'

const props = defineProps({
  /** Bridge from the page; it reads the frame we expose below. */
  bridge: { type: Object, required: true },
  /** Host URL including any ?ds= data-source parameters. */
  src: { type: String, required: true },
  /** Shown over the scene while the page is busy with something of its own. */
  busyLabel: { type: String, default: '' },
})

const emit = defineEmits(['ready', 'attached', 'error'])

const frame = ref(null)
/** booting -> the host shell is still loading; waiting -> up, but no data yet. */
const phase = ref('booting')
const failure = ref('')
let keepHidden = null

/**
 * Re-hide the chrome on a timer rather than once on load.
 *
 * The host re-opens its left sidebar whenever a data source attaches, and pops
 * it open again on its own for problem notifications. hideChrome() is idempotent
 * -- the stylesheet is inserted once and the sidebar toggles are only clicked
 * when their aria-label says the sidebar is open -- so this settles to a single
 * cheap querySelector per second.
 */
function startKeepingChromeHidden() {
  stopKeepingChromeHidden()
  keepHidden = window.setInterval(() => {
    try {
      props.bridge.hideChrome()
      const waiting = props.bridge.needsSource()
      if (!waiting && phase.value !== 'ready') {
        phase.value = 'ready'
        emit('attached')
      } else if (waiting && phase.value === 'ready') {
        phase.value = 'waiting'
      }
    } catch {
      // The frame is mid-navigation. onLoad will run again.
    }
  }, 1000)
}

function stopKeepingChromeHidden() {
  if (keepHidden != null) window.clearInterval(keepHidden)
  keepHidden = null
}

async function onLoad() {
  phase.value = 'booting'
  failure.value = ''
  try {
    await props.bridge.ready({ timeout: 120_000 })
    props.bridge.hideChrome()
    phase.value = props.bridge.needsSource() ? 'waiting' : 'ready'
    if (phase.value === 'ready') emit('attached')
    startKeepingChromeHidden()
    emit('ready')
  } catch (error) {
    failure.value = error.message
    emit('error', error)
  }
}

// Changing the data source means changing the host URL, which is a full reload:
// drop back to the boot overlay so the host's own splash never shows through.
watch(() => props.src, () => {
  phase.value = 'booting'
  stopKeepingChromeHidden()
})

onMounted(() => {
  // A cached frame can finish loading before this component mounts, in which
  // case the load event already fired and will not fire again.
  if (frame.value?.contentDocument?.readyState === 'complete') onLoad()
})

onBeforeUnmount(stopKeepingChromeHidden)

defineExpose({ frame })
</script>

<template>
  <section class="scene-frame" data-testid="replay-scene">
    <iframe
      ref="frame"
      class="scene-host"
      :src="src"
      title="三维场景"
      @load="onLoad"
    />

    <div v-if="failure" class="scene-overlay error">
      <p class="eyebrow">场景引擎</p>
      <h3>三维场景未能启动</h3>
      <p>{{ failure }}</p>
    </div>
    <div v-else-if="busyLabel" class="scene-overlay">
      <span class="spinner" />
      <h3>{{ busyLabel }}</h3>
    </div>
    <div v-else-if="phase === 'booting'" class="scene-overlay">
      <span class="spinner" />
      <h3>正在装载三维场景…</h3>
      <p>首次打开需要解压约 170 MB 渲染资源</p>
    </div>
    <div v-else-if="phase === 'waiting'" class="scene-overlay">
      <p class="eyebrow">等待数据</p>
      <slot name="waiting">
        <h3>尚未接入数据源</h3>
      </slot>
    </div>

    <div v-if="phase === 'ready'" class="scene-legend">
      <span><i style="background: #e5484d" />定位轨迹</span>
      <span><i style="background: #3b82f6" />Nav2 轨迹</span>
      <span><i style="background: #22c55e" />底盘里程计</span>
      <span><i style="background: #eab308" />配准位姿</span>
      <span><i style="background: #8a94a6" />规划路径</span>
    </div>
  </section>
</template>

<style scoped>
.scene-frame { position: relative; overflow: hidden; border: 1px solid var(--line); border-radius: var(--radius-lg); background: var(--stage-bg); }
.scene-host { display: block; width: 100%; height: 100%; border: 0; background: transparent; }
.scene-overlay { position: absolute; inset: 0; display: grid; gap: 10px; align-content: center; justify-items: center; padding: 24px; text-align: center; background: var(--overlay-bg); backdrop-filter: blur(6px); }
.scene-overlay h3 { margin: 0; font-size: 17px; color: var(--text); }
.scene-overlay p { margin: 0; color: var(--muted); font-size: 13px; }
.scene-overlay.error h3 { color: var(--danger); }
.spinner { width: 26px; height: 26px; border: 2px solid var(--chip-border); border-top-color: var(--cyan); border-radius: 50%; animation: scene-spin 0.9s linear infinite; }
@keyframes scene-spin { to { transform: rotate(360deg); } }
.scene-legend { position: absolute; left: 14px; bottom: 12px; display: flex; flex-wrap: wrap; gap: 6px 14px; padding: 8px 12px; border: 1px solid var(--line); border-radius: 12px; background: var(--overlay-bg); font-size: 12px; color: var(--text); pointer-events: none; }
.scene-legend span { display: inline-flex; align-items: center; gap: 6px; }
.scene-legend i { width: 14px; height: 3px; border-radius: 2px; }
@media (prefers-reduced-motion: reduce) { .spinner { animation-duration: 3s; } }
</style>
