<script setup>
/**
 * Zone 1: the transport bar.
 *
 * There is exactly one playhead on this page and it belongs to the host's
 * player. This bar owns no time of its own -- every button here reaches into the
 * frame through the bridge, and everything it displays is read back out of the
 * host. That is what keeps the 3D scene and our own charts from drifting apart,
 * and it is why live and replay need no separate synchronisation logic.
 */
import { computed, onBeforeUnmount, ref, watch } from 'vue'

import { SPEEDS } from '../../services/lichtblickBridge'

const props = defineProps({
  bridge: { type: Object, required: true },
  /** { text, fraction } as last read from the host. */
  clock: { type: Object, default: () => ({ text: null, fraction: null }) },
  /** Host label for the attached source, e.g. "本地 Bag · nav_sway.mcap". */
  sourceLabel: { type: String, default: '' },
  sourceKind: { type: String, default: 'bag' },
  /** False until a data source is attached; the host has no controls before then. */
  ready: { type: Boolean, default: false },
})

const speeds = SPEEDS
const speed = ref(1)
const loop = ref(false)
const playing = ref(false)
const scrubbing = ref(false)
const track = ref(null)

/**
 * Whether the host is playing.
 *
 * The host exposes no play state -- its button is a single toggle with no
 * aria-pressed -- so this is inferred from the clock advancing. The click sets it
 * optimistically so the icon responds immediately; the two watchers below correct
 * it if the host disagreed (end of recording, a seek that auto-paused).
 */
let lastMoved = 0
watch(
  () => props.clock.text,
  () => {
    lastMoved = Date.now()
    if (!playing.value) playing.value = true
  },
)

const stall = window.setInterval(() => {
  if (playing.value && Date.now() - lastMoved > 900) playing.value = false
  // Looping is ours, not the host's: the host's own repeat control is not part
  // of the verified selector set, and re-seeking to zero needs nothing beyond
  // the primitives we already trust.
  if (loop.value && props.clock.fraction != null && props.clock.fraction > 0.999) {
    props.bridge.seekFraction(0)
  }
}, 300)

onBeforeUnmount(() => window.clearInterval(stall))

/**
 * The host formats its clock in US style ("7:37:18.261 PM CST"). Reformat to
 * 24-hour so no upstream convention shows through; fall back to the raw string
 * if the host ever changes the format, since a wrong time is worse than a
 * foreign-looking one.
 */
const time = computed(() => {
  const raw = props.clock.text
  if (!raw) return { clock: '--:--:--.---', date: '' }
  const match = /^(\S+)\s+(\d{1,2}):(\d{2}):(\d{2}(?:\.\d+)?)\s*(AM|PM)?/i.exec(raw)
  if (!match) return { clock: raw, date: '' }
  const [, date, hour, minute, second, meridiem] = match
  let hours = Number(hour)
  if (meridiem) {
    const pm = meridiem.toUpperCase() === 'PM'
    hours = (hours % 12) + (pm ? 12 : 0)
  }
  const seconds = Number(second).toFixed(3).padStart(6, '0')
  return { clock: `${String(hours).padStart(2, '0')}:${minute}:${seconds}`, date }
})

const percent = computed(() => Math.round((props.clock.fraction ?? 0) * 1000) / 10)

function togglePlay() {
  if (props.bridge.togglePlay()) playing.value = !playing.value
}

function pick(next) {
  speed.value = next
  props.bridge.setSpeed(next)
}

function fractionAt(event) {
  const rect = track.value.getBoundingClientRect()
  if (!(rect.width > 0)) return 0
  return Math.min(1, Math.max(0, (event.clientX - rect.left) / rect.width))
}

function scrubStart(event) {
  if (!props.ready) return
  scrubbing.value = true
  track.value.setPointerCapture(event.pointerId)
  props.bridge.seekFraction(fractionAt(event))
}

function scrubMove(event) {
  if (!scrubbing.value) return
  props.bridge.seekFraction(fractionAt(event))
}

function scrubEnd(event) {
  if (!scrubbing.value) return
  scrubbing.value = false
  try {
    track.value.releasePointerCapture(event.pointerId)
  } catch {
    // The pointer was already released (the frame took the capture back).
  }
}

/** Keyboard scrubbing: the bar has to be usable without a pointer. */
function nudge(delta) {
  const from = props.clock.fraction ?? 0
  props.bridge.seekFraction(from + delta)
}
</script>

<template>
  <header class="replay-bar" data-testid="replay-bar">
    <div class="source" :class="sourceKind">
      <i />
      <span>
        <small>{{ sourceKind === 'live' ? '实时数据' : '本地回放' }}</small>
        <strong>{{ sourceLabel || '未接入' }}</strong>
      </span>
    </div>

    <div class="transport">
      <button
        type="button" title="上一帧" data-testid="replay-step-back"
        :disabled="!ready" @click="bridge.stepBackward()"
      >⏮</button>
      <button
        type="button" class="play" :title="playing ? '暂停' : '播放'"
        data-testid="replay-play" :disabled="!ready" @click="togglePlay"
      >{{ playing ? '⏸' : '▶' }}</button>
      <button
        type="button" title="下一帧" data-testid="replay-step-forward"
        :disabled="!ready" @click="bridge.stepForward()"
      >⏭</button>
    </div>

    <div class="speeds">
      <button
        v-for="item in speeds" :key="item" type="button"
        class="speed" :class="{ active: speed === item }"
        :data-testid="`replay-speed-${item}`" :disabled="!ready" @click="pick(item)"
      >{{ item }}×</button>
    </div>

    <div
      ref="track" class="scrub" role="slider" tabindex="0"
      aria-label="回放进度" aria-valuemin="0" aria-valuemax="100" :aria-valuenow="percent"
      data-testid="replay-scrub"
      :class="{ idle: !ready }"
      @pointerdown="scrubStart" @pointermove="scrubMove"
      @pointerup="scrubEnd" @pointercancel="scrubEnd"
      @keydown.left.prevent="nudge(-0.01)" @keydown.right.prevent="nudge(0.01)"
      @keydown.home.prevent="bridge.seekFraction(0)"
    >
      <i class="fill" :style="{ width: `${percent}%` }" />
      <i class="knob" :style="{ left: `${percent}%` }" />
    </div>

    <div class="clock" data-testid="replay-clock">
      <strong>{{ time.clock }}</strong>
      <small>{{ time.date || '等待时钟' }}</small>
    </div>

    <button
      type="button" class="loop" :class="{ active: loop }" data-testid="replay-loop"
      :disabled="!ready" :aria-pressed="loop" @click="loop = !loop"
    >循环</button>

    <button
      type="button" class="loop" data-testid="replay-reset-view"
      :disabled="!ready" @click="bridge.resetView()"
    >复位视角</button>
  </header>
</template>

<style scoped>
.replay-bar { display: flex; align-items: center; gap: 16px; padding: 12px 18px; border: 1px solid var(--line); border-radius: var(--radius-lg); background: var(--panel); box-shadow: var(--shadow); }
.source { display: flex; align-items: center; gap: 10px; min-width: 190px; }
.source i { width: 9px; height: 9px; border-radius: 50%; background: var(--muted); }
.source.live i { background: var(--green); box-shadow: 0 0 0 4px rgba(31, 191, 120, 0.16); }
.source.bag i { background: var(--cyan); box-shadow: 0 0 0 4px rgba(37, 136, 255, 0.14); }
.source span { display: grid; gap: 2px; min-width: 0; }
.source small { color: var(--muted); font-size: 11px; letter-spacing: 0.08em; }
.source strong { overflow: hidden; font-size: 14px; color: var(--text); text-overflow: ellipsis; white-space: nowrap; }
.transport, .speeds { display: flex; align-items: center; gap: 6px; }
.transport button, .speed, .loop { border: 1px solid var(--line); border-radius: 12px; background: var(--input-bg); color: var(--text); cursor: pointer; font: inherit; }
.transport button { width: 40px; height: 40px; font-size: 15px; }
.transport .play { width: 48px; border-color: transparent; background: var(--cyan); color: #fff; font-size: 17px; }
.speed, .loop { padding: 8px 12px; font-size: 13px; }
.speed.active, .loop.active { border-color: var(--menu-active-border); background: var(--menu-active-bg); color: var(--cyan); }
.transport button:disabled, .speed:disabled, .loop:disabled { opacity: 0.4; cursor: not-allowed; }
.scrub { position: relative; flex: 1; min-width: 120px; height: 26px; cursor: pointer; touch-action: none; }
.scrub.idle { cursor: default; opacity: 0.5; }
.scrub::before { content: ""; position: absolute; top: 11px; right: 0; left: 0; height: 4px; border-radius: 99px; background: var(--chip-bg); }
.fill { position: absolute; top: 11px; left: 0; height: 4px; border-radius: 99px; background: linear-gradient(90deg, var(--cyan), #6fd0ff); }
.knob { position: absolute; top: 7px; width: 12px; height: 12px; margin-left: -6px; border: 2px solid var(--cyan); border-radius: 50%; background: #fff; }
.scrub:focus-visible { outline: 2px solid var(--cyan); outline-offset: 4px; border-radius: 8px; }
.clock { display: grid; gap: 2px; min-width: 104px; text-align: right; }
.clock strong { font-family: "SFMono-Regular", ui-monospace, monospace; font-size: 16px; color: var(--text); }
.clock small { color: var(--muted); font-size: 11px; }
@media (max-width: 1100px) {
  .replay-bar { flex-wrap: wrap; }
  .scrub { order: 9; flex-basis: 100%; }
}
</style>
