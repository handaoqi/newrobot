<script setup>
import Hls from 'hls.js'
import mpegts from 'mpegts.js'
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'

import { fetchRobotStreamAudioCommand, setRobotStreamAudioCapture } from '../services/api'

const props = defineProps({
  playUrls: { type: Object, default: () => ({}) },
  robotId: { type: [Number, String], default: null },
  available: { type: Boolean, default: true },
  loading: { type: Boolean, default: false },
  objectFit: { type: String, default: 'cover' },
})

const emit = defineEmits(['notice', 'stream-error'])

const videoRef = ref(null)
const streamUnavailable = ref(false)
const streamLoading = ref(false)
const liveAudioEnabled = ref(false)
const browserAudioMuted = ref(true)
const browserAudioVolume = ref(1)
const playbackMode = ref('live')
const historyPlaybackPaused = ref(false)
const historyOffsetSeconds = ref(30)

let flvPlayer = null
let hlsPlayer = null
let liveGuardTimer = null
let historySeekTimer = null
let historyManifestUrl = ''
let playerResetInProgress = false
let applyingBrowserAudio = false
let setupVersion = 0

const HISTORY_BUFFER_SECONDS = 30 * 60
const playUrls = computed(() => props.playUrls || {})
const sourceKey = computed(() => `${playUrls.value.flv || ''}\n${playUrls.value.hls || ''}`)
const hasHistoryStream = computed(() => Boolean(playUrls.value.hls))
const hasStream = computed(() => props.available && !streamUnavailable.value && Boolean(playUrls.value.flv || playUrls.value.hls))
const isHistoryPlayback = computed(() => playbackMode.value === 'history')
const isFrozenPlayback = computed(() => playbackMode.value === 'paused' || isHistoryPlayback.value)
const historyPlaybackStatus = computed(() => {
  if (isHistoryPlayback.value) return historyPlaybackPaused.value ? '历史回放已暂停' : '历史回放 · 00:00 起播'
  if (playbackMode.value === 'paused') return historyPlaybackPaused.value ? '直播已暂停，可拖动播放条' : '暂停片段播放中，可拖动播放条'
  return '实时直播'
})

function notify(message, variant = 'info') {
  emit('notice', { message, variant })
}

function applyBrowserAudio(element, { muted = browserAudioMuted.value, volume = browserAudioVolume.value } = {}) {
  if (!element) return
  applyingBrowserAudio = true
  element.muted = muted
  element.volume = volume
  applyingBrowserAudio = false
}

function handleBrowserAudioChange(event) {
  if (applyingBrowserAudio) return
  const element = event.currentTarget
  browserAudioMuted.value = element.muted
  browserAudioVolume.value = element.volume
}

async function setLiveAudio(enabled) {
  if (!props.robotId) {
    notify('当前没有可控制的机器人音频采集', 'alert')
    return
  }
  try {
    const command = await setRobotStreamAudioCapture(props.robotId, enabled)
    if (enabled && command?.id) {
      for (let attempt = 0; attempt < 10; attempt += 1) {
        await new Promise((resolve) => window.setTimeout(resolve, 800))
        const result = await fetchRobotStreamAudioCommand(props.robotId, command.id)
        if (result?.status === 'failed') throw new Error(result.error_message || '麦克风未启动，请打开音响电源后重试')
        if (result?.status === 'finished') break
        if (attempt === 9) throw new Error('现场收音启动超时，请打开音响电源后重试')
      }
    }
    liveAudioEnabled.value = enabled
    notify(enabled ? '已请求开启 NX 现场音频采集' : '已请求关闭 NX 现场音频采集')
  } catch (error) {
    notify(error.message || 'NX 现场音频采集控制失败', 'alert')
  }
}

function releaseHistoryManifest() {
  if (!historyManifestUrl) return
  URL.revokeObjectURL(historyManifestUrl)
  historyManifestUrl = ''
}

function stopLiveGuard() {
  if (!liveGuardTimer) return
  window.clearInterval(liveGuardTimer)
  liveGuardTimer = null
}

function seekLatestFrame() {
  const element = videoRef.value
  if (!element) return
  const ranges = element.buffered
  if (ranges?.length) {
    const liveEnd = ranges.end(ranges.length - 1)
    if (Number.isFinite(liveEnd) && liveEnd - element.currentTime > 0.8) {
      element.currentTime = Math.max(0, liveEnd - 0.12)
    }
  } else if (Number.isFinite(element.duration) && element.duration > 0 && element.duration - element.currentTime > 0.8) {
    element.currentTime = Math.max(0, element.duration - 0.12)
  }
}

function keepLivePlaying() {
  if (playbackMode.value !== 'live') return
  const element = videoRef.value
  if (!element) return
  seekLatestFrame()
  if (element.paused) element.play().catch(() => {})
}

function startLiveGuard() {
  if (playbackMode.value !== 'live') return
  stopLiveGuard()
  keepLivePlaying()
  liveGuardTimer = window.setInterval(keepLivePlaying, 800)
}

function destroyPlayers() {
  if (historySeekTimer) {
    window.clearTimeout(historySeekTimer)
    historySeekTimer = null
  }
  stopLiveGuard()
  const element = videoRef.value
  const replacingAttachedPlayer = Boolean(element && (flvPlayer || hlsPlayer || element.currentSrc))
  if (replacingAttachedPlayer) playerResetInProgress = true
  flvPlayer?.destroy()
  hlsPlayer?.destroy()
  flvPlayer = null
  hlsPlayer = null
  releaseHistoryManifest()
  if (element) {
    element.removeAttribute('src')
    element.load()
  }
  if (replacingAttachedPlayer) {
    window.setTimeout(() => { playerResetInProgress = false }, 250)
  }
}

function markStreamUnavailable() {
  streamUnavailable.value = true
  streamLoading.value = false
  destroyPlayers()
  emit('stream-error')
}

function playbackRange(element) {
  const ranges = element?.seekable?.length ? element.seekable : element?.buffered
  if (!ranges?.length) return null
  const start = ranges.start(0)
  const end = ranges.end(ranges.length - 1)
  return Number.isFinite(start) && Number.isFinite(end) && end > start ? { start, end } : null
}

async function freezeHistoryManifest(hlsUrl) {
  const response = await fetch(hlsUrl, { cache: 'no-store' })
  if (!response.ok) throw new Error(`历史播放清单读取失败（${response.status}）`)
  const playlist = await response.text()
  const lines = playlist
    .split(/\r?\n/)
    .map((line) => (line && !line.startsWith('#') ? new URL(line, hlsUrl).href : line))
    .filter((line) => line !== '')
  if (!lines.includes('#EXT-X-ENDLIST')) lines.push('#EXT-X-ENDLIST')
  const blob = new Blob([`${lines.join('\n')}\n`], { type: 'application/vnd.apple.mpegurl' })
  historyManifestUrl = URL.createObjectURL(blob)
  return historyManifestUrl
}

function applyHistoryPosition(element, attempts = 0) {
  if (!isFrozenPlayback.value || !element) return
  const range = playbackRange(element)
  if (!range) {
    if (attempts < 12) historySeekTimer = window.setTimeout(() => applyHistoryPosition(element, attempts + 1), 250)
    return
  }
  const target = historyOffsetSeconds.value >= HISTORY_BUFFER_SECONDS
    ? range.start
    : Math.max(range.start, range.end - historyOffsetSeconds.value)
  historyOffsetSeconds.value = Math.max(0, Math.round(range.end - target))
  hlsPlayer?.startLoad?.(target)
  element.currentTime = target
  if (playbackMode.value === 'paused' || historyPlaybackPaused.value) {
    const pauseAfterFrame = () => element.pause()
    element.addEventListener('canplay', pauseAfterFrame, { once: true })
  }
  element.play().then(() => applyBrowserAudio(element)).catch(() => {})
}

async function startHistoryPlayback(offsetSeconds = HISTORY_BUFFER_SECONDS, { paused = false } = {}) {
  if (!hasHistoryStream.value) {
    notify('历史播放需要 HLS 视频流', 'alert')
    return
  }
  historyOffsetSeconds.value = Math.min(HISTORY_BUFFER_SECONDS, Math.max(0, Number(offsetSeconds) || 0))
  historyPlaybackPaused.value = paused
  playbackMode.value = 'history'
  await setupPlayer()
}

async function pauseLivePlayback({ notifyUser = true } = {}) {
  if (!hasHistoryStream.value) {
    if (notifyUser) notify('暂停需要 HLS 视频流', 'alert')
    return
  }
  if (playbackMode.value === 'paused') return
  stopLiveGuard()
  historyOffsetSeconds.value = 0
  historyPlaybackPaused.value = true
  playbackMode.value = 'paused'
  await setupPlayer()
  if (notifyUser) notify('直播已暂停，片段已冻结，可拖动播放条')
}

async function returnToLive() {
  if (playbackMode.value === 'live') return
  playbackMode.value = 'live'
  historyPlaybackPaused.value = false
  await setupPlayer()
  notify('已返回实时画面')
}

function handleVideoPause() {
  if (playerResetInProgress) return
  if (isFrozenPlayback.value) {
    historyPlaybackPaused.value = true
    return
  }
  // Browsers pause media during route changes, background throttling, and
  // source replacement. Only the explicit page-level pause control may enter
  // frozen playback; a live source should resume at its current edge.
  if (playbackMode.value === 'live') window.setTimeout(keepLivePlaying, 0)
}

function handleVideoPlay() {
  if (isFrozenPlayback.value) {
    historyPlaybackPaused.value = false
    return
  }
  if (playbackMode.value === 'live') startLiveGuard()
}

async function setupPlayer() {
  const version = ++setupVersion
  streamLoading.value = true
  await nextTick()
  if (version !== setupVersion) return
  destroyPlayers()
  const element = videoRef.value
  if (!element || !hasStream.value) {
    streamLoading.value = false
    return
  }
  applyBrowserAudio(element, { muted: true, volume: browserAudioVolume.value })
  const { flv, hls } = playUrls.value
  try {
    if (!isFrozenPlayback.value && flv && mpegts.getFeatureList().mseLivePlayback) {
      flvPlayer = mpegts.createPlayer({ type: 'flv', isLive: true, url: flv }, {
        enableStashBuffer: false,
        lazyLoad: false,
        liveSync: true,
        liveSyncMaxLatency: 1.0,
        liveSyncTargetLatency: 0.35,
        liveSyncPlaybackRate: 1.75,
        liveBufferLatencyChasing: true,
        liveBufferLatencyMaxLatency: 3.0,
        liveBufferLatencyMinRemain: 0.35,
      })
      flvPlayer.on(mpegts.Events.ERROR, markStreamUnavailable)
      flvPlayer.attachMediaElement(element)
      flvPlayer.load()
      await element.play()
      applyBrowserAudio(element)
      startLiveGuard()
      streamLoading.value = false
      return
    }
    if (hls && Hls.isSupported()) {
      const frozen = isFrozenPlayback.value
      const hlsSource = frozen ? await freezeHistoryManifest(hls) : hls
      if (version !== setupVersion) return
      hlsPlayer = new Hls({
        lowLatencyMode: !frozen,
        startPosition: frozen ? 0 : -1,
        liveSyncDurationCount: 1,
        liveMaxLatencyDurationCount: 2,
        maxLiveSyncPlaybackRate: frozen ? 1 : 1.75,
      })
      hlsPlayer.on(Hls.Events.ERROR, (_event, data) => {
        if (data?.fatal) markStreamUnavailable()
      })
      hlsPlayer.on(Hls.Events.MANIFEST_PARSED, () => {
        if (frozen) applyHistoryPosition(element)
        else element.play().then(() => applyBrowserAudio(element)).then(startLiveGuard).then(() => { streamLoading.value = false }).catch(markStreamUnavailable)
        if (frozen) streamLoading.value = false
      })
      hlsPlayer.loadSource(hlsSource)
      hlsPlayer.attachMedia(element)
      return
    }
    if (hls) {
      element.src = isFrozenPlayback.value ? await freezeHistoryManifest(hls) : hls
      element.addEventListener('loadedmetadata', () => {
        if (isFrozenPlayback.value) applyHistoryPosition(element)
        else element.play().then(() => applyBrowserAudio(element)).then(startLiveGuard).then(() => { streamLoading.value = false }).catch(markStreamUnavailable)
        if (isFrozenPlayback.value) streamLoading.value = false
      }, { once: true })
      return
    }
    markStreamUnavailable()
  } catch (error) {
    if (version === setupVersion) {
      notify(error.message || '视频流播放失败', 'alert')
      markStreamUnavailable()
    }
  }
}

watch(sourceKey, (nextSource, previousSource) => {
  if (nextSource === previousSource) return
  streamUnavailable.value = false
  playbackMode.value = 'live'
  historyPlaybackPaused.value = false
  void setupPlayer()
})

watch(() => props.available, (available) => {
  if (!available) destroyPlayers()
  else {
    streamUnavailable.value = false
    void setupPlayer()
  }
})

watch(() => props.robotId, () => {
  liveAudioEnabled.value = false
})

onMounted(() => { void setupPlayer() })
onBeforeUnmount(destroyPlayers)

defineExpose({ returnToLive })
</script>

<template>
  <div class="live-video-player">
    <video
      v-if="hasStream && !loading && !streamLoading"
      ref="videoRef"
      class="live-video-player__video"
      :style="{ objectFit }"
      playsinline
      autoplay
      controls
      @pause="handleVideoPause"
      @play="handleVideoPlay"
      @volumechange="handleBrowserAudioChange"
    ></video>
    <slot v-else-if="loading || streamLoading" name="loading">
      <div class="live-video-player__empty">
        <strong>正在加载视频流</strong>
        <span>页面已就绪，正在连接现场画面</span>
      </div>
    </slot>
    <slot v-else name="empty">
      <div class="live-video-player__empty">
        <strong>视频暂不可用</strong>
      </div>
    </slot>

    <slot name="overlay"></slot>

    <button
      v-if="hasStream && !loading && !streamLoading"
      type="button"
      class="live-video-player__listen-toggle"
      :class="{ active: liveAudioEnabled }"
      @click="setLiveAudio(!liveAudioEnabled)"
    >
      {{ liveAudioEnabled ? '关闭现场收音' : '开启现场收音' }}
    </button>

    <div v-if="hasStream && !loading && !streamLoading" class="live-video-player__playback-controls" aria-label="视频播放控制">
      <button :class="{ active: playbackMode === 'live' }" @click="returnToLive">直播</button>
      <button :class="{ active: playbackMode === 'paused' }" :disabled="!hasHistoryStream" @click="pauseLivePlayback">暂停</button>
      <button :class="{ active: isHistoryPlayback }" :disabled="!hasHistoryStream" @click="startHistoryPlayback()">回放</button>
      <small>{{ historyPlaybackStatus }}</small>
    </div>
  </div>
</template>

<style scoped>
.live-video-player { position: absolute; inset: 0; overflow: hidden; }
.live-video-player__video, .live-video-player__empty { display: block; width: 100%; height: 100%; }
.live-video-player__video { background: #06111f; }
.live-video-player__empty { display: grid; place-content: center; gap: 8px; color: #d7e0e6; text-align: center; background: #152633; }
.live-video-player__listen-toggle { position: absolute; z-index: 12; top: 18px; right: 18px; min-height: 42px; padding: 0 16px; border: 1px solid rgba(255, 255, 255, .4); color: #fff; background: rgba(10, 29, 41, .82); font: inherit; font-weight: 800; cursor: pointer; }
.live-video-player__listen-toggle.active { border-color: #52d99c; background: rgba(16, 110, 73, .9); }
.live-video-player__playback-controls { position: absolute; z-index: 12; top: 18px; left: 50%; display: flex; align-items: center; justify-content: center; gap: 7px; padding: 9px; color: #fff; background: rgba(10, 29, 41, .82); transform: translateX(-50%); }
.live-video-player__playback-controls button { min-height: 32px; padding: 0 10px; border: 1px solid rgba(255, 255, 255, .36); color: #fff; background: rgba(27, 62, 81, .9); font: inherit; font-size: 12px; font-weight: 800; cursor: pointer; }
.live-video-player__playback-controls button:hover { background: rgba(42, 99, 128, .96); }
.live-video-player__playback-controls button.active { border-color: #52d99c; background: rgba(16, 110, 73, .92); }
.live-video-player__playback-controls button:disabled { cursor: not-allowed; opacity: .48; }
.live-video-player__playback-controls small { position: absolute; top: calc(100% + 5px); left: 50%; width: max-content; max-width: 260px; padding: 4px 7px; color: #dbe8ee; background: rgba(10, 29, 41, .72); font-size: 11px; transform: translateX(-50%); }

@media (max-width: 720px) {
  .live-video-player__listen-toggle { top: 12px; right: 12px; min-height: 36px; padding: 0 10px; font-size: 12px; }
  .live-video-player__playback-controls { top: 12px; padding: 6px; }
  .live-video-player__playback-controls button { min-height: 30px; padding: 0 8px; }
}
</style>
