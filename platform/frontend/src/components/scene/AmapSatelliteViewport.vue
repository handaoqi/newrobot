<script setup>
import { computed, onBeforeUnmount, onMounted, reactive, ref, watch } from 'vue'

import { mapPointToWgs84 } from '../../services/sceneData'

const TILE_SIZE = 256
const DEFAULT_ZOOM = 18
const MIN_ZOOM = 3
const MAX_ZOOM = 20
const DEFAULT_TILE_URL = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'

const props = defineProps({
  geoReference: { type: Object, default: null },
  robotPose: { type: Object, default: null },
  trail: { type: Array, default: () => [] },
  waypoints: { type: Array, default: () => [] },
})

const host = ref(null)
const state = ref('idle')
const message = ref('')
const zoom = ref(DEFAULT_ZOOM)
const centerPixels = ref(null)
const tileUrl = ref(DEFAULT_TILE_URL)
const tileError = ref(false)
const viewport = reactive({ width: 1, height: 1 })
const drag = reactive({ active: false, x: 0, y: 0 })
let mounted = false
let resizeObserver

function worldSize(level = zoom.value) {
  return TILE_SIZE * (2 ** level)
}

function project(point, level = zoom.value) {
  const latitude = Math.max(-85.05112878, Math.min(85.05112878, Number(point?.latitude)))
  const longitude = Number(point?.longitude)
  if (![latitude, longitude].every(Number.isFinite)) return null
  const size = worldSize(level)
  const sine = Math.sin(latitude * Math.PI / 180)
  return {
    x: (longitude + 180) / 360 * size,
    y: (0.5 - Math.log((1 + sine) / (1 - sine)) / (4 * Math.PI)) * size,
  }
}

function unproject(pixel, level = zoom.value) {
  if (!pixel) return null
  const size = worldSize(level)
  return {
    longitude: pixel.x / size * 360 - 180,
    latitude: 180 / Math.PI * Math.atan(Math.sinh(Math.PI * (1 - 2 * pixel.y / size))),
  }
}

function normalizedCenter(pixel, level = zoom.value) {
  const size = worldSize(level)
  return {
    x: ((pixel.x % size) + size) % size,
    y: Math.max(0, Math.min(size, pixel.y)),
  }
}

const topLeft = computed(() => centerPixels.value ? {
  x: centerPixels.value.x - viewport.width / 2,
  y: centerPixels.value.y - viewport.height / 2,
} : { x: 0, y: 0 })

const tiles = computed(() => {
  if (!centerPixels.value) return []
  const tileCount = worldSize() / TILE_SIZE
  const firstX = Math.floor(topLeft.value.x / TILE_SIZE) - 1
  const lastX = Math.floor((topLeft.value.x + viewport.width) / TILE_SIZE) + 1
  const firstY = Math.max(0, Math.floor(topLeft.value.y / TILE_SIZE) - 1)
  const lastY = Math.min(tileCount - 1, Math.floor((topLeft.value.y + viewport.height) / TILE_SIZE) + 1)
  const result = []
  for (let x = firstX; x <= lastX; x += 1) {
    for (let y = firstY; y <= lastY; y += 1) {
      result.push({
        key: `${zoom.value}:${x}:${y}`,
        x: x * TILE_SIZE - topLeft.value.x,
        y: y * TILE_SIZE - topLeft.value.y,
        src: tileUrl.value
          .replaceAll('{z}', String(zoom.value))
          .replaceAll('{x}', String(((x % tileCount) + tileCount) % tileCount))
          .replaceAll('{y}', String(y)),
      })
    }
  }
  return result
})

function screenPoint(point) {
  const pixel = project(mapPointToWgs84(point, props.geoReference))
  return pixel ? { x: pixel.x - topLeft.value.x, y: pixel.y - topLeft.value.y } : null
}

const routePoints = computed(() => props.waypoints.map(screenPoint).filter(Boolean))
const trailPoints = computed(() => props.trail.map(screenPoint).filter(Boolean))
const robotPoint = computed(() => screenPoint(props.robotPose))
const pointsString = points => points.map(point => `${point.x.toFixed(1)},${point.y.toFixed(1)}`).join(' ')
const routePath = computed(() => pointsString(routePoints.value))
const trailPath = computed(() => pointsString(trailPoints.value))
const centerGeo = computed(() => unproject(centerPixels.value))

async function loadConfig() {
  try {
    const response = await fetch('/scene-map-config.json', { cache: 'no-store' })
    if (!response.ok) return
    const value = await response.json()
    const configured = value?.satellite?.tileUrl
    if (typeof configured === 'string' && configured.includes('{z}') && configured.includes('{x}') && configured.includes('{y}')) {
      tileUrl.value = configured
    }
  } catch {
    // The built-in endpoint keeps the MVP usable without runtime configuration.
  }
}

function resize() {
  if (!host.value) return
  const bounds = host.value.getBoundingClientRect()
  viewport.width = Math.max(1, bounds.width)
  viewport.height = Math.max(1, bounds.height)
}

function recenter() {
  const pixel = project(mapPointToWgs84({ x: 0, y: 0 }, props.geoReference))
  if (pixel) centerPixels.value = normalizedCenter(pixel)
}

function zoomTo(nextZoom, anchorX = viewport.width / 2, anchorY = viewport.height / 2) {
  if (!centerPixels.value) return
  const targetZoom = Math.max(MIN_ZOOM, Math.min(MAX_ZOOM, Math.round(nextZoom)))
  if (targetZoom === zoom.value) return
  const anchor = {
    x: centerPixels.value.x - viewport.width / 2 + anchorX,
    y: centerPixels.value.y - viewport.height / 2 + anchorY,
  }
  const geo = unproject(anchor)
  const nextAnchor = project(geo, targetZoom)
  zoom.value = targetZoom
  centerPixels.value = normalizedCenter({
    x: nextAnchor.x - anchorX + viewport.width / 2,
    y: nextAnchor.y - anchorY + viewport.height / 2,
  }, targetZoom)
}

function pointerDown(event) {
  if (state.value !== 'ready') return
  drag.active = true
  drag.x = event.clientX
  drag.y = event.clientY
  host.value?.setPointerCapture?.(event.pointerId)
}

function pointerMove(event) {
  if (!drag.active || !centerPixels.value) return
  centerPixels.value = normalizedCenter({
    x: centerPixels.value.x - (event.clientX - drag.x),
    y: centerPixels.value.y - (event.clientY - drag.y),
  })
  drag.x = event.clientX
  drag.y = event.clientY
}

function pointerUp(event) {
  drag.active = false
  host.value?.releasePointerCapture?.(event.pointerId)
}

function wheel(event) {
  if (state.value !== 'ready') return
  event.preventDefault()
  const bounds = host.value.getBoundingClientRect()
  zoomTo(zoom.value + (event.deltaY < 0 ? 1 : -1), event.clientX - bounds.left, event.clientY - bounds.top)
}

function initialize() {
  state.value = 'loading'
  message.value = ''
  tileError.value = false
  if (!props.geoReference?.available) {
    state.value = 'unavailable'
    message.value = '地图缺少锁定的 GNSS 原点'
    return
  }
  const pixel = project(mapPointToWgs84({ x: 0, y: 0 }, props.geoReference))
  if (!pixel) {
    state.value = 'unavailable'
    message.value = '保存地图没有有效经纬度'
    return
  }
  centerPixels.value = normalizedCenter(pixel)
  void loadConfig().finally(() => {
    if (mounted) state.value = 'ready'
  })
}

function reset() {
  centerPixels.value = null
  initialize()
}

onMounted(() => {
  mounted = true
  resizeObserver = new ResizeObserver(resize)
  resizeObserver.observe(host.value)
  resize()
  initialize()
})

watch(() => props.geoReference, reset, { deep: true })

onBeforeUnmount(() => {
  mounted = false
  resizeObserver?.disconnect()
  resizeObserver = null
})
</script>

<template>
  <div
    ref="host"
    class="satellite-viewport"
    role="application"
    aria-label="卫星地图"
    @pointerdown="pointerDown"
    @pointermove="pointerMove"
    @pointerup="pointerUp"
    @pointercancel="pointerUp"
    @wheel="wheel"
  >
    <div class="tile-layer" :class="{ dragging: drag.active }">
      <img
        v-for="tile in tiles"
        :key="tile.key"
        class="satellite-tile"
        :src="tile.src"
        :style="{ left: `${tile.x}px`, top: `${tile.y}px` }"
        alt=""
        draggable="false"
        @error="tileError = true"
      />
    </div>
    <svg v-if="state === 'ready'" class="map-overlay" :viewBox="`0 0 ${viewport.width} ${viewport.height}`" aria-hidden="true">
      <polyline v-if="trailPath" :points="trailPath" class="trail-path" />
      <polyline v-if="routePath" :points="routePath" class="route-path" />
      <circle v-if="robotPoint" :cx="robotPoint.x" :cy="robotPoint.y" r="9" class="robot-point" />
      <circle v-if="robotPoint" :cx="robotPoint.x" :cy="robotPoint.y" r="3" class="robot-core" />
    </svg>
    <div v-if="state !== 'ready'" class="satellite-state">
      <strong>{{ state === 'loading' ? '正在加载卫星图…' : '无可用来源' }}</strong>
      <span>{{ state === 'loading' ? '定位到已保存地图的经纬度' : message }}</span>
    </div>
    <div v-if="state === 'ready'" class="map-controls">
      <button type="button" title="放大" aria-label="放大" @click.stop="zoomTo(zoom + 1)">＋</button>
      <span>{{ zoom }}</span>
      <button type="button" title="缩小" aria-label="缩小" @click.stop="zoomTo(zoom - 1)">－</button>
      <button type="button" title="回到保存地图位置" aria-label="回到保存地图位置" @click.stop="recenter">⌖</button>
    </div>
    <div v-if="state === 'ready'" class="map-status">
      <span>{{ centerGeo ? `${centerGeo.latitude.toFixed(6)}, ${centerGeo.longitude.toFixed(6)}` : '—' }}</span>
      <span v-if="tileError" class="tile-error">部分瓦片加载失败</span>
      <span>Imagery © Esri</span>
    </div>
  </div>
</template>

<style scoped>
.satellite-viewport { position: relative; width: 100%; height: 100%; min-height: 520px; overflow: hidden; border-radius: 14px; background: #d8e0e4; cursor: grab; touch-action: none; user-select: none; }
.satellite-viewport:active { cursor: grabbing; }
.tile-layer { position: absolute; inset: 0; overflow: hidden; }
.tile-layer.dragging { cursor: grabbing; }
.satellite-tile { position: absolute; width: 256px; height: 256px; max-width: none; pointer-events: none; }
.map-overlay { position: absolute; inset: 0; width: 100%; height: 100%; pointer-events: none; overflow: visible; }
.route-path { fill: none; stroke: #38bdf8; stroke-width: 5; stroke-linecap: round; stroke-linejoin: round; filter: drop-shadow(0 1px 2px rgba(0, 0, 0, .75)); }
.trail-path { fill: none; stroke: #fbbf24; stroke-width: 4; stroke-linecap: round; stroke-linejoin: round; opacity: .85; filter: drop-shadow(0 1px 2px rgba(0, 0, 0, .75)); }
.robot-point { fill: #0f172a; stroke: #f8fafc; stroke-width: 3; filter: drop-shadow(0 1px 2px rgba(0, 0, 0, .8)); }
.robot-core { fill: #22c55e; }
.satellite-state { position: absolute; inset: 0; z-index: 2; display: grid; place-content: center; gap: 8px; text-align: center; color: #dbeafe; background: linear-gradient(135deg, #111827, #1e293b); }
.satellite-state strong { font-size: 18px; }.satellite-state span { color: #94a3b8; font-size: 12px; }
.map-controls, .map-status { position: absolute; z-index: 3; display: flex; align-items: center; gap: 6px; padding: 7px 9px; border: 1px solid rgba(15, 23, 42, .18); border-radius: 8px; background: rgba(255, 255, 255, .88); color: #334155; box-shadow: 0 2px 10px rgba(15, 23, 42, .16); backdrop-filter: blur(8px); font: 11px ui-monospace, monospace; }
.map-controls { top: 12px; right: 12px; }
.map-controls button { width: 25px; height: 25px; padding: 0; border: 0; border-radius: 5px; background: #e2e8f0; color: #0f172a; font-size: 17px; line-height: 1; cursor: pointer; }
.map-controls button:hover { background: #cbd5e1; }
.map-controls span { min-width: 18px; text-align: center; }
.map-status { right: 12px; bottom: 12px; flex-wrap: wrap; max-width: calc(100% - 24px); }
.tile-error { color: #b45309; }
@media (max-width: 900px) { .satellite-viewport { min-height: 420px; } }
</style>
