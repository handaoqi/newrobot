<script setup>
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'

import { mapPointToWgs84 } from '../../services/sceneData'

const props = defineProps({
  backgroundMode: { type: String, default: 'grid' },
  robotPose: { type: Object, required: true },
  trail: { type: Array, default: () => [] },
  waypoints: { type: Array, default: () => [] },
  obstacles: { type: Array, default: () => [] },
  pointCloud: { type: Object, default: null },
  geoReference: { type: Object, default: null },
})

const emit = defineEmits(['amap-availability'])

const root = ref(null)
const mapHost = ref(null)
const canvas = ref(null)
const layers = ref({ trail: true, waypoints: true, robot: true, obstacles: true, cloud: false })
const amapMessage = ref('')

let ctx
let width = 0
let height = 0
let resizeObserver
let animationFrame
let map
let AMapApi
let amapConfig
let amapPromise
let mounted = false
let offsetX = 0
let offsetY = 0
let scale = 26
const pointers = new Map()
let pinchDistance = 0

function wgs84ToGcj02(longitude, latitude) {
  const a = 6378245
  const ee = 0.006693421622965943
  const transformLatitude = (x, y) => {
    let result = -100 + 2 * x + 3 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * Math.sqrt(Math.abs(x))
    result += (20 * Math.sin(6 * x * Math.PI) + 20 * Math.sin(2 * x * Math.PI)) * 2 / 3
    result += (20 * Math.sin(y * Math.PI) + 40 * Math.sin(y / 3 * Math.PI)) * 2 / 3
    result += (160 * Math.sin(y / 12 * Math.PI) + 320 * Math.sin(y * Math.PI / 30)) * 2 / 3
    return result
  }
  const transformLongitude = (x, y) => {
    let result = 300 + x + 2 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * Math.sqrt(Math.abs(x))
    result += (20 * Math.sin(6 * x * Math.PI) + 20 * Math.sin(2 * x * Math.PI)) * 2 / 3
    result += (20 * Math.sin(x * Math.PI) + 40 * Math.sin(x / 3 * Math.PI)) * 2 / 3
    result += (150 * Math.sin(x / 12 * Math.PI) + 300 * Math.sin(x / 30 * Math.PI)) * 2 / 3
    return result
  }
  const radLatitude = latitude / 180 * Math.PI
  let magic = Math.sin(radLatitude)
  magic = 1 - ee * magic * magic
  const rootMagic = Math.sqrt(magic)
  let latitudeDelta = transformLatitude(longitude - 105, latitude - 35)
  let longitudeDelta = transformLongitude(longitude - 105, latitude - 35)
  latitudeDelta = latitudeDelta * 180 / ((a * (1 - ee)) / (magic * rootMagic) * Math.PI)
  longitudeDelta = longitudeDelta * 180 / (a / rootMagic * Math.cos(radLatitude) * Math.PI)
  return [longitude + longitudeDelta, latitude + latitudeDelta]
}

function localToGcj02(point) {
  const value = mapPointToWgs84(point, props.geoReference)
  return value ? wgs84ToGcj02(value.longitude, value.latitude) : null
}

function localToScreen(point) {
  if (map && props.backgroundMode !== 'grid') {
    const coordinate = localToGcj02(point)
    if (!coordinate) return null
    const pixel = map.lngLatToContainer(coordinate)
    return [pixel.x, pixel.y]
  }
  return [width / 2 + point.x * scale + offsetX, height / 2 - point.y * scale + offsetY]
}

function cloudPoint(index) {
  const positions = props.pointCloud?.positions
  if (!positions) return null
  const x = Number(positions[index * 3])
  const y = Number(positions[index * 3 + 1])
  const z = Number(positions[index * 3 + 2])
  if (![x, y, z].every(Number.isFinite)) return null
  const yaw = Number(props.robotPose.yaw) || 0
  return {
    x: Number(props.robotPose.x || 0) + Math.cos(yaw) * x - Math.sin(yaw) * y,
    y: Number(props.robotPose.y || 0) + Math.sin(yaw) * x + Math.cos(yaw) * y,
    z,
  }
}

function drawGrid() {
  ctx.fillStyle = '#edf5fa'
  ctx.fillRect(0, 0, width, height)
  const spacing = Math.max(24, scale)
  const startX = ((width / 2 + offsetX) % spacing + spacing) % spacing
  const startY = ((height / 2 + offsetY) % spacing + spacing) % spacing
  ctx.strokeStyle = 'rgba(103, 166, 199, .24)'
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let x = startX; x < width; x += spacing) { ctx.moveTo(x, 0); ctx.lineTo(x, height) }
  for (let y = startY; y < height; y += spacing) { ctx.moveTo(0, y); ctx.lineTo(width, y) }
  ctx.stroke()
  ctx.strokeStyle = 'rgba(72, 133, 167, .4)'
  ctx.beginPath()
  ctx.moveTo(0, height / 2 + offsetY); ctx.lineTo(width, height / 2 + offsetY)
  ctx.moveTo(width / 2 + offsetX, 0); ctx.lineTo(width / 2 + offsetX, height)
  ctx.stroke()
}

function drawPolyline(points, color, lineWidth = 2) {
  if (points.length < 2) return
  ctx.strokeStyle = color
  ctx.lineWidth = lineWidth
  ctx.beginPath()
  let started = false
  for (const point of points) {
    const pixel = localToScreen(point)
    if (!pixel) continue
    if (!started) { ctx.moveTo(...pixel); started = true } else ctx.lineTo(...pixel)
  }
  if (started) ctx.stroke()
}

function drawCloud() {
  const count = Number(props.pointCloud?.count) || 0
  if (!count) return
  const step = Math.max(1, Math.ceil(count / 800))
  for (let index = 0; index < count; index += step) {
    const point = cloudPoint(index)
    if (!point || point.z < -0.35 || Math.hypot(point.x - props.robotPose.x, point.y - props.robotPose.y) > 12) continue
    const pixel = localToScreen(point)
    if (!pixel) continue
    ctx.fillStyle = 'rgba(70, 211, 255, .32)'
    ctx.fillRect(pixel[0] - 1, pixel[1] - 1, 2, 2)
  }
}

function drawObstacles() {
  for (const obstacle of props.obstacles) {
    const pixel = localToScreen(obstacle)
    if (!pixel) continue
    const dynamic = obstacle.dynamic !== false
    ctx.fillStyle = dynamic ? '#ff4d5f' : '#ff8a3d'
    ctx.strokeStyle = dynamic ? 'rgba(255, 77, 95, .35)' : 'rgba(255, 138, 61, .3)'
    ctx.lineWidth = 6
    ctx.beginPath(); ctx.arc(pixel[0], pixel[1], dynamic ? 5 : 4, 0, Math.PI * 2); ctx.stroke(); ctx.fill()
  }
}

function drawWaypoints() {
  props.waypoints.forEach((point, index) => {
    const pixel = localToScreen(point)
    if (!pixel) return
    const done = point.status === 'done'
    ctx.fillStyle = done ? '#2fd17e' : '#3d8cff'
    ctx.strokeStyle = done ? '#a7f3d0' : '#bfdbfe'
    ctx.lineWidth = 2
    ctx.beginPath(); ctx.arc(pixel[0], pixel[1], 8, 0, Math.PI * 2); ctx.fill(); ctx.stroke()
    ctx.fillStyle = '#fff'; ctx.font = '600 10px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'
    ctx.fillText(String(index + 1), pixel[0], pixel[1])
  })
}

function drawRobot() {
  const pixel = localToScreen(props.robotPose)
  if (!pixel) return
  ctx.save()
  ctx.translate(...pixel)
  ctx.rotate(-(Number(props.robotPose.yaw) || 0))
  ctx.fillStyle = '#ff9f43'
  ctx.shadowColor = 'rgba(255, 159, 67, .55)'
  ctx.shadowBlur = 12
  ctx.beginPath(); ctx.moveTo(16, 0); ctx.lineTo(-10, -8); ctx.lineTo(-5, 0); ctx.lineTo(-10, 8); ctx.closePath(); ctx.fill()
  ctx.restore()
}

function render() {
  animationFrame = null
  if (!ctx || !width || !height) return
  ctx.clearRect(0, 0, width, height)
  if (props.backgroundMode === 'grid' || !map) drawGrid()
  if (layers.value.cloud) drawCloud()
  if (layers.value.trail) drawPolyline(props.trail, '#ff9f43', 3)
  if (layers.value.obstacles) drawObstacles()
  if (layers.value.waypoints) drawWaypoints()
  if (layers.value.robot) drawRobot()
}

function scheduleRender() {
  if (animationFrame == null) animationFrame = requestAnimationFrame(render)
}

function resize() {
  if (!canvas.value || !root.value) return
  const bounds = root.value.getBoundingClientRect()
  const ratio = Math.min(window.devicePixelRatio || 1, 2)
  width = Math.max(1, bounds.width)
  height = Math.max(1, bounds.height)
  canvas.value.width = Math.round(width * ratio)
  canvas.value.height = Math.round(height * ratio)
  canvas.value.style.width = `${width}px`
  canvas.value.style.height = `${height}px`
  ctx = canvas.value.getContext('2d')
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0)
  map?.resize?.()
  scheduleRender()
}

async function loadConfig() {
  const response = await fetch('/scene-map-config.json', { cache: 'no-store' })
  if (!response.ok) throw new Error(`HTTP ${response.status}`)
  const config = await response.json()
  amapConfig = config?.amap || {}
  const available = amapConfig.enabled === true && Boolean(String(amapConfig.key || '').trim())
  emit('amap-availability', { available, message: available ? '' : '未配置高德地图 Key' })
  return available
}

function loadAmap() {
  if (amapPromise) return amapPromise
  amapPromise = new Promise((resolve, reject) => {
    if (window.AMap) { resolve(window.AMap); return }
    if (String(amapConfig.securityJsCode || '').trim()) window._AMapSecurityConfig = { securityJsCode: amapConfig.securityJsCode }
    const script = document.createElement('script')
    script.src = `https://webapi.amap.com/maps?v=${encodeURIComponent(amapConfig.version || '2.0')}&key=${encodeURIComponent(amapConfig.key)}`
    script.async = true
    script.onload = () => window.AMap ? resolve(window.AMap) : reject(new Error('高德 API 未加载'))
    script.onerror = () => reject(new Error('高德地图脚本加载失败'))
    document.head.appendChild(script)
  })
  return amapPromise
}

function destroyMap() {
  map?.destroy?.()
  map = null
  if (mapHost.value) mapHost.value.innerHTML = ''
}

async function updateBackground() {
  destroyMap()
  amapMessage.value = ''
  if (props.backgroundMode === 'grid') { scheduleRender(); return }
  if (!props.geoReference?.available) {
    amapMessage.value = '等待 /fix 提供地理原点'
    scheduleRender()
    return
  }
  try {
    const available = amapConfig ? amapConfig.enabled === true && Boolean(String(amapConfig.key || '').trim()) : await loadConfig()
    if (!available) { amapMessage.value = '未配置高德地图 Key'; scheduleRender(); return }
    AMapApi = await loadAmap()
    if (!mounted || props.backgroundMode === 'grid') return
    const center = localToGcj02({ x: Number(props.robotPose.x) || 0, y: Number(props.robotPose.y) || 0 })
    const options = { center, zoom: 18, viewMode: '2D', resizeEnable: true }
    if (props.backgroundMode === 'satellite') options.layers = [new AMapApi.TileLayer.Satellite()]
    map = new AMapApi.Map(mapHost.value, options)
    map.on('mapmove', scheduleRender)
    map.on('zoomchange', scheduleRender)
    map.on('complete', scheduleRender)
  } catch (error) {
    amapMessage.value = error.message
    scheduleRender()
  }
}

function pointerDown(event) {
  if (props.backgroundMode !== 'grid') return
  canvas.value.setPointerCapture(event.pointerId)
  pointers.set(event.pointerId, { x: event.clientX, y: event.clientY })
}

function pointerMove(event) {
  if (!pointers.has(event.pointerId) || props.backgroundMode !== 'grid') return
  const previous = pointers.get(event.pointerId)
  pointers.set(event.pointerId, { x: event.clientX, y: event.clientY })
  if (pointers.size === 1) {
    offsetX += event.clientX - previous.x
    offsetY += event.clientY - previous.y
  } else if (pointers.size === 2) {
    const [first, second] = [...pointers.values()]
    const distance = Math.hypot(first.x - second.x, first.y - second.y)
    if (pinchDistance) scale = Math.max(6, Math.min(80, scale * distance / pinchDistance))
    pinchDistance = distance
  }
  scheduleRender()
}

function pointerUp(event) {
  pointers.delete(event.pointerId)
  if (pointers.size < 2) pinchDistance = 0
}

function wheel(event) {
  if (props.backgroundMode !== 'grid') return
  event.preventDefault()
  scale = Math.max(6, Math.min(80, scale * (event.deltaY > 0 ? 0.9 : 1.1)))
  scheduleRender()
}

function resetView() {
  offsetX = 0
  offsetY = 0
  scale = 26
  if (map && props.geoReference?.available) {
    const center = localToGcj02(props.robotPose)
    if (center) map.setZoomAndCenter(18, center)
  }
  scheduleRender()
}

watch(() => props.backgroundMode, updateBackground)
watch(() => props.geoReference, () => { if (props.backgroundMode !== 'grid') void updateBackground() }, { deep: true })
watch(() => [props.robotPose, props.trail, props.waypoints, props.obstacles, props.pointCloud, layers.value], scheduleRender, { deep: true })

onMounted(async () => {
  mounted = true
  resizeObserver = new ResizeObserver(resize)
  resizeObserver.observe(root.value)
  resize()
  try { await loadConfig() } catch (error) { emit('amap-availability', { available: false, message: error.message }) }
  if (props.backgroundMode !== 'grid') await updateBackground()
})

onBeforeUnmount(() => {
  mounted = false
  resizeObserver?.disconnect()
  if (animationFrame != null) cancelAnimationFrame(animationFrame)
  destroyMap()
})
</script>

<template>
  <section ref="root" class="map-panel">
    <div ref="mapHost" class="map-host" />
    <canvas
      ref="canvas" class="map-canvas" :class="{ passive: backgroundMode !== 'grid' && !amapMessage }"
      @pointerdown="pointerDown" @pointermove="pointerMove" @pointerup="pointerUp" @pointercancel="pointerUp" @wheel="wheel"
    />
    <div class="panel-heading">
      <div><strong>全局巡检地图</strong><small>{{ backgroundMode === 'grid' ? 'LOCAL MAP' : 'AMAP' }}</small></div>
      <button type="button" @click="resetView">复位视图</button>
    </div>
    <div class="layer-switches">
      <label><input v-model="layers.trail" type="checkbox" />轨迹</label>
      <label><input v-model="layers.waypoints" type="checkbox" />航点</label>
      <label><input v-model="layers.robot" type="checkbox" />机器狗</label>
      <label><input v-model="layers.obstacles" type="checkbox" />障碍</label>
      <label><input v-model="layers.cloud" type="checkbox" />点云</label>
    </div>
    <div v-if="amapMessage && backgroundMode !== 'grid'" class="map-message">{{ amapMessage }}，已显示本地坐标画布</div>
    <div class="map-coordinate">
      <span>X {{ Number(robotPose.x || 0).toFixed(2) }} m</span>
      <span>Y {{ Number(robotPose.y || 0).toFixed(2) }} m</span>
      <span v-if="geoReference?.available">原点 {{ Number(geoReference.origin_longitude).toFixed(6) }}, {{ Number(geoReference.origin_latitude).toFixed(6) }}</span>
    </div>
  </section>
</template>

<style scoped>
.map-panel { position: relative; min-width: 0; min-height: 540px; overflow: hidden; border: 1px solid #c8dbe8; border-radius: 12px; background: #edf5fa; }
.map-host, .map-canvas { position: absolute; inset: 0; width: 100%; height: 100%; }
.map-canvas { z-index: 2; touch-action: none; }
.map-canvas.passive { pointer-events: none; }
.panel-heading, .layer-switches, .map-coordinate, .map-message { position: absolute; z-index: 3; border: 1px solid rgba(83, 137, 173, .24); background: rgba(255, 255, 255, .88); backdrop-filter: blur(8px); }
.panel-heading { top: 12px; right: 12px; left: 12px; display: flex; align-items: center; justify-content: space-between; padding: 10px 12px; border-radius: 9px; color: #1b3a54; pointer-events: none; }
.panel-heading > div { display: flex; align-items: baseline; gap: 10px; }
.panel-heading small { color: #1689b5; font-size: 10px; letter-spacing: .14em; }
.panel-heading button { min-height: 32px; padding: 5px 10px; border: 1px solid #b6cede; border-radius: 7px; background: #eef6fb; color: #2b506b; pointer-events: auto; }
.layer-switches { top: 70px; left: 12px; display: flex; flex-wrap: wrap; gap: 7px 12px; max-width: calc(100% - 24px); padding: 8px 10px; border-radius: 8px; color: #526f87; font-size: 11px; }
.layer-switches label { display: flex; align-items: center; gap: 4px; cursor: pointer; }
.layer-switches input { width: 13px !important; height: 13px !important; min-width: 13px !important; min-height: 0 !important; margin: 0; accent-color: #168fbd; }
.map-coordinate { right: 12px; bottom: 12px; display: flex; flex-wrap: wrap; gap: 8px 14px; max-width: calc(100% - 24px); padding: 7px 10px; border-radius: 7px; color: #607d94; font: 11px ui-monospace, monospace; }
.map-message { top: 118px; left: 50%; translate: -50% 0; padding: 8px 12px; border-radius: 8px; color: #986900; font-size: 12px; white-space: nowrap; }
@media (max-width: 900px) { .map-panel { min-height: 420px; } }
@media (orientation: portrait) { .map-panel { min-height: 360px; } }
</style>
