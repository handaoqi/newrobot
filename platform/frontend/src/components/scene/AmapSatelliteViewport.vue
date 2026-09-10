<script setup>
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'

import { mapPointToWgs84 } from '../../services/sceneData'

const props = defineProps({
  geoReference: { type: Object, default: null },
  robotPose: { type: Object, default: null },
  trail: { type: Array, default: () => [] },
  waypoints: { type: Array, default: () => [] },
})

const host = ref(null)
const state = ref('idle')
const message = ref('')
let map
let mounted = false
let renderToken = 0
let resizeObserver
let amapPromise

function unavailable(reason) {
  state.value = 'unavailable'
  message.value = reason
}

async function loadConfig() {
  const response = await fetch('/scene-map-config.json', {
    cache: 'no-store',
    headers: { Accept: 'application/json' },
  })
  if (!response.ok) throw new Error(`配置 HTTP ${response.status}`)
  const value = await response.json()
  if (value?.schema !== 'roamerx.scene-map-config.v1') throw new Error('配置 schema 不匹配')
  return value.amap || {}
}

function loadAmap(config) {
  if (amapPromise) return amapPromise
  amapPromise = new Promise((resolve, reject) => {
    if (window.AMap) {
      resolve(window.AMap)
      return
    }
    const script = document.createElement('script')
    const version = encodeURIComponent(config.version || '2.0')
    const key = encodeURIComponent(config.key)
    script.src = `https://webapi.amap.com/maps?v=${version}&key=${key}`
    script.async = true
    script.onload = () => window.AMap ? resolve(window.AMap) : reject(new Error('高德 API 未暴露 AMap'))
    script.onerror = () => reject(new Error('高德 API 脚本加载失败'))
    document.head.appendChild(script)
  })
  return amapPromise
}

function convertFromGps(AMap, points) {
  if (!points.length) return Promise.resolve([])
  const chunks = []
  for (let index = 0; index < points.length; index += 40) chunks.push(points.slice(index, index + 40))
  return chunks.reduce(async (promise, chunk) => {
    const converted = await promise
    const locations = await new Promise((resolve, reject) => {
      AMap.convertFrom(chunk.map(point => [point.longitude, point.latitude]), 'gps', (status, result) => {
        if (status === 'complete' && result?.info === 'ok' && Array.isArray(result.locations)) resolve(result.locations)
        else reject(new Error(result?.info || `坐标转换 ${status || '失败'}`))
      })
    })
    return [...converted, ...locations.map(location => [location.lng, location.lat])]
  }, Promise.resolve([]))
}

function localPoints() {
  const result = []
  if (props.robotPose && Number.isFinite(Number(props.robotPose.x)) && Number.isFinite(Number(props.robotPose.y))) {
    result.push({ kind: 'robot', point: props.robotPose })
  }
  for (const point of props.waypoints) result.push({ kind: 'waypoint', point })
  for (const point of props.trail) result.push({ kind: 'trail', point })
  return result
}

function clearOverlays() {
  map?.clearMap?.()
}

async function renderOverlays(AMap) {
  if (!map || !props.geoReference?.available) return
  const token = ++renderToken
  const items = localPoints()
  const mappedItems = items.map(item => ({ item, point: mapPointToWgs84(item.point, props.geoReference) })).filter(entry => entry.point)
  if (!mappedItems.length) {
    clearOverlays()
    return
  }
  try {
    const converted = await convertFromGps(AMap, mappedItems.map(entry => entry.point))
    if (!mounted || token !== renderToken) return
    clearOverlays()
    const overlays = []
    const route = []
    const trail = []
    converted.forEach((position, index) => {
      const item = mappedItems[index].item
      if (item.kind === 'robot') overlays.push(new AMap.Marker({ position, title: '机器狗' }))
      if (item.kind === 'waypoint') route.push(position)
      if (item.kind === 'trail') trail.push(position)
    })
    if (route.length > 1) overlays.push(new AMap.Polyline({ path: route, strokeColor: '#38bdf8', strokeWeight: 5, strokeOpacity: .85 }))
    if (trail.length > 1) overlays.push(new AMap.Polyline({ path: trail, strokeColor: '#fbbf24', strokeWeight: 4, strokeOpacity: .7 }))
    if (overlays.length) map.add(overlays)
  } catch (error) {
    if (mounted && token === renderToken) unavailable(`坐标转换失败：${error.message}`)
  }
}

async function initialize() {
  state.value = 'loading'
  message.value = ''
  if (!props.geoReference?.available) {
    unavailable('地图缺少锁定的 GNSS 原点')
    return
  }
  try {
    const config = await loadConfig()
    if (config.enabled !== true || !String(config.key || '').trim()) {
      unavailable('未配置高德 API Key')
      return
    }
    if (String(config.securityJsCode || '').trim()) {
      window._AMapSecurityConfig = { securityJsCode: config.securityJsCode }
    }
    const AMap = await loadAmap(config)
    if (!mounted) return
    const centerWgs84 = mapPointToWgs84({ x: 0, y: 0 }, props.geoReference)
    const [center] = await convertFromGps(AMap, centerWgs84 ? [centerWgs84] : [])
    if (!mounted || !center) return
    map = new AMap.Map(host.value, {
      center,
      zoom: 18,
      viewMode: '2D',
      layers: [new AMap.TileLayer.Satellite()],
      resizeEnable: true,
    })
    map.on('complete', () => { if (mounted) void renderOverlays(AMap) })
    resizeObserver = new ResizeObserver(() => map?.resize?.())
    resizeObserver.observe(host.value)
    state.value = 'ready'
    await renderOverlays(AMap)
  } catch (error) {
    if (mounted) unavailable(`高德地图加载失败：${error.message}`)
  }
}

function reset() {
  renderToken += 1
  resizeObserver?.disconnect()
  resizeObserver = null
  map?.destroy?.()
  map = null
  if (mounted) void initialize()
}

onMounted(() => {
  mounted = true
  void initialize()
})

watch(() => props.geoReference, reset, { deep: true })
watch(() => [props.robotPose, props.trail, props.waypoints], () => {
  if (map && window.AMap) void renderOverlays(window.AMap)
}, { deep: true })

onBeforeUnmount(() => {
  mounted = false
  renderToken += 1
  resizeObserver?.disconnect()
  map?.destroy?.()
  map = null
})
</script>

<template>
  <div ref="host" class="satellite-viewport" role="img" aria-label="高德卫星地图">
    <div v-if="state !== 'ready'" class="satellite-state">
      <strong>无可用来源</strong>
      <span>{{ state === 'loading' ? '正在连接高德地图…' : message }}</span>
    </div>
  </div>
</template>

<style scoped>
.satellite-viewport { position: relative; width: 100%; height: 100%; min-height: 520px; overflow: hidden; border-radius: 14px; background: #111827; }
.satellite-state { position: absolute; inset: 0; z-index: 1; display: grid; place-content: center; gap: 8px; text-align: center; color: #dbeafe; background: linear-gradient(135deg, #111827, #1e293b); }
.satellite-state strong { font-size: 18px; }.satellite-state span { color: #94a3b8; font-size: 12px; }
@media (max-width: 900px) { .satellite-viewport { min-height: 420px; } }
</style>
