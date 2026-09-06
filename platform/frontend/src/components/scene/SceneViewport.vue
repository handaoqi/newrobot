<script setup>
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import { PCDLoader } from 'three/examples/jsm/loaders/PCDLoader.js'

import {
  assetForClass, nextSemanticZoom, normalizeSceneAssetCatalog, normalizeSceneAssetInstance,
  sceneAssetIdForClass, SCENE_ASSET_CATALOG_URL, semanticZoomMode,
} from '../../services/sceneData'

const props = defineProps({
  manifest: { type: Object, default: null },
  cloudBuffer: { type: ArrayBuffer, default: null },
  liveCloud: { type: Object, default: null },
  obstacles: { type: Object, default: null },
  trail: { type: Array, default: () => [] },
  correction: { type: Object, default: null },
  robotPose: { type: Object, default: null },
  waypoints: { type: Array, default: () => [] },
  semanticObjects: { type: Array, default: () => [] },
  layers: { type: Object, required: true },
  mode: { type: String, default: '2d' },
  cameraPreset: { type: String, default: 'overview' },
})
const emit = defineEmits(['mode-change', 'stats', 'error'])

const host = ref(null)
const zoom = ref(props.mode === '3d' ? 0.75 : 0.25)
let renderer
let scene
let perspective
let orthographic
let activeCamera
let controls
let resizeObserver
let animationFrame
let lastStatsAt = performance.now()
let renderedFrames = 0
const groups = {}
const assetLoader = new GLTFLoader()
const assetModelCache = new Map()
const assetModelRequests = new Map()
let assetCatalog = normalizeSceneAssetCatalog(null)
let assetCatalogUrl = ''
let assetCatalogRequest = null
let semanticRenderToken = 0
let mounted = false

function bounds() {
  const value = props.manifest?.bounds || {}
  return {
    minX: Number(value.min_x || -10), maxX: Number(value.max_x || 10),
    minY: Number(value.min_y || -10), maxY: Number(value.max_y || 10),
  }
}

function centerAndRadius() {
  const box = bounds()
  const center = new THREE.Vector3((box.minX + box.maxX) / 2, (box.minY + box.maxY) / 2, 0)
  return { center, radius: Math.max(5, Math.hypot(box.maxX - box.minX, box.maxY - box.minY) / 2) }
}

function disposeNode(node) {
  node.geometry?.dispose?.()
  const materials = Array.isArray(node.material) ? node.material : node.material ? [node.material] : []
  materials.forEach(material => {
    material.map?.dispose?.()
    material.dispose?.()
  })
}

function clearGroup(name) {
  const group = groups[name]
  if (!group) return
  while (group.children.length) {
    const child = group.children.pop()
    // GLB instances share geometry and materials with the cache. Dispose
    // procedural fallbacks here, but keep cached model resources alive.
    if (!child.userData.sceneAssetInstance) child.traverse(disposeNode)
  }
}

function catalogUrl() {
  return props.manifest?.asset_catalog?.url || props.manifest?.asset_catalog_url || SCENE_ASSET_CATALOG_URL
}

async function loadAssetCatalog() {
  const url = catalogUrl()
  if (assetCatalogUrl === url && assetCatalog.assets.length) return assetCatalog
  if (assetCatalogRequest && assetCatalogUrl === url) return assetCatalogRequest
  assetCatalogUrl = url
  assetCatalogRequest = fetch(url, { cache: 'no-store', headers: { Accept: 'application/json' } })
    .then(response => {
      if (!response.ok) throw new Error(`HTTP ${response.status}`)
      return response.json()
    })
    .then(value => {
      const normalized = normalizeSceneAssetCatalog(value)
      if (!normalized.assets.length) throw new Error('目录为空或 schema 不匹配')
      assetCatalog = normalized
      return normalized
    })
    .catch(error => {
      assetCatalog = normalizeSceneAssetCatalog(null)
      emit('error', `场景资产目录加载失败：${error.message}`)
      return assetCatalog
    })
    .finally(() => { assetCatalogRequest = null })
  await assetCatalogRequest
  if (mounted) await updateSemanticObjects()
  return assetCatalog
}

function applyInstanceTransform(node, item) {
  node.position.set(item.position.x, item.position.y, item.position.z)
  node.quaternion.set(item.orientation.x, item.orientation.y, item.orientation.z, item.orientation.w)
  const scale = item.scale
  if (Number.isFinite(Number(scale))) node.scale.setScalar(Math.max(.05, Number(scale)))
  else if (Array.isArray(scale) && scale.length >= 3) node.scale.set(Math.max(.05, Number(scale[0]) || 1), Math.max(.05, Number(scale[1]) || 1), Math.max(.05, Number(scale[2]) || 1))
  else if (scale && typeof scale === 'object') node.scale.set(Math.max(.05, Number(scale.x) || 1), Math.max(.05, Number(scale.y) || 1), Math.max(.05, Number(scale.z) || 1))
  node.userData = { ...node.userData, id: item.id, className: item.className, confidence: item.confidence, assetId: item.assetId }
}

async function loadAssetModel(assetId) {
  const entry = assetCatalog.byId.get(assetId)
  if (!entry) return null
  if (assetModelCache.has(assetId)) return assetModelCache.get(assetId).clone(true)
  if (!assetModelRequests.has(assetId)) {
    const request = assetLoader.loadAsync(entry.url)
      .then(result => {
        assetModelCache.set(assetId, result.scene)
        return result.scene
      })
      .catch(error => {
        emit('error', `场景资产 ${assetId} 加载失败：${error.message}`)
        return null
      })
      .finally(() => assetModelRequests.delete(assetId))
    assetModelRequests.set(assetId, request)
  }
  const model = await assetModelRequests.get(assetId)
  return model?.clone(true) || null
}

function pointMaterial(size = 0.07, opacity = 1) {
  return new THREE.PointsMaterial({ size, vertexColors: true, transparent: opacity < 1, opacity, sizeAttenuation: true })
}

function updateCloudBuffer() {
  clearGroup('globalCloud')
  if (!props.cloudBuffer) return
  try {
    const points = new PCDLoader().parse(props.cloudBuffer, '')
    points.material.size = 0.07
    points.material.vertexColors = Boolean(points.geometry.getAttribute('color'))
    if (!points.material.vertexColors) points.material.color.set('#5bb8ff')
    groups.globalCloud.add(points)
  } catch (error) {
    emit('error', `三维地图解析失败：${error.message}`)
  }
}

function updateLiveCloud() {
  clearGroup('localCloud')
  if (!props.liveCloud?.count) return
  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(props.liveCloud.positions, 3))
  geometry.setAttribute('color', new THREE.BufferAttribute(props.liveCloud.colors, 3))
  groups.localCloud.add(new THREE.Points(geometry, pointMaterial(0.09, 0.92)))
}

function updateObstacles() {
  clearGroup('obstacles')
  if (!props.obstacles?.count) return
  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(props.obstacles.positions, 3))
  geometry.setAttribute('color', new THREE.BufferAttribute(props.obstacles.colors, 3))
  groups.obstacles.add(new THREE.Points(geometry, pointMaterial(.12, .95)))
}

function updateTrail() {
  clearGroup('trail')
  const points = props.trail.map(point => new THREE.Vector3(Number(point.x), Number(point.y), .05)).filter(point => [point.x, point.y].every(Number.isFinite))
  if (points.length > 1) groups.trail.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({ color: '#fbbf24', transparent: true, opacity: .75 })))
}

function updateCorrection() {
  clearGroup('corrections')
  const from = props.correction?.from
  const to = props.correction?.to
  if (!from || !to) return
  const points = [new THREE.Vector3(Number(from.x), Number(from.y), .16), new THREE.Vector3(Number(to.x), Number(to.y), .16)]
  if (!points.every(point => [point.x, point.y].every(Number.isFinite))) return
  groups.corrections.add(new THREE.Line(points.length ? new THREE.BufferGeometry().setFromPoints(points) : undefined, new THREE.LineDashedMaterial({ color: '#f472b6', dashSize: .2, gapSize: .1 })))
  groups.corrections.children[0].computeLineDistances()
}

function updateBoundary() {
  clearGroup('boundary')
  const boundary = props.manifest?.boundary || []
  const addPolygon = (polygon, color, z = .12) => {
    const points = (polygon || []).map(point => new THREE.Vector3(Number(point.x ?? point[0]), Number(point.y ?? point[1]), z)).filter(point => [point.x, point.y].every(Number.isFinite))
    if (points.length < 3) return
    points.push(points[0].clone())
    groups.boundary.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(points), new THREE.LineBasicMaterial({ color })))
  }
  addPolygon(boundary.points || boundary, '#fb7185')
  const zoneColors = { forbidden: '#ef4444', restricted: '#a78bfa', warning: '#f59e0b' }
  for (const zone of boundary.zones || []) addPolygon(zone.polygon, zoneColors[zone.zone_type] || '#f59e0b', .14)
}

function updateOccupancy() {
  clearGroup('occupancy')
  if (!props.manifest?.occupancy_url) return
  const box = bounds()
  const width = Math.max(1, box.maxX - box.minX)
  const height = Math.max(1, box.maxY - box.minY)
  const material = new THREE.MeshBasicMaterial({ color: '#26374c', transparent: true, opacity: 0.58, side: THREE.DoubleSide })
  const plane = new THREE.Mesh(new THREE.PlaneGeometry(width, height), material)
  plane.position.set((box.minX + box.maxX) / 2, (box.minY + box.maxY) / 2, -0.04)
  groups.occupancy.add(plane)
  new THREE.TextureLoader().load(props.manifest.occupancy_url, texture => {
    texture.colorSpace = THREE.SRGBColorSpace
    material.map = texture
    material.color.set('#ffffff')
    material.needsUpdate = true
  }, undefined, () => {})
}

function updateRoute() {
  clearGroup('route')
  const points = props.waypoints.map(point => new THREE.Vector3(Number(point.x), Number(point.y), 0.08)).filter(point => [point.x, point.y].every(Number.isFinite))
  if (points.length > 1) {
    const geometry = new THREE.BufferGeometry().setFromPoints(points)
    groups.route.add(new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: '#38bdf8', linewidth: 2 })))
  }
  points.forEach((point, index) => {
    const marker = new THREE.Mesh(new THREE.CylinderGeometry(0.16, 0.16, 0.08, 18), new THREE.MeshStandardMaterial({ color: index ? '#22c55e' : '#f59e0b' }))
    marker.rotation.x = Math.PI / 2
    marker.position.copy(point)
    groups.route.add(marker)
  })
}

function updateRobot() {
  clearGroup('robot')
  if (!props.robotPose || !Number.isFinite(Number(props.robotPose.x))) return
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.75, 0.38, 0.34), new THREE.MeshStandardMaterial({ color: '#22d3ee', roughness: 0.55 }))
  body.position.set(Number(props.robotPose.x), Number(props.robotPose.y), Number(props.robotPose.z || 0) + 0.32)
  body.rotation.z = Number(props.robotPose.yaw || 0)
  groups.robot.add(body)
  const cone = new THREE.Mesh(new THREE.ConeGeometry(0.18, 0.65, 16), new THREE.MeshBasicMaterial({ color: '#67e8f9', transparent: true, opacity: 0.28 }))
  cone.rotation.z = -Math.PI / 2
  cone.position.set(0.55, 0, 0)
  body.add(cone)
}

function primitiveFor(item) {
  const asset = assetForClass(item.className)
  const d = item.dimensions || { x: .6, y: .6, z: 1 }
  let geometry
  if (asset.kind === 'person') geometry = new THREE.CapsuleGeometry(Math.min(d.x, d.y) * .3, Math.max(.2, d.z * .65), 5, 10)
  else if (asset.kind === 'bicycle') geometry = new THREE.TorusGeometry(Math.max(.18, d.z * .25), .045, 8, 18)
  else geometry = new THREE.BoxGeometry(d.x, d.y, d.z)
  const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({ color: asset.color, transparent: true, opacity: item.dynamic ? .88 : .72 }))
  mesh.position.set(item.position.x, item.position.y, item.position.z + d.z / 2)
  if (asset.kind === 'bicycle') mesh.rotation.x = Math.PI / 2
  const q = item.orientation
  if (q && [q.x, q.y, q.z, q.w].every(value => Number.isFinite(Number(value)))) mesh.quaternion.set(q.x, q.y, q.z, q.w)
  mesh.userData = { id: item.id, className: item.className, confidence: item.confidence }
  return mesh
}

async function updateSemanticObjects() {
  const token = ++semanticRenderToken
  clearGroup('semantic')
  const all = [...(props.manifest?.static_assets || []), ...props.semanticObjects]
  for (const [index, item] of all.entries()) {
    const normalized = normalizeSceneAssetInstance(item, index)
    const assetId = sceneAssetIdForClass(normalized.assetId, assetCatalog) || sceneAssetIdForClass(normalized.className, assetCatalog)
    const model = await loadAssetModel(assetId)
    if (!mounted || token !== semanticRenderToken) return
    if (model) {
      model.userData.sceneAssetInstance = true
      applyInstanceTransform(model, { ...normalized, assetId })
      groups.semantic.add(model)
    } else {
      groups.semantic.add(primitiveFor(normalized))
    }
  }
  updateVisibility()
}

function updateVisibility() {
  for (const [name, group] of Object.entries(groups)) group.visible = props.layers[name] !== false
}

function setCamera(mode = props.mode) {
  if (!renderer || !controls) return
  const { center, radius } = centerAndRadius()
  const previous = activeCamera
  activeCamera = mode === '2d' ? orthographic : perspective
  if (mode === '2d') {
    orthographic.position.set(center.x, center.y, radius * 2.4)
    orthographic.up.set(0, 1, 0)
    orthographic.lookAt(center)
  } else if (props.cameraPreset === 'dog' && props.robotPose) {
    const yaw = Number(props.robotPose.yaw || 0)
    perspective.position.set(Number(props.robotPose.x), Number(props.robotPose.y), Number(props.robotPose.z || 0) + .65)
    controls.target.set(perspective.position.x + Math.cos(yaw) * 4, perspective.position.y + Math.sin(yaw) * 4, .5)
  } else if (props.cameraPreset === 'follow' && props.robotPose) {
    const yaw = Number(props.robotPose.yaw || 0)
    const x = Number(props.robotPose.x)
    const y = Number(props.robotPose.y)
    perspective.position.set(x - Math.cos(yaw) * 4, y - Math.sin(yaw) * 4, Number(props.robotPose.z || 0) + 2.6)
    controls.target.set(x + Math.cos(yaw) * 1.2, y + Math.sin(yaw) * 1.2, .35)
  } else {
    perspective.position.set(center.x + radius * .9, center.y - radius * 1.1, radius * .8)
    perspective.up.set(0, 0, 1)
    perspective.lookAt(center)
  }
  controls.object = activeCamera
  if (!(['dog', 'follow'].includes(props.cameraPreset) && props.robotPose)) controls.target.copy(center)
  controls.enableRotate = mode === '3d'
  controls.update()
  if (previous !== activeCamera) resize()
}

function resize() {
  if (!renderer || !host.value) return
  const width = Math.max(1, host.value.clientWidth)
  const height = Math.max(1, host.value.clientHeight)
  renderer.setSize(width, height, false)
  perspective.aspect = width / height
  perspective.updateProjectionMatrix()
  const { radius } = centerAndRadius()
  const halfHeight = radius * 1.18
  orthographic.left = -halfHeight * width / height
  orthographic.right = halfHeight * width / height
  orthographic.top = halfHeight
  orthographic.bottom = -halfHeight
  orthographic.updateProjectionMatrix()
}

function onWheel(event) {
  zoom.value = nextSemanticZoom(zoom.value, event.deltaY)
  const next = semanticZoomMode(props.mode, zoom.value)
  if (next !== props.mode) emit('mode-change', next)
}

function animate(now) {
  animationFrame = requestAnimationFrame(animate)
  controls?.update()
  renderer?.render(scene, activeCamera)
  renderedFrames += 1
  if (now - lastStatsAt >= 1000) {
    emit('stats', { fps: Math.round(renderedFrames * 1000 / (now - lastStatsAt)), calls: renderer.info.render.calls, points: renderer.info.render.points })
    lastStatsAt = now
    renderedFrames = 0
  }
}

onMounted(() => {
  mounted = true
  scene = new THREE.Scene()
  scene.background = new THREE.Color('#07111f')
  scene.fog = new THREE.FogExp2('#07111f', 0.006)
  perspective = new THREE.PerspectiveCamera(55, 1, .05, 5000)
  orthographic = new THREE.OrthographicCamera(-10, 10, 10, -10, .05, 5000)
  activeCamera = props.mode === '2d' ? orthographic : perspective
  renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' })
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.75))
  renderer.outputColorSpace = THREE.SRGBColorSpace
  host.value.appendChild(renderer.domElement)
  controls = new OrbitControls(activeCamera, renderer.domElement)
  controls.enableDamping = true
  controls.dampingFactor = .12
  controls.screenSpacePanning = true
  scene.add(new THREE.HemisphereLight('#d8f1ff', '#132238', 2.5))
  const directional = new THREE.DirectionalLight('#ffffff', 2.2)
  directional.position.set(6, -4, 10)
  scene.add(directional)
  for (const name of ['occupancy', 'globalCloud', 'localCloud', 'obstacles', 'route', 'trail', 'corrections', 'boundary', 'robot', 'semantic']) {
    groups[name] = new THREE.Group(); groups[name].name = name; scene.add(groups[name])
  }
  const grid = new THREE.GridHelper(60, 60, '#284c68', '#173044')
  grid.rotation.x = Math.PI / 2
  scene.add(grid)
  resizeObserver = new ResizeObserver(resize)
  resizeObserver.observe(host.value)
  host.value.addEventListener('wheel', onWheel, { passive: true })
  updateOccupancy(); updateCloudBuffer(); updateLiveCloud(); updateObstacles(); updateRoute(); updateTrail(); updateCorrection(); updateBoundary(); void updateSemanticObjects(); void loadAssetCatalog(); updateVisibility(); setCamera(); resize()
  animationFrame = requestAnimationFrame(animate)
})

watch(() => props.manifest, () => { updateOccupancy(); void updateSemanticObjects(); void loadAssetCatalog(); updateBoundary(); setCamera(); resize() }, { deep: true })
watch(() => props.cloudBuffer, updateCloudBuffer)
watch(() => props.liveCloud, updateLiveCloud)
watch(() => props.obstacles, updateObstacles)
watch(() => props.trail, updateTrail, { deep: true })
watch(() => props.correction, updateCorrection, { deep: true })
watch(() => props.robotPose, () => { updateRobot(); if (['dog', 'follow'].includes(props.cameraPreset)) setCamera() }, { deep: true })
watch(() => props.waypoints, updateRoute, { deep: true })
watch(() => props.semanticObjects, () => { void updateSemanticObjects() }, { deep: true })
watch(() => props.layers, updateVisibility, { deep: true })
watch(() => [props.mode, props.cameraPreset], () => {
  zoom.value = props.mode === '3d' ? Math.max(zoom.value, .56) : Math.min(zoom.value, .44)
  setCamera()
})

onBeforeUnmount(() => {
  mounted = false
  semanticRenderToken += 1
  cancelAnimationFrame(animationFrame)
  resizeObserver?.disconnect()
  host.value?.removeEventListener('wheel', onWheel)
  controls?.dispose()
  scene?.traverse(disposeNode)
  renderer?.dispose()
  renderer?.forceContextLoss()
  renderer?.domElement?.remove()
})
</script>

<template>
  <div ref="host" class="scene-viewport" role="img" aria-label="机器狗二维三维场景视图"></div>
</template>

<style scoped>
.scene-viewport { position: relative; width: 100%; height: 100%; min-height: 520px; overflow: hidden; border-radius: 14px; background: #07111f; touch-action: none; }
.scene-viewport :deep(canvas) { display: block; width: 100%; height: 100%; }
@media (max-width: 900px) { .scene-viewport { min-height: 420px; } }
</style>
