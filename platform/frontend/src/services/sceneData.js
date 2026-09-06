export const SCENE_TOPICS = Object.freeze([
  '/tf', '/tf_static', '/front_lidar', '/local_costmap/costmap_raw',
  '/plan', '/transformed_global_plan', '/odom/localization_odom',
  '/localization_info', '/status', '/localization/decision',
  '/perception/semantic_objects', '/perception/projection_status',
])

export const SCENE_LAYER_DEFAULTS = Object.freeze({
  occupancy: true,
  globalCloud: true,
  localCloud: true,
  obstacles: true,
  route: true,
  trail: true,
  corrections: true,
  semantic: true,
  boundary: true,
})

export const SCENE_ASSET_CATALOG_SCHEMA = 'roamerx.scene-assets.v1'
export const SCENE_ASSET_CATALOG_URL = '/scene-assets/catalog.json'

export const ASSET_REGISTRY = Object.freeze({
  person: { label: '行人', aliases: ['pedestrian'], color: '#f59e0b', kind: 'person', dynamic: true },
  bicycle: { label: '自行车', aliases: ['bike', '自行车'], color: '#38bdf8', kind: 'bicycle', dynamic: true },
  vehicle: { label: '车辆', aliases: ['car', 'truck', 'bus', 'motorcycle'], color: '#ef4444', kind: 'vehicle', dynamic: true },
  wall: { label: '墙体', aliases: [], color: '#94a3b8', kind: 'wall', dynamic: false },
  building: { label: '建筑', aliases: [], color: '#64748b', kind: 'building', dynamic: false },
  tree: { label: '树木', aliases: ['bush', 'shrub', 'conifer', 'pine'], color: '#22c55e', kind: 'tree', dynamic: false },
  road: { label: '道路', aliases: ['path', 'walkway', 'crossroad', 'intersection'], color: '#64748b', kind: 'road', dynamic: false },
  unknown_obstacle: { label: '未知障碍', aliases: ['unknown'], color: '#a78bfa', kind: 'obstacle', dynamic: true },
})

export function assetForClass(value) {
  const normalized = String(value || '').trim().toLowerCase()
  const category = normalized.split('.', 1)[0]
  for (const [key, asset] of Object.entries(ASSET_REGISTRY)) {
    if (key === normalized || key === category || asset.aliases.includes(normalized)) return { key, ...asset }
  }
  return { key: 'unknown_obstacle', ...ASSET_REGISTRY.unknown_obstacle }
}

export function normalizeSceneAssetCatalog(value) {
  if (!value || typeof value !== 'object' || value.schema !== SCENE_ASSET_CATALOG_SCHEMA) {
    return { schema: '', version: '', assets: [], byId: new Map(), byAlias: new Map(), defaultByCategory: new Map() }
  }
  const assets = Array.isArray(value.assets)
    ? value.assets.filter(item => item && typeof item.asset_id === 'string' && typeof item.url === 'string')
    : []
  const byId = new Map(assets.map(item => [item.asset_id, item]))
  const byAlias = new Map()
  for (const item of assets) {
    byAlias.set(item.asset_id.toLowerCase(), item.asset_id)
    for (const alias of Array.isArray(item.aliases) ? item.aliases : []) {
      if (typeof alias === 'string' && alias.trim()) byAlias.set(alias.trim().toLowerCase(), item.asset_id)
    }
  }
  const defaultByCategory = new Map(Object.entries(value.default_asset_by_category || {}).filter(([category, assetId]) => byId.has(assetId) && category))
  return { ...value, assets, byId, byAlias, defaultByCategory }
}

export function sceneAssetIdForClass(value, catalog) {
  const normalized = String(value || '').trim().toLowerCase()
  if (!normalized || !catalog) return null
  const direct = catalog.byId?.get(normalized)
  if (direct) return direct.asset_id || direct
  const alias = catalog.byAlias?.get(normalized)
  if (alias) return alias
  const semantic = assetForClass(normalized)
  return catalog.defaultByCategory?.get(semantic.key === 'unknown_obstacle' ? 'obstacle' : semantic.key) || null
}

export function normalizeSceneAssetInstance(item, index = 0) {
  const source = item && typeof item === 'object' ? item : {}
  const position = source.position || source.pose?.position || {}
  const orientation = source.orientation || source.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 }
  const dimensions = source.dimensions || (source.scale && typeof source.scale === 'object' && !Array.isArray(source.scale) ? source.scale : { x: .6, y: .6, z: 1.7 })
  return {
    ...source,
    id: String(source.id || `static-${index}`),
    assetId: String(source.asset_id || source.asset || source.class_name || source.className || 'unknown'),
    className: String(source.class_name || source.className || source.asset_id || source.asset || 'unknown'),
    confidence: Number.isFinite(Number(source.confidence)) ? Number(source.confidence) : 1,
    position: { x: Number(position.x) || 0, y: Number(position.y) || 0, z: Number(position.z) || 0 },
    orientation: {
      x: Number(orientation.x) || 0, y: Number(orientation.y) || 0,
      z: Number(orientation.z) || 0, w: Number.isFinite(Number(orientation.w)) ? Number(orientation.w) : 1,
    },
    dimensions: { x: Math.max(.1, Number(dimensions.x) || .6), y: Math.max(.1, Number(dimensions.y) || .6), z: Math.max(.1, Number(dimensions.z) || 1.7) },
    dynamic: source.dynamic !== false,
  }
}

export function nextSemanticZoom(current, deltaY) {
  const delta = Math.sign(Number(deltaY) || 0) * -0.08
  return Math.max(0, Math.min(1, Number(current || 0) + delta))
}

export function semanticZoomMode(currentMode, zoom) {
  if (zoom <= 0.44) return '2d'
  if (zoom >= 0.56) return '3d'
  return currentMode === '3d' ? '3d' : '2d'
}

export function decodeJsonString(message) {
  if (message && typeof message === 'object' && typeof message.data === 'string') {
    try { return JSON.parse(message.data) } catch { return null }
  }
  return message && typeof message === 'object' ? message : null
}

function numeric(value, fallback = 0) {
  const number = Number(value)
  return Number.isFinite(number) ? number : fallback
}

const IDENTITY_4 = Object.freeze([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])

function normalizedFrame(value) {
  return String(value || '').replace(/^\/+/, '')
}

function multiply4(left, right) {
  const output = new Array(16).fill(0)
  for (let row = 0; row < 4; row += 1) for (let column = 0; column < 4; column += 1) {
    for (let index = 0; index < 4; index += 1) output[row * 4 + column] += left[row * 4 + index] * right[index * 4 + column]
  }
  return output
}

export function poseMatrix(position = {}, orientation = {}) {
  let x = numeric(orientation.x); let y = numeric(orientation.y); let z = numeric(orientation.z); let w = numeric(orientation.w, 1)
  const norm = Math.hypot(x, y, z, w)
  if (!norm) return null
  x /= norm; y /= norm; z /= norm; w /= norm
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), numeric(position.x),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), numeric(position.y),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), numeric(position.z),
    0, 0, 0, 1,
  ]
}

export function createTfTree(fixedFrame = 'map') {
  const fixed = normalizedFrame(fixedFrame)
  const edges = new Map()
  return {
    clear() { edges.clear() },
    update(message) {
      for (const item of message?.transforms || []) {
        const parent = normalizedFrame(item.header?.frame_id)
        const child = normalizedFrame(item.child_frame_id)
        const matrix = poseMatrix(item.transform?.translation, item.transform?.rotation)
        if (parent && child && parent !== child && matrix) edges.set(child, { parent, matrix })
      }
    },
    matrixFrom(sourceFrame) {
      let current = normalizedFrame(sourceFrame)
      if (!current || current === fixed) return [...IDENTITY_4]
      let result = [...IDENTITY_4]
      const visited = new Set()
      while (current !== fixed && !visited.has(current)) {
        visited.add(current)
        const edge = edges.get(current)
        if (!edge) return null
        result = multiply4(edge.matrix, result)
        current = edge.parent
      }
      return current === fixed ? result : null
    },
  }
}

export function transformPointData(cloud, matrix) {
  if (!cloud?.count || !matrix) return cloud
  const positions = new Float32Array(cloud.positions.length)
  for (let index = 0; index < cloud.count; index += 1) {
    const offset = index * 3
    const x = cloud.positions[offset]; const y = cloud.positions[offset + 1]; const z = cloud.positions[offset + 2]
    positions[offset] = matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3]
    positions[offset + 1] = matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7]
    positions[offset + 2] = matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11]
  }
  return { ...cloud, positions }
}

export function transformPoseTo2D(position, orientation, frameTransform = null) {
  const local = poseMatrix(position, orientation)
  if (!local) return null
  const matrix = frameTransform ? multiply4(frameTransform, local) : local
  return { x: matrix[3], y: matrix[7], z: matrix[11], yaw: Math.atan2(matrix[4], matrix[0]) }
}

export function normalizeSemanticObjects(message, timestamp = 0) {
  const items = Array.isArray(message?.objects) ? message.objects : []
  return items.map((item, index) => ({
    id: String(item.track_id || item.id || `object-${index}`),
    className: String(item.class_name || item.class_id || item.label || 'unknown'),
    confidence: numeric(item.confidence),
    position: {
      x: numeric(item.pose?.position?.x ?? item.position?.x),
      y: numeric(item.pose?.position?.y ?? item.position?.y),
      z: numeric(item.pose?.position?.z ?? item.position?.z),
    },
    orientation: item.pose?.orientation || item.orientation || { x: 0, y: 0, z: 0, w: 1 },
    dimensions: {
      x: Math.max(0.1, numeric(item.dimensions?.x, 0.6)),
      y: Math.max(0.1, numeric(item.dimensions?.y, 0.6)),
      z: Math.max(0.1, numeric(item.dimensions?.z, 1.7)),
    },
    dynamic: item.dynamic !== false,
    positionStdM: numeric(item.position_std_m, NaN),
    projectionMethod: String(item.projection_method || 'lidar_camera'),
    calibrationId: String(item.calibration_id || ''),
    observedAt: numeric(item.observed_at_unix, timestamp),
  }))
}

const POINT_FIELD_TYPES = Object.freeze({
  1: ['getInt8', 1], 2: ['getUint8', 1], 3: ['getInt16', 2], 4: ['getUint16', 2],
  5: ['getInt32', 4], 6: ['getUint32', 4], 7: ['getFloat32', 4], 8: ['getFloat64', 8],
})

function colorRamp(value) {
  const t = Math.max(0, Math.min(1, value))
  return [Math.max(0, 1.5 - Math.abs(4 * t - 3)), Math.max(0, 1.5 - Math.abs(4 * t - 2)), Math.max(0, 1.5 - Math.abs(4 * t - 1))]
}

export function pointCloud2ToArrays(message, { maxPoints = 80_000, colorMode = 'height' } = {}) {
  const fields = new Map((message?.fields || []).map(field => [field.name, field]))
  if (!fields.has('x') || !fields.has('y') || !fields.has('z')) return { positions: new Float32Array(), colors: new Float32Array(), count: 0 }
  const data = message.data instanceof Uint8Array ? message.data : new Uint8Array(message.data || [])
  const pointStep = numeric(message.point_step)
  const total = Math.min(numeric(message.width) * Math.max(1, numeric(message.height, 1)), Math.floor(data.byteLength / pointStep))
  if (!pointStep || !total) return { positions: new Float32Array(), colors: new Float32Array(), count: 0 }
  const stride = Math.max(1, Math.ceil(total / maxPoints))
  const outputCount = Math.ceil(total / stride)
  const positions = new Float32Array(outputCount * 3)
  const samples = new Float32Array(outputCount)
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength)
  const littleEndian = !message.is_bigendian
  const readField = (pointOffset, name) => {
    const field = fields.get(name)
    const type = POINT_FIELD_TYPES[numeric(field?.datatype)]
    if (!field || !type) return NaN
    return view[type[0]](pointOffset + numeric(field.offset), littleEndian)
  }
  let written = 0
  let min = Infinity
  let max = -Infinity
  for (let point = 0; point < total; point += stride) {
    const offset = point * pointStep
    const x = readField(offset, 'x'); const y = readField(offset, 'y'); const z = readField(offset, 'z')
    if (![x, y, z].every(Number.isFinite)) continue
    positions[written * 3] = x; positions[written * 3 + 1] = y; positions[written * 3 + 2] = z
    const sample = colorMode === 'intensity' && fields.has('intensity') ? readField(offset, 'intensity') : z
    samples[written] = Number.isFinite(sample) ? sample : 0
    min = Math.min(min, samples[written]); max = Math.max(max, samples[written])
    written += 1
  }
  const colors = new Float32Array(written * 3)
  const span = Math.max(1e-6, max - min)
  for (let index = 0; index < written; index += 1) colors.set(colorRamp((samples[index] - min) / span), index * 3)
  return { positions: positions.slice(0, written * 3), colors, count: written }
}

export function occupancyGridToPoints(message, { threshold = 50, maxPoints = 30_000, frameTransform = null } = {}) {
  const width = numeric(message?.info?.width)
  const height = numeric(message?.info?.height)
  const resolution = numeric(message?.info?.resolution)
  const data = message?.data || []
  if (!width || !height || !resolution || data.length < width * height) return { positions: new Float32Array(), colors: new Float32Array(), count: 0 }
  const occupied = []
  for (let index = 0; index < width * height; index += 1) if (numeric(data[index], -1) >= threshold) occupied.push(index)
  const stride = Math.max(1, Math.ceil(occupied.length / maxPoints))
  const count = Math.ceil(occupied.length / stride)
  const positions = new Float32Array(count * 3)
  const colors = new Float32Array(count * 3)
  const origin = message.info.origin?.position || {}
  const originMatrix = poseMatrix(origin, message.info.origin?.orientation)
  const matrix = frameTransform && originMatrix ? multiply4(frameTransform, originMatrix) : originMatrix
  let written = 0
  for (let item = 0; item < occupied.length; item += stride) {
    const index = occupied[item]
    const x = index % width
    const y = Math.floor(index / width)
    const localX = (x + .5) * resolution; const localY = (y + .5) * resolution
    positions.set([
      matrix[0] * localX + matrix[1] * localY + matrix[3],
      matrix[4] * localX + matrix[5] * localY + matrix[7],
      matrix[8] * localX + matrix[9] * localY + matrix[11] + .08,
    ], written * 3)
    colors.set([1, .22, .16], written * 3)
    written += 1
  }
  return { positions, colors, count: written }
}

export function localizationProcess(status = {}) {
  const quality = status.localization_quality || {}
  const decision = quality.decision || {}
  const source = decision.active_source || 'unavailable'
  const state = status.localization_status || 'unknown'
  const stages = [
    ['map', '地图一致', status.map_id ? 'ok' : 'waiting'],
    ['sensors', '传感器输入', status.sensors?.lidar?.online && status.sensors?.imu?.online ? 'ok' : 'waiting'],
    ['seed', '初始位姿', ['normal', 'relocalized'].includes(state) ? 'ok' : state === 'relocalizing' ? 'active' : 'waiting'],
    ['match', 'NDT匹配', quality.has_converged ? 'ok' : 'active'],
    ['fusion', '融合门控', source === 'unavailable' ? 'warning' : 'ok'],
    ['stable', '稳定确认', decision.absolute_stable ? 'ok' : 'active'],
  ]
  return stages.map(([key, label, tone]) => ({ key, label, tone }))
}
