import crypto from 'node:crypto'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

import * as THREE from 'three'
import { GLTFExporter } from 'three/examples/jsm/exporters/GLTFExporter.js'

// GLTFExporter is browser-first. The exporter does not need a DOM for these
// texture-free assets, but it does use FileReader to turn a Blob into an
// ArrayBuffer. This small polyfill keeps the generator dependency-free.
if (typeof globalThis.FileReader === 'undefined') {
  globalThis.FileReader = class FileReader {
    readAsArrayBuffer(blob) {
      blob.arrayBuffer().then(result => {
        this.result = result
        this.onloadend?.()
      }).catch(error => { this.error = error; this.onerror?.(error) })
    }
  }
}

const HERE = path.dirname(fileURLToPath(import.meta.url))
const OUTPUT_DIR = path.resolve(HERE, '../public/scene-assets')
const CATALOG_SCHEMA = 'roamerx.scene-assets.v1'
const CATALOG_VERSION = '2026.09.0'

const COLORS = Object.freeze({
  skin: '#d99a72',
  skinLight: '#f0b48b',
  shirt: '#2dd4bf',
  shirtAlt: '#38bdf8',
  pants: '#334155',
  pantsAlt: '#475569',
  wall: '#9ca3af',
  wallLow: '#d6b98c',
  building: '#64748b',
  buildingLight: '#94a3b8',
  roof: '#b45309',
  roofDark: '#475569',
  road: '#4b5563',
  roadMark: '#e5e7eb',
  vehicle: '#2563eb',
  vehicleAlt: '#f97316',
  vehicleUtility: '#16a34a',
  wheel: '#111827',
  trunk: '#854d0e',
  foliage: '#15803d',
  foliageLight: '#22c55e',
  foliageDark: '#166534',
})

function material(color, roughness = .82) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness: 0 })
}

function addMesh(group, geometry, mat, position = [0, 0, 0], rotation = [0, 0, 0]) {
  const mesh = new THREE.Mesh(geometry, mat)
  mesh.position.set(...position)
  mesh.rotation.set(...rotation)
  group.add(mesh)
  return mesh
}

function box(group, size, position, color, rotation = [0, 0, 0]) {
  return addMesh(group, new THREE.BoxGeometry(...size), material(color), position, rotation)
}

function cylinder(group, radius, height, position, color, radialSegments = 8) {
  return addMesh(group, new THREE.CylinderGeometry(radius, radius, height, radialSegments), material(color), position, [Math.PI / 2, 0, 0])
}

function sphere(group, radius, position, color, detail = 0) {
  return addMesh(group, new THREE.IcosahedronGeometry(radius, detail), material(color), position)
}

function cone(group, radius, height, position, color, radialSegments = 8) {
  return addMesh(group, new THREE.ConeGeometry(radius, height, radialSegments), material(color), position, [Math.PI / 2, 0, 0])
}

function createPerson({ scale = 1, walking = false } = {}) {
  const group = new THREE.Group()
  const width = .5 * scale
  const depth = .32 * scale
  const legHeight = .72 * scale
  box(group, [width, depth, 1.02 * scale], [0, 0, legHeight + .51 * scale], COLORS.shirt)
  sphere(group, .18 * scale, [0, 0, 1.72 * scale], COLORS.skin, 1)
  const legRotation = walking ? .28 : 0
  box(group, [.14 * scale, .18 * scale, legHeight], [-.12 * scale, 0, legHeight / 2], COLORS.pants, [0, legRotation, 0])
  box(group, [.14 * scale, .18 * scale, legHeight], [.12 * scale, 0, legHeight / 2], COLORS.pantsAlt, [0, -legRotation, 0])
  const armRotation = walking ? .38 : 0
  box(group, [.12 * scale, .15 * scale, .68 * scale], [(-width / 2 - .06 * scale), 0, 1.08 * scale], COLORS.shirt, [0, armRotation, 0])
  box(group, [.12 * scale, .15 * scale, .68 * scale], [(width / 2 + .06 * scale), 0, 1.08 * scale], COLORS.shirt, [0, -armRotation, 0])
  return group
}

function createWall(type) {
  const group = new THREE.Group()
  if (type === 'straight') box(group, [4, .2, 2], [2, 0, 1], COLORS.wall)
  if (type === 'low') box(group, [4, .35, .8], [2, 0, .4], COLORS.wallLow)
  if (type === 'corner') {
    box(group, [2.5, .2, 2], [1.25, 0, 1], COLORS.wall)
    box(group, [.2, 2.5, 2], [0, 1.25, 1], COLORS.wall)
  }
  return group
}

function createBuilding(type) {
  const group = new THREE.Group()
  if (type === 'kiosk') {
    box(group, [4, 3, 2.2], [0, 0, 1.1], COLORS.buildingLight)
    box(group, [4.5, 3.5, .35], [0, 0, 2.38], COLORS.roof)
    box(group, [1.2, .12, .7], [0, -1.56, 1.2], COLORS.roofDark)
  }
  if (type === 'service-room') {
    box(group, [8, 5, 3], [0, 0, 1.5], COLORS.building)
    box(group, [8.4, 5.4, .4], [0, 0, 3.2], COLORS.roofDark)
    box(group, [1.2, .12, 2], [0, -2.56, 1], COLORS.roof)
  }
  if (type === 'restroom') {
    box(group, [5, 4, 2.7], [0, 0, 1.35], COLORS.buildingLight)
    box(group, [5.4, 4.4, .35], [0, 0, 2.88], COLORS.roof)
    box(group, [1, .12, 1.8], [-1.2, -2.06, .9], COLORS.roofDark)
    box(group, [1, .12, 1.8], [1.2, -2.06, .9], COLORS.roofDark)
  }
  return group
}

function createVehicle(type) {
  const group = new THREE.Group()
  const config = {
    sedan: { length: 4.5, width: 1.8, height: 1.5, color: COLORS.vehicle, roof: 1.15 },
    van: { length: 5, width: 2, height: 2.2, color: COLORS.vehicleAlt, roof: 1.65 },
    'golf-cart': { length: 2.8, width: 1.3, height: 1.8, color: COLORS.vehicleUtility, roof: 1.55 },
    bus: { length: 10, width: 2.6, height: 3.2, color: COLORS.vehicleAlt, roof: 2.65 },
    bicycle: { length: 1.8, width: .55, height: 1.35, color: COLORS.vehicleUtility, roof: 0 },
  }[type]
  const wheelRadius = type === 'golf-cart' ? .24 : .32
  box(group, [config.length, config.width, .55], [0, 0, .58], config.color)
  box(group, [config.length * .55, config.width * .86, config.height - .9], [config.length * .05, 0, config.roof], config.color)
  for (const x of [-config.length * .32, config.length * .32]) {
    for (const y of [-config.width / 2 - .03, config.width / 2 + .03]) cylinder(group, wheelRadius, .16, [x, y, wheelRadius], COLORS.wheel, 10)
  }
  if (type === 'golf-cart') {
    box(group, [1.9, 1.15, .1], [0, 0, 1.68], COLORS.roof)
    box(group, [.12, 1.05, 1.1], [-.82, 0, 1.1], COLORS.roofDark)
  }
  if (type === 'bicycle') {
    group.clear()
    for (const x of [-.55, .55]) cylinder(group, .3, .06, [x, 0, .35], COLORS.wheel, 12)
    box(group, [.08, .08, .85], [0, 0, .78], COLORS.vehicleUtility, [0, 0, -.55])
    box(group, [.08, .08, .7], [.55, 0, .75], COLORS.vehicleUtility, [0, 0, .55])
    box(group, [.08, .08, .55], [-.1, 0, 1.05], COLORS.vehicleUtility, [0, 0, 1.1])
  }
  return group
}

function createTrafficCone() {
  const group = new THREE.Group()
  box(group, [.55, .55, .08], [0, 0, .04], COLORS.roadMark)
  addMesh(group, new THREE.ConeGeometry(.22, .65, 12), material(COLORS.vehicleAlt), [0, 0, .4])
  box(group, [.25, .25, .06], [0, 0, .52], COLORS.roadMark)
  return group
}

function createBarrier() {
  const group = new THREE.Group()
  box(group, [2, .55, .8], [0, 0, .4], COLORS.wall)
  return group
}

function createGroundAsset(type) {
  const group = new THREE.Group()
  if (type === 'vegetation') {
    cylinder(group, .7, .08, [0, 0, .04], COLORS.foliage, 12)
    for (const x of [-.35, 0, .35]) sphere(group, .18, [x, 0, .2], COLORS.foliageLight, 0)
  } else if (type === 'debris') {
    sphere(group, .45, [0, 0, .3], COLORS.wallLow, 1)
    box(group, [.35, .25, .18], [.35, 0, .12], COLORS.wall)
  } else {
    box(group, [.12, .12, 1.6], [0, 0, .8], COLORS.wall, [0, .25, 0])
  }
  return group
}

function createTree(type) {
  const group = new THREE.Group()
  if (type === 'deciduous') {
    cylinder(group, .22, 2.4, [0, 0, 1.2], COLORS.trunk, 8)
    sphere(group, 1.35, [0, 0, 3.25], COLORS.foliage, 1)
    sphere(group, .85, [-.75, 0, 3.1], COLORS.foliageLight, 0)
    sphere(group, .8, [.75, .1, 3.2], COLORS.foliageDark, 0)
  }
  if (type === 'conifer') {
    cylinder(group, .18, 2.2, [0, 0, 1.1], COLORS.trunk, 8)
    cone(group, 1.2, 2.2, [0, 0, 2.35], COLORS.foliageDark, 8)
    cone(group, .92, 1.8, [0, 0, 3.25], COLORS.foliage, 8)
    cone(group, .62, 1.35, [0, 0, 4.05], COLORS.foliageLight, 8)
  }
  if (type === 'shrub') {
    cylinder(group, .12, .45, [0, 0, .23], COLORS.trunk, 7)
    sphere(group, .7, [0, 0, .72], COLORS.foliage, 0)
    sphere(group, .52, [-.45, .08, .65], COLORS.foliageLight, 0)
    sphere(group, .5, [.45, .05, .66], COLORS.foliageDark, 0)
  }
  return group
}

function roadMesh(length, width, thickness = .08) {
  return new THREE.BoxGeometry(length, width, thickness)
}

function createCurveRoad() {
  const group = new THREE.Group()
  const radius = 3
  const width = 2.5
  const segments = 10
  const vertices = []
  const indices = []
  for (let index = 0; index <= segments; index += 1) {
    const angle = -Math.PI / 2 + (Math.PI / 2) * index / segments
    for (const offset of [-width / 2, width / 2]) {
      const r = radius + offset
      vertices.push(0 + r * Math.cos(angle), radius + r * Math.sin(angle), .04)
    }
  }
  for (let index = 0; index < segments; index += 1) {
    const start = index * 2
    indices.push(start, start + 1, start + 2, start + 1, start + 3, start + 2)
  }
  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3))
  geometry.setIndex(indices)
  geometry.computeVertexNormals()
  addMesh(group, geometry, material(COLORS.road))
  return group
}

function createRoad(type) {
  const group = new THREE.Group()
  if (type === 'straight') {
    addMesh(group, roadMesh(4, 3, .08), material(COLORS.road), [2, 0, .04])
    box(group, [3.6, .08, .015], [2, 0, .09], COLORS.roadMark)
  }
  if (type === 'curve90') addMesh(group, createCurveRoad().children[0].geometry, material(COLORS.road))
  if (type === 'intersection') {
    box(group, [4, 3, .08], [0, 0, .04], COLORS.road)
    box(group, [3, 4, .08], [0, 0, .045], COLORS.road)
    box(group, [3.6, .08, .015], [0, 0, .09], COLORS.roadMark)
    box(group, [.08, 3.6, .015], [0, 0, .095], COLORS.roadMark)
  }
  return group
}

const DEFINITIONS = [
  { asset_id: 'person.adult', category: 'person', label_zh: '成人行人', aliases: ['person', 'pedestrian', 'adult'], role: 'dynamic_entity', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createPerson(), collision: { type: 'capsule', radius_m: .3, height_m: 1.75 } },
  { asset_id: 'person.child', category: 'person', label_zh: '儿童行人', aliases: ['child'], role: 'dynamic_entity', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createPerson({ scale: .7 }), collision: { type: 'capsule', radius_m: .23, height_m: 1.23 } },
  { asset_id: 'person.walking', category: 'person', label_zh: '行走行人', aliases: ['walking_person', 'walking'], role: 'dynamic_entity', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createPerson({ walking: true }), collision: { type: 'capsule', radius_m: .32, height_m: 1.75 } },
  { asset_id: 'wall.straight', category: 'wall', label_zh: '直墙', aliases: ['wall'], role: 'static_environment', anchor: 'segment_start', scale_mode: 'length_only', build: () => createWall('straight'), collision: { type: 'box', dimensions_m: { x: 4, y: .2, z: 2 } } },
  { asset_id: 'wall.low', category: 'wall', label_zh: '矮墙', aliases: ['low_wall'], role: 'static_environment', anchor: 'segment_start', scale_mode: 'length_only', build: () => createWall('low'), collision: { type: 'box', dimensions_m: { x: 4, y: .35, z: .8 } } },
  { asset_id: 'wall.corner', category: 'wall', label_zh: '转角墙', aliases: ['corner_wall'], role: 'static_environment', anchor: 'inner_corner', scale_mode: 'uniform', build: () => createWall('corner'), collision: { type: 'box_union', dimensions_m: { x: 2.5, y: 2.5, z: 2 } } },
  { asset_id: 'building.kiosk', category: 'building', label_zh: '公园小亭', aliases: ['kiosk', 'pavilion'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createBuilding('kiosk'), collision: { type: 'box', dimensions_m: { x: 4.5, y: 3.5, z: 2.73 } } },
  { asset_id: 'building.service-room', category: 'building', label_zh: '服务房', aliases: ['service_building'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createBuilding('service-room'), collision: { type: 'box', dimensions_m: { x: 8.4, y: 5.4, z: 3.4 } } },
  { asset_id: 'building.restroom', category: 'building', label_zh: '公共卫生间', aliases: ['restroom', 'toilet'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createBuilding('restroom'), collision: { type: 'box', dimensions_m: { x: 5.4, y: 4.4, z: 3.08 } } },
  { asset_id: 'vehicle.sedan', category: 'vehicle', label_zh: '轿车', aliases: ['car', 'sedan'], role: 'dynamic_entity', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createVehicle('sedan'), collision: { type: 'box', dimensions_m: { x: 4.5, y: 1.8, z: 1.5 } } },
  { asset_id: 'vehicle.van', category: 'vehicle', label_zh: '面包车', aliases: ['van', 'truck'], role: 'dynamic_entity', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createVehicle('van'), collision: { type: 'box', dimensions_m: { x: 5, y: 2, z: 2.2 } } },
  { asset_id: 'vehicle.golf-cart', category: 'vehicle', label_zh: '电瓶车', aliases: ['golf_cart', 'service_cart'], role: 'dynamic_entity', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createVehicle('golf-cart'), collision: { type: 'box', dimensions_m: { x: 2.8, y: 1.3, z: 1.8 } } },
  { asset_id: 'vehicle.bus', category: 'vehicle', label_zh: '公交车', aliases: ['bus'], role: 'dynamic_entity', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createVehicle('bus'), collision: { type: 'box', dimensions_m: { x: 10, y: 2.6, z: 3.2 } } },
  { asset_id: 'bicycle.standard', category: 'bicycle', label_zh: '自行车', aliases: ['bicycle', 'bike'], role: 'dynamic_entity', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createVehicle('bicycle'), collision: { type: 'box', dimensions_m: { x: 1.8, y: .55, z: 1.35 } } },
  { asset_id: 'traffic-cone.standard', category: 'traffic_cone', label_zh: '交通锥', aliases: ['traffic_cone', 'cone'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: createTrafficCone, collision: { type: 'box', dimensions_m: { x: .55, y: .55, z: .7 } } },
  { asset_id: 'barrier.concrete', category: 'barrier', label_zh: '混凝土隔离墩', aliases: ['barrier', 'concrete_barrier'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'length_only', build: createBarrier, collision: { type: 'box', dimensions_m: { x: 2, y: .55, z: .8 } } },
  { asset_id: 'vegetation.groundcover', category: 'vegetation', label_zh: '地被植被', aliases: ['vegetation', 'groundcover'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createGroundAsset('vegetation'), collision: { type: 'cylinder', radius_m: .7, height_m: .25 } },
  { asset_id: 'debris.pile', category: 'debris', label_zh: '杂物堆', aliases: ['debris'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createGroundAsset('debris'), collision: { type: 'box', dimensions_m: { x: 1, y: 1, z: .6 } } },
  { asset_id: 'wall.vertical-thin', category: 'wall', label_zh: '细立柱', aliases: ['vertical_thin', 'pole'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createGroundAsset('vertical_thin'), collision: { type: 'box', dimensions_m: { x: .2, y: .2, z: 1.6 } } },
  { asset_id: 'tree.deciduous', category: 'tree', label_zh: '乔木', aliases: ['tree', 'deciduous_tree'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createTree('deciduous'), collision: { type: 'cylinder', radius_m: 1.35, height_m: 4.6 } },
  { asset_id: 'tree.conifer', category: 'tree', label_zh: '针叶树', aliases: ['conifer', 'pine'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createTree('conifer'), collision: { type: 'cylinder', radius_m: 1.2, height_m: 4.7 } },
  { asset_id: 'tree.shrub', category: 'tree', label_zh: '灌木', aliases: ['shrub', 'bush'], role: 'static_environment', anchor: 'footprint_center', scale_mode: 'uniform', build: () => createTree('shrub'), collision: { type: 'cylinder', radius_m: .7, height_m: 1.25 } },
  { asset_id: 'road.straight', category: 'road', label_zh: '直道路段', aliases: ['road', 'straight_road'], role: 'navigation_surface', anchor: 'segment_start', scale_mode: 'length_width', build: () => createRoad('straight'), collision: { type: 'box', dimensions_m: { x: 4, y: 3, z: .08 } } },
  { asset_id: 'road.curve90', category: 'road', label_zh: '九十度弯道', aliases: ['curved_road', 'road_curve'], role: 'navigation_surface', anchor: 'segment_start', scale_mode: 'length_width', build: () => createRoad('curve90'), collision: { type: 'mesh_bounds', dimensions_m: { x: 4.25, y: 4.25, z: .08 } } },
  { asset_id: 'road.intersection', category: 'road', label_zh: '十字路口', aliases: ['intersection', 'crossroad'], role: 'navigation_surface', anchor: 'junction_center', scale_mode: 'length_width', build: () => createRoad('intersection'), collision: { type: 'box_union', dimensions_m: { x: 4, y: 4, z: .08 } } },
]

function sha256(buffer) {
  return crypto.createHash('sha256').update(buffer).digest('hex')
}

function round(value) {
  return Math.round(value * 1000) / 1000
}

function inspectGroup(group) {
  group.updateMatrixWorld(true)
  const bounds = new THREE.Box3().setFromObject(group)
  const size = new THREE.Vector3()
  bounds.getSize(size)
  let triangles = 0
  group.traverse(node => {
    if (!node.isMesh || !node.geometry) return
    const count = node.geometry.index?.count || node.geometry.getAttribute('position')?.count || 0
    triangles += Math.floor(count / 3)
  })
  return {
    dimensions_m: { x: round(size.x), y: round(size.y), z: round(size.z) },
    bounds_m: {
      min: { x: round(bounds.min.x), y: round(bounds.min.y), z: round(bounds.min.z) },
      max: { x: round(bounds.max.x), y: round(bounds.max.y), z: round(bounds.max.z) },
    },
    triangle_count: triangles,
  }
}

async function exportGlb(group) {
  const scene = new THREE.Scene()
  scene.name = group.name
  scene.add(group)
  const result = await new GLTFExporter().parseAsync(scene, { binary: true, trs: true, animations: [] })
  return Buffer.from(result)
}

async function main() {
  await fs.mkdir(OUTPUT_DIR, { recursive: true })
  const assets = []
  for (const definition of DEFINITIONS) {
    const group = definition.build()
    group.name = definition.asset_id
    const inspection = inspectGroup(group)
    const buffer = await exportGlb(group)
    if (buffer.subarray(0, 4).toString('ascii') !== 'glTF') throw new Error(`${definition.asset_id}: invalid GLB header`)
    const filename = `${definition.asset_id}.glb`
    await fs.writeFile(path.join(OUTPUT_DIR, filename), buffer)
    assets.push({
      asset_id: definition.asset_id,
      category: definition.category,
      label_zh: definition.label_zh,
      aliases: definition.aliases,
      role: definition.role,
      url: `/scene-assets/${filename}`,
      anchor: definition.anchor,
      scale_mode: definition.scale_mode,
      dimensions_m: inspection.dimensions_m,
      bounds_m: inspection.bounds_m,
      collision: definition.collision,
      triangle_count: inspection.triangle_count,
      byte_size: buffer.byteLength,
      sha256: sha256(buffer),
      revision: 1,
    })
  }
  const catalog = {
    schema: CATALOG_SCHEMA,
    version: CATALOG_VERSION,
    units: 'm',
    coordinate_system: 'right_handed_z_up',
    generated_by: 'platform/frontend/scripts/generate_scene_assets.mjs',
    default_asset_by_category: {
      person: 'person.adult',
      wall: 'wall.straight',
      building: 'building.kiosk',
      vehicle: 'vehicle.sedan',
      tree: 'tree.deciduous',
      road: 'road.straight',
      barrier: 'barrier.concrete',
      traffic_cone: 'traffic-cone.standard',
      vegetation: 'vegetation.groundcover',
      debris: 'debris.pile',
    },
    assets,
  }
  await fs.writeFile(path.join(OUTPUT_DIR, 'catalog.json'), `${JSON.stringify(catalog, null, 2)}\n`, 'utf8')
  const totalBytes = assets.reduce((sum, asset) => sum + asset.byte_size, 0)
  const totalTriangles = assets.reduce((sum, asset) => sum + asset.triangle_count, 0)
  console.log(`Generated ${assets.length} GLB assets (${totalBytes} bytes, ${totalTriangles} triangles) in ${OUTPUT_DIR}`)
}

await main()
