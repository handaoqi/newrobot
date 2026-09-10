import assert from 'node:assert/strict'
import test from 'node:test'

import {
  assetForClass, createTfTree, filterDynamicSceneObjects, filterStaticSceneAssets, isRobotMoving,
  inferStaticSceneAssets, mapPointToWgs84, normalizeSemanticObjects, occupancyGridToPoints,
  normalizeSceneAssetCatalog, normalizeSceneAssetInstance, pointCloud2ToArrays,
  sceneAssetIdForClass, semanticZoomMode, transformPointData, transformPoseTo2D,
} from '../src/services/sceneData.js'

test('semantic zoom uses hysteresis and known aliases use stable assets', () => {
  assert.equal(semanticZoomMode('2d', .5), '2d')
  assert.equal(semanticZoomMode('2d', .56), '3d')
  assert.equal(semanticZoomMode('3d', .44), '2d')
  assert.equal(assetForClass('car').key, 'vehicle')
  assert.equal(assetForClass('tree.deciduous').key, 'tree')
  assert.equal(assetForClass('road.curve90').key, 'road')
  assert.equal(assetForClass('not-trained').key, 'unknown_obstacle')
})

test('TF tree composes sensor frames into map and refuses incomplete chains', () => {
  const tree = createTfTree('map')
  tree.update({ transforms: [
    { header: { frame_id: 'map' }, child_frame_id: 'odom', transform: { translation: { x: 10, y: 0, z: 0 }, rotation: { w: 1 } } },
    { header: { frame_id: 'odom' }, child_frame_id: 'lidar', transform: { translation: { x: 1, y: 2, z: 0 }, rotation: { w: 1 } } },
  ] })
  const matrix = tree.matrixFrom('/lidar')
  const cloud = transformPointData({ positions: new Float32Array([1, 1, 0]), colors: new Float32Array(3), count: 1 }, matrix)
  assert.deepEqual([...cloud.positions], [12, 3, 0])
  assert.deepEqual(transformPoseTo2D({ x: 1, y: 1 }, { w: 1 }, matrix), { x: 12, y: 3, z: 0, yaw: 0 })
  assert.equal(tree.matrixFrom('camera_missing'), null)
})

test('point cloud decoder samples finite xyz and produces pseudo-colour', () => {
  const data = new ArrayBuffer(32)
  const view = new DataView(data)
  ;[1, 2, 0, 4, 3, 4, 2, 8].forEach((value, index) => view.setFloat32(index * 4, value, true))
  const cloud = pointCloud2ToArrays({
    width: 2, height: 1, point_step: 16, is_bigendian: false, data: new Uint8Array(data),
    fields: ['x', 'y', 'z', 'intensity'].map((name, index) => ({ name, offset: index * 4, datatype: 7 })),
  }, { colorMode: 'intensity' })
  assert.equal(cloud.count, 2)
  assert.deepEqual([...cloud.positions], [1, 2, 0, 3, 4, 2])
  assert.equal(cloud.colors.length, 6)
})

test('occupancy and semantic messages become bounded scene primitives', () => {
  const obstacles = occupancyGridToPoints({
    info: { width: 3, height: 2, resolution: .5, origin: { position: { x: -1, y: 2 } } },
    data: [0, 60, -1, 100, 20, 70],
  })
  assert.equal(obstacles.count, 3)
  assert.deepEqual([...obstacles.positions.slice(0, 2)], [-.25, 2.25])
  assert.ok(Math.abs(obstacles.positions[2] - .08) < 1e-6)

  const objects = normalizeSemanticObjects({ objects: [{ track_id: 'p1', class_name: 'person', confidence: .9, pose: { position: { x: 1, y: 2, z: 0 } } }] })
  assert.equal(objects[0].id, 'p1')
  assert.equal(objects[0].dimensions.z, 1.7)
})

test('scene asset catalog resolves stable ids, aliases, and normalized instances', () => {
  const catalog = normalizeSceneAssetCatalog({
    schema: 'roamerx.scene-assets.v1',
    version: 'test',
    default_asset_by_category: { person: 'person.adult' },
    assets: [
      { asset_id: 'person.adult', category: 'person', aliases: ['person', 'pedestrian'], url: '/scene-assets/person.adult.glb' },
      { asset_id: 'tree.deciduous', category: 'tree', aliases: ['tree'], url: '/scene-assets/tree.deciduous.glb' },
    ],
  })
  assert.equal(sceneAssetIdForClass('pedestrian', catalog), 'person.adult')
  assert.equal(sceneAssetIdForClass('tree.deciduous', catalog), 'tree.deciduous')
  assert.equal(sceneAssetIdForClass('person', catalog), 'person.adult')
  const instance = normalizeSceneAssetInstance({
    asset_id: 'tree.deciduous', position: { x: 2, y: 3 }, scale: 1.5,
  })
  assert.equal(instance.assetId, 'tree.deciduous')
  assert.deepEqual(instance.position, { x: 2, y: 3, z: 0 })
  assert.equal(instance.scale, 1.5)
})

test('scene modes keep static assets separate from motion-gated objects', () => {
  const items = [
    { asset_id: 'wall.straight' },
    { asset_id: 'building.kiosk' },
    { class_name: 'tree' },
    { class_name: 'road' },
    { class_name: 'person' },
    { class_name: 'vehicle' },
    { class_name: 'unknown_obstacle' },
  ]
  assert.deepEqual(filterStaticSceneAssets(items).map(item => item.asset_id || item.class_name), [
    'wall.straight', 'building.kiosk', 'tree', 'road',
  ])
  assert.deepEqual(filterDynamicSceneObjects(items).map(item => item.class_name), [])
  assert.deepEqual(filterDynamicSceneObjects(items, { robotMoving: true }).map(item => item.class_name), ['person', 'vehicle'])
  assert.equal(isRobotMoving({ speed_mps: .05 }), false)
  assert.equal(isRobotMoving({ speed_mps: .051 }), true)
})

test('point-cloud preview infers conservative static GLB instances', () => {
  const points = []
  for (let x = 4; x < 5.8; x += .25) for (let y = 4; y < 5.8; y += .25) for (let z = 0; z < 4.5; z += .35) points.push(x, y, z)
  for (let x = 12; x < 14.4; x += .25) for (let y = 1; y < 3.4; y += .25) for (let z = 0; z < 3.5; z += .3) points.push(x, y, z)
  for (let x = 0; x < 20; x += .4) for (let y = 8; y < 10; y += .4) points.push(x, y, 0)
  const result = inferStaticSceneAssets(new Float32Array(points))
  assert.equal(result.source, 'point_cloud_heuristic')
  assert.equal(result.status, 'generated')
  assert.ok(result.assets.some(item => item.class_name === 'tree' && item.asset_id === 'tree.deciduous'))
  assert.ok(result.assets.some(item => item.class_name === 'building' && item.asset_id === 'building.kiosk'))
  assert.ok(result.assets.some(item => item.class_name === 'road' && item.asset_id === 'road.straight'))
  assert.ok(result.assets.every(item => !['person', 'vehicle', 'bicycle'].includes(item.class_name)))
})

test('map coordinates transform through the stored ENU-to-map alignment', () => {
  const geo = {
    available: true,
    origin_latitude: 39.9,
    origin_longitude: 116.4,
    map_offset_x: 1.25,
    map_offset_y: -.75,
    enu_to_map_yaw: 0,
  }
  const point = mapPointToWgs84({ x: 1.25, y: -.75 }, geo)
  assert.ok(Math.abs(point.latitude - 39.9) < 1e-10)
  assert.ok(Math.abs(point.longitude - 116.4) < 1e-10)
  const north = mapPointToWgs84({ x: 1.25, y: 99.25 }, geo)
  assert.ok(north.latitude > point.latitude)
})
