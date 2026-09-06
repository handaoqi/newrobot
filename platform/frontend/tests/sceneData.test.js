import assert from 'node:assert/strict'
import test from 'node:test'

import {
  assetForClass, createTfTree, normalizeSemanticObjects, occupancyGridToPoints,
  pointCloud2ToArrays, semanticZoomMode, transformPointData, transformPoseTo2D,
} from '../src/services/sceneData.js'

test('semantic zoom uses hysteresis and known aliases use stable assets', () => {
  assert.equal(semanticZoomMode('2d', .5), '2d')
  assert.equal(semanticZoomMode('2d', .56), '3d')
  assert.equal(semanticZoomMode('3d', .44), '2d')
  assert.equal(assetForClass('car').key, 'vehicle')
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
