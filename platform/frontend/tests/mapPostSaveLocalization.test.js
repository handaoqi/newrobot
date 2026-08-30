import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { fileURLToPath } from 'node:url'

const source = readFileSync(fileURLToPath(
  new URL('../src/views/MapsPage.vue', import.meta.url),
), 'utf8')

test('map page shows this save localization outcome and map pose', () => {
  assert.match(source, /本次\{\{ validationMappingTypeLabel \}\}定位/)
  assert.match(source, /\{\{ validationCoordinateLabel \}\} 定位位置/)
  assert.match(source, /\{\{ validationPoseLabel \}\}/)
  assert.match(source, /位置偏差/)
  assert.match(source, /航向偏差/)
  assert.match(source, /NDT 匹配误差/)
  assert.match(source, /内点率/)
})

test('map page reports queued check as loading the exact saved map without motion', () => {
  assert.match(source, /正在加载本次地图并定位/)
  assert.match(source, /不会下发运动指令/)
  assert.match(source, /postSaveValidationResult\.value\.accurate === true/)
})
