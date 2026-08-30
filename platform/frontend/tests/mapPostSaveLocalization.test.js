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
  assert.match(source, /最大位置跳变/)
  assert.match(source, /最大航向跳变/)
  assert.match(source, /判定代码/)
  assert.match(source, /候选地图/)
})

test('map page reports queued check as loading the exact saved map without navigation motion', () => {
  assert.match(source, /正在加载本次地图并定位/)
  assert.match(source, /不会启动导航或下发速度指令/)
  assert.match(source, /postSaveValidationResult\.value\.accurate === true/)
  assert.match(source, /保存终点初始位姿已发布/)
  assert.match(source, /候选地图 RTK\/ENU 初始化/)
})
