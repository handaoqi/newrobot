import assert from 'node:assert/strict'
import test from 'node:test'

import {
  hasActiveMappingWorkflow,
  isActiveMappingState,
} from '../src/utils/mappingWorkflowState.js'

test('mapping cancellation stays available throughout every workflow stage', () => {
  for (const mappingState of [
    'starting',
    'origin_waiting',
    'slam_warmup',
    'imu_initializing',
    'waiting_first_keyframe',
    'ready_to_map',
    'mapping',
    'recovering',
    'saving',
    'packaging',
    'uploading',
    'stopping',
  ]) {
    assert.equal(isActiveMappingState(mappingState), true, mappingState)
    assert.equal(hasActiveMappingWorkflow({ mappingState }), true, mappingState)
  }
})

test('in-flight commands and origin preparation count as active workflows', () => {
  assert.equal(hasActiveMappingWorkflow({ commandStatus: 'created' }), true)
  assert.equal(hasActiveMappingWorkflow({ commandStatus: 'executing' }), true)
  assert.equal(hasActiveMappingWorkflow({ originState: 'waiting_fix' }), true)
  assert.equal(hasActiveMappingWorkflow({ originState: 'locked' }), true)
  assert.equal(hasActiveMappingWorkflow({ processAlive: true }), true)
})

test('configuration step disables cancellation after workflow cleanup', () => {
  assert.equal(hasActiveMappingWorkflow(), false)
  assert.equal(hasActiveMappingWorkflow({
    mappingState: 'idle',
    commandStatus: 'succeeded',
    processAlive: false,
    originState: 'cancelled',
  }), false)
  assert.equal(hasActiveMappingWorkflow({
    mappingState: 'exited',
    commandStatus: 'succeeded',
    processAlive: false,
    originState: 'idle',
  }), false)
})
