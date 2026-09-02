import assert from 'node:assert/strict'
import test from 'node:test'

import {
  appendRelocalizationMarker,
  createRelocalizationMarker,
  relocalizationMarkerTitle,
} from '../src/services/relocalizationMarkers.js'

test('relocalization markers dedupe repeated command ids and nearby poses', () => {
  const first = appendRelocalizationMarker([], { x: 1, y: 2, yaw: 0.2 }, {
    source: 'mapping_origin_bounded',
    commandType: 'nav.relocalize',
    commandId: 'cmd-1',
  })
  assert.equal(first.length, 1)
  assert.equal(first[0].sequence, 1)

  const duplicateCommand = appendRelocalizationMarker(first, { x: 1.01, y: 2.01, yaw: 0.2 }, {
    source: 'mapping_origin_bounded',
    commandType: 'nav.relocalize',
    commandId: 'cmd-1',
  })
  assert.equal(duplicateCommand.length, 1)

  const nearbyRuntime = appendRelocalizationMarker(duplicateCommand, { x: 1.02, y: 2.01, yaw: 0.2 }, {
    source: 'runtime',
    commandType: 'runtime.relocalized',
    occurredAt: new Date(Date.parse(first[0].occurredAt) + 1000).toISOString(),
  })
  assert.equal(nearbyRuntime.length, 1)

  const laterRuntime = appendRelocalizationMarker(nearbyRuntime, { x: 4, y: 5, yaw: 1.1 }, {
    source: 'runtime',
    commandType: 'runtime.relocalized',
    occurredAt: new Date(Date.parse(first[0].occurredAt) + 20_000).toISOString(),
  })
  assert.equal(laterRuntime.length, 2)
  assert.match(relocalizationMarkerTitle(createRelocalizationMarker({ x: 4, y: 5, yaw: 1.1 })), /x 4.000/)
})
