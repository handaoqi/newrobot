import assert from 'node:assert/strict'
import test from 'node:test'

import { hasRescueMetadata, parseMapDescription } from '../src/utils/mapDescription.js'

test('normal maps with empty rescue metadata are not labelled as rescue maps', () => {
  assert.equal(hasRescueMetadata(JSON.stringify({ rescue: {} })), false)
  assert.equal(hasRescueMetadata(JSON.stringify({ rescue: null })), false)
  assert.equal(hasRescueMetadata(JSON.stringify({ source: 'edge_mapping' })), false)
})

test('explicit and populated rescue metadata are labelled as rescue maps', () => {
  assert.equal(hasRescueMetadata(JSON.stringify({ rescue: true })), true)
  assert.equal(hasRescueMetadata(JSON.stringify({
    rescue: { source_dir: '/maps/failed', valid_keyframe_count: 42 },
  })), true)
})

test('map descriptions accept objects and preserve invalid legacy text', () => {
  const objectDescription = { rescue: {}, scene_scope: 'indoor' }
  assert.equal(parseMapDescription(objectDescription), objectDescription)
  assert.deepEqual(parseMapDescription('legacy map'), { raw: 'legacy map' })
  assert.deepEqual(parseMapDescription('{invalid'), { raw: '{invalid' })
})
