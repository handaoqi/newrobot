import assert from 'node:assert/strict'
import test from 'node:test'

import { normalizeLivePlayUrls, normalizeLiveVideoUrl } from '../src/services/liveVideoUrl.js'

test('same-host stream URLs use the current operator UI origin', () => {
  assert.equal(
    normalizeLiveVideoUrl('https://39.107.250.69/live/cam/hls.m3u8', 'http://39.107.250.69:8088'),
    'http://39.107.250.69:8088/live/cam/hls.m3u8',
  )
})

test('different-host stream URLs remain unchanged for CORS deployments', () => {
  assert.equal(
    normalizeLiveVideoUrl('https://stream.example/live/cam/hls.m3u8', 'http://39.107.250.69:8088'),
    'https://stream.example/live/cam/hls.m3u8',
  )
})

test('play URL normalization preserves unrelated fields', () => {
  assert.deepEqual(
    normalizeLivePlayUrls({ flv: 'https://39.107.250.69/live/cam.flv', hls: 'https://39.107.250.69/live/cam.m3u8', stream_id: 'cam' }, 'http://39.107.250.69:8088'),
    { flv: 'http://39.107.250.69:8088/live/cam.flv', hls: 'http://39.107.250.69:8088/live/cam.m3u8', stream_id: 'cam' },
  )
})

