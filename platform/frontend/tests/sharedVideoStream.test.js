import assert from 'node:assert/strict'
import test from 'node:test'

import { useSharedVideoStream } from '../src/composables/useSharedVideoStream.js'

test('shared video source keeps one source identity across consumers', () => {
  const stream = useSharedVideoStream()
  stream.clearSharedVideoSource()

  stream.setSharedVideoSource({
    robotId: 7,
    playUrls: { flv: '/live/7.flv', hls: '/live/7.m3u8' },
  })
  const firstKey = stream.sourceKey.value

  stream.setSharedVideoSource({
    robotId: 7,
    playUrls: { flv: '/live/7.flv', hls: '/live/7.m3u8' },
  })

  assert.equal(stream.sourceKey.value, firstKey)
  assert.equal(stream.hasSource.value, true)
  assert.equal(stream.available.value, true)
})

test('changing source resets the shared error state and leaving video pages clears it', () => {
  const stream = useSharedVideoStream()
  stream.setSharedVideoSource({ robotId: 7, playUrls: { flv: '/live/7.flv' } })
  stream.markSharedVideoUnavailable()
  assert.equal(stream.streamUnavailable.value, true)
  assert.equal(stream.available.value, false)

  stream.setSharedVideoSource({ robotId: 8, playUrls: { flv: '/live/8.flv' } })
  assert.equal(stream.streamUnavailable.value, false)
  assert.equal(stream.available.value, true)
  assert.notEqual(stream.sourceKey.value, '')

  stream.clearSharedVideoSource()
  assert.equal(stream.hasSource.value, false)
  assert.equal(stream.available.value, false)
  assert.equal(stream.loading.value, true)
})
