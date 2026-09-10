import assert from 'node:assert/strict'
import test from 'node:test'

import { audioPreviewResultLabel, waitForAudioPreview } from '../src/services/audioPreviewState.js'

test('polls until an NX-only preview finishes', async () => {
  const calls = []
  const result = await waitForAudioPreview({
    initialCommand: { id: 20, status: 'queued' },
    robotId: 7,
    fetchCommand: async (robotId, commandId) => {
      calls.push([robotId, commandId])
      return { id: commandId, status: 'finished', response_payload: { playback_mode: 'single_nx' } }
    },
    sleep: async () => {},
  })

  assert.deepEqual(calls, [[7, 20]])
  assert.equal(audioPreviewResultLabel(result), 'NX 单音响试播完成（3588不可用）')
})

test('labels a 3588-only preview', () => {
  assert.equal(audioPreviewResultLabel({
    status: 'finished',
    response_payload: { playback_mode: 'single_3588' },
  }), '3588 单音响试播完成（NX不可用）')
})

test('surfaces the device failure reason', () => {
  assert.throws(
    () => audioPreviewResultLabel({ status: 'failed', error_message: 'nx=offline; 3588=offline' }),
    /nx=offline; 3588=offline/,
  )
})

test('times out without cancelling the command', async () => {
  let timestamp = 0
  await assert.rejects(
    waitForAudioPreview({
      initialCommand: { id: 20, status: 'running' },
      robotId: 7,
      fetchCommand: async () => ({ id: 20, status: 'running' }),
      timeoutMs: 1000,
      intervalMs: 1000,
      sleep: async () => { timestamp = 1000 },
      now: () => timestamp,
    }),
    /1秒内未收到播放结果/,
  )
})
