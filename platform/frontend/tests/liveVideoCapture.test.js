import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { fileURLToPath } from 'node:url'

import {
  captureExtensionForMime,
  captureFileStamp,
  captureVideoFrameBlob,
  downloadBlob,
  pickMediaRecorderMimeType,
  recordVideoElementClip,
} from '../src/utils/liveVideoCapture.js'

test('names captured files with a local timestamp', () => {
  assert.equal(captureFileStamp(new Date(2026, 8, 14, 3, 9, 7)), '20260914-030907')
})

test('picks a recorder mime type and file extension', () => {
  const Recorder = { isTypeSupported: (type) => type === 'video/webm' }
  assert.equal(pickMediaRecorderMimeType(Recorder), 'video/webm')
  assert.equal(captureExtensionForMime('video/webm;codecs=vp9'), 'webm')
  assert.equal(captureExtensionForMime('video/mp4'), 'mp4')
  assert.equal(captureExtensionForMime('image/jpeg'), 'jpg')
})

test('captures the current video frame as a jpeg blob', async () => {
  const drawn = []
  const canvas = {
    width: 0,
    height: 0,
    getContext() {
      return {
        drawImage(video, x, y, width, height) {
          drawn.push({ video, x, y, width, height })
        },
      }
    },
    toBlob(callback, type) {
      callback(new Blob(['frame'], { type }))
    },
  }
  const video = { readyState: 2, videoWidth: 640, videoHeight: 360 }
  const blob = await captureVideoFrameBlob(video, { createCanvas: () => canvas })
  assert.equal(canvas.width, 640)
  assert.equal(canvas.height, 360)
  assert.deepEqual(drawn, [{ video, x: 0, y: 0, width: 640, height: 360 }])
  assert.equal(blob.type, 'image/jpeg')
  assert.equal(blob.size, 5)
})

test('records a short clip from the playing video element', async () => {
  class FakeRecorder {
    static isTypeSupported(type) {
      return type === 'video/webm'
    }

    constructor(stream, options) {
      this.stream = stream
      this.options = options
      this.mimeType = options?.mimeType || 'video/webm'
      this.state = 'inactive'
      this.ondataavailable = null
      this.onstop = null
      this.onerror = null
    }

    start() {
      this.state = 'recording'
    }

    stop() {
      this.state = 'inactive'
      this.ondataavailable?.({ data: new Blob(['clip'], { type: this.mimeType }) })
      this.onstop?.()
    }
  }

  const video = { captureStream: () => ({ id: 'live' }) }
  const blob = await recordVideoElementClip(video, {
    durationMs: 3000,
    MediaRecorderCtor: FakeRecorder,
    wait: async () => {},
  })
  assert.equal(blob.type, 'video/webm')
  assert.equal(await blob.text(), 'clip')
})

test('downloads a blob under the given filename', () => {
  const clicks = []
  const revoked = []
  const child = { href: '', download: '', rel: '', click() { clicks.push(this.download) } }
  const doc = {
    body: {
      appended: [],
      appendChild(node) { this.appended.push(node) },
    },
    createElement() { return child },
  }
  doc.body.appendChild = doc.body.appendChild.bind(doc.body)
  child.remove = () => {}
  downloadBlob(new Blob(['file']), '20260914-030907.jpg', {
    document: doc,
    URLCtor: {
      createObjectURL: () => 'blob:capture',
      revokeObjectURL: (url) => revoked.push(url),
    },
  })
  assert.equal(child.href, 'blob:capture')
  assert.deepEqual(clicks, ['20260914-030907.jpg'])
  assert.deepEqual(revoked, ['blob:capture'])
})

test('guard-duty snapshot uses the live player clip capture', () => {
  const page = readFileSync(fileURLToPath(
    new URL('../src/views/GuardDutyPage.vue', import.meta.url),
  ), 'utf8')
  const player = readFileSync(fileURLToPath(
    new URL('../src/components/LiveVideoPlayer.vue', import.meta.url),
  ), 'utf8')
  assert.match(page, /视频截图/)
  assert.match(page, /captureSnapshotAndClip/)
  assert.match(page, /已保存截图和3秒视频/)
  assert.match(player, /clipCaptureActive/)
  assert.match(player, /defineExpose\(\{ returnToLive, captureSnapshotAndClip \}\)/)
})
