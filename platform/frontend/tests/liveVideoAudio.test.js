import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { fileURLToPath } from 'node:url'

const source = readFileSync(fileURLToPath(
  new URL('../src/components/LiveVideoPlayer.vue', import.meta.url),
), 'utf8')

test('live audio toggles browser mute state from the user gesture', () => {
  const handler = source.match(
    /async function setLiveAudio\(enabled\) \{([\s\S]*?)\n\}\n\nfunction releaseHistoryManifest/,
  )?.[1] || ''

  assert.match(handler, /setBrowserAudioMuted\(!enabled\)/)
  assert.match(handler, /setBrowserAudioMuted\(previousMuted\)/)
  assert.match(handler, /if \(enabled\) reconnectAfterAudioCaptureChange\(\)/)
})

test('audio capture reconnect keeps autoplay muted only until playback starts', () => {
  assert.match(source, /applyBrowserAudio\(element, \{ muted: true, volume: browserAudioVolume\.value \}\)/)
  assert.match(source, /element\.play\(\)\.then\(\(\) => applyBrowserAudio\(element\)\)/)
  assert.match(source, /if \(liveAudioEnabled\.value\) applyBrowserAudio\(element, \{ muted: false \}\)/)
})
