import { expect, test } from '@playwright/test'
import { readFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import net from 'node:net'
import path from 'node:path'

import { LAYOUT_PORTS } from '../../playwright.foxglove.config.js'

const REPO = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../../..')
const DOCS = path.join(REPO, 'docs/yuwang')

const readLayout = (file) => JSON.parse(readFileSync(path.join(DOCS, file), 'utf8'))

/** Panels a layout claims to contain. Read from the file so the test cannot drift from the docs. */
const panelIds = (layout) => Object.keys(layout.configById)

/** Panel id encodes its type: "Image!democam" -> "Image". */
const panelType = (id) => id.split('!')[0]

/**
 * Image panels whose configured topic must show up in the rendered panel.
 *
 * This is the assertion that catches a silently-dropped config key. Lichtblick
 * does not reject an unknown key: it keeps the panel, ignores the key, and lets
 * the panel auto-select a topic instead. "The panel mounted" therefore proves
 * nothing about whether it is showing what the layout asked for. The Image panel
 * prints its topic in the header even with no data source attached, so this works
 * offline.
 */
const imageTopics = (layout) =>
  Object.entries(layout.configById)
    .filter(([id]) => panelType(id) === 'Image')
    .map(([id, config]) => [id, config?.imageMode?.imageTopic])

const isListening = (port, host = '127.0.0.1') =>
  new Promise((resolve) => {
    const socket = net.connect({ port, host })
    const done = (result) => {
      socket.destroy()
      resolve(result)
    }
    socket.setTimeout(2000)
    socket.once('connect', () => done(true))
    socket.once('error', () => done(false))
    socket.once('timeout', () => done(false))
  })

// Console noise that says nothing about whether the layout is valid.
const IGNORED_CONSOLE = [
  /Failed to load resource/i,
  /favicon/i,
  /\[Violation\]/i,
  /downloadable font/i,
  /Content-Security-Policy/i,
]

function watchConsole(page) {
  const errors = []
  page.on('console', (message) => {
    if (message.type() !== 'error') return
    const text = message.text()
    if (!IGNORED_CONSOLE.some((pattern) => pattern.test(text))) errors.push(text)
  })
  page.on('pageerror', (error) => errors.push(`pageerror: ${error.message}`))
  return errors
}

/** Wait for the app shell, not just for load: the bundle boots asynchronously. */
async function waitForApp(page) {
  await expect(page.getByTestId('AppMenuButton')).toBeVisible({ timeout: 120_000 })
}

test.describe('Lichtblick renders the layouts shipped in docs/yuwang', () => {
  for (const [file, port] of Object.entries(LAYOUT_PORTS)) {
    test(`${file} loads with every declared panel`, async ({ page }) => {
      const layout = readLayout(file)
      const expected = panelIds(layout)
      const errors = watchConsole(page)

      await page.goto(`http://127.0.0.1:${port}/`, { waitUntil: 'load' })
      await waitForApp(page)

      // WebGL has to be real, otherwise the 3D panel "passes" by rendering an
      // error card and this whole suite would be worthless.
      const renderer = await page.evaluate(() => {
        const gl = document.createElement('canvas').getContext('webgl2')
        return gl ? gl.getParameter(gl.RENDERER) : null
      })
      expect(renderer, 'headless Chromium produced no WebGL2 context').not.toBeNull()

      // The default layout was injected into index.html by foxglove_web_serve.py.
      // If Lichtblick rejected it, it silently falls back to its welcome layout
      // and none of these panels exist.
      for (const id of expected) {
        await expect(
          page.getByTestId(`panel-mouseenter-container ${id}`),
          `panel ${id} from ${file} was not mounted`,
        ).toBeVisible()
      }

      // A panel that mounted but threw still shows an error card.
      await expect(page.getByText('This panel encountered an unexpected error')).toHaveCount(0)

      // Mounting is not enough: a config key the host does not know is dropped in
      // silence and the panel picks its own topic. Assert the host is actually
      // honouring what the layout declared.
      for (const [id, topic] of imageTopics(layout)) {
        expect(topic, `${id} in ${file} declares no imageMode.imageTopic`).toBeTruthy()
        await expect(
          page.getByTestId(`panel-mouseenter-container ${id}`),
          `${id} in ${file} is not showing its configured topic ${topic}`,
        ).toContainText(topic)
      }

      const fatal = errors.filter((text) => /panel|layout|migrat|webgl|schema/i.test(text))
      expect(fatal, `console errors while rendering ${file}`).toEqual([])
    })
  }

  // Static counterpart to the topic assertion above: catches the mistake in the
  // file rather than in the browser, and names the key that was actually wrong.
  // `cameraTopic` reads plausibly and appears in older Foxglove docs, but 1.28.1
  // has no such key -- the whole Image config lives under `imageMode`.
  test('Image panels configure topics under imageMode, not at the top level', () => {
    const stale = ['cameraTopic', 'imageTopic', 'calibrationTopic', 'transformMarkers', 'rotation']
    for (const file of Object.keys(LAYOUT_PORTS)) {
      const layout = readLayout(file)
      for (const [id, config] of Object.entries(layout.configById)) {
        if (panelType(id) !== 'Image') continue
        expect(config.imageMode?.imageTopic, `${id} in ${file} must set imageMode.imageTopic`).toBeTruthy()
        for (const key of stale) {
          expect(config, `${id} in ${file} sets ${key} at the top level; it belongs under imageMode`).not.toHaveProperty(key)
        }
      }
    }
  })

  test('demo layout shows live data through the /ws proxy', async ({ page }) => {
    const bridgeUp = await isListening(8765)
    test.skip(!bridgeUp, 'no foxglove_bridge on 8765; start it with foxglove_demo.sh web')

    const port = LAYOUT_PORTS['demo_patrol_layout.json']
    const errors = watchConsole(page)

    // Same-origin websocket: the browser only ever talks to the web port, and the
    // bridge stays bound to 127.0.0.1.
    const wsUrl = `ws://127.0.0.1:${port}/ws`
    await page.goto(`http://127.0.0.1:${port}/?ds=foxglove-websocket&ds.url=${encodeURIComponent(wsUrl)}`, {
      waitUntil: 'load',
    })
    await waitForApp(page)

    // The dialog only stays up while there is no source; its disappearance is the
    // signal that the proxied connection was accepted.
    await expect(page.getByTestId('DataSourceDialog')).toHaveCount(0, { timeout: 90_000 })

    // /patrol/status is published only by the fake dog, so real field values here
    // prove data crossed the proxy, not just that a socket opened.
    const rawMessages = page.getByTestId('panel-mouseenter-container RawMessages!demostatus')
    await expect(rawMessages).toBeVisible()
    await expect(rawMessages.getByText('Waiting for next message')).toHaveCount(0, { timeout: 90_000 })

    await expect(page.getByText('This panel encountered an unexpected error')).toHaveCount(0)
    const fatal = errors.filter((text) => /panel|layout|migrat|webgl|schema/i.test(text))
    expect(fatal, 'console errors while streaming live data').toEqual([])
  })
})
