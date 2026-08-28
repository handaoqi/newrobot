import { defineConfig } from '@playwright/test'
import { fileURLToPath } from 'node:url'
import path from 'node:path'

// Verifies the Lichtblick layouts shipped in docs/yuwang against a real browser.
//
// Kept out of playwright.config.js on purpose: that config is the tablet UI suite
// for this Vue app, is hermetic, and starts its own vite server. This one drives a
// vendored third-party bundle served from the NX runtime tree and is only useful
// on a machine that has run fetch_lichtblick_web.sh. Run it explicitly:
//
//     npm run test:foxglove
//
// It lives in platform/frontend because this is the only package in the repo with
// browser-test infrastructure installed; nothing here touches the Vue app.

const HERE = path.dirname(fileURLToPath(import.meta.url))
const REPO = path.resolve(HERE, '../..')
const SERVE = path.join(REPO, 'robot/script/robot/foxglove_web_serve.py')
const DOCS = path.join(REPO, 'docs/yuwang')

// One server per layout. The bundle injects a single default layout into
// index.html, so serving both from one port would mean driving the import dialog
// by hand -- two ports exercises the real injection path instead.
export const LAYOUT_PORTS = {
  'demo_patrol_layout.json': 8091,
  'nav_debug_layout.json': 8092,
  // Public-dataset demo. Its data source is a 488 MB remote MCAP, so this suite
  // only checks that Lichtblick accepts the layout -- no data source is attached
  // and nothing is fetched over the network.
  'nuscenes_demo_layout.json': 8093,
}

export default defineConfig({
  testDir: './tests/foxglove',
  fullyParallel: false,
  forbidOnly: Boolean(process.env.CI),
  retries: 0,
  workers: 1,
  // The bundle is 170 MB of JS/WASM and this is an NX, not a workstation.
  timeout: 180_000,
  expect: { timeout: 45_000 },
  outputDir: 'test-results/foxglove',
  reporter: [['line'], ['html', { outputFolder: 'playwright-report/foxglove', open: 'never' }]],
  webServer: Object.entries(LAYOUT_PORTS).map(([layout, port]) => ({
    command: `python3 ${SERVE} --port ${port} --default-layout ${path.join(DOCS, layout)}`,
    url: `http://127.0.0.1:${port}/`,
    // Never reuse: foxglove_web_serve.py injects the layout once at startup, so a
    // server left over from an earlier run serves the layout as it was then. Editing
    // a layout JSON and re-running would silently re-verify the stale one and pass.
    // Starting a stdlib http server costs well under a second; a stale pass costs more.
    reuseExistingServer: false,
    timeout: 60_000,
    stdout: 'ignore',
    stderr: 'pipe',
  })),
  use: {
    trace: 'retain-on-failure',
    screenshot: 'only-on-failure',
    video: 'off',
    viewport: { width: 1600, height: 1000 },
    launchOptions: {
      args: [
        '--no-sandbox',
        // The 3D panel is WebGL. Headless Chromium has no GPU here, so ANGLE has
        // to fall back to SwiftShader; without --enable-unsafe-swiftshader recent
        // Chromium refuses the fallback outright.
        '--use-gl=angle',
        '--use-angle=swiftshader',
        '--enable-unsafe-swiftshader',
      ],
      // DISPLAY is set on this box to a remote X server (X11 forwarding). ANGLE
      // sees it, picks the Vulkan/XCB backend, fails xcb_connect against a display
      // it cannot reach, and then never falls back -- every WebGL context creation
      // fails and the 3D panel renders an error card. Clearing DISPLAY for the
      // browser process is what actually fixes it; --ozone-platform=headless alone
      // does not.
      env: Object.fromEntries(Object.entries(process.env).filter(([k]) => k !== 'DISPLAY')),
    },
  },
  projects: [{ name: 'lichtblick-chromium', use: { browserName: 'chromium' } }],
})
