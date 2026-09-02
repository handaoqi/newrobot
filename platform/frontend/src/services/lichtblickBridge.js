/**
 * Remote control for the self-hosted Lichtblick host embedded at /foxglove/.
 *
 * The host exposes no postMessage or window API -- the only global it defines is
 * LICHTBLICK_SUITE_DEFAULT_LAYOUT. What makes this page possible is that the
 * bundle is served from our own origin, so the parent document can reach into
 * `iframe.contentDocument` directly: inject CSS to hide the upstream chrome,
 * click the upstream playback controls, and read the upstream clock.
 *
 * Every selector below was verified against Lichtblick 1.28.1 in a real browser.
 * Three of them are counter-intuitive and were each wrong on the first attempt,
 * so they are called out where they are used. Keep this file as the single place
 * that knows anything about the host's DOM: when the pinned host version moves,
 * this is the only file that should need to change.
 */

/** Host DOM contract. Everything we depend on, in one place. */
export const HOST = {
  /** Present once the app shell has booted. */
  appShell: '[data-testid="AppMenuButton"]',
  /** Present only while no data source is attached. */
  sourceDialog: '[data-testid="DataSourceDialog"]',
  /** The host keeps one of these mounted at all times; it is how we hand it a file. */
  fileInput: 'input[type=file]',
  playButton: '[data-testid="play-button"]',
  stepBack: '[data-testid="seek-backward-button"]',
  stepForward: '[data-testid="seek-forward-button"]',
  speedDropdown: '[data-testid="PlaybackSpeedControls-Dropdown"]',
  /** Upstream's own bar. We hide it and drive its controls from ours. */
  playbackBar: '[data-testid="playback-controls"]',
  /**
   * The clock. The primary selector targets the MUI TextField's descendant
   * input; Lichtblick 1.28.1 omits that wrapper testid after a file source is
   * attached, so the disabled text input is the stable fallback.
   */
  clockInput: '[data-testid="PlaybackTime-text"] input, [data-testid="playback-controls"] input[type="text"][disabled]',
  /** Playhead marker; its inline `left` is the position as a percentage. */
  playheadMarker: '[data-testid="playback-slider"] [style*="left"]',
  /**
   * Seek target. Pointer events dispatched at the element carrying the testid do
   * nothing at all -- that is a layout wrapper. The slider that listens is its
   * child.
   */
  sliderOuter: '[data-testid="playback-slider"]',
  sliderInner: '[data-testid="playback-slider"] > div',
  menuItem: '[role=menuitem], li',
  /**
   * Sidebar toggles. These carry the sidebar's state in their aria-label
   * ("Hide left sidebar" when open, "Show ..." when closed), which is the only
   * state readout they expose -- there is no aria-pressed and no selected class.
   */
  leftSidebarButton: '[data-testid="left-sidebar-button"]',
  rightSidebarButton: '[data-testid="right-sidebar-button"]',
  /**
   * Recentre the camera. Rendered by the 3D panel only once the camera has been
   * moved off the configured pose, so its absence is normal and not a warning.
   */
  resetView: '[data-testid="reset-view"]',
}

/** Playback rates the host actually offers. Upstream has no 0.25x. */
export const HOST_SPEEDS = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 0.8, 1, 2]

/** Rates we surface in our own bar, chosen from what the host supports. */
export const SPEEDS = [0.2, 0.5, 1, 2]

const CHROME_STYLE_ID = 'roamerx-embed-chrome'

/**
 * CSS injected into the host document so only the scene remains.
 *
 * The host shell is three rows inside one flex container:
 *   <header> (MuiAppBar, 44px)  -- brand, app menu, source name
 *   [data-testid=sidebars-wrapper]  -- sidebar-left + the panel mosaic
 *   [data-testid=playback-controls] (78px)  -- the bar we replace with our own
 *
 * Hiding the outer rows leaves the mosaic filling the frame. Selectors are
 * testids and semantic tags, never class names: every class here is a content
 * hash (`mui-3epqd6-root`) that changes on each upstream build.
 *
 * The playback bar is the exception, and it is not cosmetic: it must stay
 * *measurable*. `display: none` collapses the slider to zero width, so
 * seekFraction's click lands at `left + width * f` = `left` no matter the
 * fraction, the host divides by that zero width, and the resulting NaN reaches a
 * BigInt conversion that kills the player outright -- the clock goes "Invalid
 * date" and every playback node unmounts. So it is moved out of flow and made
 * transparent instead, which keeps getBoundingClientRect() honest.
 * `pointer-events: none` keeps it from eating clicks meant for the 3D canvas;
 * our own events are dispatched at the element directly, which that does not
 * block.
 *
 * Note the bar is only in the DOM once a data source is attached. That is fine
 * for a stylesheet, but it means anything *reading* these nodes must wait.
 *
 * Hiding `header` covers two different things at once: the app bar, and each
 * panel's own toolbar, which PanelToolbar/index.tsx also renders as a <header>
 * (carrying the panel title, the drag handle and the "Remove panel" menu).
 *
 * The third rule covers what the 3D panel floats over its own canvas
 * (RendererOverlay.tsx): a 2D/3D-and-measure card, an "Inspect objects" card,
 * the publish tool, and "Reset view". Those cards have no wrapper of their own,
 * so each is matched through a button inside it -- the MUI global classes are
 * part of MUI's public class API and, unlike the component classes here, are not
 * content hashes. The publish tool is not merely cosmetic: this console is
 * read-only and must never send a goal to a real dog. "Reset view" comes back as
 * our own button in the transport bar; object inspection belongs to zone 4.
 */
export const CHROME_CSS = `
  header, [role="banner"] { display: none !important; }
  [data-testid="sidebar-left"], [data-testid="sidebar-right"] { display: none !important; }
  .MuiPaper-root:has([data-testid="measure-button"]),
  .MuiPaper-root:has([data-testid^="ExpandingToolbar-"]),
  [data-testid="publish-button"],
  [data-testid="reset-view"] { display: none !important; }
  [data-testid="playback-controls"] {
    position: absolute !important;
    left: 0 !important;
    right: 0 !important;
    bottom: 0 !important;
    opacity: 0 !important;
    pointer-events: none !important;
    z-index: -1 !important;
  }
`

class HostUnavailable extends Error {}

/**
 * @param {() => HTMLIFrameElement | null | undefined} getFrame
 */
export function createLichtblickBridge(getFrame) {
  let warned = new Set()

  /** Same-origin access. Throws rather than returning null so callers say why. */
  const doc = () => {
    const frame = getFrame()
    if (!frame) throw new HostUnavailable('scene frame is not mounted')
    let d
    try {
      d = frame.contentDocument
    } catch (error) {
      // Only reachable if the host stops being same-origin, which would break
      // the whole page design, so say so loudly rather than degrading quietly.
      throw new HostUnavailable(`scene frame is not same-origin: ${error.message}`)
    }
    if (!d) throw new HostUnavailable('scene frame has no document yet')
    return d
  }

  const win = () => getFrame()?.contentWindow

  /**
   * Look up a control. A missing control means the host version moved under us;
   * warn once naming the selector so it is obvious which line of HOST to fix,
   * and let the caller disable the button instead of silently doing nothing.
   */
  const control = (selector) => {
    const el = doc().querySelector(selector)
    if (!el && !warned.has(selector)) {
      warned.add(selector)
      console.warn(`[lichtblick] host control not found: ${selector} -- check HOST in lichtblickBridge.js`)
    }
    return el
  }

  /**
   * The single source of truth for time on this page.
   *
   * `text` is the host's formatted absolute time; `fraction` is the playhead
   * position in [0,1], read off the marker's inline style. Returns nulls rather
   * than throwing so a render loop can poll it unguarded.
   */
  const readClock = () => {
    try {
      const d = doc()
      const text = d.querySelector(HOST.clockInput)?.value ?? null
      const left = d.querySelector(HOST.playheadMarker)?.style?.left
      const fraction = left ? Number.parseFloat(left) / 100 : null
      return { text, fraction: Number.isFinite(fraction) ? fraction : null }
    } catch {
      return { text: null, fraction: null }
    }
  }

  /**
   * Close both sidebars through the host's own toggles.
   *
   * Not stylable: the host mounts each open sidebar as a react-mosaic *tile*
   * (`mosaic-tile` with `inset: 0% 75% 0% 0%`) alongside the panel tiles, so the
   * width is computed in JS from the mosaic tree. Hiding the sidebar element
   * leaves its 25% tile behind as a dead column -- which is exactly what the
   * CSS-only version produced.
   *
   * State comes from the aria-label ("Hide left sidebar" when open); there is no
   * aria-pressed and no selected class. Acting only on "Hide ..." keeps this
   * idempotent instead of a blind toggle.
   */
  const collapseSidebars = () => {
    for (const selector of [HOST.leftSidebarButton, HOST.rightSidebarButton]) {
      const button = doc().querySelector(selector)
      if (/^hide/i.test(button?.getAttribute('aria-label') ?? '')) button.click()
    }
  }

  const waitFor = async (selector, { timeout = 120_000, absent = false } = {}) => {
    const deadline = Date.now() + timeout
    for (;;) {
      let found = null
      try {
        found = doc().querySelector(selector)
      } catch (error) {
        if (Date.now() > deadline) throw error
      }
      if (absent ? !found : found) return found
      if (Date.now() > deadline) {
        throw new Error(`timed out waiting for ${selector}${absent ? ' to disappear' : ''}`)
      }
      await new Promise((resolve) => setTimeout(resolve, 120))
    }
  }

  /**
   * A missing mounted bundle is easy to mistake for a slow first boot: the
   * platform SPA also answers unknown paths with its own index.html. Detect
   * that specific fallback before waiting two minutes for a Lichtblick test id
   * that can never appear.
   */
  const assertHostDocument = () => {
    const d = doc()
    if (d.title === 'Vite + Vue' && d.querySelector('#app') && !d.querySelector('#root')) {
      throw new Error('Lichtblick 宿主未部署：/foxglove/ 返回了平台前端页面')
    }
  }

  return {
    HOST,

    /**
     * Resolves once the host app shell is up inside the frame. Deliberately
     * resolves to nothing: handing out a host DOM node invites callers to hold a
     * reference across a reload, and it cannot cross a Playwright page.evaluate
     * boundary.
     */
    async ready(options) {
      assertHostDocument()
      await waitFor(HOST.appShell, options)
    },

    /** True while the host is still asking the user to pick a data source. */
    needsSource() {
      try {
        return Boolean(doc().querySelector(HOST.sourceDialog))
      } catch {
        return false
      }
    },

    /**
     * Strip the upstream chrome so the frame reads as our own 3D viewport.
     *
     * Idempotent, and safe -- indeed necessary -- to call again after a data
     * source is attached: the host opens its left sidebar at that point, and
     * collapseSidebars is the half of this that CSS cannot do.
     */
    hideChrome() {
      const d = doc()
      d.documentElement.classList.add('roamerx-embedded')
      if (!d.getElementById(CHROME_STYLE_ID)) {
        const style = d.createElement('style')
        style.id = CHROME_STYLE_ID
        style.textContent = CHROME_CSS
        d.head.appendChild(style)
      }
      collapseSidebars()
    },

    collapseSidebars,

    /**
     * Hand the operator's file to the host without an upload and without a
     * second file picker.
     *
     * Assigning to the host's own <input type=file> rather than simulating a
     * drop: a synthetic DragEvent dispatched at document.body never reaches the
     * host's drop handler, because the handler is on a descendant and events
     * dispatched at an ancestor only propagate upward.
     */
    async openLocalFile(file) {
      const w = win()
      if (!w) throw new HostUnavailable('scene frame has no window')
      const input = control(HOST.fileInput)
      if (!input) throw new Error('host exposes no file input')
      const transfer = new w.DataTransfer()
      // Re-wrap through the frame's own File so the host's instanceof checks pass.
      transfer.items.add(new w.File([await file.arrayBuffer()], file.name, { type: file.type }))
      input.files = transfer.files
      input.dispatchEvent(new w.Event('change', { bubbles: true }))
      await waitFor(HOST.sourceDialog, { absent: true, timeout: 120_000 })
    },

    /** Toggle play/pause. The host has one button for both. */
    togglePlay() {
      const button = control(HOST.playButton)
      button?.click()
      return Boolean(button)
    },

    stepForward() {
      const button = control(HOST.stepForward)
      button?.click()
      return Boolean(button)
    },

    stepBackward() {
      const button = control(HOST.stepBack)
      button?.click()
      return Boolean(button)
    },

    /**
     * Jump to a fraction of the recording.
     *
     * Dispatched at HOST.sliderInner, not sliderOuter: the element carrying the
     * testid is a layout wrapper with no listeners, and events sent to it are
     * swallowed without any error -- the playhead simply does not move.
     */
    seekFraction(fraction) {
      const w = win()
      const el = control(HOST.sliderInner) ?? control(HOST.sliderOuter)
      if (!el || !w) return false
      const clamped = Math.min(1, Math.max(0, fraction))
      const rect = el.getBoundingClientRect()
      // A zero-width slider makes every fraction land on the same pixel; the
      // host divides by that width and the NaN it produces crashes its player
      // for good. Refusing is recoverable, seeking into that state is not.
      if (!(rect.width > 0)) {
        console.warn(
          '[lichtblick] seek target has zero width -- the host playback bar is ' +
            'collapsed. It must stay laid out; see CHROME_CSS in lichtblickBridge.js.',
        )
        return false
      }
      const at = (buttons) => ({
        bubbles: true,
        cancelable: true,
        composed: true,
        button: 0,
        buttons,
        pointerId: 1,
        pointerType: 'mouse',
        isPrimary: true,
        clientX: rect.left + rect.width * clamped,
        clientY: rect.top + rect.height / 2,
      })
      el.dispatchEvent(new w.PointerEvent('pointerdown', at(1)))
      el.dispatchEvent(new w.MouseEvent('mousedown', at(1)))
      w.dispatchEvent(new w.PointerEvent('pointermove', at(1)))
      w.dispatchEvent(new w.MouseEvent('mousemove', at(1)))
      w.dispatchEvent(new w.PointerEvent('pointerup', at(0)))
      w.dispatchEvent(new w.MouseEvent('mouseup', at(0)))
      return true
    },

    /** Pick a playback rate from the host's own menu, by its label. */
    async setSpeed(speed) {
      const dropdown = control(HOST.speedDropdown)
      if (!dropdown) return false
      dropdown.click()
      const label = `${speed}×`
      const deadline = Date.now() + 4000
      for (;;) {
        const item = [...doc().querySelectorAll(HOST.menuItem)].find(
          (el) => el.textContent?.trim() === label,
        )
        if (item) {
          item.click()
          return true
        }
        if (Date.now() > deadline) {
          console.warn(`[lichtblick] no "${label}" entry in the host speed menu`)
          return false
        }
        await new Promise((resolve) => setTimeout(resolve, 80))
      }
    },

    /**
     * Recentre the camera on the display frame.
     *
     * Queried directly rather than through control(): the host only renders this
     * button once the camera has been dragged off its configured pose, so a miss
     * means "already centred", not a broken selector.
     */
    resetView() {
      try {
        doc().querySelector(HOST.resetView)?.click()
        return true
      } catch {
        return false
      }
    },

    /** Current speed as the host reports it, e.g. "0.5×". */
    readSpeed() {
      try {
        return control(HOST.speedDropdown)?.textContent?.trim() ?? null
      } catch {
        return null
      }
    },

    readClock,

    /**
     * Poll the host clock on every frame and report changes. Returns a stop
     * function. Polling rather than subscribing because the host emits nothing.
     */
    watchClock(onTick) {
      let running = true
      let last = null
      const tick = () => {
        if (!running) return
        const clock = readClock()
        if (!last || clock.text !== last.text || clock.fraction !== last.fraction) {
          last = clock
          onTick(clock)
        }
        requestAnimationFrame(tick)
      }
      requestAnimationFrame(tick)
      return () => {
        running = false
      }
    },
  }
}
