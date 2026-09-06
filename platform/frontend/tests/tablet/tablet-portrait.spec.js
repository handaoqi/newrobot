import { expect, test } from '@playwright/test'

import { installTabletMocks } from './mock-api.js'

const captureOnly = process.env.TABLET_CAPTURE === '1'
const pages = [
  { name: 'login', path: '/login', ready: '.login-card', authenticated: false },
  { name: 'overview', path: '/dashboard/overview', ready: '.overview-page', authenticated: true },
  { name: 'guard-duty', path: '/dashboard/guard-duty', ready: '.guard-grid', authenticated: true },
  { name: 'remote-control', path: '/dashboard/remote-control', ready: '.remote-control-page', authenticated: true },
]

const shellRoutes = [
  { name: 'remote-development', path: '/dashboard/remote-development' },
  { name: 'analytics', path: '/dashboard/analytics' },
  { name: 'events', path: '/dashboard/events' },
  { name: 'robots', path: '/dashboard/robots' },
  { name: 'tasks', path: '/dashboard/tasks' },
  { name: 'patrol-calendar', path: '/dashboard/tasks/calendar' },
  { name: 'task-execution', path: '/dashboard/task-executions/1' },
  { name: 'maps', path: '/dashboard/tasks/maps' },
  { name: 'routes', path: '/dashboard/tasks/routes' },
  { name: 'zones', path: '/dashboard/tasks/zones' },
  { name: 'tracks', path: '/dashboard/tasks/tracks' },
]

async function collectLayoutReport(page) {
  return page.evaluate(() => {
    const viewportWidth = window.innerWidth
    const root = document.documentElement
    const visible = (element, style, rect) => (
      style.display !== 'none'
      && style.visibility !== 'hidden'
      && Number(style.opacity || 1) > 0
      && rect.width > 0
      && rect.height > 0
    )
    const describe = (element) => {
      const id = element.id ? `#${element.id}` : ''
      const classes = [...element.classList].slice(0, 3).map((name) => `.${name}`).join('')
      return `${element.tagName.toLowerCase()}${id}${classes}`
    }

    const horizontalOffenders = []
    for (const element of document.body.querySelectorAll('*')) {
      const style = getComputedStyle(element)
      const rect = element.getBoundingClientRect()
      if (!visible(element, style, rect)) continue
      if (rect.left < -2 || rect.right > viewportWidth + 2) {
        const scrollContainer = element.closest('[data-allow-horizontal-scroll], .menu-list')
        if (!scrollContainer) {
          horizontalOffenders.push({ selector: describe(element), left: Math.round(rect.left), right: Math.round(rect.right) })
        }
      }
    }

    const smallTouchTargets = []
    const targetSelector = 'button, input:not([type="hidden"]):not([type="checkbox"]):not([type="radio"]), select, textarea, a.menu-item'
    for (const element of document.querySelectorAll(targetSelector)) {
      const style = getComputedStyle(element)
      const rect = element.getBoundingClientRect()
      if (!visible(element, style, rect)) continue
      if (rect.width < 44 || rect.height < 44) {
        smallTouchTargets.push({ selector: describe(element), width: Math.round(rect.width), height: Math.round(rect.height) })
      }
    }

    return {
      viewport: { width: viewportWidth, height: window.innerHeight },
      document: { width: root.scrollWidth, height: root.scrollHeight },
      hasHorizontalScroll: root.scrollWidth > root.clientWidth + 1,
      horizontalOffenders: horizontalOffenders.slice(0, 30),
      smallTouchTargets: smallTouchTargets.slice(0, 30),
    }
  })
}

for (const pageCase of pages) {
  test(`${pageCase.name} adapts to tablet viewport`, async ({ page }, testInfo) => {
    await installTabletMocks(page, pageCase)
    await page.goto(pageCase.path)
    await expect(page.locator(pageCase.ready)).toBeVisible()
    await page.emulateMedia({ reducedMotion: 'reduce', colorScheme: 'light' })
    await page.addStyleTag({ content: '*, *::before, *::after { animation: none !important; transition: none !important; caret-color: transparent !important; }' })

    const report = await collectLayoutReport(page)
    await testInfo.attach('layout-report', {
      body: Buffer.from(JSON.stringify(report, null, 2)),
      contentType: 'application/json',
    })

    expect(report.hasHorizontalScroll, JSON.stringify(report, null, 2)).toBe(false)
    expect(report.horizontalOffenders, JSON.stringify(report, null, 2)).toEqual([])
    expect(report.smallTouchTargets, JSON.stringify(report, null, 2)).toEqual([])

    if (pageCase.authenticated) {
      const menuButton = page.locator('.tablet-menu-button')
      const viewport = page.viewportSize()
      if (viewport.height >= viewport.width) {
        await expect(menuButton).toBeVisible()
        await expect(menuButton).toHaveAttribute('aria-label', '打开导航菜单')
        await expect(menuButton).toHaveCSS('border-radius', '50%')
        await expect(menuButton).toHaveJSProperty('offsetWidth', 56)
        await expect(menuButton).toHaveJSProperty('offsetHeight', 56)
        await menuButton.click()
        await expect(menuButton).toHaveAttribute('aria-expanded', 'true')
        await expect(menuButton).toHaveAttribute('aria-label', '关闭导航菜单')
        await expect(page.locator('#dashboard-navigation')).toHaveClass(/tablet-open/)
        await page.keyboard.press('Escape')
        await expect(menuButton).toHaveAttribute('aria-expanded', 'false')
        await expect(menuButton).toHaveAttribute('aria-label', '打开导航菜单')
        await expect(menuButton).toBeFocused()
      } else {
        await expect(menuButton).toBeHidden()
        await expect(page.locator('#dashboard-navigation')).toBeVisible()
      }
    }

    if (pageCase.name === 'overview') {
      await expect(page.getByTestId('device-battery')).toHaveText('63%')
    }

    if (pageCase.name === 'guard-duty') {
      const actionMetrics = await page.locator('.guard-task-actions > button').evaluateAll((buttons) => buttons.map((button) => {
        const rect = button.getBoundingClientRect()
        const style = getComputedStyle(button)
        return {
          top: Math.round(rect.top),
          width: Math.round(rect.width),
          fontSize: Number.parseFloat(style.fontSize),
          whiteSpace: style.whiteSpace,
          fits: button.scrollWidth <= button.clientWidth + 1,
        }
      }))
      expect(actionMetrics).toHaveLength(3)
      expect(new Set(actionMetrics.map(item => item.top)).size, JSON.stringify(actionMetrics, null, 2)).toBe(1)
      expect(actionMetrics.every(item => item.width > 0 && item.fontSize <= 12 && item.whiteSpace === 'nowrap' && item.fits), JSON.stringify(actionMetrics, null, 2)).toBe(true)
    }

    const fullPagePath = testInfo.outputPath(`${pageCase.name}-full-page.png`)
    await page.screenshot({ path: fullPagePath, fullPage: true, animations: 'disabled', caret: 'hide' })
    await testInfo.attach('full-page', { path: fullPagePath, contentType: 'image/png' })

    if (!captureOnly) {
      await expect(page).toHaveScreenshot(`${pageCase.name}-viewport.png`, { fullPage: false })
    }

    // Navigate away while request interception is still active so page-level
    // pollers are stopped before the browser context is torn down.
    await page.goto('about:blank')
  })
}

for (const routeCase of shellRoutes) {
  test(`${routeCase.name} keeps the left desktop menu within the 2000x1200 landscape viewport`, async ({ page }, testInfo) => {
    test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'The full route shell matrix runs at the target landscape tablet size')
    await installTabletMocks(page, { authenticated: true })
    await page.goto(routeCase.path)
    await expect(page.locator('.tablet-app-bar')).toBeHidden()
    await expect(page.locator('#dashboard-navigation')).toBeVisible()
    await page.waitForTimeout(700)
    await page.emulateMedia({ reducedMotion: 'reduce', colorScheme: 'light' })
    await page.addStyleTag({ content: '*, *::before, *::after { animation: none !important; transition: none !important; caret-color: transparent !important; }' })

    const report = await collectLayoutReport(page)
    await testInfo.attach('layout-report', {
      body: Buffer.from(JSON.stringify(report, null, 2)),
      contentType: 'application/json',
    })

    expect(report.hasHorizontalScroll, JSON.stringify(report, null, 2)).toBe(false)
    expect(report.horizontalOffenders, JSON.stringify(report, null, 2)).toEqual([])
    expect(report.smallTouchTargets, JSON.stringify(report, null, 2)).toEqual([])

    await expect(page.locator('.menu-list')).toHaveCSS('display', 'grid')
    const taskMenu = page.getByRole('button', { name: '巡检任务' })
    const taskLink = page.getByRole('link', { name: '任务列表', exact: true })
    if (!(await taskLink.isVisible())) await taskMenu.click()
    await expect(taskLink).toBeVisible()
    await page.goto('about:blank')
  })
}

test('portrait orientation uses the right-side floating menu even with a desktop user agent', async ({ browser }, testInfo) => {
  test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'The device-mode regression runs once in Chromium')
  const context = await browser.newContext({
    baseURL: 'http://127.0.0.1:4173',
    colorScheme: 'light',
    locale: 'zh-CN',
    timezoneId: 'Asia/Shanghai',
    hasTouch: false,
    isMobile: false,
    userAgent: 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36',
    viewport: { width: 800, height: 1000 },
    screen: { width: 800, height: 1000 },
  })
  try {
    const page = await context.newPage()
    await installTabletMocks(page, { authenticated: true })
    await page.goto('/dashboard/overview')
    const menuButton = page.locator('.tablet-menu-button')
    await expect(menuButton).toBeVisible()
    await menuButton.click()
    const sidebar = page.locator('#dashboard-navigation')
    await expect(sidebar).toHaveClass(/tablet-open/)
    await expect.poll(async () => {
      const sidebarBox = await sidebar.boundingBox()
      const viewportWidth = await page.evaluate(() => window.innerWidth)
      return sidebarBox ? Math.abs((sidebarBox.x + sidebarBox.width) - viewportWidth) : Number.POSITIVE_INFINITY
    }).toBeLessThanOrEqual(1)
    await expect(page.locator('.menu-list')).toHaveCSS('display', 'grid')
    await expect(page.locator('.menu-list')).toHaveJSProperty('scrollWidth', await page.locator('.menu-list').evaluate(element => element.clientWidth))
    const report = await collectLayoutReport(page)
    expect(report.hasHorizontalScroll, JSON.stringify(report, null, 2)).toBe(false)
    expect(report.horizontalOffenders, JSON.stringify(report, null, 2)).toEqual([])
  } finally {
    await context.close()
  }
})

test('landscape orientation uses the permanent left desktop menu even on mobile', async ({ browser }, testInfo) => {
  test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'The device-mode regression runs once in Chromium')
  const context = await browser.newContext({
    baseURL: 'http://127.0.0.1:4173',
    colorScheme: 'light',
    locale: 'zh-CN',
    timezoneId: 'Asia/Shanghai',
    hasTouch: true,
    isMobile: true,
    userAgent: 'Mozilla/5.0 (Linux; Android 14; Tablet) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36',
    viewport: { width: 1200, height: 800 },
    screen: { width: 1200, height: 800 },
  })
  try {
    const page = await context.newPage()
    await installTabletMocks(page, { authenticated: true })
    await page.goto('/dashboard/overview')
    const menuButton = page.locator('.tablet-menu-button')
    await expect(menuButton).toBeHidden()
    const sidebar = page.locator('#dashboard-navigation')
    await expect(sidebar).toBeVisible()
    await expect.poll(async () => {
      const sidebarBox = await sidebar.boundingBox()
      return sidebarBox ? Math.abs(sidebarBox.x) : Number.POSITIVE_INFINITY
    }).toBeLessThanOrEqual(1)
  } finally {
    await context.close()
  }
})

test('remote development keeps live output ahead of contained history on a phone', async ({ browser }, testInfo) => {
  test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'The handset regression runs once in Chromium')
  const context = await browser.newContext({
    baseURL: 'http://127.0.0.1:4173',
    colorScheme: 'light',
    locale: 'zh-CN',
    timezoneId: 'Asia/Shanghai',
    hasTouch: true,
    isMobile: true,
    viewport: { width: 390, height: 844 },
    screen: { width: 390, height: 844 },
  })
  try {
    const page = await context.newPage()
    await installTabletMocks(page, { authenticated: true })
    await page.goto('/dashboard/remote-development')
    await expect(page.locator('.dev-history')).toBeVisible()
    await expect(page.locator('.terminal-shell')).toBeVisible()
    const layout = await page.evaluate(() => {
      const history = document.querySelector('.dev-history')
      const terminal = document.querySelector('.terminal-shell')
      const historyRect = history.getBoundingClientRect()
      const terminalRect = terminal.getBoundingClientRect()
      const style = getComputedStyle(history)
      return {
        historyTop: historyRect.top,
        terminalBottom: terminalRect.bottom,
        historyScrollHeight: history.scrollHeight,
        historyClientHeight: history.clientHeight,
        overflowY: style.overflowY,
      }
    })
    expect(layout.historyTop).toBeGreaterThanOrEqual(layout.terminalBottom)
    expect(layout.historyScrollHeight).toBeGreaterThan(layout.historyClientHeight)
    expect(['auto', 'scroll']).toContain(layout.overflowY)
    await page.goto('about:blank')
  } finally {
    await context.close()
  }
})

test('a square viewport follows the portrait right-side floating menu rule', async ({ browser }, testInfo) => {
  test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'The orientation boundary regression runs once in Chromium')
  const context = await browser.newContext({
    baseURL: 'http://127.0.0.1:4173',
    colorScheme: 'light',
    locale: 'zh-CN',
    timezoneId: 'Asia/Shanghai',
    hasTouch: true,
    isMobile: true,
    userAgent: 'Mozilla/5.0 (Linux; Android 14; Tablet) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36',
    viewport: { width: 900, height: 900 },
    screen: { width: 900, height: 900 },
  })
  try {
    const page = await context.newPage()
    await installTabletMocks(page, { authenticated: true })
    await page.goto('/dashboard/overview')
    const menuButton = page.locator('.tablet-menu-button')
    await expect(menuButton).toBeVisible()
    await menuButton.click()
    const sidebar = page.locator('#dashboard-navigation')
    await expect(sidebar).toHaveClass(/tablet-open/)
    await expect.poll(async () => {
      const sidebarBox = await sidebar.boundingBox()
      const viewportWidth = await page.evaluate(() => window.innerWidth)
      return sidebarBox ? Math.abs((sidebarBox.x + sidebarBox.width) - viewportWidth) : Number.POSITIVE_INFINITY
    }).toBeLessThanOrEqual(1)
  } finally {
    await context.close()
  }
})
