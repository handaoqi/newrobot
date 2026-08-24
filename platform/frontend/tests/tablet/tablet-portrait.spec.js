import { expect, test } from '@playwright/test'

import { installTabletMocks } from './mock-api.js'

const captureOnly = process.env.TABLET_CAPTURE === '1'
const pages = [
  { name: 'login', path: '/login', ready: '.login-card', authenticated: false },
  { name: 'overview', path: '/dashboard/overview', ready: '.overview-page', authenticated: true },
  { name: 'guard-duty', path: '/dashboard/guard-duty', ready: '.guard-grid', authenticated: true },
  { name: 'remote-control', path: '/dashboard/remote-control', ready: '.remote-control-page', authenticated: true },
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
  test(`${pageCase.name} adapts to tablet portrait`, async ({ page }, testInfo) => {
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
      const menuButton = page.getByRole('button', { name: '打开导航菜单' })
      await expect(menuButton).toBeVisible()
      await menuButton.click()
      await expect(menuButton).toHaveAttribute('aria-expanded', 'true')
      await expect(page.locator('#dashboard-navigation')).toHaveClass(/tablet-open/)
      await page.keyboard.press('Escape')
      await expect(menuButton).toHaveAttribute('aria-expanded', 'false')
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
