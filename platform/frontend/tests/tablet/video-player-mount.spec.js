import { expect, test } from '@playwright/test'

import { installTabletMocks } from './mock-api.js'

const videoPages = [
  { path: '/dashboard/overview', ready: '.overview-page', stage: '.video-stage' },
  { path: '/dashboard/guard-duty', ready: '.guard-grid', stage: '.guard-video-stage' },
  { path: '/dashboard/remote-control', ready: '.remote-control-page', stage: '.remote-video-stage' },
]

async function expectMountedPlayer(page, pageCase) {
  await expect(page.locator(pageCase.ready)).toBeVisible()
  const player = page.locator(`${pageCase.stage} > .live-video-player`)
  await expect(player).toHaveCount(1)
  await expect(player).toBeVisible()
}

async function navigateFromMenu(page, path) {
  const menuButton = page.locator('.tablet-menu-button')
  if (await menuButton.isVisible()) {
    if (await menuButton.getAttribute('aria-expanded') !== 'true') await menuButton.click()
  }
  await page.locator(`a.menu-item[href="${path}"]`).click()
  await expect(page).toHaveURL(new RegExp(`${path.replaceAll('/', '\\/')}$`))
}

test('monitoring video players mount on direct entry and route changes', async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'Run the video mount regression once in Chromium')

  const teleportWarnings = []
  page.on('console', (message) => {
    if (message.type() === 'warning' && message.text().includes('Teleport')) {
      teleportWarnings.push(message.text())
    }
  })
  await installTabletMocks(page, { authenticated: true })

  for (const pageCase of videoPages) {
    await page.goto(pageCase.path)
    await expectMountedPlayer(page, pageCase)
  }

  await page.goto(videoPages[0].path)
  await expectMountedPlayer(page, videoPages[0])
  for (const pageCase of [...videoPages.slice(1), videoPages[0]]) {
    await navigateFromMenu(page, pageCase.path)
    await expectMountedPlayer(page, pageCase)
  }

  expect(teleportWarnings).toEqual([])
})
