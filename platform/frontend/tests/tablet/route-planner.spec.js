import { expect, test } from '@playwright/test'

import { installTabletMocks } from './mock-api.js'

test('saved route hydrates waypoint details and keeps per-waypoint global controllers independent', async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'Run the route planner regression once in Chromium')
  await installTabletMocks(page, { authenticated: true })

  await page.goto('/dashboard/tasks/routes')

  const waypointItems = page.locator('.waypoint-item')
  await expect(waypointItems).toHaveCount(3)
  await expect(page.locator('.waypoint-details')).toHaveCount(3)
  await expect(page.locator('.waypoint-details').first()).toBeVisible()
  await expect(page.locator('.waypoint-load-error')).toHaveCount(0)

  const firstWaypoint = waypointItems.first()
  const waypointList = page.locator('.waypoint-list')
  const fixedListHeight = await waypointList.evaluate(element => element.getBoundingClientRect().height)
  await firstWaypoint.locator('.waypoint-expand-toggle').click()
  await expect(firstWaypoint.locator('.waypoint-details')).toHaveCount(0)
  await expect.poll(() => waypointList.evaluate(element => element.getBoundingClientRect().height)).toBe(fixedListHeight)
  await firstWaypoint.locator('.waypoint-expand-toggle').click()
  await expect(firstWaypoint.locator('.waypoint-details')).toBeVisible()

  const waypointControllers = waypointItems.locator('label').filter({ hasText: '全局规划器' }).locator('select')
  await expect(waypointControllers.nth(0)).toHaveValue('theta_star')
  await expect(waypointControllers.nth(1)).toHaveValue('navfn')
  await expect(waypointControllers.nth(2)).toHaveValue('navfn')

  await waypointControllers.nth(1).selectOption('theta_star')

  await expect(waypointControllers.nth(0)).toHaveValue('theta_star')
  await expect(waypointControllers.nth(1)).toHaveValue('theta_star')
  await expect(waypointControllers.nth(2)).toHaveValue('navfn')

  page.on('dialog', dialog => dialog.accept())
  const saveRequestPromise = page.waitForRequest(request => (
    request.method() === 'PUT' && new URL(request.url()).pathname === '/api/routes/10/'
  ))
  await page.getByRole('button', { name: '保存路线', exact: true }).click()
  const saveRequest = await saveRequestPromise
  expect(saveRequest.postDataJSON().waypoints.map(point => point.global_controller)).toEqual([
    'theta_star',
    'theta_star',
    'navfn',
  ])
})

test('drill record follows the compact waypoint panel and aligns with keyframes', async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'Run the desktop grid regression once in Chromium')
  await installTabletMocks(page, { authenticated: true, mapThumbnailUrl: '/images/map.png' })

  await page.goto('/dashboard/tasks/routes')

  const keyframes = page.locator('.route-keyframe-row')
  const waypointPanel = page.locator('.route-step-3')
  const timeline = page.locator('.route-timeline-column')
  const navigationTest = page.locator('.route-step-5')
  await expect(keyframes).toBeVisible()
  await expect(timeline).toBeVisible()
  await expect(navigationTest).toBeVisible()

  const positions = await Promise.all([waypointPanel, keyframes, timeline, navigationTest].map(async locator => locator.boundingBox()))
  const [waypointBox, keyframeBox, timelineBox, navigationBox] = positions
  expect(timelineBox.y).toBeGreaterThanOrEqual(waypointBox.y + waypointBox.height)
  expect(timelineBox.y - (waypointBox.y + waypointBox.height)).toBeLessThanOrEqual(10)
  expect(Math.abs(timelineBox.y + timelineBox.height - (keyframeBox.y + keyframeBox.height))).toBeLessThanOrEqual(2)
  expect(navigationBox.y).toBeGreaterThan(Math.max(keyframeBox.y + keyframeBox.height, timelineBox.y + timelineBox.height))

  const toolbar = page.locator('.map-toolbar')
  await expect(toolbar).toBeVisible()
  const toolbarLayout = await toolbar.evaluate(element => ({
    flexWrap: getComputedStyle(element).flexWrap,
    height: element.getBoundingClientRect().height,
  }))
  expect(toolbarLayout.flexWrap).toBe('nowrap')
  expect(toolbarLayout.height).toBeLessThanOrEqual(54)
})

test('real preview shows target dispatch and waypoint arrival events in the shared drill record', async ({ page }, testInfo) => {
  test.skip(testInfo.project.name !== 'tablet-2000-landscape-chromium', 'Run the route preview regression once in Chromium')
  const routeExecution = {
    id: 'route-preview-live-10',
    route: 10,
    route_name: '南门—主步道—活动广场',
    state: 'completed',
    completed_waypoints: 3,
    current_waypoint_index: 2,
    created_at: '2026-08-24T08:30:00+08:00',
    started_at: '2026-08-24T08:30:01+08:00',
    finished_at: '2026-08-24T08:30:12+08:00',
    route_snapshot: {
      waypoints: [
        { map_point_number: 1, x: 1, y: 1 },
        { map_point_number: 2, x: 3, y: 2 },
        { map_point_number: 3, x: 5, y: 4 },
      ],
    },
    events: [
      { id: 1, event_type: 'task.created', state_version: 1, occurred_at: '2026-08-24T08:30:00+08:00', payload: {} },
      { id: 2, event_type: 'task.started', state_version: 2, occurred_at: '2026-08-24T08:30:01+08:00', payload: {} },
      { id: 3, event_type: 'task.target_dispatched', state_version: 3, occurred_at: '2026-08-24T08:30:02+08:00', payload: { execution_waypoint_index: 0, waypoint: { map_point_number: 1, x: 1, y: 1 } } },
      { id: 4, event_type: 'task.waypoint_reached', state_version: 4, occurred_at: '2026-08-24T08:30:04+08:00', payload: { execution_waypoint_index: 0, waypoint: { map_point_number: 1 } } },
      { id: 5, event_type: 'task.target_dispatched', state_version: 5, occurred_at: '2026-08-24T08:30:05+08:00', payload: { execution_waypoint_index: 1, waypoint: { map_point_number: 2, x: 3, y: 2 } } },
      { id: 6, event_type: 'task.waypoint_reached', state_version: 6, occurred_at: '2026-08-24T08:30:08+08:00', payload: { execution_waypoint_index: 1, waypoint: { map_point_number: 2 } } },
      { id: 7, event_type: 'task.completed', state_version: 7, occurred_at: '2026-08-24T08:30:12+08:00', payload: {} },
    ],
  }
  await installTabletMocks(page, { authenticated: true, routeExecution })
  page.on('dialog', dialog => dialog.accept())

  await page.goto('/dashboard/tasks/routes')
  await page.getByRole('button', { name: '▶ 预演', exact: true }).click()

  const timeline = page.locator('.drill-timeline-panel')
  await expect(timeline).toContainText('时间轴 · 真实预演')
  await expect(timeline).toContainText('1号点目标已下发')
  await expect(timeline).toContainText('1号点已到达')
  await expect(timeline).toContainText('2号点目标已下发')
  await expect(timeline).toContainText('2号点已到达')
  await expect(timeline).toContainText('预演完成')

  await timeline.getByRole('button', { name: '折叠', exact: true }).click()
  await expect(timeline.locator('.drill-timeline-list')).toHaveCount(0)
  await expect(timeline).toContainText('演练记录')
})
