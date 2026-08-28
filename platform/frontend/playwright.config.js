import { defineConfig } from '@playwright/test'

const commonUse = {
  baseURL: 'http://127.0.0.1:4173',
  colorScheme: 'light',
  locale: 'zh-CN',
  timezoneId: 'Asia/Shanghai',
  hasTouch: true,
  isMobile: true,
  deviceScaleFactor: 2,
  reducedMotion: 'reduce',
  trace: 'retain-on-failure',
  screenshot: 'only-on-failure',
}

export default defineConfig({
  testDir: './tests/tablet',
  fullyParallel: true,
  forbidOnly: Boolean(process.env.CI),
  retries: process.env.CI ? 1 : 0,
  workers: process.env.CI ? 2 : undefined,
  timeout: 30_000,
  expect: {
    timeout: 8_000,
    toHaveScreenshot: {
      animations: 'disabled',
      caret: 'hide',
      maxDiffPixelRatio: 0.01,
    },
  },
  outputDir: 'test-results/tablet',
  snapshotPathTemplate: '{testDir}/__screenshots__/{projectName}/{arg}{ext}',
  reporter: process.env.CI
    ? [['line'], ['html', { outputFolder: 'playwright-report/tablet', open: 'never' }]]
    : [['line'], ['html', { outputFolder: 'playwright-report/tablet', open: 'never' }]],
  webServer: {
    command: 'npm run dev -- --host 127.0.0.1 --port 4173',
    url: 'http://127.0.0.1:4173/login',
    reuseExistingServer: !process.env.CI,
    timeout: 120_000,
  },
  projects: [
    {
      name: 'ipad-768-webkit',
      use: {
        ...commonUse,
        browserName: 'webkit',
        userAgent: 'Mozilla/5.0 (iPad; CPU OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/17.5 Mobile/15E148 Safari/604.1',
        viewport: { width: 768, height: 1024 },
        screen: { width: 768, height: 1024 },
      },
    },
    {
      name: 'android-800-chromium',
      use: {
        ...commonUse,
        browserName: 'chromium',
        userAgent: 'Mozilla/5.0 (Linux; Android 14; Tablet) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36',
        viewport: { width: 800, height: 1280 },
        screen: { width: 800, height: 1280 },
      },
    },
    {
      name: 'ipad-834-webkit',
      use: {
        ...commonUse,
        browserName: 'webkit',
        userAgent: 'Mozilla/5.0 (iPad; CPU OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/17.5 Mobile/15E148 Safari/604.1',
        viewport: { width: 834, height: 1194 },
        screen: { width: 834, height: 1194 },
      },
    },
    {
      name: 'tablet-2000-landscape-chromium',
      use: {
        ...commonUse,
        browserName: 'chromium',
        userAgent: 'Mozilla/5.0 (Linux; Android 14; Tablet) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Safari/537.36',
        viewport: { width: 2000, height: 1200 },
        screen: { width: 2000, height: 1200 },
      },
    },
  ],
})
