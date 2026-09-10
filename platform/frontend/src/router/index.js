import { createRouter, createWebHistory } from 'vue-router'

import DashboardLayout from '../views/DashboardLayout.vue'
import ConfigurationErrorPage from '../views/ConfigurationErrorPage.vue'
import LoginPage from '../views/LoginPage.vue'
import ModuleNotInstalledPage from '../views/ModuleNotInstalledPage.vue'
import NotFoundPage from '../views/NotFoundPage.vue'
import PageLoadErrorPage from '../views/PageLoadErrorPage.vue'
import { MODULES } from '../modules/index.js'
import { loadRuntimeConfig, homePathForConfig } from '../services/runtimeConfig.js'
import { claimAssetLoadRetry, clearAssetLoadRetry, isAssetLoadError } from '../services/assetLoadRecovery.js'

const routes = [
  { path: '/login', name: 'login', component: LoginPage },
  { path: '/configuration-error', name: 'configuration-error', component: ConfigurationErrorPage },
  { path: '/page-load-error', name: 'page-load-error', component: PageLoadErrorPage },
  {
    path: '/dashboard',
    component: DashboardLayout,
    meta: { requiresAuth: true },
    children: [
      ...MODULES.map((module) => ({
        path: module.path.replace(/^\/dashboard\/?/, ''),
        name: module.id,
        component: module.component,
        meta: { title: module.title, moduleId: module.id, requiresAuth: true },
      })),
      { path: 'module-not-installed', name: 'module-not-installed', component: ModuleNotInstalledPage, meta: { title: '页面未安装', requiresAuth: true } },
    ],
  },
  { path: '/', name: 'home', component: { template: '<div />' } },
  { path: '/:pathMatch(.*)*', name: 'not-found', component: NotFoundPage, meta: { notFound: true } },
]

const router = createRouter({ history: createWebHistory(), routes })
let runtimeConfigPromise = null

function readConfig() {
  if (!runtimeConfigPromise) {
    runtimeConfigPromise = loadRuntimeConfig().catch((error) => {
      // Do not keep a transient network failure cached for the whole session.
      runtimeConfigPromise = null
      throw error
    })
  }
  return runtimeConfigPromise
}

export function resetRuntimeConfigForTests() {
  runtimeConfigPromise = null
}

router.beforeEach(async (to) => {
  let config
  try {
    config = await readConfig()
  } catch (error) {
    if (to.name !== 'configuration-error') return { name: 'configuration-error', query: { message: error.message } }
    return true
  }

  const token = localStorage.getItem('inspection_token')
  if (to.name === 'home') return { path: homePathForConfig(config) }
  if (to.meta.requiresAuth && !token) return { name: 'login', query: { redirect: to.fullPath } }
  if (to.name === 'login' && token) return { path: homePathForConfig(config) }
  if (to.meta.moduleId && !config.enabled_modules.includes(to.meta.moduleId)) {
    return { name: 'module-not-installed', query: { moduleId: to.meta.moduleId, redirect: to.fullPath } }
  }
  return true
})

router.afterEach((to, _from, failure) => {
  if (!failure && to.name !== 'page-load-error') clearAssetLoadRetry()
})

router.onError((error, to) => {
  if (isAssetLoadError(error)) {
    if (claimAssetLoadRetry() && typeof window !== 'undefined' && typeof window.location?.reload === 'function') {
      window.location.reload()
      return
    }
    if (to?.name !== 'page-load-error') {
      router.replace({ name: 'page-load-error', query: { message: '页面资源加载失败，可能是页面版本已更新，请刷新页面后重试。' } })
    }
    return
  }
  if (to?.name !== 'page-load-error') {
    router.replace({ name: 'page-load-error', query: { message: `页面加载失败：${error?.message || '未知错误'}` } })
  }
})

export default router
