import { createRouter, createWebHistory } from 'vue-router'

import DashboardLayout from '../views/DashboardLayout.vue'
import ConfigurationErrorPage from '../views/ConfigurationErrorPage.vue'
import LoginPage from '../views/LoginPage.vue'
import ModuleNotInstalledPage from '../views/ModuleNotInstalledPage.vue'
import NotFoundPage from '../views/NotFoundPage.vue'
import { MODULES } from '../modules/index.js'
import { loadRuntimeConfig, homePathForConfig } from '../services/runtimeConfig.js'

const routes = [
  { path: '/login', name: 'login', component: LoginPage },
  { path: '/configuration-error', name: 'configuration-error', component: ConfigurationErrorPage },
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
  if (!runtimeConfigPromise) runtimeConfigPromise = loadRuntimeConfig()
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

router.onError((error, to) => {
  if (to?.name !== 'configuration-error') {
    router.push({ name: 'configuration-error', query: { message: `页面加载失败：${error?.message || '未知错误'}` } })
  }
})

export default router
