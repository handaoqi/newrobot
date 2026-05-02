import { createRouter, createWebHistory } from 'vue-router'

import DashboardLayout from '../views/DashboardLayout.vue'
import DashboardOverview from '../views/DashboardOverview.vue'
import EventsPage from '../views/EventsPage.vue'
import LoginPage from '../views/LoginPage.vue'
import RobotsPage from '../views/RobotsPage.vue'
import TasksPage from '../views/TasksPage.vue'

const routes = [
  { path: '/', redirect: '/dashboard/overview' },
  { path: '/login', name: 'login', component: LoginPage },
  {
    path: '/dashboard',
    component: DashboardLayout,
    meta: { requiresAuth: true },
    children: [
      { path: 'overview', name: 'overview', meta: { title: '实时监测中心' }, component: DashboardOverview },
      { path: 'events', name: 'events', meta: { title: '事件中心' }, component: EventsPage },
      { path: 'robots', name: 'robots', meta: { title: '机器人管理' }, component: RobotsPage },
      { path: 'tasks', name: 'tasks', meta: { title: '巡检任务' }, component: TasksPage },
    ],
  },
]

const router = createRouter({
  history: createWebHistory(),
  routes,
})

router.beforeEach((to) => {
  const token = localStorage.getItem('inspection_token')
  if (to.meta.requiresAuth && !token) {
    return { name: 'login' }
  }
  if (to.name === 'login' && token) {
    return { name: 'overview' }
  }
  return true
})

export default router
