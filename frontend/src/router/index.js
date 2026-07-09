import { createRouter, createWebHistory } from 'vue-router'

import DashboardLayout from '../views/DashboardLayout.vue'
import AnalyticsPage from '../views/AnalyticsPage.vue'
import DashboardOverview from '../views/DashboardOverview.vue'
import EventsPage from '../views/EventsPage.vue'
import LoginPage from '../views/LoginPage.vue'
import RemoteControlPage from '../views/RemoteControlPage.vue'
import RobotsPage from '../views/RobotsPage.vue'
import TasksPage from '../views/TasksPage.vue'
import TaskExecutionPage from '../views/TaskExecutionPage.vue'
import PatrolCalendarPage from '../views/PatrolCalendarPage.vue'
import MapsPage from '../views/MapsPage.vue'
import RoutePlannerPage from '../views/RoutePlannerPage.vue'
import ZoneManagerPage from '../views/ZoneManagerPage.vue'
import TrackPlaybackPage from '../views/TrackPlaybackPage.vue'

const routes = [
  { path: '/', redirect: '/dashboard/overview' },
  { path: '/login', name: 'login', component: LoginPage },
  {
    path: '/dashboard',
    component: DashboardLayout,
    meta: { requiresAuth: true },
    children: [
      { path: 'overview', name: 'overview', meta: { title: '实时监测中心' }, component: DashboardOverview },
      { path: 'remote-control', name: 'remote-control', meta: { title: '远程控制' }, component: RemoteControlPage },
      { path: 'analytics', name: 'analytics', meta: { title: '统计分析中心' }, component: AnalyticsPage },
      { path: 'events', name: 'events', meta: { title: '事件中心' }, component: EventsPage },
      { path: 'robots', name: 'robots', meta: { title: '机器人管理' }, component: RobotsPage },
      { path: 'tasks', name: 'tasks', meta: { title: '巡检任务' }, component: TasksPage },
      { path: 'tasks/calendar', name: 'patrol-calendar', meta: { title: '巡检日历' }, component: PatrolCalendarPage },
      { path: 'task-executions/:executionId', name: 'task-execution', meta: { title: '任务执行详情' }, component: TaskExecutionPage },
      { path: 'tasks/maps', name: 'maps', meta: { title: '地图管理' }, component: MapsPage },
      { path: 'tasks/routes', name: 'routes', meta: { title: '路径规划' }, component: RoutePlannerPage },
      { path: 'tasks/zones', name: 'zones', meta: { title: '禁区管理' }, component: ZoneManagerPage },
      { path: 'tasks/tracks', name: 'tracks', meta: { title: '轨迹回放' }, component: TrackPlaybackPage },
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
