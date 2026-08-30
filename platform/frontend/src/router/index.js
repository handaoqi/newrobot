import { createRouter, createWebHistory } from 'vue-router'

import DashboardLayout from '../views/DashboardLayout.vue'

const AnalyticsPage = () => import('../views/AnalyticsPage.vue')
const DashboardOverview = () => import('../views/DashboardOverview.vue')
const EventsPage = () => import('../views/EventsPage.vue')
const GuardDutyPage = () => import('../views/GuardDutyPage.vue')
const LoginPage = () => import('../views/LoginPage.vue')
const RemoteControlPage = () => import('../views/RemoteControlPage.vue')
const RobotsPage = () => import('../views/RobotsPage.vue')
const TasksPage = () => import('../views/TasksPage.vue')
const TaskExecutionPage = () => import('../views/TaskExecutionPage.vue')
const PatrolCalendarPage = () => import('../views/PatrolCalendarPage.vue')
const MapsPage = () => import('../views/MapsPage.vue')
const RoutePlannerPage = () => import('../views/RoutePlannerPage.vue')
const ZoneManagerPage = () => import('../views/ZoneManagerPage.vue')
const TrackPlaybackPage = () => import('../views/TrackPlaybackPage.vue')
const RemoteDevelopmentPage = () => import('../views/RemoteDevelopmentPage.vue')
const ValidationJobsPage = () => import('../views/ValidationJobsPage.vue')
const ReplayDebugPage = () => import('../views/ReplayDebugPage.vue')

const routes = [
  { path: '/', redirect: '/dashboard/overview' },
  { path: '/login', name: 'login', component: LoginPage },
  {
    path: '/dashboard',
    component: DashboardLayout,
    meta: { requiresAuth: true },
    children: [
      { path: 'overview', name: 'overview', meta: { title: '实时监测中心' }, component: DashboardOverview },
      { path: 'guard-duty', name: 'guard-duty', meta: { title: '保安值守' }, component: GuardDutyPage },
      { path: 'remote-control', name: 'remote-control', meta: { title: '远程控制' }, component: RemoteControlPage },
      { path: 'remote-development', name: 'remote-development', meta: { title: '远程 AI 开发' }, component: RemoteDevelopmentPage },
      { path: 'validation', name: 'validation', meta: { title: '仿真与回放检查' }, component: ValidationJobsPage },
      { path: 'replay-debug', name: 'replay-debug', meta: { title: '回放调试台' }, component: ReplayDebugPage },
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
