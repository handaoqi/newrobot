/**
 * The one source of truth for installable frontend modules.
 *
 * Keep loaders in this file lazy: importing the registry must not pull page
 * code into the application shell. The server build also exposes the three
 * development/replay pages; the container build omits them at compile time.
 */
export const CORE_MODULES = Object.freeze(['auth', 'shell', 'theme'])

const productionDefinitions = [
  { id: 'overview', title: '监测中心', path: '/dashboard/overview', component: () => import('../views/DashboardOverview.vue'), recommended: ['robots'] },
  { id: 'guard-duty', title: '保安值守', path: '/dashboard/guard-duty', component: () => import('../views/GuardDutyPage.vue'), recommended: ['overview', 'remote-control', 'events', 'tasks', 'task-execution'] },
  { id: 'remote-control', title: '远程控制', path: '/dashboard/remote-control', component: () => import('../views/RemoteControlPage.vue'), recommended: ['overview', 'guard-duty'] },
  { id: 'remote-development', title: '远程 AI 开发', path: '/dashboard/remote-development', component: () => import('../views/RemoteDevelopmentPage.vue'), recommended: [] },
  { id: 'analytics', title: '统计分析', path: '/dashboard/analytics', component: () => import('../views/AnalyticsPage.vue'), recommended: ['events'] },
  { id: 'events', title: '事件中心', path: '/dashboard/events', component: () => import('../views/EventsPage.vue'), recommended: ['guard-duty', 'analytics'] },
  { id: 'robots', title: '机器人管理', path: '/dashboard/robots', component: () => import('../views/RobotsPage.vue'), recommended: ['overview'] },
  { id: 'tasks', title: '巡检任务', path: '/dashboard/tasks', component: () => import('../views/TasksPage.vue'), recommended: ['patrol-calendar', 'task-execution', 'maps', 'routes'] },
  { id: 'patrol-calendar', title: '巡检日历', path: '/dashboard/tasks/calendar', parentId: 'tasks', component: () => import('../views/PatrolCalendarPage.vue'), recommended: ['tasks', 'task-execution'] },
  { id: 'task-execution', title: '执行详情', path: '/dashboard/task-executions/:executionId', parentId: 'tasks', component: () => import('../views/TaskExecutionPage.vue'), recommended: ['tasks', 'patrol-calendar'] },
  { id: 'maps', title: '地图管理', path: '/dashboard/tasks/maps', parentId: 'tasks', component: () => import('../views/MapsPage.vue'), recommended: ['routes', 'zones'] },
  { id: 'routes', title: '路径规划', path: '/dashboard/tasks/routes', parentId: 'tasks', component: () => import('../views/RoutePlannerPage.vue'), recommended: ['maps', 'zones', 'tasks'] },
  { id: 'zones', title: '禁区管理', path: '/dashboard/tasks/zones', parentId: 'tasks', component: () => import('../views/ZoneManagerPage.vue'), recommended: ['maps', 'routes'] },
]

const serverReplayDefinitions = [
  { id: 'validation', title: '仿真与回放检查', path: '/dashboard/validation', component: () => import('../views/ValidationJobsPage.vue'), recommended: [] },
  { id: 'replay-debug', title: '回放调试台', path: '/dashboard/replay-debug', component: () => import('../views/ReplayDebugPage.vue'), recommended: [] },
  { id: 'tracks', title: '轨迹回放', path: '/dashboard/tasks/tracks', parentId: 'tasks', component: () => import('../views/TrackPlaybackPage.vue'), recommended: [] },
]

// Vite replaces import.meta.env at build time. Keeping the fallback empty is
// important for the Node-based module tests, which exercise the server catalog.
const isContainerBuild = import.meta.env?.VITE_BUILD_TARGET === 'container'
const definitions = isContainerBuild
  ? productionDefinitions
  : [...productionDefinitions, ...serverReplayDefinitions]

export const MODULES = Object.freeze(definitions.map((module) => Object.freeze({
  ...module,
  recommended: Object.freeze([...module.recommended]),
})))

export const MODULE_BY_ID = Object.freeze(Object.fromEntries(MODULES.map((module) => [module.id, module])))
export const STABLE_MODULE_IDS = Object.freeze(MODULES.map((module) => module.id))

export function getModule(id) {
  return MODULE_BY_ID[id] || null
}

export function getModuleForPath(path) {
  return MODULES.find((module) => {
    const pattern = `^${module.path.replace(/:[^/]+/g, '[^/]+')}$`
    return new RegExp(pattern).test(path)
  }) || null
}

export function defaultModuleIds() {
  return [...STABLE_MODULE_IDS]
}

export function defaultHomePath(enabledIds = STABLE_MODULE_IDS) {
  const enabled = new Set(enabledIds)
  return MODULES.find((module) => enabled.has(module.id))?.path || '/dashboard/overview'
}

export function navigationItems(enabledIds = STABLE_MODULE_IDS) {
  const enabled = new Set(enabledIds)
  const visible = MODULES.filter((module) => enabled.has(module.id))
  const tasks = visible.filter((module) => module.parentId === 'tasks')
  const taskModule = visible.find((module) => module.id === 'tasks')
  const topLevel = visible.filter((module) => !module.parentId && module.id !== 'tasks')
  return [
    ...topLevel.map((module) => ({ label: module.title, path: module.path, moduleId: module.id })),
    ...(taskModule ? [{
      label: taskModule.title,
      path: taskModule.path,
      moduleId: taskModule.id,
      children: [
        { label: '任务列表', path: taskModule.path, moduleId: taskModule.id },
        ...tasks.map((module) => ({ label: module.title, path: module.path, moduleId: module.id })),
      ],
    }] : []),
  ]
}

export function recommendedMissing(moduleId, enabledIds) {
  const module = getModule(moduleId)
  const enabled = new Set(enabledIds)
  return (module?.recommended || []).filter((id) => !enabled.has(id)).map((id) => MODULE_BY_ID[id])
}
