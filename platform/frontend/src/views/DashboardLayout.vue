<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import { useRoute, useRouter } from 'vue-router'

import { useTheme } from '../composables/useTheme'
import { API_BASE, fetchRobotMappingStatus, fetchRobots } from '../services/api'

const route = useRoute()
const router = useRouter()
const { toggleLabel, toggleTheme } = useTheme()

const expandedMenus = ref({})
const tabletMenuOpen = ref(false)
const tabletMenuButton = ref(null)
const tabletSidebar = ref(null)
const mappingAlert = ref(null)
const robots = ref([])
let mappingPollTimer = null
let mappingAlertEventSource = null
let lastMappingAlertKey = ''
let mappingStatusRefreshing = false

const menuItems = [
  { label: '保安值守', path: '/dashboard/guard-duty' },
  { label: '监测中心', path: '/dashboard/overview' },
  { label: '远程控制', path: '/dashboard/remote-control' },
  { label: '远程 AI 开发', path: '/dashboard/remote-development' },
  { label: '统计分析', path: '/dashboard/analytics' },
  { label: '事件中心', path: '/dashboard/events' },
  { label: '机器人管理', path: '/dashboard/robots' },
  { label: '巡检任务', path: '/dashboard/tasks', children: [
    { label: '任务列表', path: '/dashboard/tasks' },
    { label: '巡检日历', path: '/dashboard/tasks/calendar' },
    { label: '地图管理', path: '/dashboard/tasks/maps' },
    { label: '路径规划', path: '/dashboard/tasks/routes' },
    { label: '禁区管理', path: '/dashboard/tasks/zones' },
    { label: '轨迹回放', path: '/dashboard/tasks/tracks' },
  ]},
]

const user = computed(() => {
  try {
    return JSON.parse(localStorage.getItem('inspection_user') || '{}')
  } catch {
    return {}
  }
})

function toggleMenu(index) {
  expandedMenus.value[index] = !expandedMenus.value[index]
}

function menuContainsPath(item) {
  return route.path === item.path || Boolean(item.children?.some(menuContainsPath))
}

function expandActiveMenu() {
  menuItems.forEach((item, index) => {
    if (item.children?.some(menuContainsPath)) expandedMenus.value[index] = true
    item.children?.forEach((child, childIndex) => {
      if (child.children?.some(menuContainsPath)) expandedMenus.value[`${index}-${childIndex}`] = true
    })
  })
}

async function openTabletMenu() {
  expandActiveMenu()
  tabletMenuOpen.value = true
  document.body.classList.add('tablet-menu-locked')
  await nextTick()
  tabletSidebar.value?.querySelector('a, button')?.focus()
}

function closeTabletMenu({ returnFocus = false } = {}) {
  if (!tabletMenuOpen.value) return
  tabletMenuOpen.value = false
  document.body.classList.remove('tablet-menu-locked')
  if (returnFocus) nextTick(() => tabletMenuButton.value?.focus())
}

function toggleTabletMenu() {
  if (tabletMenuOpen.value) closeTabletMenu({ returnFocus: true })
  else openTabletMenu()
}

function handleTabletMenuKeydown(event) {
  if (event.key === 'Escape' && tabletMenuOpen.value) {
    closeTabletMenu({ returnFocus: true })
  }
}

function isMenuActive(item) {
  return menuContainsPath(item)
}

function logout() {
  localStorage.removeItem('inspection_token')
  localStorage.removeItem('inspection_user')
  router.push('/login')
}

function mappingHealthFromStatus(status) {
  const progress = status?.result?.save_progress || {}
  const health = progress.slam_health || {}
  const diverged = progress.error_code === 'SLAM_DIVERGED' || health.state === 'diverged'
  const degraded = health.state === 'degraded'
  if (!diverged && !degraded) return null
  return {
    robotId: status.robot_id,
    robotCode: status.robot_code || '',
    diverged,
    title: diverged ? '建图已停采，请立即停止遥控' : '建图质量正在恶化',
    message: diverged
      ? '定位已发散，关键帧不再记录。请停止走场，到地图页点击“停止并生成救援地图”。'
      : (health.warning || progress.error || '位姿异常，请放慢或原地停下'),
  }
}

function announceMappingAlert(alert) {
  const key = `${alert.robotId}:${alert.diverged ? 'diverged' : 'degraded'}`
  if (key === lastMappingAlertKey) return
  lastMappingAlertKey = key
  try {
    const context = new window.AudioContext()
    const oscillator = context.createOscillator()
    const gain = context.createGain()
    oscillator.type = 'square'
    oscillator.frequency.value = alert.diverged ? 880 : 520
    gain.gain.value = 0.08
    oscillator.connect(gain)
    gain.connect(context.destination)
    oscillator.start()
    oscillator.stop(context.currentTime + (alert.diverged ? 0.45 : 0.2))
  } catch {}
  if (!window.speechSynthesis) return
  window.speechSynthesis.cancel()
  const utterance = new SpeechSynthesisUtterance(alert.diverged
    ? '建图定位已发散，请立即停止遥控走场'
    : '建图质量正在恶化，请放慢或停下')
  utterance.lang = 'zh-CN'
  utterance.rate = 1
  window.speechSynthesis.speak(utterance)
}

function applyMappingAlert(alert) {
  mappingAlert.value = alert
  if (alert) announceMappingAlert(alert)
  else lastMappingAlertKey = ''
}

async function refreshMappingAlerts() {
  if (mappingStatusRefreshing) return
  mappingStatusRefreshing = true
  try {
    if (!robots.value.length) robots.value = await fetchRobots()
    const snapshots = await Promise.all(robots.value.map(async (robot) => {
      try {
        return await fetchRobotMappingStatus(robot.id)
      } catch {
        return null
      }
    }))
    const next = snapshots.map(mappingHealthFromStatus).find(item => item)
    if (next) applyMappingAlert(next)
    else {
      mappingAlert.value = null
      lastMappingAlertKey = ''
    }
  } catch {
    // Keep the last banner if polling fails; SSE still covers the diverge event.
  } finally {
    mappingStatusRefreshing = false
  }
}

function setupMappingAlertStream() {
  const token = localStorage.getItem('inspection_token')
  if (!token || typeof EventSource === 'undefined') return
  mappingAlertEventSource?.close()
  mappingAlertEventSource = new EventSource(`${API_BASE}/events/stream/?token=${encodeURIComponent(token)}`)
  mappingAlertEventSource.addEventListener('inspection_event_created', (message) => {
    let payload = {}
    try {
      payload = JSON.parse(message.data || '{}')
    } catch {
      return
    }
    const event = payload.event || {}
    const eventType = String(event.event_type || '')
    const sourceCode = String(event.source_code || '')
    if (eventType !== 'slam_diverged' && sourceCode !== 'SLAM_DIVERGED' && event.title !== '建图定位已发散') return
    applyMappingAlert({
      robotId: event.robot || payload.robot?.id,
      robotCode: payload.robot?.code || '',
      diverged: true,
      title: '建图已停采，请立即停止遥控',
      message: event.description || '定位已发散，关键帧不再记录。请停止走场，到地图页点击“停止并生成救援地图”。',
    })
  })
}

onMounted(() => {
  expandActiveMenu()
  document.addEventListener('keydown', handleTabletMenuKeydown)
  refreshMappingAlerts()
  mappingPollTimer = window.setInterval(refreshMappingAlerts, 2000)
  setupMappingAlertStream()
})

onBeforeUnmount(() => {
  document.removeEventListener('keydown', handleTabletMenuKeydown)
  document.body.classList.remove('tablet-menu-locked')
  if (mappingPollTimer) window.clearInterval(mappingPollTimer)
  mappingAlertEventSource?.close()
  window.speechSynthesis?.cancel()
})

watch(() => route.path, () => {
  expandActiveMenu()
  closeTabletMenu()
  if (mappingAlert.value?.diverged) return
  refreshMappingAlerts()
})
</script>

<template>
  <div class="dashboard-shell">
    <header class="tablet-app-bar">
      <button
        ref="tabletMenuButton"
        class="tablet-menu-button"
        type="button"
        :aria-label="tabletMenuOpen ? '关闭导航菜单' : '打开导航菜单'"
        aria-controls="dashboard-navigation"
        :aria-expanded="tabletMenuOpen"
        @click="toggleTabletMenu"
      >
        <span></span><span></span><span></span>
      </button>
      <div class="tablet-app-title">
        <span>智能巡检平台</span>
        <strong>{{ route.meta?.title || '平台页面' }}</strong>
      </div>
      <span class="tablet-user">{{ user.display_name || '值班员' }}</span>
    </header>

    <button
      v-if="tabletMenuOpen"
      class="tablet-menu-backdrop"
      type="button"
      aria-label="关闭导航菜单"
      @click="closeTabletMenu({ returnFocus: true })"
    ></button>

    <aside
      id="dashboard-navigation"
      ref="tabletSidebar"
      class="sidebar"
      :class="{ 'tablet-open': tabletMenuOpen }"
      aria-label="平台导航"
    >
      <div class="brand-block">
        <span class="eyebrow">Robot Patrol</span>
        <h1>智能巡检平台</h1>
      </div>

      <nav class="menu-list">
        <template v-for="(item, index) in menuItems" :key="item.path">
          <div v-if="item.children" class="menu-group">
            <button
              class="menu-item menu-group-header"
              :class="{ active: isMenuActive(item) }"
              type="button"
              :aria-expanded="Boolean(expandedMenus[index])"
              @click="toggleMenu(index)"
            >
              <span>{{ item.label }}</span>
              <span class="menu-arrow">{{ expandedMenus[index] ? '▼' : '▶' }}</span>
            </button>
            <div v-if="expandedMenus[index]" class="menu-submenu">
              <template v-for="(child, childIndex) in item.children" :key="child.path">
                <div v-if="child.children" class="menu-subgroup">
                  <button
                    class="menu-item menu-subitem menu-group-header"
                    :class="{ active: isMenuActive(child) }"
                    type="button"
                    :aria-expanded="Boolean(expandedMenus[`${index}-${childIndex}`])"
                    @click="toggleMenu(`${index}-${childIndex}`)"
                  >
                    <span>{{ child.label }}</span>
                    <span class="menu-arrow">{{ expandedMenus[`${index}-${childIndex}`] ? '▼' : '▶' }}</span>
                  </button>
                  <div v-if="expandedMenus[`${index}-${childIndex}`]" class="menu-submenu menu-third-level">
                    <router-link
                      v-for="grandchild in child.children"
                      :key="grandchild.path"
                      :to="grandchild.path"
                      class="menu-item menu-subitem menu-third-item"
                      :class="{ active: route.path === grandchild.path }"
                    >
                      {{ grandchild.label }}
                    </router-link>
                  </div>
                </div>
                <router-link
                  v-else
                  :to="child.path"
                  class="menu-item menu-subitem"
                  :class="{ active: route.path === child.path }"
                >
                  {{ child.label }}
                </router-link>
              </template>
            </div>
          </div>
          <router-link
            v-else
            :to="item.path"
            class="menu-item"
            :class="{ active: route.path === item.path }"
          >
            {{ item.label }}
          </router-link>
        </template>
      </nav>

      <div class="sidebar-note">
        <strong>值守提醒</strong>
        <p>建议优先处理高风险事件，同时持续关注低电量设备与异常停留区域。</p>
      </div>
    </aside>

    <section class="main-layout">
      <header class="main-header">
          <div class="desktop-page-heading">
          <p class="header-kicker">AI Patrol Workspace</p>
          <h2>{{ route.meta?.title || '平台页面' }}</h2>
        </div>
        <div class="header-actions">
          <button class="theme-btn" @click="toggleTheme">{{ toggleLabel }}</button>
          <div class="header-user">
            <strong>{{ user.display_name || '值班员' }}</strong>
          </div>
          <button class="ghost-btn" @click="logout">退出登录</button>
        </div>
      </header>

      <div
        v-if="mappingAlert"
        class="mapping-live-alert"
        :class="{ diverged: mappingAlert.diverged }"
        role="alert"
      >
        <div>
          <strong>{{ mappingAlert.title }}</strong>
          <span>{{ mappingAlert.robotCode ? `${mappingAlert.robotCode}：` : '' }}{{ mappingAlert.message }}</span>
        </div>
        <router-link class="mapping-live-alert-link" to="/dashboard/tasks/maps">
          去地图页处理
        </router-link>
      </div>

      <router-view />
    </section>
  </div>
</template>

<style scoped>
.tablet-app-bar,
.tablet-menu-backdrop {
  display: none;
}

.mapping-live-alert {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 16px;
  margin: 0 0 16px;
  padding: 12px 16px;
  border: 1px solid #f0c36d;
  border-radius: 12px;
  background: #fff7e6;
  color: #7a4e00;
}
.mapping-live-alert.diverged {
  border-color: #e36d6d;
  background: #fff1f0;
  color: #8a1f11;
}
.mapping-live-alert > div {
  display: grid;
  gap: 2px;
}
.mapping-live-alert strong {
  font-size: 15px;
}
.mapping-live-alert span {
  font-size: 13px;
  line-height: 1.45;
}
.mapping-live-alert-link {
  flex: 0 0 auto;
  padding: 8px 12px;
  border-radius: 8px;
  background: #8a1f11;
  color: #fff;
  font-size: 13px;
  font-weight: 700;
  text-decoration: none;
}
.mapping-live-alert:not(.diverged) .mapping-live-alert-link {
  background: #7a4e00;
}

@media (max-width: 1024px) and (orientation: portrait),
  (min-width: 1200px) and (max-width: 2048px) and (min-height: 900px) and (max-height: 1280px) and (orientation: landscape) {
  :global(body.tablet-menu-locked) {
    overflow: hidden;
  }

  .tablet-app-bar {
    position: sticky;
    z-index: 1100;
    top: 0;
    display: grid;
    grid-template-columns: 56px minmax(0, 1fr) auto;
    align-items: center;
    gap: 12px;
    min-height: 76px;
    padding: 10px 18px;
    border-bottom: 1px solid var(--line);
    background: color-mix(in srgb, var(--panel) 94%, transparent);
    box-shadow: 0 10px 32px rgba(25, 55, 90, 0.12);
    backdrop-filter: blur(22px);
  }

  .tablet-menu-button {
    display: grid;
    place-content: center;
    gap: 5px;
    width: 56px;
    height: 56px;
    padding: 0;
    border: 1px solid var(--chip-border);
    border-radius: 50%;
    color: var(--text);
    background: var(--ghost-bg);
    transition: background 0.2s ease, border-color 0.2s ease, transform 0.2s ease;
  }

  .tablet-menu-button:hover {
    border-color: var(--cyan);
    background: var(--menu-active-bg);
  }

  .tablet-menu-button[aria-expanded="true"] {
    transform: rotate(90deg);
  }

  .tablet-menu-button span {
    display: block;
    width: 21px;
    height: 2px;
    border-radius: 999px;
    background: currentColor;
    transition: transform 0.2s ease, opacity 0.2s ease;
  }

  .tablet-app-title {
    display: grid;
    min-width: 0;
    gap: 2px;
  }

  .tablet-app-title span {
    overflow: hidden;
    color: var(--muted);
    font-size: 12px;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .tablet-app-title strong {
    overflow: hidden;
    font-size: 18px;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .tablet-user {
    max-width: 120px;
    overflow: hidden;
    color: var(--muted);
    font-size: 13px;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .tablet-menu-backdrop {
    position: fixed;
    z-index: 1150;
    inset: 0;
    display: block;
    width: 100%;
    height: 100%;
    padding: 0;
    border: 0;
    border-radius: 0;
    background: rgba(4, 12, 25, 0.46);
    backdrop-filter: blur(2px);
  }

  .sidebar {
    position: fixed;
    z-index: 1200;
    inset: 0 auto 0 0;
    width: min(340px, calc(100vw - 72px));
    max-height: 100dvh;
    overflow-y: auto;
    padding: 28px 22px;
    border-right: 1px solid var(--line);
    border-bottom: 0;
    transform: translateX(-105%);
    visibility: hidden;
    pointer-events: none;
    transition: transform 0.22s ease, visibility 0.22s ease;
  }

  .sidebar.tablet-open {
    transform: translateX(0);
    visibility: visible;
    pointer-events: auto;
  }

  .sidebar .menu-list {
    display: grid;
    overflow: visible;
    gap: 8px;
    padding-bottom: 0;
  }

  .sidebar .menu-item,
  .sidebar .menu-group,
  .sidebar .menu-group-header,
  .sidebar .menu-subitem {
    width: 100%;
  }

  .sidebar .menu-group {
    position: static;
  }

  .sidebar .menu-subgroup {
    width: 100%;
    min-width: 0;
  }

  .sidebar .menu-submenu {
    position: static;
    width: 100%;
    padding: 6px 0 0 12px;
    border: 0;
    background: transparent;
    box-shadow: none;
  }

  .sidebar .menu-third-level {
    padding-left: 20px;
  }

  .desktop-page-heading {
    display: none;
  }

  .main-header {
    justify-content: flex-end;
    margin-bottom: 14px;
  }

  .header-actions {
    width: 100%;
  }

  .mapping-live-alert {
    align-items: flex-start;
  }
}

@media (max-width: 1024px) and (orientation: portrait) {
  .tablet-app-bar {
    position: fixed;
    top: 12px;
    right: 12px;
    left: auto;
    display: block;
    width: 56px;
    min-height: 56px;
    padding: 0;
    border: 0;
    background: transparent;
    box-shadow: none;
    backdrop-filter: none;
  }

  .tablet-app-title,
  .tablet-user {
    display: none;
  }

  .tablet-menu-button {
    box-shadow: 0 10px 28px rgba(25, 55, 90, 0.22);
    backdrop-filter: blur(18px);
  }

  .sidebar {
    inset: 0 0 0 auto;
    border-right: 0;
    border-left: 1px solid var(--line);
    transform: translateX(105%);
  }

  .main-layout {
    padding-top: 78px;
  }
}

@media (min-width: 1200px) and (max-width: 2048px) and (min-height: 900px) and (max-height: 1280px) and (orientation: landscape) {
  .main-layout {
    min-width: 0;
    padding: 24px clamp(24px, 3vw, 56px);
  }

  .main-header {
    margin-bottom: 20px;
  }
}
</style>
