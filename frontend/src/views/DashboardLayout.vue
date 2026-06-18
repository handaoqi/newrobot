<script setup>
import { computed, ref } from 'vue'
import { useRoute, useRouter } from 'vue-router'

import { useTheme } from '../composables/useTheme'

const route = useRoute()
const router = useRouter()
const { toggleLabel, toggleTheme } = useTheme()

const expandedMenus = ref({})

const menuItems = [
  { label: '监测中心', path: '/dashboard/overview' },
  { label: '统计分析', path: '/dashboard/analytics' },
  { label: '事件中心', path: '/dashboard/events' },
  { label: '机器人管理', path: '/dashboard/robots' },
  { label: '巡检任务', path: '/dashboard/tasks', children: [
    { label: '任务列表', path: '/dashboard/tasks' },
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

function isMenuActive(item) {
  if (item.children) {
    return item.children.some(child => route.path === child.path)
  }
  return route.path === item.path
}

function logout() {
  localStorage.removeItem('inspection_token')
  localStorage.removeItem('inspection_user')
  router.push('/login')
}
</script>

<template>
  <div class="dashboard-shell">
    <aside class="sidebar">
      <div class="brand-block">
        <span class="eyebrow">Robot Patrol</span>
        <h1>智能巡检平台</h1>
      </div>

      <nav class="menu-list">
        <template v-for="(item, index) in menuItems" :key="item.path">
          <div v-if="item.children" class="menu-group">
            <div
              class="menu-item menu-group-header"
              :class="{ active: isMenuActive(item) }"
              @click="toggleMenu(index)"
            >
              <span>{{ item.label }}</span>
              <span class="menu-arrow">{{ expandedMenus[index] ? '▼' : '▶' }}</span>
            </div>
            <div v-if="expandedMenus[index]" class="menu-submenu">
              <router-link
                v-for="child in item.children"
                :key="child.path"
                :to="child.path"
                class="menu-item menu-subitem"
                :class="{ active: route.path === child.path }"
              >
                {{ child.label }}
              </router-link>
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
        <div>
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

      <router-view />
    </section>
  </div>
</template>
