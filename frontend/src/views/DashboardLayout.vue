<script setup>
import { computed } from 'vue'
import { useRoute, useRouter } from 'vue-router'

import { useTheme } from '../composables/useTheme'

const route = useRoute()
const router = useRouter()
const { toggleLabel, toggleTheme } = useTheme()

const menuItems = [
  { label: '监测中心', path: '/dashboard/overview' },
  { label: '统计分析', path: '/dashboard/analytics' },
  { label: '事件中心', path: '/dashboard/events' },
  { label: '机器人管理', path: '/dashboard/robots' },
  { label: '巡检任务', path: '/dashboard/tasks' },
]

const user = computed(() => {
  try {
    return JSON.parse(localStorage.getItem('inspection_user') || '{}')
  } catch {
    return {}
  }
})

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
        <router-link
          v-for="item in menuItems"
          :key="item.path"
          :to="item.path"
          class="menu-item"
          :class="{ active: route.path === item.path }"
        >
          {{ item.label }}
        </router-link>
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
            <span>{{ user.username || 'operator' }} · 在线值守</span>
          </div>
          <button class="ghost-btn" @click="logout">退出登录</button>
        </div>
      </header>

      <router-view />
    </section>
  </div>
</template>
