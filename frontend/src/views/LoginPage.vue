<script setup>
import { reactive, ref } from 'vue'
import { useRouter } from 'vue-router'

import { useTheme } from '../composables/useTheme'
import { login } from '../services/api'

const router = useRouter()
const { toggleLabel, toggleTheme } = useTheme()
const form = reactive({
  username: 'operator',
  password: 'admin123456',
})
const errorMessage = ref('')
const loading = ref(false)

async function submit() {
  loading.value = true
  errorMessage.value = ''
  try {
    const data = await login(form)
    localStorage.setItem('inspection_token', data.token)
    localStorage.setItem('inspection_user', JSON.stringify(data.user))
    router.push('/dashboard/overview')
  } catch (error) {
    errorMessage.value = error.message
  } finally {
    loading.value = false
  }
}
</script>

<template>
  <main class="login-shell">
    <section class="login-hero">
      <div class="login-copy">
        <div class="login-toolbar">
          <button class="theme-btn" type="button" @click="toggleTheme">{{ toggleLabel }}</button>
        </div>
        <span class="eyebrow">Inspection Command Center</span>
        <h1>智能机器人巡检监测平台</h1>
        <p>
          面向工作人员的实时监控、告警处置与巡检协同平台。支持视频监看、远程喊话、事件闭环和设备健康监测。
        </p>
        <div class="hero-metrics">
          <article>
            <strong>24h</strong>
            <span>设备在线监护</span>
          </article>
          <article>
            <strong>12+</strong>
            <span>AI 异常识别类型</span>
          </article>
          <article>
            <strong>秒级</strong>
            <span>告警推送与人工响应</span>
          </article>
        </div>
      </div>
    </section>

    <section class="login-panel">
      <div class="login-card">
        <div class="card-head">
          <h2>登录平台</h2>
          <p>默认演示账号已预置，可直接进入平台。</p>
        </div>

        <form class="login-form" @submit.prevent="submit">
          <label>
            <span>用户名</span>
            <input v-model="form.username" type="text" placeholder="请输入用户名" />
          </label>
          <label>
            <span>密码</span>
            <input v-model="form.password" type="password" placeholder="请输入密码" />
          </label>
          <p v-if="errorMessage" class="form-error">{{ errorMessage }}</p>
          <button class="primary-btn" type="submit" :disabled="loading">
            {{ loading ? '登录中...' : '进入监测平台' }}
          </button>
        </form>

        <div class="login-tip">
          <strong>演示账号</strong>
          <span>operator / admin123456</span>
        </div>
      </div>
    </section>
  </main>
</template>
