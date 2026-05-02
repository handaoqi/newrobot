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
      <div class="login-artboard" aria-hidden="true">
        <div class="art-glow art-glow-a"></div>
        <div class="art-glow art-glow-b"></div>
        <div class="art-grid"></div>
        <div class="art-orbit orbit-a"></div>
        <div class="art-orbit orbit-b"></div>
        <div class="art-card art-card-top">
          <span>AI 巡检识别</span>
          <strong>实时异常预警</strong>
        </div>
        <div class="art-card art-card-bottom">
          <span>远程联动处置</span>
          <strong>视频、喊话、事件闭环一体化</strong>
        </div>
        <div class="art-radar">
          <div class="radar-ring ring-1"></div>
          <div class="radar-ring ring-2"></div>
          <div class="radar-ring ring-3"></div>
          <div class="radar-sweep"></div>
          <span class="radar-dot dot-a"></span>
          <span class="radar-dot dot-b"></span>
          <span class="radar-dot dot-c"></span>
        </div>
      </div>
      <div class="login-copy">
        <div class="login-toolbar">
          <button class="theme-btn" type="button" @click="toggleTheme">{{ toggleLabel }}</button>
        </div>
        <span class="eyebrow">Urban Safety Intelligence</span>
        <h1>智能巡检监测与预警协同平台</h1>
        <p>
          面向园区、社区与重点通道的智能巡检工作台，融合实时视频监看、AI 异常识别、远程喊话处置与设备健康监管。
        </p>
        <div class="hero-metrics">
          <article>
            <strong>24h</strong>
            <span>在线巡检与安全守护</span>
          </article>
          <article>
            <strong>12+</strong>
            <span>异常识别与事件分类能力</span>
          </article>
          <article>
            <strong>秒级</strong>
            <span>预警推送与联动响应效率</span>
          </article>
        </div>
      </div>
    </section>

    <section class="login-panel">
      <div class="login-card">
        <div class="card-head">
          <h2>登录指挥席</h2>
          <p>演示账号已预置，可直接体验巡检监测、预警处置与统计分析能力。</p>
        </div>

        <form class="login-form" @submit.prevent="submit">
          <label>
            <span>账号名称</span>
            <input v-model="form.username" type="text" placeholder="请输入用户名" />
          </label>
          <label>
            <span>登录密码</span>
            <input v-model="form.password" type="password" placeholder="请输入密码" />
          </label>
          <p v-if="errorMessage" class="form-error">{{ errorMessage }}</p>
          <button class="primary-btn" type="submit" :disabled="loading">
            {{ loading ? '正在验证身份...' : '进入监测平台' }}
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
