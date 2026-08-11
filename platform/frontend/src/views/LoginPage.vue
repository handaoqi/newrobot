<script setup>
import { reactive, ref } from 'vue'
import { useRouter } from 'vue-router'

import { login } from '../services/api'

const router = useRouter()
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
    <div class="login-artboard" aria-hidden="true"></div>
    <section class="login-hero">
      <div class="login-copy">
        <h1>
          <span>智能巡检监测与预警</span><span>协同平台</span>
        </h1>
        <p>
          <span>面向园区、社区与重点通道的智能巡检工作台，</span><span>融合实时视频监看、AI 异常识别、</span><span>远程喊话处置与设备健康监管。</span>
        </p>
      </div>
    </section>

    <section class="login-panel">
      <div class="login-card">
        <div class="card-head">
          <h2>登录</h2>
          <p>欢迎使用智能监测平台</p>
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
