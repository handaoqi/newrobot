<script setup>
import { computed } from 'vue'
import { useRoute } from 'vue-router'

const route = useRoute()
const message = computed(() => route.query.message || '模块配置无效，页面未启动。请修正 modules.json 后重试。')
function reload() { window.location.reload() }
</script>

<template>
  <main class="login-shell configuration-error-page">
    <section class="login-panel">
      <div class="login-card" role="alert">
        <span class="eyebrow">FRONTEND CONFIGURATION ERROR</span>
        <h1>前端配置错误</h1>
        <p>{{ message }}</p>
        <button type="button" class="primary-btn" @click="reload">重新读取配置</button>
      </div>
    </section>
  </main>
</template>
