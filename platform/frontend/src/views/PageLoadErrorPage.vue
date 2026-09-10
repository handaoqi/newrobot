<script setup>
import { computed } from 'vue'
import { useRoute } from 'vue-router'

const route = useRoute()
const message = computed(() => route.query.message || '页面资源加载失败，请刷新后重试。')

function reload() {
  window.location.reload()
}
</script>

<template>
  <main class="login-shell configuration-error-page">
    <section class="login-panel">
      <div class="login-card" role="alert">
        <span class="eyebrow">PAGE LOAD ERROR</span>
        <h1>页面加载失败</h1>
        <p>{{ message }}</p>
        <button type="button" class="primary-btn" @click="reload">重新加载页面</button>
      </div>
    </section>
  </main>
</template>
