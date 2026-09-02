<script setup>
import { onErrorCaptured, onMounted, ref } from 'vue'

import { initTheme } from './composables/useTheme'

const renderError = ref('')
onErrorCaptured((error) => {
  renderError.value = error?.message || '页面渲染失败'
  return false
})
function reload() { window.location.reload() }

onMounted(() => {
  initTheme()
})
</script>

<template>
  <main v-if="renderError" class="page-render-error" role="alert">
    <h1>页面加载失败</h1>
    <p>{{ renderError }}</p>
    <button type="button" class="primary-btn" @click="reload">重新加载</button>
  </main>
  <router-view v-else />
</template>
