<script setup>
import { onMounted } from 'vue'

import { initTheme } from './composables/useTheme'

onMounted(() => {
  initTheme()
})
</script>

<template>
  <router-view />
</template>
