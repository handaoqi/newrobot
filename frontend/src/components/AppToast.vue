<script setup>
defineProps({
  show: {
    type: Boolean,
    default: false,
  },
  message: {
    type: String,
    default: '',
  },
  variant: {
    type: String,
    default: 'default',
  },
})
</script>

<template>
  <transition name="toast-fade">
    <div v-if="show" :class="['app-toast', `app-toast-${variant}`]">{{ message }}</div>
  </transition>
</template>
