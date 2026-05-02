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
})
</script>

<template>
  <transition name="toast-fade">
    <div v-if="show" class="app-toast">{{ message }}</div>
  </transition>
</template>
