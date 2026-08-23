<script setup>
import dogIcon from '../assets/robot-dog.svg'

defineProps({
  size: { type: [Number, String], default: 26 },
  label: { type: String, default: '机器狗' },
})
</script>

<template>
  <img
    class="robot-dog-icon"
    :src="dogIcon"
    :width="size"
    :height="size"
    :alt="label"
    aria-hidden="true"
  />
</template>

<style scoped>
.robot-dog-icon {
  display: block;
  color: currentColor;
  object-fit: contain;
}
</style>
