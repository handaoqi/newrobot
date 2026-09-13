<script setup>
import { onBeforeUnmount, onMounted, ref, watch } from 'vue'

const props = defineProps({
  cloud: { type: Object, default: null },
  yaw: { type: Number, default: 0 },
  radius: { type: Number, default: 12 },
})

const canvas = ref(null)
const host = ref(null)
let ctx
let width = 0
let height = 0
let resizeObserver
let frame

function resize() {
  const bounds = host.value.getBoundingClientRect()
  const ratio = Math.min(window.devicePixelRatio || 1, 2)
  width = bounds.width
  height = bounds.height
  canvas.value.width = Math.max(1, Math.round(width * ratio))
  canvas.value.height = Math.max(1, Math.round(height * ratio))
  canvas.value.style.width = `${width}px`
  canvas.value.style.height = `${height}px`
  ctx = canvas.value.getContext('2d')
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0)
  schedule()
}

function render() {
  frame = null
  if (!ctx) return
  ctx.fillStyle = '#f4f8fb'
  ctx.fillRect(0, 0, width, height)
  const size = Math.min(width, height) - 46
  const pixelsPerMeter = size / (props.radius * 2)
  const centerX = width / 2
  const centerY = height / 2 + 7

  ctx.strokeStyle = 'rgba(78, 135, 171, .22)'
  ctx.lineWidth = 1
  for (let meter = 3; meter <= props.radius; meter += 3) {
    ctx.beginPath(); ctx.arc(centerX, centerY, meter * pixelsPerMeter, 0, Math.PI * 2); ctx.stroke()
  }
  ctx.save()
  ctx.translate(centerX, centerY)
  ctx.rotate(-props.yaw)
  const fanRadius = props.radius * pixelsPerMeter
  ctx.fillStyle = 'rgba(46, 169, 211, .08)'
  ctx.strokeStyle = 'rgba(46, 151, 191, .24)'
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.arc(0, 0, fanRadius, -Math.PI * .74, Math.PI * .74); ctx.closePath(); ctx.fill(); ctx.stroke()
  ctx.restore()

  const positions = props.cloud?.positions
  const colors = props.cloud?.colors
  const count = Number(props.cloud?.count) || 0
  const step = Math.max(1, Math.ceil(count / 3500))
  for (let index = 0; index < count; index += step) {
    const x = Number(positions?.[index * 3])
    const y = Number(positions?.[index * 3 + 1])
    const z = Number(positions?.[index * 3 + 2])
    if (![x, y, z].every(Number.isFinite) || z < -0.35 || Math.hypot(x, y) > props.radius) continue
    const red = Math.round((colors?.[index * 3] ?? .2) * 255)
    const green = Math.round((colors?.[index * 3 + 1] ?? .8) * 255)
    const blue = Math.round((colors?.[index * 3 + 2] ?? 1) * 255)
    ctx.fillStyle = `rgba(${red}, ${green}, ${blue}, .82)`
    ctx.fillRect(centerX + x * pixelsPerMeter, centerY - y * pixelsPerMeter, 2, 2)
  }

  ctx.fillStyle = '#ff9f43'
  ctx.beginPath(); ctx.moveTo(centerX + 8, centerY); ctx.lineTo(centerX - 6, centerY - 5); ctx.lineTo(centerX - 6, centerY + 5); ctx.closePath(); ctx.fill()
  ctx.fillStyle = '#7895ac'
  ctx.font = '10px ui-monospace, monospace'
  ctx.fillText(`${props.radius} m`, centerX + props.radius * pixelsPerMeter - 25, centerY - 6)
}

function schedule() {
  if (frame == null) frame = requestAnimationFrame(render)
}

watch(() => [props.cloud, props.yaw, props.radius], schedule, { deep: true })
onMounted(() => {
  resizeObserver = new ResizeObserver(resize)
  resizeObserver.observe(host.value)
  resize()
})
onBeforeUnmount(() => {
  resizeObserver?.disconnect()
  if (frame != null) cancelAnimationFrame(frame)
})
</script>

<template>
  <section ref="host" class="cloud-panel">
    <canvas ref="canvas" />
    <header><strong>局部点云俯视图</strong><span>{{ cloud?.count || 0 }} pts · {{ radius }} m</span></header>
    <div class="height-legend"><span>低</span><i /><span>高</span></div>
  </section>
</template>

<style scoped>
.cloud-panel { position: relative; min-height: 260px; overflow: hidden; border: 1px solid #c8dbe8; border-radius: 12px; background: #f4f8fb; }
canvas { position: absolute; inset: 0; }
header { position: absolute; top: 0; right: 0; left: 0; display: flex; justify-content: space-between; gap: 12px; padding: 11px 13px; color: #1b3a54; background: linear-gradient(180deg, rgba(244, 248, 251, .96), rgba(244, 248, 251, 0)); }
header strong { font-size: 13px; } header span { color: #6d879d; font: 10px ui-monospace, monospace; }
.height-legend { position: absolute; right: 11px; bottom: 9px; display: flex; align-items: center; gap: 5px; color: #6d879d; font-size: 9px; }
.height-legend i { width: 58px; height: 4px; border-radius: 4px; background: linear-gradient(90deg, #1ad8ff, #44df91, #ffd166, #ff4d5f); }
@media (orientation: portrait) { .cloud-panel { min-height: 240px; } }
</style>
