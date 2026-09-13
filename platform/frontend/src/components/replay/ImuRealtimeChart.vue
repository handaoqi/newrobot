<script setup>
import { computed } from 'vue'
import { LineChart } from 'echarts/charts'
import { GridComponent, LegendComponent, TooltipComponent } from 'echarts/components'
import { use } from 'echarts/core'
import { CanvasRenderer } from 'echarts/renderers'
import VChart from 'vue-echarts'

use([CanvasRenderer, LineChart, GridComponent, LegendComponent, TooltipComponent])

const props = defineProps({ samples: { type: Array, default: () => [] } })

const option = computed(() => ({
  animation: false,
  color: ['#35c8f4', '#54dd8b', '#ff9f43'],
  grid: { top: 38, right: 14, bottom: 22, left: 46 },
  legend: { top: 8, right: 10, data: ['gx', 'gy', 'gz'], textStyle: { color: '#5f7890', fontSize: 10 } },
  tooltip: { trigger: 'axis', backgroundColor: '#ffffff', borderColor: '#c7d9e6', textStyle: { color: '#24435d' } },
  xAxis: {
    type: 'category', boundaryGap: false, data: props.samples.map(item => item.label),
    axisLabel: { show: false }, axisTick: { show: false }, axisLine: { lineStyle: { color: '#294052' } },
  },
  yAxis: {
    type: 'value', axisLabel: { color: '#7890a5', fontSize: 9 }, axisLine: { show: false }, axisTick: { show: false },
    splitLine: { lineStyle: { color: 'rgba(78, 134, 164, .2)', type: 'dashed' } },
  },
  series: ['gx', 'gy', 'gz'].map(key => ({
    name: key, type: 'line', showSymbol: false, smooth: .16,
    data: props.samples.map(item => item[key]), lineStyle: { width: 1.5 },
  })),
}))
</script>

<template>
  <section class="imu-panel">
    <header><strong>IMU 实时监控</strong><span>ANGULAR VELOCITY · 200 帧</span></header>
    <VChart class="chart" :option="option" autoresize />
  </section>
</template>

<style scoped>
.imu-panel { display: grid; grid-template-rows: auto minmax(0, 1fr); min-height: 240px; overflow: hidden; border: 1px solid #c8dbe8; border-radius: 12px; background: #f8fbfd; }
header { display: flex; justify-content: space-between; gap: 10px; padding: 11px 13px 0; color: #1b3a54; }
header strong { font-size: 13px; } header span { color: #7890a5; font: 9px ui-monospace, monospace; letter-spacing: .06em; }
.chart { width: 100%; min-height: 190px; }
@media (orientation: portrait) { .imu-panel { min-height: 250px; } }
</style>
