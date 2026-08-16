<script setup>
import { computed } from 'vue'
import { LineChart } from 'echarts/charts'
import {
  GridComponent,
  LegendComponent,
  MarkLineComponent,
  TooltipComponent,
} from 'echarts/components'
import { CanvasRenderer } from 'echarts/renderers'
import { use } from 'echarts/core'
import VChart from 'vue-echarts'

use([CanvasRenderer, LineChart, GridComponent, TooltipComponent, LegendComponent, MarkLineComponent])

const props = defineProps({
  title: {
    type: String,
    default: '',
  },
  subtitle: {
    type: String,
    default: '',
  },
  valueSuffix: {
    type: String,
    default: '',
  },
  series: {
    type: Array,
    default: () => [],
  },
  summary: {
    type: Object,
    default: () => null,
  },
  accent: {
    type: String,
    default: '#2f7bff',
  },
})

const stats = computed(() => {
  const values = props.series.map((item) => Number(item.value) || 0)
  const max = Math.max(...values, 1)
  const min = Math.min(...values, 0)
  const range = max - min || 1
  const total = values.reduce((sum, value) => sum + value, 0)
  const latest = values.at(-1) || 0
  const previous = values.at(-2) || latest
  const delta = latest - previous
  const direction = delta >= 0 ? '较上一周期上升' : '较上一周期回落'

  return {
    max,
    min,
    range,
    total,
    latest,
    delta,
    direction,
  }
})

const average = computed(() => {
  if (!props.series.length) return 0
  return Math.round((stats.value.total / props.series.length) * 10) / 10
})

const displaySummary = computed(() => ({
  latest: props.summary?.latest ?? stats.value.latest,
  delta: props.summary?.delta ?? Math.abs(stats.value.delta),
  direction: props.summary?.direction ?? stats.value.direction,
  total: props.summary?.total ?? stats.value.total,
  average: props.summary?.average ?? average.value,
}))

const chartOption = computed(() => ({
  animation: true,
  grid: { left: 10, right: 10, top: 18, bottom: 24, containLabel: true },
  tooltip: {
    trigger: 'axis',
    backgroundColor: 'rgba(11, 25, 46, 0.92)',
    borderColor: 'rgba(146, 197, 255, 0.22)',
    textStyle: { color: '#eff7ff' },
    formatter: (params) => {
      const point = params?.[0]
      if (!point) return ''
      return `${point.axisValue}<br/>${props.title}: ${point.data}${props.valueSuffix}`
    },
  },
  xAxis: {
    type: 'category',
    boundaryGap: false,
    data: props.series.map((item) => item.label),
    axisLine: { lineStyle: { color: 'rgba(123, 154, 194, 0.18)' } },
    axisTick: { show: false },
    axisLabel: { color: '#8aa4c6', fontSize: 12 },
  },
  yAxis: {
    type: 'value',
    minInterval: 1,
    splitLine: {
      lineStyle: {
        color: 'rgba(123, 154, 194, 0.18)',
        type: 'dashed',
      },
    },
    axisLine: { show: false },
    axisTick: { show: false },
    axisLabel: { color: '#8aa4c6', fontSize: 12 },
  },
  series: [
    {
      type: 'line',
      smooth: 0.32,
      data: props.series.map((item) => Number(item.value) || 0),
      symbol: 'circle',
      symbolSize: 9,
      showSymbol: true,
      lineStyle: {
        color: props.accent,
        width: 4,
      },
      itemStyle: {
        color: props.accent,
        borderColor: '#ffffff',
        borderWidth: 2,
      },
      areaStyle: {
        color: {
          type: 'linear',
          x: 0,
          y: 0,
          x2: 0,
          y2: 1,
          colorStops: [
            { offset: 0, color: `${props.accent}66` },
            { offset: 1, color: `${props.accent}05` },
          ],
        },
      },
      emphasis: {
        focus: 'series',
      },
      markLine: {
        symbol: 'none',
        label: {
          color: '#8aa4c6',
          formatter: `均值 ${displaySummary.value.average}${props.valueSuffix}`,
        },
        lineStyle: {
          color: `${props.accent}88`,
          type: 'dashed',
        },
        data: [{ yAxis: displaySummary.value.average }],
      },
    },
  ],
}))
</script>

<template>
  <article class="panel trend-panel">
    <div class="panel-head trend-head">
      <div>
        <h3>{{ title }}</h3>
        <p>{{ subtitle }}</p>
      </div>
      <div class="trend-highlight">
        <strong>{{ displaySummary.latest }}{{ valueSuffix }}</strong>
        <span>{{ displaySummary.direction }} {{ displaySummary.delta }}{{ valueSuffix }}</span>
      </div>
    </div>

    <div class="trend-stage">
      <VChart class="trend-chart" :option="chartOption" autoresize />
    </div>

    <div class="trend-footer">
      <div class="trend-meta">
        <strong>7期累计</strong>
        <span>{{ displaySummary.total }}{{ valueSuffix }}</span>
      </div>
      <div class="trend-meta">
        <strong>平均值</strong>
        <span>{{ displaySummary.average }}{{ valueSuffix }}</span>
      </div>
      <div class="trend-labels">
        <span v-for="item in series" :key="item.label">{{ item.label }}</span>
      </div>
    </div>
  </article>
</template>
