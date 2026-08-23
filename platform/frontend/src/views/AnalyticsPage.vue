<script setup>
import { computed, onMounted, ref } from 'vue'

import TrendLineChart from '../components/TrendLineChart.vue'
import { fetchAnalytics } from '../services/api'

const loading = ref(true)
const analytics = ref(null)
const loadError = ref('')

const skeletonCards = [
  { title: '累计预警', value: '', note: '正在汇总近 7 个统计周期' },
  { title: '检测识别', value: '', note: '正在汇总识别抓拍数据' },
  { title: '平均完成度', value: '', note: '正在计算任务完成度' },
  { title: '值守响应', value: '', note: '正在计算平均响应时长' },
]

const skeletonTrends = [
  { title: '预警次数趋势', subtitle: '正在加载异常波动数据', unit: '次', accent: '#fb7b4d' },
  { title: '检测次数趋势', subtitle: '正在加载识别活跃度数据', unit: '次', accent: '#2d8cff' },
  { title: '巡检时长趋势', subtitle: '正在加载巡检时长数据', unit: '分钟', accent: '#19b97f' },
  { title: '执行里程趋势', subtitle: '正在加载巡检覆盖数据', unit: '公里', accent: '#7b6cff' },
]

const summaryCards = computed(() => {
  return analytics.value?.cards || skeletonCards
})

const trends = computed(() => analytics.value?.trends || skeletonTrends)
const dataPending = computed(() => loading.value || !analytics.value)

async function loadAnalytics() {
  loading.value = true
  loadError.value = ''
  try {
    analytics.value = await fetchAnalytics()
  } catch (error) {
    loadError.value = error?.message || '统计数据加载失败'
  } finally {
    loading.value = false
  }
}

onMounted(loadAnalytics)
</script>

<template>
  <section class="page-section analytics-page" :aria-busy="dataPending">
    <header class="analytics-page-head">
      <div>
        <span class="eyebrow">ANALYTICS</span>
        <h2>统计分析</h2>
        <p>查看巡检、识别与值守运行趋势。</p>
      </div>
      <span class="analytics-load-state" :class="{ 'is-ready': !dataPending }">
        {{ dataPending ? '数据加载中' : '数据已更新' }}
      </span>
    </header>

    <div v-if="loading" class="analytics-loading-banner" role="status">
      页面框架已就绪，正在加载统计数据，请稍候…
    </div>
    <div v-else-if="loadError" class="analytics-error-banner" role="alert">
      <span>{{ loadError }}，当前显示占位框架。</span>
      <button type="button" class="ghost-btn" @click="loadAnalytics">重新加载</button>
    </div>

    <section class="analytics-summary">
      <article
        v-for="card in summaryCards"
        :key="card.title"
        class="metric-card analytics-card"
        :class="{ 'is-loading': dataPending }"
      >
        <span class="analytics-label">{{ card.title }}</span>
        <strong v-if="dataPending" class="analytics-skeleton-value" aria-label="加载中"></strong>
        <strong v-else>{{ card.value }}</strong>
        <p>{{ card.note }}</p>
      </article>
    </section>

    <section class="analytics-grid">
      <TrendLineChart
        v-for="trend in trends"
        :key="trend.title"
        :class="{ 'is-loading': dataPending }"
        :title="trend.title"
        :subtitle="trend.subtitle"
        :value-suffix="trend.unit"
        :series="trend.series || []"
        :summary="trend.summary || { latest: 0, delta: 0, direction: '数据加载中', total: 0, average: 0 }"
        :accent="trend.accent"
      />
    </section>
  </section>
</template>
