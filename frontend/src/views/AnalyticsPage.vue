<script setup>
import { computed, onMounted, ref } from 'vue'

import TrendLineChart from '../components/TrendLineChart.vue'
import { fetchAnalytics } from '../services/api'

const loading = ref(true)
const analytics = ref(null)

const summaryCards = computed(() => {
  return analytics.value?.cards || []
})

const trends = computed(() => analytics.value?.trends || [])

onMounted(async () => {
  try {
    analytics.value = await fetchAnalytics()
  } finally {
    loading.value = false
  }
})
</script>

<template>
  <section class="page-section">
    <div class="section-head">
      <div>
        <h3>统计分析</h3>
        <p>从预警、检测、巡检时长与执行里程等维度，快速判断平台运行趋势。</p>
      </div>
      <span class="panel-badge">Analytics</span>
    </div>

    <template v-if="!loading">
      <section class="analytics-summary">
        <article v-for="card in summaryCards" :key="card.title" class="metric-card analytics-card">
          <span class="analytics-label">{{ card.title }}</span>
          <strong>{{ card.value }}</strong>
          <p>{{ card.note }}</p>
        </article>
      </section>

      <section class="analytics-grid">
        <TrendLineChart
          v-for="trend in trends"
          :key="trend.title"
          :title="trend.title"
          :subtitle="trend.subtitle"
          :value-suffix="trend.unit"
          :series="trend.series"
          :summary="trend.summary"
          :accent="trend.accent"
        />
      </section>
    </template>
  </section>
</template>
