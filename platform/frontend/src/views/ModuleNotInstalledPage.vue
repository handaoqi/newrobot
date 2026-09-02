<script setup>
import { computed } from 'vue'
import { useRoute, useRouter } from 'vue-router'
import { getModule } from '../modules/index.js'

const route = useRoute()
const router = useRouter()
const module = computed(() => getModule(String(route.query.moduleId || '')))

function goHome() {
  router.push('/dashboard/overview')
}
</script>

<template>
  <section class="page-section module-not-installed" data-testid="module-not-installed">
    <span class="eyebrow">MODULE NOT INSTALLED</span>
    <h2>页面未安装</h2>
    <p v-if="module">模块 <code>{{ module.id }}</code>（{{ module.title }}）不在当前前端安装配置中。</p>
    <p v-else>当前地址对应的页面模块不在此版本的安装目录中。</p>
    <div class="module-install-command">
      <span>启用命令</span>
      <code>frontendctl modules enable {{ module?.id || '&lt;module-id&gt;' }}</code>
    </div>
    <button type="button" class="primary-btn" @click="goHome">返回可用首页</button>
  </section>
</template>
