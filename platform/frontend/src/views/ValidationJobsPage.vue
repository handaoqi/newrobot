<script setup>
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'

import {
  approveValidationBaseline,
  cancelValidationJob,
  createValidationLiveTicket,
  createValidationJob,
  fetchValidationJob,
  fetchValidationJobs,
  fetchValidationProfiles,
  fetchValidationRecordings,
  fetchValidationRunners,
  uploadValidationRecording,
} from '../services/api'

const recordings = ref([])
const profiles = ref([])
const runners = ref([])
const jobs = ref([])
const selectedJob = ref(null)
const selectedRecordingId = ref('')
const selectedProfileId = ref('')
const file = ref(null)
const uploadLabel = ref('')
const busy = ref(false)
const error = ref('')
let refreshTimer = null

const activeStates = new Set(['queued', 'staging', 'running', 'analyzing', 'uploading', 'cancelling'])
const activeJob = computed(() => jobs.value.find(item => activeStates.has(item.state)))
const resultArtifact = computed(() => selectedJob.value?.artifacts?.find(item => item.role === 'result_mcap'))

const stateLabels = {
  queued: '排队中', staging: '准备中', running: '重算中', analyzing: '分析中', uploading: '上传结果',
  cancelling: '取消中', completed: '已完成', cancelled: '已取消', infra_error: '基础设施失败',
}

function formatTime(value) {
  return value ? new Date(value).toLocaleString('zh-CN', { hour12: false }) : '--'
}

function formatBytes(value) {
  const size = Number(value || 0)
  if (size >= 1024 ** 3) return `${(size / 1024 ** 3).toFixed(2)} GiB`
  if (size >= 1024 ** 2) return `${(size / 1024 ** 2).toFixed(1)} MiB`
  return `${Math.round(size / 1024)} KiB`
}

function artifactUrl(artifact) {
  return artifact ? `${window.location.origin}${artifact.signed_download_path}` : ''
}

function foxgloveUrl(artifact) {
  const source = artifactUrl(artifact)
  return `/foxglove/?ds=remote-file&ds.url=${encodeURIComponent(source)}`
}

async function openLiveFoxglove(job) {
  const popup = window.open('about:blank', '_blank')
  try {
    const { ticket } = await createValidationLiveTicket(job.id)
    const scheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const source = `${scheme}//${window.location.host}/validation-ws/?ticket=${encodeURIComponent(ticket)}`
    const target = `/foxglove/?ds=foxglove-websocket&ds.url=${encodeURIComponent(source)}`
    if (popup) popup.location = target
    else window.location.assign(target)
  } catch (nextError) {
    popup?.close()
    error.value = nextError.message
  }
}

async function refresh({ preserveSelection = true } = {}) {
  try {
    const [nextRecordings, nextProfiles, nextRunners, nextJobs] = await Promise.all([
      fetchValidationRecordings(), fetchValidationProfiles(), fetchValidationRunners(), fetchValidationJobs(),
    ])
    recordings.value = nextRecordings
    profiles.value = nextProfiles
    runners.value = nextRunners
    jobs.value = nextJobs
    if (!selectedRecordingId.value && nextRecordings.length) selectedRecordingId.value = nextRecordings[0].id
    if (!selectedProfileId.value && nextProfiles.length) selectedProfileId.value = nextProfiles[0].id
    const selectedId = preserveSelection ? selectedJob.value?.id : nextJobs[0]?.id
    if (selectedId) selectedJob.value = await fetchValidationJob(selectedId)
    else selectedJob.value = nextJobs[0] || null
  } catch (nextError) {
    error.value = nextError.message
  }
}

async function submitUpload() {
  if (!file.value) return
  busy.value = true
  error.value = ''
  try {
    const recording = await uploadValidationRecording(file.value, { label: uploadLabel.value })
    file.value = null
    uploadLabel.value = ''
    await refresh()
    selectedRecordingId.value = recording.id
  } catch (nextError) {
    error.value = nextError.message
  } finally {
    busy.value = false
  }
}

async function submitJob() {
  if (!selectedRecordingId.value || !selectedProfileId.value) return
  busy.value = true
  error.value = ''
  try {
    const job = await createValidationJob({
      recording_id: selectedRecordingId.value,
      profile_id: selectedProfileId.value,
      idempotency_key: crypto.randomUUID(),
    })
    selectedJob.value = job
    await refresh()
  } catch (nextError) {
    error.value = nextError.message
  } finally {
    busy.value = false
  }
}

async function selectJob(job) {
  selectedJob.value = await fetchValidationJob(job.id)
}

async function cancelJob() {
  if (!selectedJob.value) return
  selectedJob.value = await cancelValidationJob(selectedJob.value.id)
  await refresh()
}

async function approveBaseline() {
  if (!selectedJob.value) return
  try {
    await approveValidationBaseline(selectedJob.value.id)
    await refresh()
  } catch (nextError) {
    error.value = nextError.message
  }
}

onMounted(async () => {
  await refresh({ preserveSelection: false })
  refreshTimer = window.setInterval(() => refresh().catch(() => {}), 4000)
})

onBeforeUnmount(() => window.clearInterval(refreshTimer))
</script>

<template>
  <main class="validation-page">
    <header class="page-head">
      <div>
        <p class="eyebrow">VALIDATION LOOP</p>
        <h1>仿真与回放检查</h1>
        <p>真实 MCAP 隔离重算、规则/黄金基线判定和 Foxglove 证据回放。</p>
      </div>
      <div class="runner-summary">
        <strong>{{ runners.filter(item => item.state !== 'offline').length }}</strong>
        <span>在线 Runner</span>
      </div>
    </header>

    <p v-if="error" class="error-banner">{{ error }}</p>

    <section class="composer-grid">
      <article class="validation-card">
        <h2>上传 MCAP</h2>
        <input v-model="uploadLabel" placeholder="录制名称（可选）" />
        <input type="file" accept=".mcap" @change="file = $event.target.files?.[0] || null" />
        <button :disabled="busy || !file" @click="submitUpload">{{ busy ? '处理中…' : '登记并上传' }}</button>
        <small>生产环境使用 64 MiB 分片直传对象存储；文件不经过 MQTT。</small>
      </article>

      <article class="validation-card">
        <h2>创建检查作业</h2>
        <select v-model="selectedRecordingId">
          <option disabled value="">选择录制</option>
          <option v-for="item in recordings" :key="item.id" :value="item.id">
            {{ item.label }} · {{ formatBytes(item.size_bytes) }} · {{ item.state }}
          </option>
        </select>
        <select v-model="selectedProfileId">
          <option disabled value="">选择检查 Profile</option>
          <option v-for="item in profiles" :key="item.id" :value="item.id">
            {{ item.name }} v{{ item.version }} · {{ item.mode }}
          </option>
        </select>
        <button :disabled="busy || !selectedRecordingId || !selectedProfileId" @click="submitJob">创建隔离作业</button>
        <small>默认按需触发、CPU Runner 单并发；MATRiX/UE 未部署时明确拒绝。</small>
      </article>
    </section>

    <section class="workspace-grid">
      <article class="validation-card job-list-card">
        <div class="card-title"><h2>作业记录</h2><span v-if="activeJob">有作业运行中</span></div>
        <button
          v-for="job in jobs" :key="job.id" class="job-row"
          :class="{ selected: selectedJob?.id === job.id }" @click="selectJob(job)"
        >
          <span><strong>{{ job.recording_label }}</strong><small>{{ formatTime(job.created_at) }}</small></span>
          <span class="job-state"><b :class="`verdict-${job.verdict}`">{{ job.verdict }}</b>{{ stateLabels[job.state] }}</span>
        </button>
        <p v-if="!jobs.length" class="empty">尚无检查作业</p>
      </article>

      <article v-if="selectedJob" class="validation-card job-detail-card">
        <div class="card-title">
          <div><h2>{{ selectedJob.recording_label }}</h2><small>{{ selectedJob.profile_name }} v{{ selectedJob.profile_version }}</small></div>
          <strong :class="`verdict verdict-${selectedJob.verdict}`">{{ selectedJob.verdict }}</strong>
        </div>
        <div class="progress"><i :style="{ width: `${selectedJob.progress_percent}%` }" /></div>
        <div class="detail-metrics">
          <span><small>状态</small>{{ stateLabels[selectedJob.state] }}</span>
          <span><small>Runner</small>{{ selectedJob.runner_name || '等待领取' }}</span>
          <span><small>尝试</small>{{ selectedJob.attempt_count }}</span>
          <span><small>消息</small>{{ selectedJob.summary?.metrics?.message_count ?? '--' }}</span>
        </div>
        <div class="actions">
          <button v-if="activeStates.has(selectedJob.state)" class="secondary" @click="cancelJob">取消作业</button>
          <button v-if="selectedJob.state === 'completed' && selectedJob.verdict !== 'FAIL'" class="secondary" @click="approveBaseline">批准为黄金基线</button>
          <button v-if="activeStates.has(selectedJob.state) && selectedJob.live_bridge_url" @click="openLiveFoxglove(selectedJob)">实时查看采样话题</button>
          <a v-if="resultArtifact" :href="foxgloveUrl(resultArtifact)" target="_blank">在 Foxglove 中回放</a>
        </div>
        <p v-if="selectedJob.error_message" class="error-banner">{{ selectedJob.error_code }}：{{ selectedJob.error_message }}</p>

        <h3>检查结果</h3>
        <div class="checks">
          <button v-for="check in selectedJob.checks" :key="check.id" class="check-row">
            <b :class="`verdict-${check.status}`">{{ check.status }}</b>
            <span><strong>{{ check.title }}</strong><small>{{ check.message || check.rule_id }}</small></span>
            <time v-if="check.start_time_ns">{{ (check.start_time_ns / 1e9).toFixed(3) }}s</time>
          </button>
          <p v-if="!selectedJob.checks?.length" class="empty">作业完成后显示结构化检查结果</p>
        </div>

        <h3>制品</h3>
        <div class="artifacts">
          <a v-for="artifact in selectedJob.artifacts" :key="artifact.id" :href="artifactUrl(artifact)" target="_blank">
            {{ artifact.role }} · {{ artifact.name }} · {{ formatBytes(artifact.size_bytes) }}
          </a>
        </div>
      </article>
      <article v-else class="validation-card empty-detail">选择一个作业查看报告</article>
    </section>
  </main>
</template>

<style scoped>
.validation-page { display: grid; gap: 20px; padding: 4px; }
.page-head, .card-title, .detail-metrics, .actions { display: flex; align-items: center; justify-content: space-between; gap: 16px; }
.page-head h1, .validation-card h2, .validation-card h3 { margin: 0; }
.page-head p { margin: 7px 0 0; color: var(--muted); }
.eyebrow { color: #35d6c5 !important; font-size: 12px; font-weight: 800; letter-spacing: .16em; }
.runner-summary { min-width: 130px; padding: 16px; border: 1px solid var(--line); border-radius: 18px; text-align: center; }
.runner-summary strong, .runner-summary span { display: block; }
.runner-summary strong { font-size: 28px; }
.runner-summary span, small { color: var(--muted); }
.composer-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 16px; }
.workspace-grid { display: grid; grid-template-columns: minmax(280px, .8fr) minmax(0, 1.8fr); gap: 16px; align-items: start; }
.validation-card { display: grid; gap: 14px; padding: 20px; border: 1px solid var(--line); border-radius: 20px; background: var(--panel-bg); }
input, select { width: 100%; box-sizing: border-box; padding: 11px 13px; border: 1px solid var(--line); border-radius: 12px; background: var(--input-bg, transparent); color: inherit; }
button, .actions a { border: 0; border-radius: 12px; padding: 11px 15px; background: #1e88e5; color: #fff; cursor: pointer; text-decoration: none; font: inherit; }
button:disabled { opacity: .45; cursor: not-allowed; }
button.secondary { background: transparent; color: inherit; border: 1px solid var(--line); }
.job-row, .check-row { display: flex; justify-content: space-between; align-items: center; width: 100%; text-align: left; background: transparent; color: inherit; border: 1px solid transparent; }
.job-row:hover, .job-row.selected { border-color: #35d6c5; background: rgba(53, 214, 197, .08); }
.job-row span, .check-row span { display: grid; gap: 4px; }
.job-state { text-align: right; justify-items: end; font-size: 12px; }
.progress { height: 7px; overflow: hidden; border-radius: 99px; background: rgba(127, 127, 127, .18); }
.progress i { display: block; height: 100%; background: linear-gradient(90deg, #1e88e5, #35d6c5); }
.detail-metrics { display: grid; grid-template-columns: repeat(4, 1fr); }
.detail-metrics span { display: grid; gap: 4px; }
.checks, .artifacts { display: grid; gap: 8px; }
.check-row { cursor: default; border-color: var(--line); }
.artifacts a { color: #37a7ff; }
.verdict { padding: 8px 12px; border-radius: 999px; }
.verdict-PASS { color: #28c58b; } .verdict-WARN, .verdict-NOT_EVALUATED { color: #e9a23b; } .verdict-FAIL { color: #f26464; }
.error-banner { margin: 0; padding: 12px 14px; border-radius: 12px; background: rgba(242, 100, 100, .12); color: #f26464; }
.empty, .empty-detail { color: var(--muted); text-align: center; }
@media (max-width: 900px) { .composer-grid, .workspace-grid { grid-template-columns: 1fr; } .detail-metrics { grid-template-columns: repeat(2, 1fr); } }
</style>
