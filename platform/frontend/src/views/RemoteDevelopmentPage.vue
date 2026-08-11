<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref } from 'vue'

import {
  API_BASE,
  cancelDevelopmentTask,
  createDevelopmentTask,
  fetchDevelopmentAgents,
  fetchDevelopmentTask,
  fetchDevelopmentTasks,
  fetchRobots,
} from '../services/api'

const robots = ref([])
const agents = ref([])
const tasks = ref([])
const selectedTask = ref(null)
const events = ref([])
const selectedRobotId = ref('')
const workspace = ref('robot-main')
const prompt = ref('')
const submitting = ref(false)
const cancelling = ref(false)
const error = ref('')
const terminal = ref(null)
let eventSource = null
let refreshTimer = null

const terminalStates = new Set(['succeeded', 'failed', 'cancelled', 'timed_out', 'rejected'])

const selectedAgent = computed(() => (
  agents.value.find(item => String(item.robot) === String(selectedRobotId.value))
))

const canSubmit = computed(() => (
  selectedRobotId.value && prompt.value.trim() && !submitting.value && !activeTask.value
))

const activeTask = computed(() => (
  tasks.value.find(item => String(item.robot) === String(selectedRobotId.value) && !terminalStates.has(item.status))
))

const statusText = computed(() => selectedTask.value?.status_label || '未执行')

function formatTime(value) {
  if (!value) return '--'
  return new Date(value).toLocaleString('zh-CN', { hour12: false })
}

function eventDisplayText(event) {
  if (event.text) {
    const raw = event.text
    try {
      const parsed = JSON.parse(raw)
      const item = parsed.item || parsed
      return item.text || item.message || item.command || raw
    } catch {
      return raw
    }
  }
  return event.type === 'status' ? (event.payload?.status || '状态更新') : 'Codex 事件'
}

function eventClass(event) {
  if (event.stream === 'stderr') return 'stderr'
  if (event.type === 'result') return 'result'
  if (event.type === 'status') return 'status'
  return 'stdout'
}

async function scrollTerminal() {
  await nextTick()
  if (terminal.value) terminal.value.scrollTop = terminal.value.scrollHeight
}

function closeStream() {
  eventSource?.close()
  eventSource = null
}

function connectStream(task) {
  closeStream()
  if (!task || terminalStates.has(task.status) || typeof EventSource === 'undefined') return
  const token = localStorage.getItem('inspection_token') || ''
  const lastSequence = events.value.at(-1)?.sequence || 0
  const url = `${API_BASE}/development/tasks/${task.id}/stream/?token=${encodeURIComponent(token)}&after=${lastSequence}`
  eventSource = new EventSource(url)
  eventSource.addEventListener('dev_output', (message) => {
    const event = JSON.parse(message.data)
    if (!events.value.some(item => item.sequence === event.sequence)) events.value.push(event)
    scrollTerminal()
  })
  eventSource.addEventListener('dev_status', (message) => {
    const nextTask = JSON.parse(message.data)
    selectedTask.value = { ...selectedTask.value, ...nextTask }
    const index = tasks.value.findIndex(item => item.id === nextTask.id)
    if (index >= 0) tasks.value[index] = { ...tasks.value[index], ...nextTask }
    if (terminalStates.has(nextTask.status)) {
      closeStream()
      refreshTasks()
    }
  })
}

async function selectTask(task) {
  error.value = ''
  selectedTask.value = await fetchDevelopmentTask(task.id)
  events.value = selectedTask.value.events || []
  await scrollTerminal()
  connectStream(selectedTask.value)
}

async function refreshTasks() {
  const result = await fetchDevelopmentTasks(selectedRobotId.value)
  tasks.value = result
  if (selectedTask.value) {
    const updated = result.find(item => item.id === selectedTask.value.id)
    if (updated) selectedTask.value = { ...selectedTask.value, ...updated }
  }
}

async function refreshAgents() {
  agents.value = await fetchDevelopmentAgents()
}

async function chooseRobot() {
  closeStream()
  selectedTask.value = null
  events.value = []
  await refreshTasks()
  if (tasks.value[0]) await selectTask(tasks.value[0])
}

async function submitTask() {
  if (!canSubmit.value) return
  submitting.value = true
  error.value = ''
  try {
    const task = await createDevelopmentTask({
      robot: selectedRobotId.value,
      workspace: workspace.value,
      prompt: prompt.value.trim(),
    })
    prompt.value = ''
    await refreshTasks()
    await selectTask(task)
  } catch (exc) {
    error.value = exc.message
    if (exc.payload?.active_task_id) {
      const task = tasks.value.find(item => item.id === exc.payload.active_task_id)
      if (task) await selectTask(task)
    }
  } finally {
    submitting.value = false
  }
}

async function cancelTask() {
  if (!selectedTask.value?.can_cancel) return
  cancelling.value = true
  error.value = ''
  try {
    selectedTask.value = await cancelDevelopmentTask(selectedTask.value.id)
    await refreshTasks()
  } catch (exc) {
    error.value = exc.message
  } finally {
    cancelling.value = false
  }
}

onMounted(async () => {
  try {
    ;[robots.value, agents.value] = await Promise.all([fetchRobots(), fetchDevelopmentAgents()])
    selectedRobotId.value = robots.value[0]?.id || ''
    if (selectedRobotId.value) await chooseRobot()
    refreshTimer = window.setInterval(() => {
      refreshAgents().catch(() => {})
      refreshTasks().catch(() => {})
    }, 5000)
  } catch (exc) {
    error.value = exc.message
  }
})

onBeforeUnmount(() => {
  closeStream()
  if (refreshTimer) window.clearInterval(refreshTimer)
})
</script>

<template>
  <section class="page-section dev-page">
    <div class="dev-toolbar panel">
      <div>
        <span class="dev-kicker">CODEX CLI BRIDGE</span>
        <h3>远程 AI 开发控制台</h3>
        <p>指令通过云端下发到机器狗，本页实时显示 Codex CLI 输出。</p>
      </div>
      <div class="agent-state" :class="selectedAgent?.status === 'online' ? 'online' : 'offline'">
        <i></i>
        <span>Dev Agent {{ selectedAgent?.status === 'online' ? '在线' : '离线' }}</span>
        <small v-if="selectedAgent">v{{ selectedAgent.agent_version }}</small>
      </div>
    </div>

    <div class="dev-grid">
      <aside class="panel dev-history">
        <div class="panel-head compact">
          <div>
            <h3>开发任务</h3>
            <p>最近 {{ tasks.length }} 条</p>
          </div>
        </div>
        <div class="robot-field">
          <label>机器狗</label>
          <select v-model="selectedRobotId" @change="chooseRobot">
            <option v-for="robot in robots" :key="robot.id" :value="robot.id">
              {{ robot.name }} · {{ robot.code }}
            </option>
          </select>
        </div>
        <div class="task-list">
          <button
            v-for="task in tasks"
            :key="task.id"
            class="task-card"
            :class="{ selected: selectedTask?.id === task.id }"
            @click="selectTask(task)"
          >
            <span class="task-card-top">
              <strong>{{ task.status_label }}</strong>
              <small>{{ formatTime(task.created_at) }}</small>
            </span>
            <span class="task-prompt">{{ task.prompt }}</span>
            <small>{{ task.workspace }}</small>
          </button>
          <p v-if="!tasks.length" class="empty-copy">暂无远程开发任务</p>
        </div>
      </aside>

      <main class="dev-main">
        <section class="panel composer">
          <div class="composer-row">
            <label>
              <span>工作区</span>
              <select v-model="workspace">
                <option value="robot-main">机器狗主工程</option>
                <option value="cloud-platform">云端平台工程</option>
              </select>
            </label>
            <div class="current-state">
              <span>当前状态</span>
              <strong :class="selectedTask?.status || 'idle'">{{ statusText }}</strong>
            </div>
          </div>
          <label class="prompt-field">
            <span>开发指令</span>
            <textarea
              v-model="prompt"
              rows="5"
              placeholder="例如：检查导航模块的速度限制实现，修复问题并执行相关测试"
              @keydown.ctrl.enter.prevent="submitTask"
            ></textarea>
          </label>
          <div class="composer-actions">
            <span v-if="error" class="dev-error">{{ error }}</span>
            <span v-else class="shortcut">Ctrl + Enter 发送</span>
            <button
              v-if="selectedTask?.can_cancel"
              class="danger-btn"
              :disabled="cancelling"
              @click="cancelTask"
            >{{ cancelling ? '取消中…' : '停止任务' }}</button>
            <button class="primary-btn" :disabled="!canSubmit" @click="submitTask">
              {{ submitting ? '正在提交…' : activeTask ? '已有任务执行中' : '发送给 Codex' }}
            </button>
          </div>
        </section>

        <section class="terminal-shell">
          <header>
            <div class="terminal-lights"><i></i><i></i><i></i></div>
            <strong>Codex CLI 实时输出</strong>
            <span v-if="selectedTask">
              对话 {{ selectedTask.codex_thread_id?.slice(0, 8) || '待创建' }} · 任务 {{ selectedTask.id.slice(0, 8) }}
            </span>
          </header>
          <div ref="terminal" class="terminal-body">
            <div v-if="!events.length" class="terminal-empty">
              <strong>等待任务</strong>
              <span>提交开发指令后，Codex 的分析、命令和结果会显示在这里。</span>
            </div>
            <div v-for="event in events" :key="event.sequence" class="terminal-line" :class="eventClass(event)">
              <span class="line-sequence">{{ String(event.sequence).padStart(4, '0') }}</span>
              <span class="line-time">{{ formatTime(event.occurred_at) }}</span>
              <pre>{{ eventDisplayText(event) }}</pre>
            </div>
          </div>
          <footer v-if="selectedTask">
            <span>开始：{{ formatTime(selectedTask.started_at) }}</span>
            <span>结束：{{ formatTime(selectedTask.finished_at) }}</span>
            <span>退出码：{{ selectedTask.exit_code ?? '--' }}</span>
          </footer>
        </section>
      </main>
    </div>
  </section>
</template>

<style scoped>
.dev-page { display: grid; gap: 18px; }
.dev-toolbar { display: flex; align-items: center; justify-content: space-between; gap: 20px; padding: 22px 26px; }
.dev-toolbar h3 { margin: 4px 0 5px; font-size: 24px; }
.dev-toolbar p { margin: 0; color: var(--muted); }
.dev-kicker { color: var(--cyan); font-size: 11px; font-weight: 800; letter-spacing: .16em; }
.agent-state { display: flex; align-items: center; gap: 8px; padding: 10px 14px; border: 1px solid var(--line); border-radius: 999px; background: var(--panel-soft); }
.agent-state i { width: 9px; height: 9px; border-radius: 50%; background: var(--danger); box-shadow: 0 0 12px var(--danger); }
.agent-state.online i { background: var(--green); box-shadow: 0 0 12px var(--green); }
.agent-state small { color: var(--muted); }
.dev-grid { display: grid; grid-template-columns: minmax(250px, 320px) minmax(0, 1fr); gap: 18px; min-height: 680px; }
.dev-history { padding: 18px; min-height: 0; }
.panel-head.compact { margin-bottom: 14px; }
.panel-head.compact h3, .panel-head.compact p { margin: 0; }
.robot-field { display: grid; gap: 6px; margin-bottom: 14px; color: var(--muted); font-size: 12px; }
.robot-field select, .composer select, .prompt-field textarea { width: 100%; border: 1px solid var(--line); border-radius: 12px; color: var(--text); background: var(--input-bg); }
.robot-field select, .composer select { height: 42px; padding: 0 11px; }
.task-list { display: grid; gap: 9px; max-height: 580px; overflow-y: auto; }
.task-card { display: grid; gap: 7px; width: 100%; padding: 12px; border: 1px solid var(--line); border-radius: 14px; color: var(--text); text-align: left; background: var(--panel-soft); }
.task-card.selected { border-color: var(--cyan); box-shadow: 0 0 0 2px color-mix(in srgb, var(--cyan) 16%, transparent); }
.task-card-top { display: flex; justify-content: space-between; gap: 8px; }
.task-card small { color: var(--muted); }
.task-prompt { overflow: hidden; color: var(--text); font-size: 13px; line-height: 1.45; text-overflow: ellipsis; white-space: nowrap; }
.empty-copy { color: var(--muted); text-align: center; }
.dev-main { display: grid; grid-template-rows: auto minmax(420px, 1fr); gap: 18px; min-width: 0; }
.composer { padding: 20px; }
.composer-row { display: grid; grid-template-columns: minmax(180px, 280px) 1fr; gap: 16px; margin-bottom: 14px; }
.composer label, .prompt-field { display: grid; gap: 7px; color: var(--muted); font-size: 12px; }
.current-state { display: flex; align-items: flex-end; justify-content: flex-end; gap: 9px; color: var(--muted); }
.current-state strong { padding: 8px 12px; border-radius: 10px; color: var(--text); background: var(--chip-bg); }
.current-state strong.running { color: var(--green); }
.current-state strong.failed, .current-state strong.rejected { color: var(--danger); }
.prompt-field textarea { min-height: 120px; padding: 13px; resize: vertical; line-height: 1.6; }
.composer-actions { display: flex; align-items: center; justify-content: flex-end; gap: 10px; margin-top: 12px; }
.composer-actions .shortcut, .dev-error { margin-right: auto; color: var(--muted); font-size: 12px; }
.dev-error { color: var(--danger); }
.primary-btn, .danger-btn { min-height: 42px; padding: 0 18px; border: 0; border-radius: 12px; color: white; font-weight: 700; }
.primary-btn { background: linear-gradient(135deg, #2588ff, #4e63ff); }
.danger-btn { background: rgba(255, 93, 93, .9); }
.primary-btn:disabled, .danger-btn:disabled { cursor: not-allowed; opacity: .45; }
.terminal-shell { display: grid; grid-template-rows: 46px minmax(0, 1fr) 38px; min-height: 0; overflow: hidden; border: 1px solid rgba(111, 180, 255, .2); border-radius: 18px; color: #dcecff; background: #07111f; box-shadow: var(--shadow); }
.terminal-shell header, .terminal-shell footer { display: flex; align-items: center; gap: 14px; padding: 0 16px; border-bottom: 1px solid rgba(146, 197, 255, .12); background: #0c1a2d; }
.terminal-shell header > span { margin-left: auto; color: #7897b9; font-family: monospace; }
.terminal-lights { display: flex; gap: 6px; }
.terminal-lights i { width: 9px; height: 9px; border-radius: 50%; background: #ff6b6b; }
.terminal-lights i:nth-child(2) { background: #ffc857; }
.terminal-lights i:nth-child(3) { background: #38d996; }
.terminal-body { min-height: 0; overflow: auto; padding: 14px 0; font: 12px/1.6 "SFMono-Regular", Consolas, monospace; }
.terminal-empty { display: grid; place-content: center; gap: 8px; height: 100%; color: #7897b9; text-align: center; }
.terminal-line { display: grid; grid-template-columns: 48px 150px minmax(0, 1fr); gap: 10px; padding: 4px 16px; border-left: 2px solid transparent; }
.terminal-line:hover { background: rgba(255,255,255,.035); }
.terminal-line.stderr { border-left-color: #ff6b6b; color: #ff9b9b; }
.terminal-line.status { border-left-color: #43d5ff; color: #7ee8ff; }
.terminal-line.result { border-left-color: #52df9a; color: #7ff1ca; }
.terminal-line pre { margin: 0; overflow-wrap: anywhere; white-space: pre-wrap; }
.line-sequence { color: #466583; }
.line-time { color: #7897b9; }
.terminal-shell footer { justify-content: flex-end; border-top: 1px solid rgba(146, 197, 255, .12); border-bottom: 0; color: #7897b9; font-size: 11px; }
@media (max-width: 900px) {
  .dev-toolbar { align-items: flex-start; flex-direction: column; }
  .dev-grid { grid-template-columns: 1fr; }
  .dev-history { max-height: 300px; }
  .dev-main { grid-template-rows: auto 560px; }
}
@media (max-width: 620px) {
  .composer-row { grid-template-columns: 1fr; }
  .current-state { justify-content: flex-start; }
  .terminal-line { grid-template-columns: 42px minmax(0, 1fr); }
  .line-time { display: none; }
  .terminal-shell footer { justify-content: flex-start; overflow-x: auto; }
}
</style>
