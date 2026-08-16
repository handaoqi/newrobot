<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref } from 'vue'

import {
  API_BASE,
  cancelDevelopmentTask,
  createDevelopmentTask,
  fetchDevelopmentAgents,
  fetchDevelopmentConversation,
  fetchDevelopmentTasks,
  fetchVoiceRecognitions,
  fetchRobots,
} from '../services/api'

const robots = ref([])
const agents = ref([])
const tasks = ref([])
const voiceRecognitions = ref([])
const selectedTask = ref(null)
const turns = ref([])
const conversationThreadId = ref('')
const selectedRobotId = ref('')
const workspace = ref('robot-main')
const model = ref('gpt-5.6-terra')
const prompt = ref('')
const submitting = ref(false)
const cancelling = ref(false)
const error = ref('')
const terminal = ref(null)
const composer = ref(null)
const autoFollowLatest = ref(true)
let eventSource = null
let refreshTimer = null

const terminalStates = new Set(['succeeded', 'failed', 'cancelled', 'timed_out', 'rejected'])
const modelOptions = [
  { value: 'gpt-5.6-terra', label: 'Terra（默认 · 均衡）' },
  { value: 'gpt-5.6-sol', label: 'Sol（复杂任务）' },
  { value: 'gpt-5.6-luna', label: 'Luna（快速任务）' },
]

const selectedAgent = computed(() => (
  agents.value.find(item => String(item.robot) === String(selectedRobotId.value))
))

const canSubmit = computed(() => (
  selectedRobotId.value && prompt.value.trim() && !submitting.value && !activeTask.value
))

const activeTask = computed(() => (
  tasks.value.find(item => String(item.robot) === String(selectedRobotId.value) && !terminalStates.has(item.status))
))

const statusText = computed(() => (
  activeTask.value?.status_label || selectedTask.value?.status_label || '未执行'
))

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
      if (parsed.type === 'thread.started') return '已连接主会话，继续原有上下文'
      if (parsed.type === 'turn.started') return 'Codex 开始处理本轮指令'
      if (parsed.type === 'turn.completed') return '本轮指令处理完成'
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

function visibleEvents(events) {
  return events.filter(event => !(
    event.stream === 'stderr'
    && event.text?.includes('codex_models_manager::cache: failed to load models cache:')
    && event.text.includes('base_instructions')
  ))
}

function updateAutoFollow() {
  if (!terminal.value) return
  const { scrollTop, scrollHeight, clientHeight } = terminal.value
  autoFollowLatest.value = scrollHeight - scrollTop - clientHeight < 48
}

async function scrollTerminal(force = false) {
  await nextTick()
  await new Promise(resolve => window.requestAnimationFrame(resolve))
  await new Promise(resolve => window.requestAnimationFrame(resolve))
  if (terminal.value && (force || autoFollowLatest.value)) {
    terminal.value.scrollTop = terminal.value.scrollHeight
    autoFollowLatest.value = true
  }
}

function closeStream() {
  eventSource?.close()
  eventSource = null
}

function turnForTask(taskId) {
  return turns.value.find(turn => turn.task.id === taskId)
}

function eventKey(taskId, event) {
  return `${taskId}:${event.sequence}`
}

function upsertTask(nextTask) {
  const index = tasks.value.findIndex(item => item.id === nextTask.id)
  if (index >= 0) tasks.value[index] = { ...tasks.value[index], ...nextTask }
  else tasks.value.unshift(nextTask)
  const turn = turnForTask(nextTask.id)
  if (turn) turn.task = { ...turn.task, ...nextTask }
  if (nextTask.codex_thread_id) conversationThreadId.value = nextTask.codex_thread_id
  if (!selectedTask.value || selectedTask.value.id === nextTask.id || !terminalStates.has(nextTask.status)) {
    selectedTask.value = { ...(selectedTask.value?.id === nextTask.id ? selectedTask.value : {}), ...nextTask }
  }
}

function connectStream(task) {
  closeStream()
  if (!task || terminalStates.has(task.status) || typeof EventSource === 'undefined') return
  const token = localStorage.getItem('inspection_token') || ''
  const turn = turnForTask(task.id)
  const lastSequence = turn?.events.at(-1)?.sequence || 0
  const url = `${API_BASE}/development/tasks/${task.id}/stream/?token=${encodeURIComponent(token)}&after=${lastSequence}`
  eventSource = new EventSource(url)
  eventSource.addEventListener('dev_output', (message) => {
    const event = JSON.parse(message.data)
    if (event.payload?.codex_thread_id) conversationThreadId.value = event.payload.codex_thread_id
    let target = turnForTask(task.id)
    if (!target) {
      target = { task, events: [] }
      turns.value.push(target)
    }
    if (!target.events.some(item => item.sequence === event.sequence)) target.events.push(event)
    scrollTerminal()
  })
  eventSource.addEventListener('dev_status', (message) => {
    const nextTask = JSON.parse(message.data)
    upsertTask(nextTask)
    if (terminalStates.has(nextTask.status)) {
      closeStream()
      refreshConversation().catch(() => {})
    }
  })
}

async function refreshConversation() {
  error.value = ''
  const result = await fetchDevelopmentConversation(selectedRobotId.value)
  tasks.value = result.tasks || []
  turns.value = result.turns || []
  conversationThreadId.value = result.codex_thread_id || ''
  selectedTask.value = activeTask.value || tasks.value[0] || null
  await scrollTerminal(true)
  if (activeTask.value) connectStream(activeTask.value)
}

async function refreshTasks() {
  const result = await fetchDevelopmentTasks(selectedRobotId.value)
  const visibleTaskIds = new Set(turns.value.map(turn => turn.task.id))
  tasks.value = result.filter(item => visibleTaskIds.has(item.id) || !terminalStates.has(item.status))
  for (const task of tasks.value) {
    const turn = turnForTask(task.id)
    if (turn) turn.task = { ...turn.task, ...task }
  }
  selectedTask.value = activeTask.value || tasks.value[0] || null
  if (!eventSource && activeTask.value) connectStream(activeTask.value)
}

async function refreshAgents() {
  agents.value = await fetchDevelopmentAgents()
}

async function refreshVoiceRecognitions() {
  if (!selectedRobotId.value) {
    voiceRecognitions.value = []
    return
  }
  voiceRecognitions.value = await fetchVoiceRecognitions(selectedRobotId.value)
}

async function chooseRobot() {
  closeStream()
  selectedTask.value = null
  turns.value = []
  conversationThreadId.value = ''
  await Promise.all([refreshConversation(), refreshVoiceRecognitions()])
  await scrollTerminal(true)
}

async function submitTask() {
  if (!canSubmit.value) return
  submitting.value = true
  error.value = ''
  try {
    const task = await createDevelopmentTask({
      robot: selectedRobotId.value,
      workspace: workspace.value,
      model: model.value,
      conversation_id: 'main',
      prompt: prompt.value.trim(),
    })
    prompt.value = ''
    tasks.value.unshift(task)
    turns.value.push({ task, events: [] })
    selectedTask.value = task
    connectStream(task)
    await scrollTerminal(true)
  } catch (exc) {
    error.value = exc.message
    if (exc.payload?.active_task_id) {
      const task = tasks.value.find(item => item.id === exc.payload.active_task_id)
      if (task) {
        selectedTask.value = task
        connectStream(task)
      }
    }
  } finally {
    submitting.value = false
  }
}

async function cancelTask() {
  const task = activeTask.value
  if (!task?.can_cancel) return
  cancelling.value = true
  error.value = ''
  try {
    upsertTask(await cancelDevelopmentTask(task.id))
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
      refreshVoiceRecognitions().catch(() => {})
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
            <h3>主会话记录</h3>
            <p>连续上下文 · 最近 {{ turns.length }} 轮</p>
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
          <article
            v-for="task in tasks"
            :key="task.id"
            class="task-card"
            :class="{ active: !terminalStates.has(task.status) }"
          >
            <span class="task-card-top">
              <strong>{{ task.status_label }}</strong>
              <small>{{ formatTime(task.created_at) }}</small>
            </span>
            <span class="task-prompt">{{ task.prompt }}</span>
            <small>{{ task.workspace }}</small>
          </article>
          <p v-if="!tasks.length" class="empty-copy">暂无远程开发任务</p>
        </div>
        <section class="voice-history">
          <div class="voice-history-head">
            <div>
              <h4>语音识别记录</h4>
              <p>仅保存识别文本，不保存录音</p>
            </div>
            <small>最近 {{ voiceRecognitions.length }} 条</small>
          </div>
          <div class="voice-history-list">
            <article v-for="item in voiceRecognitions" :key="item.id" class="voice-record">
              <span class="voice-record-top">
                <strong :class="item.outcome">{{ item.outcome_label }}</strong>
                <small>{{ formatTime(item.created_at) }}</small>
              </span>
              <p>{{ item.transcript || '（未识别到有效语音）' }}</p>
              <small>{{ item.asr_engine || '未知引擎' }}<template v-if="item.command"> · 指令：{{ item.command }}</template></small>
            </article>
            <p v-if="!voiceRecognitions.length" class="empty-copy">暂无语音识别记录</p>
          </div>
        </section>
      </aside>

      <main class="dev-main">
        <section class="terminal-shell">
          <header>
            <div class="terminal-lights"><i></i><i></i><i></i></div>
            <strong>Codex CLI 实时输出</strong>
            <span>主会话 {{ conversationThreadId?.slice(0, 8) || '待创建' }}</span>
          </header>
          <div ref="terminal" class="terminal-body" @scroll="updateAutoFollow">
            <div v-if="!turns.length" class="terminal-empty">
              <strong>等待任务</strong>
              <span>提交开发指令后，Codex 的分析、命令和结果会显示在这里。</span>
            </div>
            <section v-for="(turn, turnIndex) in turns" :key="turn.task.id" class="conversation-turn">
              <div class="user-message">
                <span class="message-role">你</span>
                <div>
                  <p>{{ turn.task.prompt }}</p>
                  <small>
                    第 {{ turnIndex + 1 }} 轮 · {{ formatTime(turn.task.created_at) }} · {{ turn.task.workspace }}
                    · {{ turn.task.model || 'gpt-5.6-terra' }}
                  </small>
                </div>
              </div>
              <div v-if="!turn.events.length && !terminalStates.has(turn.task.status)" class="turn-pending">
                Codex 正在接收并处理本轮指令…
              </div>
              <div
                v-for="event in visibleEvents(turn.events)"
                :key="eventKey(turn.task.id, event)"
                class="terminal-line"
                :class="eventClass(event)"
              >
                <span class="line-sequence">{{ String(event.sequence).padStart(4, '0') }}</span>
                <span class="line-time">{{ formatTime(event.occurred_at) }}</span>
                <pre>{{ eventDisplayText(event) }}</pre>
              </div>
            </section>
          </div>
          <footer v-if="selectedTask">
            <span>开始：{{ formatTime(selectedTask.started_at) }}</span>
            <span>结束：{{ formatTime(selectedTask.finished_at) }}</span>
            <span>退出码：{{ selectedTask.exit_code ?? '--' }}</span>
          </footer>
        </section>

        <section ref="composer" class="panel composer">
          <div class="composer-row">
            <label>
              <span>工作区</span>
              <select v-model="workspace">
                <option value="robot-main">机器狗主工程</option>
                <option value="cloud-platform">云端平台工程</option>
              </select>
            </label>
            <label>
              <span>模型</span>
              <select v-model="model">
                <option v-for="item in modelOptions" :key="item.value" :value="item.value">
                  {{ item.label }}
                </option>
              </select>
            </label>
            <label>
              <span>Codex 对话</span>
              <strong class="conversation-name">主会话 · 连续上下文</strong>
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
              rows="4"
              placeholder="输入新指令，继续当前 Codex 主会话"
              @keydown.ctrl.enter.prevent="submitTask"
            ></textarea>
          </label>
          <div class="composer-actions">
            <span v-if="error" class="dev-error">{{ error }}</span>
            <span v-else class="shortcut">Ctrl + Enter 发送</span>
            <button
              v-if="activeTask?.can_cancel"
              class="danger-btn"
              :disabled="cancelling"
              @click="cancelTask"
            >{{ cancelling ? '取消中…' : '停止任务' }}</button>
            <button class="primary-btn" :disabled="!canSubmit" @click="submitTask">
              {{ submitting ? '正在提交…' : activeTask ? '已有任务执行中' : '发送给 Codex' }}
            </button>
          </div>
        </section>
      </main>
    </div>
  </section>
</template>

<style scoped>
.dev-page { display: grid; grid-template-rows: auto minmax(0, 1fr); gap: 18px; height: calc(100vh - 128px); min-height: 700px; }
.dev-toolbar { display: flex; align-items: center; justify-content: space-between; gap: 20px; padding: 22px 26px; }
.dev-toolbar h3 { margin: 4px 0 5px; font-size: 24px; }
.dev-toolbar p { margin: 0; color: var(--muted); }
.dev-kicker { color: var(--cyan); font-size: 11px; font-weight: 800; letter-spacing: .16em; }
.agent-state { display: flex; align-items: center; gap: 8px; padding: 10px 14px; border: 1px solid var(--line); border-radius: 999px; background: var(--panel-soft); }
.agent-state i { width: 9px; height: 9px; border-radius: 50%; background: var(--danger); box-shadow: 0 0 12px var(--danger); }
.agent-state.online i { background: var(--green); box-shadow: 0 0 12px var(--green); }
.agent-state small { color: var(--muted); }
.dev-grid { display: grid; grid-template-columns: minmax(250px, 320px) minmax(0, 1fr); gap: 18px; min-height: 0; }
.dev-history { padding: 18px; min-height: 0; }
.panel-head.compact { margin-bottom: 14px; }
.panel-head.compact h3, .panel-head.compact p { margin: 0; }
.robot-field { display: grid; gap: 6px; margin-bottom: 14px; color: var(--muted); font-size: 12px; }
.robot-field select, .composer select, .prompt-field textarea { width: 100%; border: 1px solid var(--line); border-radius: 12px; color: var(--text); background: var(--input-bg); }
.robot-field select, .composer select { height: 42px; padding: 0 11px; }
.task-list { display: grid; gap: 9px; max-height: 580px; overflow-y: auto; }
.task-card { display: grid; gap: 7px; width: 100%; padding: 12px; border: 1px solid var(--line); border-radius: 14px; color: var(--text); text-align: left; background: var(--panel-soft); }
.task-card.active { border-color: var(--cyan); box-shadow: 0 0 0 2px color-mix(in srgb, var(--cyan) 16%, transparent); }
.task-card-top { display: flex; justify-content: space-between; gap: 8px; }
.task-card small { color: var(--muted); }
.task-prompt { overflow: hidden; color: var(--text); font-size: 13px; line-height: 1.45; text-overflow: ellipsis; white-space: nowrap; }
.empty-copy { color: var(--muted); text-align: center; }
.voice-history { display: grid; gap: 10px; margin-top: 18px; padding-top: 16px; border-top: 1px solid var(--line); }
.voice-history-head, .voice-record-top { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
.voice-history-head h4, .voice-history-head p { margin: 0; }
.voice-history-head h4 { font-size: 14px; }
.voice-history-head p, .voice-history-head > small, .voice-record small { color: var(--muted); font-size: 11px; }
.voice-history-list { display: grid; gap: 8px; max-height: 280px; overflow-y: auto; }
.voice-record { display: grid; gap: 5px; padding: 10px; border: 1px solid var(--line); border-radius: 12px; background: var(--panel-soft); }
.voice-record p { margin: 0; overflow-wrap: anywhere; color: var(--text); font-size: 12px; line-height: 1.45; }
.voice-record-top strong { font-size: 12px; }
.voice-record-top strong.accepted { color: var(--green); }
.voice-record-top strong.armed { color: var(--cyan); }
.voice-record-top strong.busy { color: #ffc857; }
.voice-record-top strong.ignored, .voice-record-top strong.no_speech { color: var(--muted); }
.dev-main { display: grid; grid-template-rows: minmax(420px, 1fr) auto; gap: 18px; min-width: 0; }
.composer { padding: 20px; }
.composer-row { display: grid; grid-template-columns: minmax(145px, 210px) minmax(175px, 240px) minmax(160px, 230px) 1fr; gap: 16px; margin-bottom: 14px; }
.composer label, .prompt-field { display: grid; gap: 7px; color: var(--muted); font-size: 12px; }
.conversation-name { display: flex; align-items: center; min-height: 42px; padding: 0 12px; border: 1px solid var(--line); border-radius: 12px; color: var(--text); background: var(--input-bg); font-size: 13px; }
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
.conversation-turn { padding: 8px 0 14px; border-bottom: 1px solid rgba(146, 197, 255, .1); }
.conversation-turn:last-child { border-bottom: 0; }
.user-message { display: grid; grid-template-columns: 34px minmax(0, 1fr); gap: 10px; margin: 4px 16px 10px; padding: 12px 14px; border: 1px solid rgba(67, 213, 255, .18); border-radius: 12px; background: rgba(67, 213, 255, .07); }
.user-message p { margin: 0 0 4px; color: #ecf7ff; font: 13px/1.6 system-ui, sans-serif; white-space: pre-wrap; }
.user-message small { color: #7897b9; font: 11px/1.4 system-ui, sans-serif; }
.message-role { display: grid; place-items: center; width: 30px; height: 30px; border-radius: 9px; color: #03111d; background: #43d5ff; font: 700 12px/1 system-ui, sans-serif; }
.turn-pending { margin: 8px 18px; color: #7ee8ff; }
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
  .dev-page { height: auto; min-height: 0; }
  .dev-toolbar { align-items: flex-start; flex-direction: column; }
  .dev-grid { grid-template-columns: 1fr; }
  .dev-history { max-height: 300px; }
  .dev-main { grid-template-rows: 560px auto; }
}
@media (max-width: 620px) {
  .composer-row { grid-template-columns: 1fr; }
  .current-state { justify-content: flex-start; }
  .terminal-line { grid-template-columns: 42px minmax(0, 1fr); }
  .line-time { display: none; }
  .terminal-shell footer { justify-content: flex-start; overflow-x: auto; }
}
</style>
