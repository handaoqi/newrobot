<script setup>
import { computed, onMounted, ref } from 'vue'
import { useRouter } from 'vue-router'

import {
  createCalendarDay,
  createPatrolSchedule,
  deleteCalendarDay,
  deletePatrolSchedule,
  fetchCalendarDays,
  fetchPatrolCalendar,
  fetchPatrolSchedules,
  fetchPatrolTasks,
  fetchRobots,
  fetchRoutes,
  fetchScheduleRuns,
  runPatrolScheduleNow,
  setPatrolScheduleEnabled,
  updateCalendarDay,
  updatePatrolSchedule,
} from '../services/api'

const router = useRouter()

const loading = ref(false)
const error = ref('')
const toast = ref('')
const robots = ref([])
const tasks = ref([])
const routes = ref([])
const schedules = ref([])
const calendarDays = ref([])
const scheduleRuns = ref([])
const calendar = ref({ days: [] })
const selectedRobot = ref('')
const selectedStatus = ref('')
const weekdayOptions = [
  { value: 1, label: '周一' },
  { value: 2, label: '周二' },
  { value: 3, label: '周三' },
  { value: 4, label: '周四' },
  { value: 5, label: '周五' },
  { value: 6, label: '周六' },
  { value: 7, label: '周日' },
]

const showScheduleDialog = ref(false)
const editingSchedule = ref(null)
const scheduleForm = ref(blankSchedule())

const showDayDialog = ref(false)
const editingDay = ref(null)
const dayForm = ref(blankDay())

function todayString() {
  return toDateInput(new Date())
}

function toDateInput(date) {
  const year = date.getFullYear()
  const month = String(date.getMonth() + 1).padStart(2, '0')
  const day = String(date.getDate()).padStart(2, '0')
  return `${year}-${month}-${day}`
}

function startOfWeek(date) {
  const result = new Date(date)
  const day = result.getDay() || 7
  result.setDate(result.getDate() - day + 1)
  return result
}

function addDays(date, days) {
  const result = new Date(date)
  result.setDate(result.getDate() + days)
  return result
}

const weekStart = ref(toDateInput(startOfWeek(new Date())))
const weekEnd = computed(() => toDateInput(addDays(new Date(`${weekStart.value}T00:00:00`), 6)))
const todayItems = computed(() => calendar.value.days.find(day => day.date === todayString())?.items || [])
const filteredRuns = computed(() => scheduleRuns.value.filter(run => !selectedStatus.value || run.status === selectedStatus.value))

const routeOptions = computed(() => {
  if (!scheduleForm.value.robot) return routes.value
  return routes.value.filter(route => String(route.robot) === String(scheduleForm.value.robot))
})

const taskOptions = computed(() => {
  if (!scheduleForm.value.robot) return tasks.value
  return tasks.value.filter(task => String(task.robot) === String(scheduleForm.value.robot))
})

function blankSchedule() {
  return {
    name: '',
    robot: '',
    task_template: '',
    route: '',
    map_data: '',
    schedule_type: 'daily',
    time_of_day: '09:00',
    weekdays: [],
    run_date: '',
    priority: 10,
    enabled: true,
    note: '',
  }
}

function blankDay() {
  return {
    date: todayString(),
    name: '',
    day_type: 'holiday',
    enabled: true,
    note: '',
  }
}

function setToast(message) {
  toast.value = message
  window.setTimeout(() => {
    if (toast.value === message) toast.value = ''
  }, 2400)
}

function statusLabel(status) {
  const map = {
    pending: '待执行',
    created: '已创建',
    dispatched: '已下发',
    skipped: '已跳过',
    failed: '失败',
    completed: '已完成',
    cancelled: '已取消',
  }
  return map[status] || status || '待执行'
}

function reasonLabel(reason) {
  const map = {
    ROBOT_OFFLINE: '机器人离线',
    ROBOT_BUSY: '机器人正在执行其他任务',
    INVALID_CONFIG: '配置错误',
    TASK_DISABLED: '任务模板已停用',
    holiday_override: '节假日计划覆盖',
    lower_priority: '同时间低优先级跳过',
    HOLIDAY_OVERRIDE: '节假日计划覆盖',
    LOWER_PRIORITY: '同时间低优先级跳过',
  }
  return map[reason] || reason || ''
}

function scheduleTypeLabel(type) {
  return { daily: '日常', holiday: '节假日', once: '一次性' }[type] || type
}

function dayTypeLabel(type) {
  return { normal: '普通日', holiday: '节假日', event_day: '活动日', workday: '调休日' }[type] || type
}

function formatDate(date) {
  const parsed = new Date(date)
  return `${parsed.getMonth() + 1}/${parsed.getDate()}`
}

function formatDateTime(value) {
  if (!value) return '-'
  const date = new Date(value)
  return `${date.getMonth() + 1}/${date.getDate()} ${String(date.getHours()).padStart(2, '0')}:${String(date.getMinutes()).padStart(2, '0')}`
}

async function loadAll() {
  loading.value = true
  error.value = ''
  try {
    const params = { from: weekStart.value, to: weekEnd.value, robot: selectedRobot.value }
    ;[robots.value, tasks.value, routes.value, schedules.value, calendarDays.value, calendar.value, scheduleRuns.value] = await Promise.all([
      fetchRobots(),
      fetchPatrolTasks(),
      fetchRoutes(),
      fetchPatrolSchedules({ robot: selectedRobot.value }),
      fetchCalendarDays(),
      fetchPatrolCalendar(params),
      fetchScheduleRuns(params),
    ])
  } catch (exc) {
    error.value = exc.message
  } finally {
    loading.value = false
  }
}

function openCreateSchedule() {
  editingSchedule.value = null
  scheduleForm.value = blankSchedule()
  if (selectedRobot.value) scheduleForm.value.robot = selectedRobot.value
  showScheduleDialog.value = true
}

function openEditSchedule(schedule) {
  editingSchedule.value = schedule
  scheduleForm.value = {
    name: schedule.name,
    robot: schedule.robot,
    task_template: schedule.task_template,
    route: schedule.route,
    map_data: schedule.map_data,
    schedule_type: schedule.schedule_type,
    time_of_day: String(schedule.time_of_day || '09:00').slice(0, 5),
    weekdays: schedule.weekdays || [],
    run_date: schedule.run_date || '',
    priority: schedule.priority,
    enabled: schedule.enabled,
    note: schedule.note || '',
  }
  showScheduleDialog.value = true
}

function onRouteChange() {
  const route = routes.value.find(item => String(item.id) === String(scheduleForm.value.route))
  if (route) {
    scheduleForm.value.robot = route.robot
    scheduleForm.value.map_data = route.map_data
  }
}

function onTaskChange() {
  const task = tasks.value.find(item => String(item.id) === String(scheduleForm.value.task_template))
  if (!task) return
  scheduleForm.value.robot = task.robot
  if (task.route) scheduleForm.value.route = task.route
  const route = routes.value.find(item => String(item.id) === String(task.route))
  if (route) scheduleForm.value.map_data = route.map_data
}

function toggleWeekday(day) {
  const current = new Set(scheduleForm.value.weekdays || [])
  if (current.has(day)) current.delete(day)
  else current.add(day)
  scheduleForm.value.weekdays = [...current].sort((a, b) => a - b)
}

async function saveSchedule() {
  error.value = ''
  const payload = { ...scheduleForm.value }
  if (payload.schedule_type !== 'once') payload.run_date = null
  try {
    if (editingSchedule.value) {
      await updatePatrolSchedule(editingSchedule.value.id, payload)
      setToast('计划已更新')
    } else {
      await createPatrolSchedule(payload)
      setToast('计划已创建')
    }
    showScheduleDialog.value = false
    await loadAll()
  } catch (exc) {
    error.value = exc.message
  }
}

async function toggleSchedule(schedule) {
  await setPatrolScheduleEnabled(schedule.id, !schedule.enabled)
  setToast(schedule.enabled ? '计划已停用' : '计划已启用')
  await loadAll()
}

async function removeSchedule(schedule) {
  if (!confirm(`确定删除计划“${schedule.name}”吗？`)) return
  await deletePatrolSchedule(schedule.id)
  setToast('计划已删除')
  await loadAll()
}

async function runNow(schedule) {
  if (!schedule) {
    error.value = '未找到可执行的巡检计划'
    return
  }
  error.value = ''
  try {
    const run = await runPatrolScheduleNow(schedule.id)
    setToast('已立即触发巡检计划')
    if (run.task_execution) {
      router.push(`/dashboard/task-executions/${run.task_execution}`)
      return
    }
    await loadAll()
  } catch (exc) {
    error.value = exc.message
  }
}

function openCreateDay() {
  editingDay.value = null
  dayForm.value = blankDay()
  showDayDialog.value = true
}

function openEditDay(day) {
  editingDay.value = day
  dayForm.value = {
    date: day.date,
    name: day.name,
    day_type: day.day_type,
    enabled: day.enabled,
    note: day.note || '',
  }
  showDayDialog.value = true
}

async function saveDay() {
  error.value = ''
  try {
    if (editingDay.value) {
      await updateCalendarDay(editingDay.value.id, dayForm.value)
      setToast('特殊日期已更新')
    } else {
      await createCalendarDay(dayForm.value)
      setToast('特殊日期已创建')
    }
    showDayDialog.value = false
    await loadAll()
  } catch (exc) {
    error.value = exc.message
  }
}

async function removeDay(day) {
  if (!confirm(`确定删除特殊日期“${day.name}”吗？`)) return
  await deleteCalendarDay(day.id)
  setToast('特殊日期已删除')
  await loadAll()
}

function previousWeek() {
  weekStart.value = toDateInput(addDays(new Date(`${weekStart.value}T00:00:00`), -7))
  loadAll()
}

function nextWeek() {
  weekStart.value = toDateInput(addDays(new Date(`${weekStart.value}T00:00:00`), 7))
  loadAll()
}

onMounted(loadAll)
</script>

<template>
  <section class="page-section patrol-calendar-page">
    <div v-if="toast" class="calendar-toast">{{ toast }}</div>

    <section class="panel calendar-hero">
      <div>
        <p class="header-kicker">Patrol Calendar</p>
        <h3>巡检日历编排</h3>
        <p>为任务模板设置自动执行时间。计划启用后，到点由平台自动生成执行记录并下发机器狗，无需人工点击。</p>
      </div>
      <div class="calendar-toolbar">
        <select v-model="selectedRobot" @change="loadAll">
          <option value="">全部机器狗</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">{{ robot.name }} / {{ robot.code }}</option>
        </select>
        <button class="ghost-btn" @click="previousWeek">上一周</button>
        <strong>{{ weekStart }} 至 {{ weekEnd }}</strong>
        <button class="ghost-btn" @click="nextWeek">下一周</button>
        <button class="primary-btn" @click="openCreateSchedule">新建计划</button>
      </div>
    </section>

    <p v-if="error" class="form-error">{{ error }}</p>
    <p v-if="loading" class="loading">加载中...</p>

    <section class="panel detail-panel">
      <div class="panel-head">
        <h3>今日巡检</h3>
        <span class="panel-badge">{{ todayItems.length }} 项</span>
      </div>
      <div v-if="todayItems.length" class="calendar-card-grid">
        <article v-for="item in todayItems" :key="`${item.schedule_id}-${item.planned_start_at}`" class="calendar-task-card" :class="`state-${item.status}`">
          <div class="calendar-task-time">{{ item.time }}</div>
          <div>
            <strong>{{ item.task_name }}</strong>
            <span>{{ item.robot_name }} / {{ item.route_name }}</span>
            <small>{{ scheduleTypeLabel(item.schedule_type) }}计划 · {{ statusLabel(item.status) }}</small>
            <small v-if="item.skip_reason || item.error_message">{{ reasonLabel(item.skip_reason) }} {{ item.error_message }}</small>
          </div>
          <div class="calendar-task-actions">
            <button v-if="item.task_execution_id" class="ghost-btn" @click="router.push(`/dashboard/task-executions/${item.task_execution_id}`)">详情</button>
            <button class="primary-btn" @click="runNow(schedules.find(schedule => schedule.id === item.schedule_id))">立即执行</button>
          </div>
        </article>
      </div>
      <div v-else class="empty-state">
        今日暂无巡检计划
        <button class="primary-btn" @click="openCreateSchedule">新建计划</button>
      </div>
    </section>

    <section class="panel detail-panel">
      <div class="panel-head">
        <h3>本周计划</h3>
        <button class="ghost-btn" @click="loadAll">刷新</button>
      </div>
      <div class="calendar-week-board">
        <article v-for="day in calendar.days" :key="day.date" class="calendar-week-column" :class="{ today: day.date === todayString() }">
          <div class="calendar-week-date">
            <strong>{{ formatDate(day.date) }}</strong>
            <span :class="`day-tag ${day.day_type}`">{{ dayTypeLabel(day.day_type) }}</span>
          </div>
          <div v-if="day.items.length" class="calendar-week-items">
            <button
              v-for="item in day.items"
              :key="`${item.schedule_id}-${item.planned_start_at}`"
              class="week-item"
              :class="`state-${item.status}`"
              @click="item.task_execution_id ? router.push(`/dashboard/task-executions/${item.task_execution_id}`) : null"
            >
              <span>{{ item.time }}</span>
              <strong>{{ item.task_name }}</strong>
              <small>{{ statusLabel(item.status) }}<template v-if="item.skip_reason"> · {{ reasonLabel(item.skip_reason) }}</template></small>
            </button>
          </div>
          <span v-else class="muted-note">无计划</span>
        </article>
      </div>
    </section>

    <section class="calendar-two-col">
      <section class="panel detail-panel">
        <div class="panel-head">
          <h3>计划管理</h3>
          <button class="primary-btn" @click="openCreateSchedule">新建计划</button>
        </div>
        <div class="calendar-list">
          <article v-for="schedule in schedules" :key="schedule.id" class="calendar-list-card" :class="{ disabled: !schedule.enabled }">
            <div>
              <strong>{{ schedule.name }}</strong>
              <span>{{ scheduleTypeLabel(schedule.schedule_type) }} · {{ schedule.time_of_day?.slice(0, 5) }} · {{ schedule.robot_name }}</span>
              <small>{{ schedule.task_name }} / {{ schedule.route_name }}</small>
              <small v-if="schedule.weekdays?.length">执行日：{{ schedule.weekdays.map(day => weekdayOptions.find(item => item.value === day)?.label || day).join('、') }}</small>
            </div>
            <div class="action-row">
              <span class="panel-badge">{{ schedule.enabled ? '启用' : '停用' }}</span>
              <button class="ghost-btn" @click="runNow(schedule)">立即执行</button>
              <button class="ghost-btn" @click="openEditSchedule(schedule)">编辑</button>
              <button class="ghost-btn" @click="toggleSchedule(schedule)">{{ schedule.enabled ? '停用' : '启用' }}</button>
              <button class="ghost-btn danger-text" @click="removeSchedule(schedule)">删除</button>
            </div>
          </article>
          <div v-if="!schedules.length" class="empty-state">暂无巡检计划</div>
        </div>
      </section>

      <section class="panel detail-panel">
        <div class="panel-head">
          <h3>特殊日期</h3>
          <button class="primary-btn" @click="openCreateDay">新增日期</button>
        </div>
        <div class="calendar-list compact">
          <article v-for="day in calendarDays" :key="day.id" class="calendar-list-card">
            <div>
              <strong>{{ day.date }} {{ day.name }}</strong>
              <span>{{ day.day_type_label }} · {{ day.enabled ? '启用' : '停用' }}</span>
            </div>
            <div class="action-row">
              <button class="ghost-btn" @click="openEditDay(day)">编辑</button>
              <button class="ghost-btn danger-text" @click="removeDay(day)">删除</button>
            </div>
          </article>
          <div v-if="!calendarDays.length" class="empty-state">暂无特殊日期</div>
        </div>
      </section>
    </section>

    <section class="panel detail-panel">
      <div class="panel-head">
        <h3>调度记录</h3>
        <select v-model="selectedStatus">
          <option value="">全部状态</option>
          <option value="dispatched">已下发</option>
          <option value="skipped">已跳过</option>
          <option value="failed">失败</option>
        </select>
      </div>
      <div class="table-list">
        <article v-for="run in filteredRuns" :key="run.id" class="table-card">
          <div>
            <strong>{{ run.schedule_name }}</strong>
            <span>{{ formatDateTime(run.planned_start_at) }} / {{ run.robot_name }}</span>
            <small v-if="run.skip_reason || run.error_message">{{ reasonLabel(run.skip_reason) }} {{ run.error_message }}</small>
          </div>
          <div class="action-row">
            <span class="panel-badge">{{ statusLabel(run.status) }}</span>
            <button v-if="run.task_execution" class="ghost-btn" @click="router.push(`/dashboard/task-executions/${run.task_execution}`)">详情</button>
          </div>
        </article>
        <div v-if="!filteredRuns.length" class="empty-state">暂无调度记录</div>
      </div>
    </section>

    <div v-if="showScheduleDialog" class="modal-overlay" @click.self="showScheduleDialog = false">
      <div class="modal calendar-modal">
        <div class="modal-header">
          <h3>{{ editingSchedule ? '编辑巡检计划' : '新建巡检计划' }}</h3>
          <button class="btn-close" @click="showScheduleDialog = false">×</button>
        </div>
        <div class="modal-body calendar-form">
          <label>计划名称<input v-model="scheduleForm.name" placeholder="每日早间日常巡检" /></label>
          <label>机器狗
            <select v-model="scheduleForm.robot">
              <option value="">选择机器狗</option>
              <option v-for="robot in robots" :key="robot.id" :value="robot.id">{{ robot.name }}</option>
            </select>
          </label>
          <label>任务模板
            <select v-model="scheduleForm.task_template" @change="onTaskChange">
              <option value="">选择任务模板</option>
              <option v-for="task in taskOptions" :key="task.id" :value="task.id" :disabled="!task.enabled">
                {{ task.name }} / {{ task.route_name_display || task.route_name }}{{ task.enabled ? '' : '（已停用）' }}
              </option>
            </select>
          </label>
          <label>巡检路线
            <select v-model="scheduleForm.route" @change="onRouteChange">
              <option value="">选择路线</option>
              <option v-for="route in routeOptions" :key="route.id" :value="route.id">{{ route.name }}</option>
            </select>
          </label>
          <label>计划类型
            <select v-model="scheduleForm.schedule_type">
              <option value="daily">日常计划</option>
              <option value="holiday">节假日计划</option>
              <option value="once">一次性计划</option>
            </select>
          </label>
          <label>执行时间<input v-model="scheduleForm.time_of_day" type="time" /></label>
          <label v-if="scheduleForm.schedule_type !== 'once'" class="wide">执行星期
            <div class="weekday-picker">
              <button
                v-for="day in weekdayOptions"
                :key="day.value"
                type="button"
                class="weekday-chip"
                :class="{ active: scheduleForm.weekdays.includes(day.value) }"
                @click="toggleWeekday(day.value)"
              >
                {{ day.label }}
              </button>
            </div>
            <small class="muted-note">不选择表示每天都执行；选择后只在指定星期执行。</small>
          </label>
          <label v-if="scheduleForm.schedule_type === 'once'">执行日期<input v-model="scheduleForm.run_date" type="date" /></label>
          <label>优先级<input v-model.number="scheduleForm.priority" type="number" /></label>
          <label class="wide">备注<input v-model="scheduleForm.note" placeholder="计划说明" /></label>
          <label class="check-row"><input v-model="scheduleForm.enabled" type="checkbox" /> 启用计划</label>
        </div>
        <div class="modal-actions">
          <button class="ghost-btn" @click="showScheduleDialog = false">取消</button>
          <button class="primary-btn" :disabled="!scheduleForm.name || !scheduleForm.robot || !scheduleForm.task_template || !scheduleForm.route" @click="saveSchedule">保存</button>
        </div>
      </div>
    </div>

    <div v-if="showDayDialog" class="modal-overlay" @click.self="showDayDialog = false">
      <div class="modal calendar-modal small">
        <div class="modal-header">
          <h3>{{ editingDay ? '编辑特殊日期' : '新增特殊日期' }}</h3>
          <button class="btn-close" @click="showDayDialog = false">×</button>
        </div>
        <div class="modal-body calendar-form">
          <label>日期<input v-model="dayForm.date" type="date" /></label>
          <label>名称<input v-model="dayForm.name" placeholder="国庆节" /></label>
          <label>类型
            <select v-model="dayForm.day_type">
              <option value="holiday">节假日</option>
              <option value="event_day">特殊活动日</option>
              <option value="workday">调休日</option>
            </select>
          </label>
          <label>备注<input v-model="dayForm.note" /></label>
          <label class="check-row"><input v-model="dayForm.enabled" type="checkbox" /> 启用</label>
        </div>
        <div class="modal-actions">
          <button class="ghost-btn" @click="showDayDialog = false">取消</button>
          <button class="primary-btn" :disabled="!dayForm.date || !dayForm.name" @click="saveDay">保存</button>
        </div>
      </div>
    </div>
  </section>
</template>
