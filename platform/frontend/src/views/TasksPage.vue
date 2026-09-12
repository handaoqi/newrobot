<script setup>
import { computed, onMounted, ref } from 'vue'
import { useRouter } from 'vue-router'

import {
  createPatrolTask,
  deletePatrolTask,
  executePatrolTask,
  fetchPatrolTasks,
  fetchRobots,
  fetchRouteSummaries,
  updatePatrolTask,
} from '../services/api'
import { activateAndRelocalizeMap } from '../services/mapActivationFlow'
import { resolveBatteryPercent } from '../utils/battery'
import { isLowBatteryBlocked, lowBatteryGuardMessage } from '../utils/guardDutyLowBattery'

const router = useRouter()
const tasks = ref([])
const robots = ref([])
const routes = ref([])
const error = ref('')
const savingRecordTaskId = ref(null)
const executingTaskId = ref(null)
const executionProgress = ref('')
const form = ref({ name: '', robot: '', route: '', description: '', enabled: true, record_rosbag: false })

const routeOptions = computed(() => {
  if (!form.value.robot) return routes.value
  return routes.value.filter(route => String(route.robot) === String(form.value.robot))
})

function onRobotChange() {
  const selectedRoute = routes.value.find(route => String(route.id) === String(form.value.route))
  if (selectedRoute && String(selectedRoute.robot) !== String(form.value.robot)) {
    form.value.route = ''
  }
}

async function load() {
  error.value = ''
  const [tasksResult, robotsResult, routesResult] = await Promise.allSettled([
    fetchPatrolTasks(),
    fetchRobots(),
    fetchRouteSummaries(),
  ])
  if (tasksResult.status === 'fulfilled') tasks.value = tasksResult.value
  else {
    tasks.value = []
    error.value = tasksResult.reason?.message || '加载巡检任务失败'
  }
  if (routesResult.status === 'fulfilled') routes.value = routesResult.value
  else {
    routes.value = []
    if (!error.value) error.value = routesResult.reason?.message || '加载路线失败'
  }
  if (robotsResult.status === 'fulfilled') robots.value = robotsResult.value
  else {
    robots.value = []
    console.error('加载机器人失败:', robotsResult.reason)
  }
  seedRobotsFromRoutesAndTasks()
  if (!form.value.robot && robots.value.length) form.value.robot = robots.value[0].id
}

function seedRobotsFromRoutesAndTasks() {
  const known = new Map(robots.value.map(robot => [String(robot.id), robot]))
  for (const route of routes.value) {
    if (!route.robot || known.has(String(route.robot))) continue
    known.set(String(route.robot), {
      id: route.robot,
      name: route.robot_name || route.robot_code || `机器狗 ${route.robot}`,
      code: route.robot_code || String(route.robot),
    })
  }
  for (const task of tasks.value) {
    if (!task.robot || known.has(String(task.robot))) continue
    known.set(String(task.robot), {
      id: task.robot,
      name: task.robot_name || task.robot_code || `机器狗 ${task.robot}`,
      code: task.robot_code || String(task.robot),
    })
  }
  robots.value = Array.from(known.values())
}

async function createTask() {
  error.value = ''
  try {
    await createPatrolTask(form.value)
    form.value = { name: '', robot: '', route: '', description: '', enabled: true, record_rosbag: false }
    await load()
  } catch (exc) {
    error.value = exc.message
  }
}

async function execute(task) {
  error.value = ''
  executingTaskId.value = task.id
  const batteryPercent = resolveBatteryPercent(null, robots.value.find(robot => String(robot.id) === String(task.robot)))
  if (isLowBatteryBlocked(batteryPercent)) {
    error.value = lowBatteryGuardMessage(batteryPercent)
    executingTaskId.value = null
    return
  }
  executionProgress.value = '正在检查机器狗地图'
  try {
    await activateAndRelocalizeMap({
      mapId: task.map_id,
      robotId: task.robot,
      waypoints: task.route_snapshot?.waypoints || task.waypoints || [],
      onProgress: message => { executionProgress.value = message },
    })
    executionProgress.value = '地图与定位已就绪，正在下发巡检任务'
    const execution = await executePatrolTask(task.id)
    router.push(`/dashboard/task-executions/${execution.id}`)
  } catch (exc) {
    error.value = exc.message
  } finally {
    executingTaskId.value = null
    executionProgress.value = ''
  }
}

async function setTaskRecording(task, event) {
  const enabled = event.target.checked
  const previous = Boolean(task.record_rosbag)
  savingRecordTaskId.value = task.id
  error.value = ''
  task.record_rosbag = enabled
  try {
    const updated = await updatePatrolTask(task.id, { record_rosbag: enabled })
    Object.assign(task, updated)
  } catch (exc) {
    task.record_rosbag = previous
    event.target.checked = previous
    error.value = exc.message
  } finally {
    savingRecordTaskId.value = null
  }
}

async function removeTask(task) {
  if (!confirm(`确定要删除巡检任务模板 "${task.name}" 吗？`)) return
  error.value = ''
  try {
    await deletePatrolTask(task.id)
    await load()
  } catch (exc) {
    const refs = formatReferences(exc.payload?.references)
    if (confirm(`${exc.message}${refs ? `\n\n关联数据：${refs}` : ''}\n\n是否强制删除该任务及全部关联数据？`)) {
      await forceRemoveTask(task)
    } else {
      error.value = exc.message
    }
  }
}

async function forceRemoveTask(task) {
  if (!confirm(`强制删除会同时删除巡检任务 "${task.name}" 的日历计划、执行记录、轨迹和告警事件，且不可恢复。确定继续吗？`)) return
  error.value = ''
  try {
    await deletePatrolTask(task.id, { force: true })
    await load()
  } catch (exc) {
    error.value = exc.message
  }
}

function formatReferences(references = {}) {
  return Object.entries(references)
    .filter(([, count]) => Number(count) > 0)
    .map(([name, count]) => `${name} ${count} 个`)
    .join('，')
}

onMounted(load)
</script>

<template>
  <section class="page-section">
    <section class="panel detail-panel p0-form">
      <div class="panel-head">
        <div>
          <h3>巡检任务模板</h3>
          <p class="muted-note">任务模板只定义“哪条机器狗走哪条路线”；自动执行时间请到巡检日历编排。</p>
        </div>
        <span class="panel-badge">狗 + 地图 + 路线</span>
      </div>
      <div class="form-grid">
        <input v-model="form.name" placeholder="任务名称" />
        <select v-model="form.robot" @change="onRobotChange">
          <option value="">选择机器人</option>
          <option v-for="robot in robots" :key="robot.id" :value="robot.id">{{ robot.name }} / {{ robot.code }}</option>
        </select>
        <select v-model="form.route">
          <option value="">选择路线</option>
          <option v-for="route in routeOptions" :key="route.id" :value="route.id">{{ route.name }} / {{ route.map_name }}</option>
        </select>
        <input v-model="form.description" placeholder="任务说明" />
        <label class="diagnostic-record-toggle form-record-toggle">
          <input v-model="form.record_rosbag" type="checkbox" />
          <span>
            <strong>录制导航诊断包</strong>
            <small>路线已开启时优先生效；否则使用任务开关。循环执行保存为一个连续包。</small>
          </span>
        </label>
      </div>
      <p v-if="error" class="form-error">{{ error }}</p>
      <button class="primary-btn" :disabled="!form.name || !form.robot || !form.route" @click="createTask">保存模板</button>
    </section>

    <section class="panel detail-panel">
      <p v-if="executionProgress" class="muted-note execution-progress">{{ executionProgress }}</p>
      <div class="task-list">
        <article v-for="task in tasks" :key="task.id" class="task-card">
          <div>
            <strong>{{ task.name }}</strong>
            <span>{{ task.robot_name }} / {{ task.route_name_display || task.route_name }}</span>
            <small>任务模板 · {{ task.enabled ? '可用于日历自动调度' : '已停用，不参与自动调度' }}</small>
          </div>
          <div class="table-side action-row">
            <label class="task-record-toggle">
              <input
                :checked="task.record_rosbag"
                :disabled="savingRecordTaskId === task.id || task.route_record_rosbag"
                type="checkbox"
                @change="setTaskRecording(task, $event)"
              />
              <span>{{ task.route_record_rosbag ? '路线已开启录包' : savingRecordTaskId === task.id ? '保存中…' : '录制导航包' }}</span>
            </label>
            <span class="panel-badge">{{ task.latest_execution?.state || '未执行' }}</span>
            <button v-if="task.latest_execution" class="ghost-btn" @click="router.push(`/dashboard/task-executions/${task.latest_execution.id}`)">详情</button>
            <button class="primary-btn" :disabled="!task.enabled || executingTaskId !== null" @click="execute(task)">
              {{ executingTaskId === task.id ? '准备执行中...' : '立即执行' }}
            </button>
            <button class="ghost-btn danger-btn" @click="removeTask(task)">删除</button>
            <button class="ghost-btn danger-btn" @click="forceRemoveTask(task)">强制删除</button>
          </div>
        </article>
      </div>
    </section>
  </section>
</template>

<style scoped>
.diagnostic-record-toggle {
  display: flex;
  align-items: center;
  gap: 10px;
  padding: 10px 12px;
  border: 1px solid var(--line);
  border-radius: 8px;
  background: var(--table-bg);
  cursor: pointer;
  user-select: none;
}

.form-record-toggle {
  grid-column: 1 / -1;
}

.diagnostic-record-toggle input {
  width: 18px;
  height: 18px;
  flex: 0 0 auto;
}

.diagnostic-record-toggle span {
  display: grid;
  gap: 2px;
}

.diagnostic-record-toggle small {
  color: var(--muted);
}

.task-record-toggle {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  color: var(--muted);
  cursor: pointer;
  white-space: nowrap;
}

.task-record-toggle input {
  width: 16px;
  height: 16px;
}

.execution-progress {
  margin: -4px 0 14px;
  color: var(--cyan);
  font-weight: 700;
}
</style>
