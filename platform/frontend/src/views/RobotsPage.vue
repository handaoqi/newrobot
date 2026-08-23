<script setup>
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'

import { fetchMaps, fetchRobotCommand, fetchRobotDetail, fetchRobots, fetchRobotSessions, fetchRobotStatus, fetchRoutes, fetchTaskExecution, sendRobotCommand, startRobotChargingDock } from '../services/api'

const robots = ref([])
const selectedRobot = ref(null)
const liveStatus = ref(null)
const sessions = ref([])
const commandBusy = ref('')
const commandMessage = ref('')
const chargeCommandMessage = ref('')
const modeAlignMessage = ref('')
const speaker3588Volume = ref(100)
const speakerNxVolume = ref(50)
const volumeFeedback = ref({ speaker_3588: '', speaker_nx: '' })
const refreshing = ref(false)
const refreshingRobotId = ref(null)
const chargeTogglePending = ref(false)
const modeAlignPending = ref(false)
const dockDialogOpen = ref(false)
const dockRobot = ref(null)
const dockMaps = ref([])
const dockRoutes = ref([])
const dockMapId = ref('')
const dockRouteId = ref('')
const dockSubmitting = ref(false)
const dockMessage = ref('')
const dockExecution = ref(null)
const dockRunning = ref(false)
let chargeRefreshTimer = null
let dockProgressTimer = null
const volumeDebounceTimers = {}
const volumeRequestVersions = { speaker_3588: 0, speaker_nx: 0 }

const status = computed(() => liveStatus.value?.status || null)
const power = computed(() => status.value?.power || null)
const network = computed(() => status.value?.network || null)
const audio = computed(() => status.value?.audio || null)
const powerMode = computed(() => status.value?.power_mode || null)
const navigation = computed(() => status.value?.navigation || null)
const sensors = computed(() => status.value?.sensors || {})
const chargeControlMode = computed(() => (
  powerMode.value?.mode === 'cooling_standby' ? 'stop' : 'start'
))
const motionControlService = computed(() => {
  const services = powerMode.value?.services || {}
  const motion = services.controller_egg_motion_control
  const task = services.controller_egg_dog_task
  if (!motion && !task) return null
  return {
    available: Boolean(motion?.available && task?.available),
    active: Boolean(motion?.active && task?.active),
    expected_active: Boolean(motion?.expected_active && task?.expected_active),
  }
})
const motionControlText = computed(() => {
  if (!motionControlService.value?.available) return '状态未知'
  return motionControlService.value.active ? '已启动' : '已停止'
})
const chargeStageText = computed(() => ({
  idle: '未请求充电',
  waiting_for_dock: '低电量，等待放入充电桩',
  starting_charge: '充电桩已确认，正在停止运控',
  waiting_current: '等待充电电流',
  charging: '正在充电',
  thermal_protection: '温度保护',
  stopping_charge: '正在断开充电并恢复运控',
  error: '充电流程异常',
})[powerMode.value?.charge_stage] || '状态未知')
const activeTask = computed(() => (
  selectedRobot.value?.tasks?.find((task) => (
    ['dispatching', 'accepted', 'running', 'pausing', 'paused', 'resuming'].includes(task.latest_execution?.state)
  )) || null
))
const availableDockRoutes = computed(() => dockRoutes.value.filter((route) => (
  String(route.map_data) === String(dockMapId.value) && (route.waypoints || []).length === 2
)))

const batteryText = computed(() => (
  power.value?.available && power.value?.percent != null ? `${power.value.percent}%` : '未知'
))

const batteryEnergyText = computed(() => {
  if (!power.value?.available) return '剩余电量未知'
  const reported = Number(power.value?.remaining_energy_wh)
  const percent = Number(power.value?.percent)
  const remainingWh = Number.isFinite(reported)
    ? reported
    : (Number.isFinite(percent) && Number.isFinite(Number(power.value?.rated_capacity_wh))
      ? Number(power.value.rated_capacity_wh) * Math.max(0, Math.min(100, percent)) / 100
      : null)
  return Number.isFinite(remainingWh) ? '约 ' + remainingWh.toFixed(1) + ' Wh' : '剩余电量未知'
})

const chargeText = computed(() => {
  if (!power.value?.available) return '状态未知'
  if (power.value.thermal_protection || power.value.charge_state === 'thermal_protection') return '过热保护'
  if (power.value.charging) return '正在充电'
  if (power.value.charge_state === 'waiting') return '等待充电'
  if (power.value.charge_state === 'ready') return '充电桩已连接'
  return '未充电'
})

const batteryCurrentText = computed(() => {
  const current = Number(power.value?.current_a)
  if (!Number.isFinite(current)) {
    return power.value?.thermal_protection ? '充电已暂停 · BMS 未上报电流' : 'BMS 未上报电流'
  }
  if (power.value?.thermal_protection) return `保护中 · ${Math.abs(current).toFixed(2)} A`
  if (current > 0.05) return `充电 ${current.toFixed(2)} A`
  if (current < -0.05) return `放电 ${Math.abs(current).toFixed(2)} A`
  return '电流 0.00 A'
})

const powerModeText = computed(() => {
  if (powerMode.value?.mode === 'cooling_standby') return '冷却待机'
  if (powerMode.value?.mode === 'normal') return '正常工作'
  return '状态未知'
})

const powerModeStateText = computed(() => ({
  entering: '正在进入冷却待机',
  restoring: '正在恢复所有服务',
  ready: '模式切换完成',
  error: '模式切换异常',
})[powerMode.value?.transition_state] || '等待设备上报')

const chargingParametersText = computed(() => {
  const overheat = power.value?.charging_overheat_threshold_c
  const full = power.value?.full_battery_percent
  const low = power.value?.low_battery_start_percent
  return `温度保护 ${overheat != null ? `${overheat} °C` : '未知'} · 目标 ${full != null ? `${full}%` : '未知'} · 自动开始 ${low != null ? `${low}%` : '未知'}`
})

const modeServices = computed(() => Object.entries(powerMode.value?.services || {}).map(([key, service]) => ({
  key,
  ...service,
  currentText: service.active ? '运行' : '停止',
  expectedText: service.expected_active ? '应运行' : '应停止',
})))

const modeServicesMatched = computed(() => (
  modeServices.value.length > 0 && modeServices.value.every((service) => service.matches_mode)
))

const speakerTotalVolume = computed(() => (
  Number(speaker3588Volume.value || 0) + Number(speakerNxVolume.value || 0)
))

const signalText = computed(() => (
  network.value?.available && network.value?.signal_percent != null
    ? `${network.value.signal_percent}%`
    : '未知'
))

const networkTypeText = computed(() => (
  network.value?.available ? (network.value.type || '蜂窝网络') : '未知'
))

const statusSampleTime = computed(() => {
  const value = status.value?.sampled_at
  if (!value) return '暂无实时数据'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return value
  return `更新于 ${date.toLocaleTimeString('zh-CN', { hour12: false })}`
})

async function chooseRobot(robotId) {
  selectedRobot.value = await fetchRobotDetail(robotId)
  ;[liveStatus.value, sessions.value] = await Promise.all([
    fetchRobotStatus(robotId),
    fetchRobotSessions(robotId),
  ])
  const liveAudio = liveStatus.value?.status?.audio
  if (liveAudio?.speaker_3588?.volume_percent != null) {
    speaker3588Volume.value = liveAudio.speaker_3588.volume_percent
  }
  if (liveAudio?.speaker_nx?.volume_percent != null) {
    speakerNxVolume.value = liveAudio.speaker_nx.volume_percent
  }
}

async function refreshRobotCard(robotId) {
  if (!robotId || refreshingRobotId.value) return
  refreshingRobotId.value = robotId
  try {
    const detail = await fetchRobotDetail(robotId)
    const index = robots.value.findIndex((robot) => robot.id === robotId)
    if (index >= 0) robots.value[index] = { ...robots.value[index], ...detail }
    if (selectedRobot.value?.id === robotId) {
      selectedRobot.value = detail
      ;[liveStatus.value, sessions.value] = await Promise.all([
        fetchRobotStatus(robotId),
        fetchRobotSessions(robotId),
      ])
    }
  } finally {
    refreshingRobotId.value = null
  }
}

async function refreshChargeStatus() {
  const robotId = selectedRobot.value?.id
  if (!robotId) return
  try {
    const latest = await fetchRobotStatus(robotId)
    if (selectedRobot.value?.id !== robotId || !latest?.status) return null
    liveStatus.value = {
      ...(liveStatus.value || {}),
      connection_status: latest.connection_status,
      status: {
        ...(liveStatus.value?.status || {}),
        sampled_at: latest.status.sampled_at,
        power: latest.status.power,
        power_mode: latest.status.power_mode,
      },
    }
    return { power: latest.status.power, powerMode: latest.status.power_mode }
  } catch (_error) {
    // Keep the last valid sample; the freshness timestamp exposes stale data.
    return null
  }
}

function formatTime(value) {
  if (!value) return '暂无'
  const date = new Date(value)
  return Number.isNaN(date.getTime()) ? value : date.toLocaleString('zh-CN', { hour12: false })
}

function taskStateLabel(state) {
  return ({
    dispatching: '下发中', accepted: '已接受', running: '执行中', pausing: '暂停中',
    paused: '已暂停', resuming: '恢复中', succeeded: '已完成', failed: '失败',
    cancelled: '已取消', timed_out: '超时',
  })[state] || state || '未执行'
}

async function sendManagementCommand(action, payload = {}) {
  if (!selectedRobot.value?.id || commandBusy.value) return
  commandBusy.value = action
  commandMessage.value = ''
  try {
    await sendRobotCommand(selectedRobot.value.id, { action, payload })
    commandMessage.value = '命令已下发，等待设备状态更新'
    return true
  } catch (error) {
    commandMessage.value = error.message || '命令执行失败'
    return false
  } finally {
    commandBusy.value = ''
  }
}

async function toggleChargeControl() {
  if (chargeTogglePending.value) return
  const action = chargeControlMode.value === 'start' ? 'charge_start' : 'charge_stop'
  const expectedChargeRequested = action === 'charge_start'
  chargeTogglePending.value = true
  chargeCommandMessage.value = ''
  try {
    if (!await sendManagementCommand(action)) {
      chargeCommandMessage.value = commandMessage.value
      commandMessage.value = ''
      return
    }
    commandMessage.value = ''
    chargeCommandMessage.value = action === 'charge_start'
      ? '正在上报充电准备状态并检查蓝牙、极片和正负极'
      : '正在断开充电并恢复正常工作服务'
    let latestState = null
    for (let attempt = 0; attempt < 60; attempt += 1) {
      await new Promise((resolve) => window.setTimeout(resolve, 2000))
      latestState = await refreshChargeStatus()
      const expectedMode = action === 'charge_start' ? 'cooling_standby' : 'normal'
      if (
        latestState?.power
        && Boolean(latestState.power.charging_requested) === expectedChargeRequested
        && latestState.powerMode?.mode === expectedMode
        && latestState.powerMode?.transition_state === 'ready'
      ) break
    }
    if (latestState?.power) {
      chargeCommandMessage.value = latestState.powerMode?.transition_state === 'error'
        ? `模式切换异常：${latestState.powerMode.last_error || '请检查设备日志'}`
        : '充电与工作模式状态已刷新'
    } else {
      chargeCommandMessage.value = '状态刷新失败，请点击机器人卡片刷新按钮重试'
    }
  } finally {
    chargeTogglePending.value = false
  }
}

async function forceModeAlignment() {
  if (modeAlignPending.value || !['cooling_standby', 'normal'].includes(powerMode.value?.mode)) return
  const expectedMode = powerMode.value.mode
  const action = expectedMode === 'cooling_standby' ? 'charge_start' : 'charge_stop'
  modeAlignPending.value = true
  modeAlignMessage.value = ''
  try {
    if (!await sendManagementCommand(action)) {
      modeAlignMessage.value = commandMessage.value
      commandMessage.value = ''
      return
    }
    commandMessage.value = ''
    modeAlignMessage.value = `正在强制对齐${powerModeText.value}模式服务`
    let latestState = null
    let aligned = false
    for (let attempt = 0; attempt < 60; attempt += 1) {
      await new Promise((resolve) => window.setTimeout(resolve, 2000))
      latestState = await refreshChargeStatus()
      const services = Object.values(latestState?.powerMode?.services || {})
      aligned = latestState?.powerMode?.mode === expectedMode
        && latestState.powerMode.transition_state === 'ready'
        && services.length > 0
        && services.every((service) => service.matches_mode)
      if (aligned || latestState?.powerMode?.transition_state === 'error') break
    }
    if (aligned) {
      modeAlignMessage.value = `${powerModeText.value}模式状态对齐完成`
    } else if (latestState?.powerMode?.transition_state === 'error') {
      modeAlignMessage.value = `状态对齐失败：${latestState.powerMode.last_error || '请检查设备日志'}`
    } else {
      modeAlignMessage.value = '状态对齐未完成，服务状态仍存在不一致'
    }
  } finally {
    modeAlignPending.value = false
  }
}

function queueSpeakerVolume(target, volume) {
  volumeRequestVersions[target] += 1
  const version = volumeRequestVersions[target]
  volumeFeedback.value = { ...volumeFeedback.value, [target]: '音量下发中' }
  if (volumeDebounceTimers[target]) window.clearTimeout(volumeDebounceTimers[target])
  volumeDebounceTimers[target] = window.setTimeout(() => {
    setSpeakerVolume(target, Number(volume), version)
  }, 250)
}

async function setSpeakerVolume(target, volume, version) {
  const robotId = selectedRobot.value?.id
  if (!robotId || version !== volumeRequestVersions[target]) return
  try {
    const command = await sendRobotCommand(robotId, {
      action: 'audio_volume',
      payload: { target, volume },
    })
    const terminalStates = new Set(['succeeded', 'failed', 'rejected', 'cancelled', 'timed_out', 'expired'])
    let result = command
    for (let attempt = 0; attempt < 24 && !terminalStates.has(result.status); attempt += 1) {
      await new Promise((resolve) => window.setTimeout(resolve, 400))
      if (version !== volumeRequestVersions[target] || selectedRobot.value?.id !== robotId) return
      result = await fetchRobotCommand(robotId, command.id)
    }
    if (version !== volumeRequestVersions[target]) return
    if (result.status === 'succeeded') {
      volumeFeedback.value = { ...volumeFeedback.value, [target]: '音量下发成功' }
      return
    }
    throw new Error(result.error_message || result.ack_reason_message || '设备未确认音量设置')
  } catch (error) {
    if (version !== volumeRequestVersions[target]) return
    volumeFeedback.value = {
      ...volumeFeedback.value,
      [target]: `下发失败：${error.message || '未知错误'}`,
    }
  }
}

async function refreshRobots() {
  if (refreshing.value) return
  refreshing.value = true
  try {
    robots.value = await fetchRobots()
    const selectedRobotId = selectedRobot.value?.id || robots.value[0]?.id
    if (selectedRobotId) {
      await chooseRobot(selectedRobotId)
    }
  } finally {
    refreshing.value = false
  }
}

async function openDockDialog(robot) {
  dockRobot.value = robot
  dockMessage.value = ''
  dockExecution.value = null
  dockRunning.value = false
  dockDialogOpen.value = true
  try {
    const [maps, routes] = await Promise.all([fetchMaps(), fetchRoutes()])
    dockRoutes.value = routes.filter((route) => Number(route.robot) === Number(robot.id))
    const routeMapIds = new Set(dockRoutes.value.map((route) => Number(route.map_data)))
    dockMaps.value = maps.filter((map) => routeMapIds.has(Number(map.id)))
    dockMapId.value = String(robot.charging_config?.map_id || dockMaps.value[0]?.id || '')
    const preferredRoute = robot.charging_config?.route_id
    dockRouteId.value = availableDockRoutes.value.some((route) => Number(route.id) === Number(preferredRoute))
      ? String(preferredRoute)
      : String(availableDockRoutes.value[0]?.id || '')
  } catch (error) {
    dockMessage.value = error.message || '加载回充配置失败'
  }
}

function onDockMapChanged() {
  dockRouteId.value = String(availableDockRoutes.value[0]?.id || '')
}

async function confirmDock() {
  if (!dockRobot.value || !dockMapId.value || !dockRouteId.value || dockSubmitting.value) return
  dockSubmitting.value = true
  dockMessage.value = ''
  try {
    const result = await startRobotChargingDock(dockRobot.value.id, {
      map_id: Number(dockMapId.value),
      route_id: Number(dockRouteId.value),
    })
    dockExecution.value = result.execution
    dockRunning.value = true
    dockMessage.value = '回充任务已下发'
    await refreshRobotCard(dockRobot.value.id)
    await refreshDockProgress()
    dockProgressTimer = window.setInterval(refreshDockProgress, 2000)
  } catch (error) {
    dockMessage.value = error.message || '一键回充启动失败'
  } finally {
    dockSubmitting.value = false
  }
}

async function refreshDockProgress() {
  if (!dockRobot.value?.id || !dockExecution.value?.id) return
  try {
    dockExecution.value = await fetchTaskExecution(dockExecution.value.id)
    await refreshRobotCard(dockRobot.value.id)
    const state = dockExecution.value.state
    const stage = liveStatus.value?.status?.power_mode?.charge_stage
    if (['failed', 'cancelled', 'timed_out', 'completed'].includes(state) || ['charging', 'thermal_protection', 'error'].includes(stage)) {
      dockRunning.value = false
      window.clearInterval(dockProgressTimer)
    }
  } catch (error) { dockMessage.value = error.message || '回充状态刷新失败' }
}

function dockStepState(index) {
  const execution = dockExecution.value || {}
  const events = execution.events || []
  const types = new Set(events.map((item) => item.event_type))
  const failed = ['failed', 'cancelled', 'timed_out'].includes(execution.state) || types.has('task.docking_charge_failed')
  const waypoint = Number(execution.current_waypoint_index ?? -1)
  const stage = liveStatus.value?.status?.power_mode?.charge_stage
  if (failed && index >= 5) return 'failed'
  if (index === 0) return execution.id ? 'done' : 'waiting'
  if (index === 1) return types.has('task.started') || ['accepted', 'running', 'completed'].includes(execution.state) ? 'done' : 'waiting'
  if (index === 2) return waypoint >= 1 || execution.completed_waypoints >= 1 ? 'done' : execution.state === 'running' ? 'active' : 'waiting'
  if (index === 3) return waypoint >= 1 || execution.completed_waypoints >= 1 ? 'active' : 'waiting'
  if (index === 4) return types.has('task.docking_final_approach') ? 'done' : 'waiting'
  if (index === 5) return types.has('task.docking_contact_checking') ? 'active' : 'waiting'
  return types.has('task.docking_charge_started') || stage === 'charging' ? 'done' : stage === 'thermal_protection' || stage === 'error' ? 'failed' : 'waiting'
}

const dockSteps = [
  '回充任务已创建并下发', 'Edge Agent 已接收，导航准备完成', '正在前往第 1 点',
  '已到第 1 点，正在前往充电桩', '已进入末段微速直连', '正在检查蓝牙、极片、正负极', '已开始充电',
]

onMounted(async () => {
  await refreshRobots()
  chargeRefreshTimer = window.setInterval(refreshChargeStatus, 3000)
})

onBeforeUnmount(() => window.clearInterval(dockProgressTimer))

onBeforeUnmount(() => {
  if (chargeRefreshTimer) window.clearInterval(chargeRefreshTimer)
  Object.values(volumeDebounceTimers).forEach((timer) => window.clearTimeout(timer))
})
</script>

<template>
  <section class="page-section">
    <div class="data-grid robot-management-grid">
      <section class="panel list-panel robot-management-list-panel">
        <div class="panel-head robot-list-head">
          <div>
            <h3>机器人</h3>
            <p>选择设备查看实时状态</p>
          </div>
        </div>
        <article
          v-for="robot in robots"
          :key="robot.id"
          class="table-card robot-table-card"
          :class="{ selected: selectedRobot?.id === robot.id }"
          @click="chooseRobot(robot.id)"
        >
          <div class="robot-card-main">
            <div class="robot-card-identity">
              <strong>{{ robot.name }}</strong>
              <span>{{ robot.code }}</span>
              <small>{{ robot.location }}</small>
            </div>
            <div class="robot-card-dock-config">
              <button
                type="button"
                class="primary-btn compact-command robot-dock-button"
                :disabled="robot.connection_status !== 'online'"
                @click.stop="openDockDialog(robot)"
              >一键回充</button>
              <span :title="robot.charging_config?.map_name || '未配置'">{{ robot.charging_config?.map_name || '未配置' }}</span>
              <span :title="robot.charging_config?.route_name || '未配置'">{{ robot.charging_config?.route_name || '未配置' }}</span>
            </div>
          </div>
          <div class="table-side robot-card-side">
            <span :class="['robot-status', robot.status]">{{ robot.status_label }}</span>
            <small>{{ selectedRobot?.id === robot.id ? batteryText : `${robot.battery_level}%` }}</small>
            <button
              type="button"
              class="robot-card-refresh"
              :class="{ active: refreshingRobotId === robot.id }"
              :disabled="Boolean(refreshingRobotId)"
              title="刷新该机器人"
              aria-label="刷新该机器人"
              @click.stop="refreshRobotCard(robot.id)"
            >↻</button>
          </div>
        </article>
      </section>

      <section class="panel detail-panel" v-if="selectedRobot">
        <div class="panel-head">
          <div>
            <h3>{{ selectedRobot.name }}</h3>
            <p>{{ selectedRobot.code }} / {{ selectedRobot.area }}</p>
          </div>
          <span class="panel-badge">{{ selectedRobot.mode_label }}</span>
        </div>

        <div class="metrics-grid">
          <div class="metric-card battery-metric-card" :class="{ 'metric-card-active': power?.charging }">
            <div class="battery-metric-copy">
              <strong>{{ batteryText }}</strong>
              <span>电池电量</span>
              <small v-if="power?.available">
                {{ batteryEnergyText }} · {{ power.voltage_v != null ? power.voltage_v + ' V' : '电压未知' }}
              </small>
            </div>
            <div class="battery-charge-actions">
              <button
                type="button"
                :class="[chargeControlMode === 'start' ? 'primary-btn' : 'ghost-btn', 'compact-command', { 'danger-command': chargeControlMode === 'stop' }]"
                :disabled="Boolean(commandBusy) || chargeTogglePending || !power?.available"
                @click="toggleChargeControl"
              >{{ chargeTogglePending ? '状态刷新中' : chargeControlMode === 'start' ? '开始充电' : '断开充电' }}</button>
              <small v-if="chargeCommandMessage" class="charge-command-feedback">
                {{ chargeCommandMessage }}
              </small>
            </div>
          </div>
          <div class="metric-card" :class="{ 'metric-card-warning': power?.thermal_protection }">
            <strong>{{ chargeText }}</strong>
            <span>充电状态</span>
            <small v-if="power?.available">
              {{ batteryCurrentText }} · 电池 {{ power.temperature_c != null ? `${power.temperature_c} °C` : '温度未知' }}
            </small>
            <small>{{ chargingParametersText }}</small>
          </div>
          <div class="metric-card">
            <strong>{{ networkTypeText }}</strong>
            <span>网络制式</span>
            <small>{{ network?.modem || '通信模组未知' }}</small>
          </div>
          <div class="metric-card">
            <strong>{{ signalText }}</strong>
            <span>网络信号</span>
            <small>{{ network?.csq != null ? `CSQ ${network.csq}/31` : '信号数据未知' }}</small>
          </div>
          <div class="metric-card">
            <strong>{{ speakerTotalVolume }}%</strong>
            <span>扬声器总音量</span>
            <small>3588 {{ speaker3588Volume }}% + NX {{ speakerNxVolume }}%</small>
          </div>
          <div class="metric-card">
            <strong>{{ selectedRobot.firmware_version }}</strong>
            <span>固件版本</span>
          </div>
          <div class="metric-card" :class="{ 'metric-card-active': powerMode?.mode === 'cooling_standby', 'metric-card-warning': powerMode?.transition_state === 'error' }">
            <strong>{{ powerModeText }}</strong>
            <span>工作模式</span>
            <small>{{ powerModeStateText }}</small>
          </div>
        </div>

        <p class="telemetry-freshness" :class="{ stale: !power?.available || !network?.available }">
          {{ statusSampleTime }} · 数据源：机器狗实时遥测
        </p>

        <div class="detail-card management-control-card">
          <div class="detail-card-head">
            <strong>充电与充电桩</strong>
            <span :class="['state-chip', { ok: power?.charging, warning: power?.thermal_protection }]">{{ chargeText }}</span>
          </div>
          <div class="status-facts">
            <span>蓝牙 <b>{{ power?.bluetooth_connected ? '已连接' : '未连接' }}</b></span>
            <span>充电极片 <b>{{ power?.charge_pin == null ? '未知' : power.charge_pin === 1 ? '已接触' : '未接触' }}</b></span>
            <span>负极 <b>{{ power?.negative_contact == null ? '未知' : power.negative_contact === 1 ? '正常' : '异常' }}</b></span>
            <span>正极 <b>{{ power?.positive_contact == null ? '未知' : power.positive_contact === 1 ? '正常' : '异常' }}</b></span>
            <span>电池温度 <b>{{ power?.temperature_c != null ? `${power.temperature_c} °C` : '未知' }}</b></span>
            <span>温度保护 <b>{{ power?.thermal_protection ? `已触发（阈值 ${power.charging_overheat_threshold_c ?? '未知'} °C）` : `未触发（阈值 ${power?.charging_overheat_threshold_c != null ? `${power.charging_overheat_threshold_c} °C` : '未知'}）` }}</b></span>
            <span>BMS 状态码 <b>{{ power?.error_code ?? '未知' }}</b></span>
            <span>工作模式 <b>{{ powerModeText }}</b></span>
            <span>模式状态 <b>{{ powerModeStateText }}</b></span>
            <span>3588 运控 <b>{{ motionControlText }}（{{ motionControlService?.expected_active ? '正常模式应启动' : '冷却待机应停止' }}）</b></span>
            <span>充电流程 <b>{{ chargeStageText }}</b></span>
            <span v-if="powerMode?.charge_stage_detail">充电条件 <b>{{ powerMode.charge_stage_detail }}</b></span>
            <span>满电自动恢复 <b>{{ powerMode?.auto_charge_enabled ? '已启用' : '未启用' }}</b></span>
            <span v-if="powerMode?.last_warning">模式告警 <b>{{ powerMode.last_warning }}</b></span>
          </div>
          <div class="service-state-head">
            <b>{{ powerModeText }}模式服务</b>
            <div class="service-state-actions">
              <span :class="['state-chip', { ok: modeServicesMatched, warning: modeServices.length && !modeServicesMatched }]">
                {{ !modeServices.length ? '等待上报' : modeServicesMatched ? '全部符合' : '存在不一致' }}
              </span>
              <button
                type="button"
                class="primary-btn compact-command service-align-button"
                :disabled="modeAlignPending || Boolean(commandBusy) || !['cooling_standby', 'normal'].includes(powerMode?.mode)"
                @click="forceModeAlignment"
              >{{ modeAlignPending ? '状态对齐中' : '强制对齐' }}</button>
            </div>
          </div>
          <div v-if="modeServices.length" class="service-state-list">
            <div v-for="service in modeServices" :key="service.key" :class="['service-state-row', { mismatch: !service.matches_mode }]">
              <strong>{{ service.name }}</strong>
              <span>{{ service.currentText }}</span>
              <small>{{ service.expectedText }}</small>
              <b>{{ service.matches_mode ? '一致' : '不一致' }}</b>
            </div>
          </div>
          <p v-else class="service-state-empty">尚未收到服务状态，请刷新机器人状态。</p>
          <p v-if="modeAlignMessage" class="command-feedback mode-align-feedback">{{ modeAlignMessage }}</p>
        </div>

        <div class="detail-card management-control-card">
          <strong>扬声器音量</strong>
          <label class="volume-control">
            <span><b>3588 告警音响</b><small>{{ audio?.speaker_3588?.online ? '在线' : '离线' }}</small></span>
            <input
              v-model.number="speaker3588Volume"
              type="range"
              min="0"
              max="100"
              :disabled="Boolean(commandBusy) || !audio?.speaker_3588?.online"
              @input="queueSpeakerVolume('speaker_3588', speaker3588Volume)"
            >
            <span class="volume-result">
              <output>{{ speaker3588Volume }}%</output>
              <small :class="{ success: volumeFeedback.speaker_3588 === '音量下发成功', error: volumeFeedback.speaker_3588.startsWith('下发失败') }">
                {{ volumeFeedback.speaker_3588 }}
              </small>
            </span>
          </label>
          <label class="volume-control">
            <span><b>NX 扬声器</b><small>{{ audio?.speaker_nx?.online ? '在线' : '离线' }}</small></span>
            <input
              v-model.number="speakerNxVolume"
              type="range"
              min="0"
              max="100"
              :disabled="Boolean(commandBusy) || !audio?.speaker_nx?.online"
              @input="queueSpeakerVolume('speaker_nx', speakerNxVolume)"
            >
            <span class="volume-result">
              <output>{{ speakerNxVolume }}%</output>
              <small :class="{ success: volumeFeedback.speaker_nx === '音量下发成功', error: volumeFeedback.speaker_nx.startsWith('下发失败') }">
                {{ volumeFeedback.speaker_nx }}
              </small>
            </span>
          </label>
        </div>

        <p v-if="commandMessage" class="command-feedback">{{ commandMessage }}</p>

        <div class="detail-card">
          <strong>当前执行任务</strong>
          <div class="status-facts task-facts">
            <span>任务 <b>{{ activeTask?.name || '暂无执行中的任务' }}</b></span>
            <span>路线 <b>{{ activeTask?.route_name_display || '暂无' }}</b></span>
            <span>执行状态 <b>{{ taskStateLabel(activeTask?.latest_execution?.state) }}</b></span>
            <span>完成率 <b>{{ activeTask?.completion_rate ?? 0 }}%</b></span>
          </div>
        </div>

        <div class="detail-card">
          <strong>Edge Agent 状态</strong>
          <div class="status-facts">
            <span>云端连接 <b>{{ liveStatus?.connection_status || 'unknown' }}</b></span>
            <span>Agent 版本 <b>{{ selectedRobot.agent_version || '未知' }}</b></span>
            <span>控制模式 <b>{{ status?.control_mode || 'unknown' }}</b></span>
            <span>当前会话 <b>{{ sessions.length }}</b></span>
            <span>最后上报 <b>{{ formatTime(status?.received_at) }}</b></span>
            <span>ROS <b>{{ status?.ros_ready ? 'ready' : 'not ready' }}</b></span>
          </div>
        </div>

        <div class="detail-card">
          <strong>定位、导航与避障</strong>
          <div class="status-facts">
            <span>定位 <b>{{ status?.localization_status || 'unknown' }}</b></span>
            <span>NDT <b>{{ status?.localization_quality?.has_converged ? '已收敛' : '未收敛' }}</b></span>
            <span>Nav2 <b>{{ status?.nav_ready ? 'ready' : 'not ready' }}</b></span>
            <span>激光雷达 <b>{{ sensors?.lidar?.online ? '在线' : '离线' }}</b></span>
            <span>避障扫描 <b>{{ sensors?.laser_scan?.online ? '在线' : '离线' }}</b></span>
            <span>前方障碍 <b>{{ navigation?.front_obstacle_distance_m != null ? `${navigation.front_obstacle_distance_m.toFixed(2)} m` : '未检测到' }}</b></span>
            <span>规划速度 <b>{{ navigation?.requested_planar_speed_mps != null ? `${navigation.requested_planar_speed_mps.toFixed(2)} m/s` : '0.00 m/s' }}</b></span>
            <span>实际速度 <b>{{ navigation?.actual_planar_speed_mps != null ? `${navigation.actual_planar_speed_mps.toFixed(2)} m/s` : '0.00 m/s' }}</b></span>
          </div>
        </div>

        <div class="detail-card">
          <strong>近期识别事件</strong>
          <div v-if="selectedRobot.recent_events?.length" class="robot-event-list">
            <article v-for="event in selectedRobot.recent_events" :key="event.id" class="robot-event-row">
              <img v-if="event.annotated_snapshot_url || event.snapshot_url" :src="event.annotated_snapshot_url || event.snapshot_url" alt="告警截图">
              <div>
                <b>{{ event.title }}</b>
                <span>{{ event.status_label }} · 置信度 {{ event.confidence }}%</span>
                <small>{{ formatTime(event.detected_at) }} · {{ event.location }}</small>
              </div>
            </article>
          </div>
          <p v-else>暂无识别事件</p>
        </div>
      </section>
    </div>

    <div v-if="dockDialogOpen" class="dock-dialog-backdrop" @click.self="dockDialogOpen = false">
      <section class="dock-dialog" role="dialog" aria-modal="true" aria-label="一键回充">
        <div class="detail-card-head"><strong>一键回充</strong><button type="button" class="ghost-btn compact-command" @click="dockDialogOpen = false">关闭</button></div>
        <p>{{ dockRobot?.name }}：第 1 点正常避障，第 2 点微速直连充电桩。</p>
        <template v-if="!dockExecution">
        <label>充电地图
          <select v-model="dockMapId" @change="onDockMapChanged">
            <option value="">选择地图</option>
            <option v-for="map in dockMaps" :key="map.id" :value="String(map.id)">{{ map.name }}</option>
          </select>
        </label>
        <label>两点回充路线
          <select v-model="dockRouteId">
            <option value="">选择路线</option>
            <option v-for="route in availableDockRoutes" :key="route.id" :value="String(route.id)">{{ route.name }}</option>
          </select>
        </label>
        <small>确认后将保存为该机器人的默认回充配置。</small>
        <p v-if="dockMessage" class="command-feedback">{{ dockMessage }}</p>
        <button type="button" class="primary-btn" :disabled="dockSubmitting || !dockMapId || !dockRouteId" @click="confirmDock">{{ dockSubmitting ? '下发中' : '确认一键回充' }}</button>
        </template>
        <div v-else class="dock-progress">
          <p>{{ dockExecution.map_name }} · {{ dockExecution.route_name }} · {{ dockExecution.state }}</p>
          <div v-for="(label, index) in dockSteps" :key="label" :class="['dock-step', dockStepState(index)]"><b>{{ index + 1 }}</b><span>{{ label }}</span></div>
          <p v-if="dockMessage" class="command-feedback">{{ dockMessage }}</p>
        </div>
      </section>
    </div>
  </section>
</template>
