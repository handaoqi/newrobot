<script setup>
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'

import { fetchRobotCommand, fetchRobotDetail, fetchRobots, fetchRobotSessions, fetchRobotStatus, sendRobotCommand } from '../services/api'

const robots = ref([])
const selectedRobot = ref(null)
const liveStatus = ref(null)
const sessions = ref([])
const commandBusy = ref('')
const commandMessage = ref('')
const speaker3588Volume = ref(100)
const speakerNxVolume = ref(50)
const volumeFeedback = ref({ speaker_3588: '', speaker_nx: '' })
const refreshing = ref(false)
const refreshingRobotId = ref(null)
const chargeControlMode = ref('start')
const chargeTogglePending = ref(false)
const modeAlignPending = ref(false)
let chargeRefreshTimer = null
const volumeDebounceTimers = {}
const volumeRequestVersions = { speaker_3588: 0, speaker_nx: 0 }

const status = computed(() => liveStatus.value?.status || null)
const power = computed(() => status.value?.power || null)
const network = computed(() => status.value?.network || null)
const audio = computed(() => status.value?.audio || null)
const powerMode = computed(() => status.value?.power_mode || null)
const navigation = computed(() => status.value?.navigation || null)
const sensors = computed(() => status.value?.sensors || {})
const activeTask = computed(() => (
  selectedRobot.value?.tasks?.find((task) => (
    ['dispatching', 'accepted', 'running', 'pausing', 'paused', 'resuming'].includes(task.latest_execution?.state)
  )) || null
))

const batteryText = computed(() => (
  power.value?.available && power.value?.percent != null ? `${power.value.percent}%` : '未知'
))

const chargeText = computed(() => {
  if (!power.value?.available) return '状态未知'
  if (power.value.thermal_protection || power.value.charge_state === 'thermal_protection') return '过热保护'
  if (power.value.charging) return '正在充电'
  if (power.value.charger_controller_active) return '等待充电'
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

const modeServices = computed(() => Object.entries(powerMode.value?.services || {}).map(([key, service]) => ({
  key,
  ...service,
  currentText: key === 'power_profile'
    ? (service.value || '未知')
    : (service.active ? '运行' : '停止'),
  expectedText: key === 'power_profile'
    ? `应为 ${service.expected_value || '未知'}`
    : (service.expected_active ? '应运行' : '应停止'),
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
  const livePower = liveStatus.value?.status?.power
  chargeControlMode.value = livePower?.charging || livePower?.charger_controller_active ? 'stop' : 'start'
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
      const refreshedPower = liveStatus.value?.status?.power
      chargeControlMode.value = refreshedPower?.charging || refreshedPower?.charger_controller_active ? 'stop' : 'start'
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
  const expectedControllerActive = action === 'charge_start'
  chargeTogglePending.value = true
  try {
    if (!await sendManagementCommand(action)) return
    commandMessage.value = action === 'charge_start'
      ? '正在切换冷却待机，成功后自动开始充电'
      : '正在断开充电并恢复正常工作服务'
    let latestState = null
    for (let attempt = 0; attempt < 60; attempt += 1) {
      await new Promise((resolve) => window.setTimeout(resolve, 2000))
      latestState = await refreshChargeStatus()
      const expectedMode = action === 'charge_start' ? 'cooling_standby' : 'normal'
      if (
        latestState?.power
        && Boolean(latestState.power.charger_controller_active) === expectedControllerActive
        && latestState.powerMode?.mode === expectedMode
        && latestState.powerMode?.transition_state === 'ready'
      ) break
    }
    if (latestState?.power) {
      chargeControlMode.value = latestState.power.charging || latestState.power.charger_controller_active ? 'stop' : 'start'
      commandMessage.value = latestState.powerMode?.transition_state === 'error'
        ? `模式切换异常：${latestState.powerMode.last_error || '请检查设备日志'}`
        : '充电与工作模式状态已刷新'
    } else {
      commandMessage.value = '状态刷新失败，请点击机器人卡片刷新按钮重试'
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
  try {
    if (!await sendManagementCommand(action)) return
    commandMessage.value = `正在强制对齐${powerModeText.value}模式服务`
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
      commandMessage.value = `${powerModeText.value}模式状态对齐完成`
    } else if (latestState?.powerMode?.transition_state === 'error') {
      commandMessage.value = `状态对齐失败：${latestState.powerMode.last_error || '请检查设备日志'}`
    } else {
      commandMessage.value = '状态对齐未完成，服务状态仍存在不一致'
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

onMounted(async () => {
  await refreshRobots()
  chargeRefreshTimer = window.setInterval(refreshChargeStatus, 3000)
})

onBeforeUnmount(() => {
  if (chargeRefreshTimer) window.clearInterval(chargeRefreshTimer)
  Object.values(volumeDebounceTimers).forEach((timer) => window.clearTimeout(timer))
})
</script>

<template>
  <section class="page-section">
    <div class="data-grid">
      <section class="panel list-panel">
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
            <strong>{{ robot.name }}</strong>
            <span>{{ robot.code }}</span>
            <small>{{ robot.location }}</small>
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
                {{ power.voltage_v != null ? `${power.voltage_v} V` : '电压未知' }}
              </small>
            </div>
            <div class="battery-charge-actions">
              <button
                type="button"
                :class="[chargeControlMode === 'start' ? 'primary-btn' : 'ghost-btn', 'compact-command', { 'danger-command': chargeControlMode === 'stop' }]"
                :disabled="Boolean(commandBusy) || chargeTogglePending || !power?.available"
                @click="toggleChargeControl"
              >{{ chargeTogglePending ? '状态刷新中' : chargeControlMode === 'start' ? '开始充电' : '断开充电' }}</button>
            </div>
          </div>
          <div class="metric-card" :class="{ 'metric-card-warning': power?.thermal_protection }">
            <strong>{{ chargeText }}</strong>
            <span>充电状态</span>
            <small v-if="power?.available">
              {{ batteryCurrentText }}
            </small>
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
            <span>充电极片 <b>{{ power?.charge_pin === 1 ? '已接触' : '未接触' }}</b></span>
            <span>负极 <b>{{ power?.negative_contact === 1 ? '正常' : '异常' }}</b></span>
            <span>正极 <b>{{ power?.positive_contact === 1 ? '正常' : '异常' }}</b></span>
            <span>电池温度 <b>{{ power?.temperature_c != null ? `${power.temperature_c} °C` : '未知' }}</b></span>
            <span>温度保护 <b>{{ power?.thermal_protection ? `已触发（阈值 ${power.charging_overheat_threshold_c} °C）` : '未触发' }}</b></span>
            <span>BMS 状态码 <b>{{ power?.error_code ?? '未知' }}</b></span>
            <span>工作模式 <b>{{ powerModeText }}</b></span>
            <span>模式状态 <b>{{ powerModeStateText }}</b></span>
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
  </section>
</template>
