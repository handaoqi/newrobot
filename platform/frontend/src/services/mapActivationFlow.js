import {
  fetchRobotCommand,
  fetchRobotNavigationStatus,
  sendRobotNavigationCommand,
  setActiveMap,
} from './api.js'
import {
  expectedLegacyMapVersion,
  navigationMapIdentity,
  navigationReadyForMap,
} from './mapActivationState.js'

const TERMINAL_COMMAND_STATES = new Set([
  'succeeded',
  'failed',
  'rejected',
  'cancelled',
  'timed_out',
  'expired',
])

function sleep(milliseconds) {
  return new Promise(resolve => window.setTimeout(resolve, milliseconds))
}

function localizationNormal(payload = {}) {
  const status = payload.status || {}
  return (status.localization_status || payload.localization_status) === 'normal'
}

function navigationStackReady(payload = {}) {
  const status = payload.status || {}
  return Boolean(status.nav_ready ?? payload.nav_ready)
}

function activeTaskExecutionId(payload = {}) {
  return payload?.status?.task_execution_id || null
}

export async function waitForRobotCommand(robotId, command, {
  timeoutMs = 180_000,
  intervalMs = 1_000,
  onProgress = () => {},
  onCommand = () => {},
} = {}) {
  if (!robotId || !command?.id) throw new Error('设备命令信息不完整')
  let latest = command
  const deadline = Date.now() + timeoutMs
  while (!TERMINAL_COMMAND_STATES.has(latest.status) && Date.now() < deadline) {
    onProgress(latest)
    await sleep(intervalMs)
    latest = await fetchRobotCommand(robotId, command.id)
  }
  if (!TERMINAL_COMMAND_STATES.has(latest.status)) {
    throw new Error('等待机器狗确认命令超时')
  }
  if (latest.status !== 'succeeded') {
    const error = new Error(
      latest.error_message
      || latest.ack_reason_message
      || latest.error_code
      || latest.ack_reason_code
      || `机器狗命令执行失败：${latest.status}`,
    )
    // Preserve the terminal payload for callers that need to display a
    // committed NDT candidate even when the final LIO handoff failed.
    error.command = latest
    throw error
  }
  onProgress(latest)
  return latest
}

export async function activateAndRelocalizeMap({
  mapId,
  robotId,
  mapVersion = expectedLegacyMapVersion(mapId),
  sceneScope = 'indoor',
  coordinateMode = 'local_only',
  onProgress = () => {},
  onCommand = () => {},
  traceId = '',
}) {
  const activation = await activateRouteMap({ mapId, robotId, mapVersion, onProgress, onCommand, traceId })
  let navigationStatus = activation.navigationStatus

  if (navigationReadyForMap(navigationStatus, mapId, mapVersion)) {
    onProgress('目标地图已应用，定位与导航已就绪')
    return activation
  }
  if (activeTaskExecutionId(navigationStatus)) {
    throw new Error('机器人正在执行任务，路线已保存，但不能切换地图、重定位或重启导航栈')
  }

  if (localizationNormal(navigationStatus) && !navigationStackReady(navigationStatus)) {
    onProgress('定位已就绪，正在启动导航栈')
    const startCommand = await sendRobotNavigationCommand(robotId, 'start', {
      map_id: String(mapId),
      map_version: mapVersion,
    }, { traceId })
    await waitForRobotCommand(robotId, startCommand, {
      timeoutMs: 180_000,
      onProgress: latest => onProgress(`导航栈启动：${latest.status || 'created'}`),
    })
    navigationStatus = await fetchRobotNavigationStatus(robotId)
    if (navigationReadyForMap(navigationStatus, mapId, mapVersion)) {
      onProgress('地图、定位与导航均已就绪')
      return { ...activation, navigationStatus }
    }
  }

  onProgress('地图已应用，先快速搜索可信位置与建图原点，失败后自动进入全局搜索')
  const relocalizeCommand = await sendRobotNavigationCommand(robotId, 'relocalize', {
    map_id: String(mapId),
    map_version: mapVersion,
    seed_source: 'quick_then_global',
    scene_scope: sceneScope,
    coordinate_mode: coordinateMode,
    wait_seconds: 120,
  }, { traceId })
  await waitForRobotCommand(robotId, relocalizeCommand, {
    timeoutMs: 360_000,
    onProgress: latest => {
      onProgress(`快速定位/全局回退：${latest.status || 'created'}`)
      onCommand({ phase: 'localization', command: latest, showCandidates: true })
    },
  })

  navigationStatus = await fetchRobotNavigationStatus(robotId)
  if (localizationNormal(navigationStatus) && !navigationStackReady(navigationStatus)) {
    onProgress('定位已恢复，正在启动导航栈')
    const startCommand = await sendRobotNavigationCommand(robotId, 'start', {
      map_id: String(mapId),
      map_version: mapVersion,
    }, { traceId })
    await waitForRobotCommand(robotId, startCommand, {
      timeoutMs: 180_000,
      onProgress: latest => onProgress(`导航栈启动：${latest.status || 'created'}`),
    })
  }

  const deadline = Date.now() + 60_000
  while (Date.now() < deadline) {
    navigationStatus = await fetchRobotNavigationStatus(robotId)
    if (navigationReadyForMap(navigationStatus, mapId, mapVersion)) {
      onProgress('地图、定位与导航均已就绪')
      return { ...activation, navigationStatus }
    }
    onProgress('重定位命令已完成，正在等待定位稳定')
    await sleep(2_000)
  }
  throw new Error('地图已下发，但定位未在限定时间内恢复 normal；请到路径规划页面手动设置初始定位')
}

export async function activateRouteMap({
  mapId,
  robotId,
  mapVersion = expectedLegacyMapVersion(mapId),
  onProgress = () => {},
  onCommand = () => {},
  traceId = '',
}) {
  if (!mapId) throw new Error('路线未绑定地图')
  if (!robotId) throw new Error('路线未绑定机器狗')

  let navigationStatus = await fetchRobotNavigationStatus(robotId)
  if (navigationStatus.connection_status !== 'online') {
    throw new Error('机器狗 Edge Agent 当前离线，无法下发地图')
  }
  const currentMap = navigationMapIdentity(navigationStatus)
  const mapMatches = currentMap.mapId === String(mapId) && currentMap.mapVersion === String(mapVersion)
  if (!mapMatches) {
    if (activeTaskExecutionId(navigationStatus)) {
      throw new Error('机器人正在执行任务，不能切换到新路线地图')
    }
    onProgress('正在下发地图到机器狗')
    const activation = await setActiveMap(mapId, { traceId })
    if (activation.robot && String(activation.robot) !== String(robotId)) {
      throw new Error('路线地图绑定的机器狗与任务机器狗不一致')
    }
    const command = activation.activation_command
    if (!command) throw new Error('地图未绑定机器狗，无法下发设备切换命令')
    await waitForRobotCommand(robotId, command, {
      timeoutMs: 180_000,
      onProgress: latest => {
        onProgress(`地图下发：${latest.status || 'created'}`)
        onCommand({ phase: 'transfer', command: latest, showCandidates: false })
      },
    })
  }

  navigationStatus = await fetchRobotNavigationStatus(robotId)
  onProgress(mapMatches ? '目标地图已在机器狗上' : '路线地图下发完成')
  return { changed: !mapMatches, navigationStatus }
}
