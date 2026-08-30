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

export async function waitForRobotCommand(robotId, command, {
  timeoutMs = 180_000,
  intervalMs = 1_000,
  onProgress = () => {},
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
    throw new Error(
      latest.error_message
      || latest.ack_reason_message
      || latest.error_code
      || latest.ack_reason_code
      || `机器狗命令执行失败：${latest.status}`,
    )
  }
  onProgress(latest)
  return latest
}

export async function activateAndRelocalizeMap({
  mapId,
  robotId,
  mapVersion = expectedLegacyMapVersion(mapId),
  onProgress = () => {},
}) {
  if (!mapId) throw new Error('路线未绑定地图')
  if (!robotId) throw new Error('路线未绑定机器狗')

  let navigationStatus = await fetchRobotNavigationStatus(robotId)
  if (navigationStatus.connection_status !== 'online') {
    throw new Error('机器狗 Edge Agent 当前离线，无法下发地图')
  }
  if (navigationReadyForMap(navigationStatus, mapId, mapVersion)) {
    onProgress('目标地图已应用，定位与导航已就绪')
    return { changed: false, navigationStatus }
  }

  const currentMap = navigationMapIdentity(navigationStatus)
  const mapMatches = currentMap.mapId === String(mapId) && currentMap.mapVersion === String(mapVersion)
  if (!mapMatches) {
    onProgress('正在下发地图到机器狗')
    const activation = await setActiveMap(mapId)
    if (activation.robot && String(activation.robot) !== String(robotId)) {
      throw new Error('路线地图绑定的机器狗与任务机器狗不一致')
    }
    const command = activation.activation_command
    if (!command) throw new Error('地图未绑定机器狗，无法下发设备切换命令')
    await waitForRobotCommand(robotId, command, {
      timeoutMs: 180_000,
      onProgress: latest => onProgress(`地图下发：${latest.status || 'created'}`),
    })
  }

  navigationStatus = await fetchRobotNavigationStatus(robotId)
  const liveStatus = navigationStatus.status || {}
  if (!(liveStatus.nav_ready ?? navigationStatus.nav_ready)) {
    onProgress('地图已应用，正在启动导航栈')
    const startCommand = await sendRobotNavigationCommand(robotId, 'start', {
      map_id: String(mapId),
      map_version: mapVersion,
    })
    await waitForRobotCommand(robotId, startCommand, {
      timeoutMs: 180_000,
      onProgress: latest => onProgress(`启动导航栈：${latest.status || 'created'}`),
    })
    navigationStatus = await fetchRobotNavigationStatus(robotId)
  }

  if (navigationReadyForMap(navigationStatus, mapId, mapVersion)) {
    onProgress('地图、定位与导航均已就绪')
    return { changed: !mapMatches, navigationStatus }
  }

  onProgress('地图已应用，正在使用最近可信位置重定位')
  try {
    const relocalizeCommand = await sendRobotNavigationCommand(robotId, 'relocalize', {
      map_id: String(mapId),
      map_version: mapVersion,
      seed_source: 'last_trusted',
    })
    await waitForRobotCommand(robotId, relocalizeCommand, {
      timeoutMs: 180_000,
      onProgress: latest => onProgress(`可信位置重定位：${latest.status || 'created'}`),
    })
  } catch {
    onProgress('无可用可信位姿，正在搜索全图位置与 360° 航向')
    const globalCommand = await sendRobotNavigationCommand(robotId, 'relocalize', {
      map_id: String(mapId),
      map_version: mapVersion,
      seed_source: 'global',
      wait_seconds: 90,
    })
    await waitForRobotCommand(robotId, globalCommand, {
      timeoutMs: 180_000,
      onProgress: latest => onProgress(`全局重定位：${latest.status || 'created'}`),
    })
  }

  const deadline = Date.now() + 60_000
  while (Date.now() < deadline) {
    navigationStatus = await fetchRobotNavigationStatus(robotId)
    if (navigationReadyForMap(navigationStatus, mapId, mapVersion)) {
      onProgress('地图、定位与导航均已就绪')
      return { changed: !mapMatches, navigationStatus }
    }
    onProgress('重定位命令已完成，正在等待定位稳定')
    await sleep(2_000)
  }
  throw new Error('地图已下发，但定位未在限定时间内恢复 normal；请到路径规划页面手动设置初始定位')
}
