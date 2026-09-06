const event = {
  id: 101,
  title: '主通道发现自行车停留',
  event_type: 'bicycle_detected',
  location: '太阳宫公园主通道',
  detected_at: '2026-08-24T08:30:00+08:00',
  confidence: 0.94,
  risk_level: 'medium',
  risk_label: '中风险',
  status: 'pending',
  status_label: '待处理',
  snapshot_url: '/images/event-1.jpg',
  annotated_snapshot_url: '/images/event-1.jpg',
  description: '自行车在巡检通道内停留，请值班员关注。',
  robot_code: 'ZSL-1A-07',
}

const robot = {
  id: 1,
  code: 'ZSL-1A-07',
  name: '南入口巡检机器人',
  location: '太阳宫公园南入口',
  area: '主通道南入口',
  status: 'online',
  status_label: '在线',
  mode: 'auto',
  mode_label: '自动巡检',
  battery_level: 78,
  network_strength: 92,
  speaker_volume: 84,
  today_alerts: 3,
  current_task_name: '公园主通道例行巡检',
  stream_id: '',
  play_urls: {},
  connection_status: 'online',
  localization_status: 'normal',
  ros_ready: true,
  nav_ready: true,
  control_mode: 'auto',
  current_map_id: 20,
  current_map_version: 'v1',
  recent_events: [event],
  tasks: [],
}

const overview = {
  summary: {
    online_robot_count: 1,
    today_alert_count: 3,
    pending_event_count: 1,
    resolved_event_count: 2,
    completed_task_count: 4,
  },
  header: {
    device_code: robot.code,
    current_mode: '自动巡检',
    current_location: robot.location,
    today_alerts: 3,
    today_patrol_minutes: 126,
  },
  live_event: event,
  latest_robot: robot,
  event_distribution: [
    { status: 'pending', total: 1 },
    { status: 'resolved', total: 2 },
  ],
}

const robotStatus = {
  robot_id: 1,
  connection_status: 'online',
  localization_status: 'normal',
  nav_ready: true,
  task_execution_id: '',
  status: {
    localization_status: 'normal',
    nav_ready: true,
    control_mode: 'auto',
    battery_percent: 63,
    speed_mps: 0,
    sampled_at: '2026-08-24T08:30:00+08:00',
    audio: {
      capture_enabled: false,
      speaker_3588: { online: true, volume_percent: 76 },
      speaker_nx: { online: true, volume_percent: 82 },
    },
    power_mode: {
      services: { controller_motion: { available: true, active: true } },
    },
    localization_quality: {
      matching_error: 0.086,
      inlier_fraction: 0.91,
      decision: {
        preferred_source: 'ndt',
        active_source: 'ndt_imu',
        rtk_quality: '固定解',
        rtk_x: 12.42,
        rtk_y: 8.16,
        rtk_yaw: 0.314,
        absolute_stable: true,
      },
    },
    navigation: { front_obstacle_distance_m: 3.72 },
  },
}

const navigationStatus = {
  robot_id: 1,
  connection_status: 'online',
  current_map_id: 20,
  current_map_version: 'v1',
  status: {
    map_id: 20,
    map_version: 'v1',
    x: 2.4,
    y: 3.1,
    yaw: 0.4,
    localization_status: 'normal',
    nav_ready: true,
  },
}

const developmentTask = {
  id: 'dev-task-1',
  robot: 1,
  workspace: 'robot-main',
  model: 'gpt-5.6-terra',
  prompt: '检查室外 RTK 定位链路，并说明当前状态。',
  status: 'running',
  status_label: '执行中',
  created_at: '2026-08-24T08:30:00+08:00',
  started_at: '2026-08-24T08:30:05+08:00',
  finished_at: null,
  exit_code: null,
  can_cancel: true,
  codex_thread_id: 'thread-tablet-development',
}

const developmentConversation = {
  codex_thread_id: developmentTask.codex_thread_id,
  tasks: [developmentTask],
  turns: [{
    task: developmentTask,
    events: [
      {
        sequence: 1,
        type: 'status',
        stream: 'stdout',
        text: 'Codex 正在检查定位遥测与 RTK 状态。',
        occurred_at: '2026-08-24T08:30:06+08:00',
      },
      {
        sequence: 2,
        type: 'output',
        stream: 'stdout',
        text: '实时输出应始终可见，不应被会话记录覆盖。',
        occurred_at: '2026-08-24T08:30:07+08:00',
      },
    ],
  }],
}

const developmentVoiceRecognitions = Array.from({ length: 8 }, (_, index) => ({
  id: index + 1,
  outcome: 'accepted',
  outcome_label: '已执行',
  created_at: `2026-08-24T08:${String(20 + index).padStart(2, '0')}:00+08:00`,
  transcript: `小太阳，继续第 ${index + 1} 条远程开发指令。`,
  asr_engine: 'whisper',
  command: '继续开发',
}))

const patrolTasks = [{
  id: 40,
  name: '公园主通道例行巡检',
  robot: 1,
  robot_name: robot.name,
  route: 10,
  route_name: '南门—主步道—活动广场',
  map_id: 20,
  enabled: true,
  status: 'pending',
  latest_execution: null,
}]

const routeDetail = {
  id: 10,
  name: '南门—主步道—活动广场',
  map_data: 20,
  robot: 1,
  waypoint_names: ['南门', '主步道', '活动广场'],
  description: '主步道巡检路线',
  scene_scope: 'indoor',
  global_controller: 'navfn',
  waypoints: [
    { id: 1, name: '南门', sequence: 0, map_point_number: 1, x: 1, y: 1, yaw: 0, global_controller: 'theta_star' },
    { id: 2, name: '主步道', sequence: 1, map_point_number: 2, x: 3, y: 2, yaw: 0.4, global_controller: 'navfn' },
    { id: 3, name: '活动广场', sequence: 2, map_point_number: 3, x: 5, y: 4, yaw: 0.8 },
  ],
}

const routeSummary = {
  id: routeDetail.id,
  name: routeDetail.name,
  map_data: routeDetail.map_data,
  robot: routeDetail.robot,
  waypoint_count: routeDetail.waypoints.length,
  // Compatibility shape from older deployments: a compact summary may
  // contain an empty placeholder even though waypoint_count is non-zero.
  waypoints: [],
  global_controller: routeDetail.global_controller,
  latest_execution: {
    id: 'route-execution-10',
    state: 'completed',
    created_at: '2026-08-24T08:30:00+08:00',
  },
}

const defaultRouteExecution = {
  id: 'route-execution-10',
  route: 10,
  route_name: routeDetail.name,
  state: 'completed',
  completed_waypoints: 3,
  current_waypoint_index: 2,
  created_at: '2026-08-24T08:30:00+08:00',
  started_at: '2026-08-24T08:30:01+08:00',
  finished_at: '2026-08-24T08:30:12+08:00',
  route_snapshot: { waypoints: routeDetail.waypoints },
  events: [],
}

const mapDetail = {
  id: 20,
  name: '太阳宫园区 V1',
  version: 'v1',
  thumbnail_url: null,
  resolution: 0.05,
  width: 800,
  height: 600,
  origin: [-10, -10, 0],
}

function responseFor(pathname, method) {
  const path = pathname.replace(/^\/api/, '')
  if (path === '/auth/login/' && method === 'POST') {
    return { token: 'tablet-visual-token', user: { username: 'operator', display_name: '值班员 A' } }
  }
  if (path === '/dashboard/overview/') return overview
  if (path === '/robots/') return [robot]
  if (path === '/robots/1/') return robot
  if (path === '/robots/1/status/') return robotStatus
  if (path === '/robots/1/person-detections/') {
    return { enabled: true, available: false, stale: false, detections: [], frame_width: 1920, frame_height: 1080 }
  }
  if (path === '/robots/1/mapping/status/') {
    return { robot_id: 1, robot_code: robot.code, result: { save_progress: { slam_health: { state: 'healthy' } } } }
  }
  if (path === '/robots/1/navigation/status/') return navigationStatus
  if (path === '/development/agents/') return [{ robot: 1, status: 'online', agent_version: '0.1.0' }]
  if (path === '/development/conversations/main/') return developmentConversation
  if (path === '/development/tasks/') return [developmentTask]
  if (path === '/voice-recognitions/') return developmentVoiceRecognitions
  if (path === '/patrol-tasks/') return patrolTasks
  if (path === '/routes/') return [routeSummary]
  if (path === '/routes/10/') return routeDetail
  if (path === '/maps/') return [mapDetail]
  if (path === '/map-sets/') return []
  if (path === '/maps/20/') return mapDetail
  if (path === '/maps/20/mapping-trace/') return { samples: [] }
  if (path === '/speech-categories/') return [{ id: 1, name: '现场提醒' }]
  if (path === '/speech-templates/') {
    return [{ id: 1, name: '文明通行提醒', text: '您好，请保持通道畅通。', category: 1, category_name: '现场提醒' }]
  }
  if (path === '/recorded-audio/') return []
  if (path === '/alert-skills/') return []
  return {}
}

export async function installTabletMocks(page, {
  authenticated,
  routeExecution = defaultRouteExecution,
  mapThumbnailUrl = null,
} = {}) {
  await page.addInitScript(({ shouldAuthenticate }) => {
    if (shouldAuthenticate) {
      localStorage.setItem('inspection_token', 'tablet-visual-token')
      localStorage.setItem('inspection_user', JSON.stringify({ username: 'operator', display_name: '值班员 A' }))
    } else {
      localStorage.removeItem('inspection_token')
      localStorage.removeItem('inspection_user')
    }

    class SilentEventSource {
      addEventListener() {}
      removeEventListener() {}
      close() {}
    }
    window.EventSource = SilentEventSource
  }, { shouldAuthenticate: authenticated })

  await page.route('**/api/**', async (route) => {
    const request = route.request()
    const pathname = new URL(request.url()).pathname
    if (!pathname.startsWith('/api/')) {
      await route.fallback()
      return
    }
    const apiPath = pathname.replace(/^\/api/, '')
    let body
    if (apiPath === '/routes/10/execute/' && request.method() === 'POST') {
      body = routeExecution
    } else if (apiPath === `/task-executions/${routeExecution.id}/`) {
      body = routeExecution
    } else if (apiPath === `/task-executions/${routeExecution.id}/trajectory/`) {
      body = { points: [] }
    } else if (mapThumbnailUrl && ['/maps/', '/maps/20/'].includes(apiPath)) {
      const map = { ...mapDetail, thumbnail_url: mapThumbnailUrl }
      body = apiPath === '/maps/' ? [map] : map
    } else {
      body = responseFor(pathname, request.method())
    }
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify(body),
    })
  })
}
