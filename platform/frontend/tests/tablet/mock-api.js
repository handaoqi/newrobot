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
  waypoints: [
    { id: 1, name: '南门', sequence: 0, map_point_number: 1, x: 1, y: 1, yaw: 0 },
    { id: 2, name: '主步道', sequence: 1, map_point_number: 2, x: 3, y: 2, yaw: 0.4 },
    { id: 3, name: '活动广场', sequence: 2, map_point_number: 3, x: 5, y: 4, yaw: 0.8 },
  ],
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
  if (path === '/patrol-tasks/') return patrolTasks
  if (path === '/routes/10/') return routeDetail
  if (path === '/maps/20/') return mapDetail
  if (path === '/speech-categories/') return [{ id: 1, name: '现场提醒' }]
  if (path === '/speech-templates/') {
    return [{ id: 1, name: '文明通行提醒', text: '您好，请保持通道畅通。', category: 1, category_name: '现场提醒' }]
  }
  if (path === '/recorded-audio/') return []
  if (path === '/alert-skills/') return []
  return {}
}

export async function installTabletMocks(page, { authenticated }) {
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
    const body = responseFor(new URL(request.url()).pathname, request.method())
    await route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify(body),
    })
  })
}
