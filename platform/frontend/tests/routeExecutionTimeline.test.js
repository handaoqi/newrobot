import test from 'node:test'
import assert from 'node:assert/strict'

import {
  buildTaskExecutionTimeline,
  taskExecutionIsActive,
} from '../src/services/taskExecutionWaypointProgress.js'

function event(id, eventType, stateVersion, occurredAt, payload = {}, extra = {}) {
  return {
    id,
    event_type: eventType,
    state_version: stateVersion,
    occurred_at: occurredAt,
    payload,
    ...extra,
  }
}

test('renders durable structured navigation stages with metrics', () => {
  const timeline = buildTaskExecutionTimeline({
    state: 'running',
    created_at: '2026-09-16T08:00:00+08:00',
    route_snapshot: { waypoints: [{ map_point_number: 7, x: 1, y: 2, yaw: 0 }] },
    events: [
      event(1, 'task.navigation_stage', 2, '2026-09-16T08:00:01+08:00', {
        navigation_stage: 'path_planning',
        stage_status: 'active',
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 7, x: 1, y: 2 },
      }),
      event(2, 'task.navigation_stage', 3, '2026-09-16T08:00:02+08:00', {
        navigation_stage: 'path_tracking',
        stage_status: 'active',
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 7, x: 1, y: 2 },
        stage_metrics: { distance_remaining_m: 1.25 },
      }),
      event(3, 'task.navigation_stage', 4, '2026-09-16T08:00:03+08:00', {
        navigation_stage: 'arrival_acceptance',
        stage_status: 'completed',
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 7, x: 1, y: 2 },
        stage_metrics: { distance_m: 0.12, acceptance_tolerance_m: 0.3, reapproach_attempts: 1 },
      }),
      event(4, 'task.navigation_stage', 5, '2026-09-16T08:00:04+08:00', {
        navigation_stage: 'departure_heading',
        stage_status: 'completed',
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 7, x: 1, y: 2 },
        stage_metrics: { next_map_point_number: 8, heading_error_deg: 96.5 },
      }),
    ],
  })

  assert.deepEqual(timeline.map(item => item.title), [
    '7号点 · 全局规划与路径平滑：进行中',
    '7号点 · 路径跟踪：进行中',
    '7号点 · XY / 航向联合验收：完成',
    '7号点 · 对准下个航点：完成',
  ])
  assert.match(timeline[1].detail, /剩余 1\.25m/)
  assert.match(timeline[2].detail, /偏差 0\.12m.*验收半径 0\.30m.*追加靠近 1 次/)
  assert.match(timeline[3].detail, /对准 8 号点.*航向误差 96\.5度/)
})

test('builds an ordered real preview timeline from durable task events', () => {
  const execution = {
    state: 'completed',
    route_name: '南门路线',
    created_at: '2026-09-06T08:00:00+08:00',
    route_snapshot: {
      waypoints: [
        { map_point_number: 3, x: 3, y: 4 },
        { map_point_number: 1, x: 1, y: 2 },
      ],
    },
    events: [
      event(5, 'task.completed', 5, '2026-09-06T08:00:09+08:00'),
      event(3, 'task.waypoint_reached', 3, '2026-09-06T08:00:05+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 3, x: 3, y: 4 },
        robot_pose: { x: 3.02, y: 3.98 },
      }),
      event(1, 'task.created', 0, '2026-09-06T08:00:00+08:00'),
      event(2, 'task.target_dispatched', 2, '2026-09-06T08:00:02+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 3, x: 3, y: 4 },
      }),
      event(4, 'task.target_dispatched', 4, '2026-09-06T08:00:06+08:00', {
        execution_waypoint_index: 1,
        waypoint: { map_point_number: 1, x: 1, y: 2 },
      }),
    ],
  }

  const timeline = buildTaskExecutionTimeline(execution, {
    requestedAt: Date.parse('2026-09-06T08:00:00+08:00'),
  })

  assert.deepEqual(timeline.map(item => item.title), [
    '路线下发中',
    '任务执行已创建',
    '3号点目标已下发',
    '3号点已到达',
    '1号点目标已下发',
    '任务执行完成',
  ])
  assert.match(timeline[3].detail, /目标 x 3\.00 \/ y 4\.00/)
  assert.match(timeline[3].detail, /机器狗 x 3\.02 \/ y 3\.98/)
  assert.equal(timeline.at(-1).elapsedSeconds, 9)
})

test('falls back to execution index and removes duplicated events', () => {
  const duplicated = event('', 'task.target_dispatched', 2, '2026-09-06T08:00:02+08:00', {
    execution_waypoint_index: 1,
    waypoint: { x: 5, y: 6 },
  }, { message_id: 'same-message' })
  const timeline = buildTaskExecutionTimeline({
    state: 'running',
    route_snapshot: {
      waypoints: [
        { map_point_number: 7 },
        { map_point_number: 6 },
      ],
    },
    events: [duplicated, { ...duplicated }],
  })

  assert.equal(timeline.length, 1)
  assert.equal(timeline[0].title, '6号点目标已下发')
})

test('arrival confirmation shows correction mode, one reapproach and strict radius', () => {
  const timeline = buildTaskExecutionTimeline({
    state: 'running',
    route_snapshot: { waypoints: [{ map_point_number: 2 }] },
    events: [
      event(8, 'task.arrival_confirmed', 8, '2026-09-06T08:00:08+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 2, x: 2, y: 3 },
        arrival_mode: 'lightweight',
        localization_correction: 'skipped',
        distance_m: 0.29,
        acceptance_tolerance_m: 0.3,
        reapproach_attempts: 1,
        coarse_completed: false,
      }),
      event(9, 'task.waypoint_reached', 9, '2026-09-06T08:00:08+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 2, x: 2, y: 3 },
      }),
    ],
  })

  assert.equal(timeline.length, 1)
  assert.equal(timeline[0].title, '2号点验收完成')
  assert.match(timeline[0].detail, /轻量到达（跳过定位校正）/)
  assert.match(timeline[0].detail, /偏差 0\.29m/)
  assert.match(timeline[0].detail, /验收半径 0\.30m/)
  assert.match(timeline[0].detail, /追加靠近 1 次/)
})

test('current coarse fallback after full correction is not labeled as a legacy strategy', () => {
  const timeline = buildTaskExecutionTimeline({
    state: 'running',
    route_snapshot: { waypoints: [{ map_point_number: 4 }] },
    events: [
      event(8, 'task.arrival_degraded_accepted', 8, '2026-09-15T08:00:07+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 4, x: 4, y: 5 },
        distance_m: 0.40,
        fine_tolerance_m: 0.3,
        coarse_tolerance_m: 0.5,
        reapproach_attempts: 1,
      }),
      event(9, 'task.arrival_confirmed', 9, '2026-09-15T08:00:08+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 4, x: 4, y: 5 },
        arrival_mode: 'full_correction',
        localization_correction: 'completed',
        distance_m: 0.40,
        acceptance_tolerance_m: 0.5,
        reapproach_attempts: 1,
        coarse_completed: true,
      }),
    ],
  })

  assert.equal(timeline[0].title, '4号点一次细靠近后按 0.50m 放行')
  assert.match(timeline[0].detail, /偏差 0\.40m/)
  assert.match(timeline[0].detail, /追加靠近 1 次/)
  assert.equal(timeline[1].title, '4号点验收完成（0.50m 降级完成）')
  assert.match(timeline[1].detail, /完整定位校正/)
  assert.match(timeline[1].detail, /一次细靠近后按 0\.50m 完成/)
  assert.doesNotMatch(timeline[1].detail, /旧策略/)
})

test('legacy coarse completion without full correction keeps the historical marker', () => {
  const timeline = buildTaskExecutionTimeline({
    state: 'completed',
    route_snapshot: { waypoints: [{ map_point_number: 2 }] },
    events: [
      event(8, 'task.arrival_confirmed', 8, '2026-09-06T08:00:08+08:00', {
        execution_waypoint_index: 0,
        waypoint: { map_point_number: 2, x: 2, y: 3 },
        distance_m: 0.41,
        acceptance_tolerance_m: 0.5,
        coarse_completed: true,
      }),
    ],
  })

  assert.equal(timeline[0].title, '2号点验收完成（粗范围完成）')
  assert.match(timeline[0].detail, /旧策略：按 0\.50m 粗范围完成/)
})

test('separates Nav2 stop, one-second zero confirmation, and stationary correction', () => {
  const timeline = buildTaskExecutionTimeline({
    state: 'running',
    route_snapshot: { waypoints: [{ map_point_number: 2 }] },
    events: [
      event(1, 'task.arrival_nav2_stopping', 4, '2026-09-06T08:00:04+08:00', {
        execution_waypoint_index: 0,
        stop_confirmation_seconds: 1,
        elapsed_seconds: 0,
      }),
      event(2, 'task.arrival_zero_confirming', 4, '2026-09-06T08:00:04.5+08:00', {
        execution_waypoint_index: 0,
        stop_confirmation_seconds: 1,
        elapsed_seconds: 0.5,
      }),
      event(3, 'task.arrival_zero_confirmed', 4, '2026-09-06T08:00:05.5+08:00', {
        execution_waypoint_index: 0,
        stop_confirmation_seconds: 1,
        elapsed_seconds: 1.5,
      }),
      event(4, 'task.arrival_correcting', 4, '2026-09-06T08:00:05.6+08:00', {
        execution_waypoint_index: 0,
      }),
    ],
  })

  assert.deepEqual(timeline.map(item => item.title), [
    '2号点等待 Nav2 停止',
    '2号点零速确认中',
    '2号点零速已确认',
    '2号点静止定位校正',
  ])
  assert.match(timeline[1].detail, /连续零速 1\.0s/)
  assert.match(timeline[2].detail, /本阶段 1\.5s/)
})

test('shows a request failure before an execution exists', () => {
  const timeline = buildTaskExecutionTimeline(null, {
    requestedAt: Date.parse('2026-09-06T08:00:00+08:00'),
    requestError: '网络不可用',
  })

  assert.equal(timeline.length, 1)
  assert.equal(timeline[0].type, 'error')
  assert.equal(timeline[0].title, '预演下发失败')
  assert.equal(timeline[0].detail, '网络不可用')
})

test('identifies active and terminal execution states', () => {
  assert.equal(taskExecutionIsActive({ state: 'running' }), true)
  assert.equal(taskExecutionIsActive({ state: 'paused' }), true)
  assert.equal(taskExecutionIsActive({ state: 'completed' }), false)
  assert.equal(taskExecutionIsActive({ state: 'failed' }), false)
})

test('adds persisted diagnostic logs and a missing terminal failure event', () => {
  const timeline = buildTaskExecutionTimeline({
    id: 'execution-from-guard-duty',
    state: 'failed',
    created_at: '2026-09-14T21:40:00+08:00',
    finished_at: '2026-09-14T21:41:00+08:00',
    failure_code: 'ROBOT_STANDUP_FAILED',
    failure_message: 'recovery requires a confirmed stop',
    events: [
      event(1, 'task.created', 0, '2026-09-14T21:40:00+08:00'),
    ],
    system_logs: [{
      id: 'log-1',
      level: 'WARNING',
      event_code: 'task.safe_hold',
      message: '等待安全条件满足',
      occurred_at: '2026-09-14T21:40:30+08:00',
      repeat_count: 3,
      data: { reason_code: 'ROBOT_NOT_STOPPED' },
    }],
  })

  assert.deepEqual(timeline.map(item => item.title), [
    '任务执行已创建',
    '等待安全条件满足',
    '任务执行失败',
  ])
  assert.match(timeline[1].detail, /task\.safe_hold/)
  assert.match(timeline[1].detail, /重复 3 次/)
  assert.equal(timeline[2].detail, 'ROBOT_STANDUP_FAILED · recovery requires a confirmed stop')
})

test('expands startup actions and navigation modules in drill records', () => {
  const timeline = buildTaskExecutionTimeline({
    id: 'execution-progress-detail',
    state: 'running',
    created_at: '2026-09-16T08:00:00+08:00',
    route_snapshot: { waypoints: [{ map_point_number: 2 }] },
    events: [],
    system_logs: [
      {
        id: 'start-progress',
        level: 'INFO',
        event_code: 'task.start.progress',
        message: '启动任务：智能初始化定位：验证 RTK 固定解',
        occurred_at: '2026-09-16T08:00:01+08:00',
        data: {
          startup_progress: {
            current_action: '智能初始化定位：验证 RTK 固定解',
            next_action: '定位接管后应用航段策略并下发首航点',
            actions: [
              { label: '检查导航栈与安全状态', status: 'completed' },
              { label: '校验地图、边界与路线', status: 'completed' },
              { label: '初始化定位并确认主定位源', status: 'in_progress' },
            ],
          },
        },
      },
      {
        id: 'navigation-progress',
        level: 'INFO',
        event_code: 'navigation.progress',
        message: '导航至2号点：路径跟踪',
        waypoint_index: 0,
        occurred_at: '2026-09-16T08:00:02+08:00',
        data: {
          navigation_progress: {
            progress: { completed_waypoints: 0, total_waypoints: 1, distance_remaining_m: 3.25 },
            modules: {
              global_planner: 'theta_star',
              local_controller: 'mppi',
              smoother: 'savitzky_golay',
              goal_checker: 'general_goal_checker',
              localization_mode: 'ukf',
            },
            strategy: {
              speed_level: 'micro',
              speed_profile: 'cruise',
              configured_linear_limit_mps: 0.3,
              detour_enabled: true,
              collision_slowdown_enabled: true,
              collision_stop_enabled: true,
              arrival_policy: 'stop_and_confirm',
              xy_goal_tolerance_m: 0.3,
            },
          },
        },
      },
    ],
  })

  assert.equal(timeline[0].title, '启动任务：智能初始化定位：验证 RTK 固定解')
  assert.match(timeline[0].detail, /已完成：检查导航栈与安全状态、校验地图、边界与路线/)
  assert.match(timeline[0].detail, /当前：初始化定位并确认主定位源/)
  assert.match(timeline[0].detail, /下一步：定位接管后应用航段策略并下发首航点/)
  assert.equal(timeline[1].title, '导航至2号点：路径跟踪')
  assert.match(timeline[1].detail, /进度 0\/1/)
  assert.match(timeline[1].detail, /距航点 3\.25m/)
  assert.match(timeline[1].detail, /执行模块：ThetaStar → MPPI（FollowPath） → Savitzky-Golay → 通用 GoalChecker/)
  assert.match(timeline[1].detail, /定位策略：UKF/)
  assert.match(timeline[1].detail, /微速、巡航、线速度上限 0\.30m\/s、绕行开启、碰撞减速开启、硬急停开启/)
})
