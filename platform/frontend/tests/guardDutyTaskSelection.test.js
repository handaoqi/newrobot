import test from 'node:test'
import assert from 'node:assert/strict'

import {
  guardDutyTaskOptions,
  initialGuardDutyExecution,
} from '../src/utils/guardDutyTaskSelection.js'

const tasks = [
  {
    id: 75,
    name: '最新任务',
    robot: 1,
    enabled: true,
    scheduled_start: '2026-08-25T17:30:54+08:00',
    latest_execution: null,
  },
  {
    id: 62,
    name: '旧的被拒绝任务',
    robot: 1,
    enabled: true,
    scheduled_start: '2026-08-17T23:49:22+08:00',
    latest_execution: {
      id: 'rejected-execution',
      state: 'rejected',
      created_at: '2026-08-17T16:38:04Z',
    },
  },
  {
    id: 51,
    name: '较新执行记录',
    robot: 1,
    enabled: true,
    scheduled_start: '2026-08-03T12:17:11+08:00',
    latest_execution: {
      id: 'completed-execution',
      state: 'completed',
      created_at: '2026-08-18T12:00:00Z',
    },
  },
]

test('guard duty keeps the first configured task first and sorts remaining history by execution time', () => {
  assert.deepEqual(guardDutyTaskOptions(tasks, 1).map((task) => task.id), [75, 51, 62])
})

test('an old rejected execution never replaces a first task that has not run', () => {
  const options = guardDutyTaskOptions(tasks, 1)
  assert.equal(initialGuardDutyExecution(options), null)
})

test('an active execution takes precedence so operators retain its controls', () => {
  const activeTasks = tasks.map((task) => ({ ...task }))
  activeTasks[1] = {
    ...activeTasks[1],
    latest_execution: {
      ...activeTasks[1].latest_execution,
      id: 'running-execution',
      state: 'running',
    },
  }
  assert.equal(initialGuardDutyExecution(guardDutyTaskOptions(activeTasks, 1))?.id, 'running-execution')
})

test('disabled and other-robot tasks are omitted from the selector', () => {
  const extraTasks = [
    ...tasks,
    { id: 80, robot: 1, enabled: false, scheduled_start: '2026-08-26T00:00:00Z' },
    { id: 81, robot: 2, enabled: true, scheduled_start: '2026-08-27T00:00:00Z' },
  ]
  assert.deepEqual(guardDutyTaskOptions(extraTasks, 1).map((task) => task.id), [75, 51, 62])
})
