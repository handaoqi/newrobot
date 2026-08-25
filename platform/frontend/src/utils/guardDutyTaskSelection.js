import { isExecutionActive } from '../services/executionState.js'

function timestamp(value) {
  const parsed = new Date(value || 0).getTime()
  return Number.isFinite(parsed) ? parsed : 0
}

export function guardDutyTaskOptions(tasks, robotId) {
  const matchingTasks = (Array.isArray(tasks) ? tasks : [])
    .filter((task) => task?.enabled && (!robotId || String(task.robot) === String(robotId)))

  if (matchingTasks.length < 2) return matchingTasks

  const [firstTask, ...remainingTasks] = matchingTasks
  const recentExecutedTasks = remainingTasks.filter((task) => task.latest_execution)
  recentExecutedTasks.sort((left, right) => {
    const executionDelta = timestamp(right.latest_execution?.created_at) - timestamp(left.latest_execution?.created_at)
    if (executionDelta) return executionDelta
    return timestamp(right.scheduled_start) - timestamp(left.scheduled_start)
  })
  return [firstTask, ...recentExecutedTasks.slice(0, 9)]
}

export function initialGuardDutyExecution(tasks) {
  const activeExecution = tasks
    .map((task) => task.latest_execution)
    .filter((item) => item && isExecutionActive(item.state))
    .sort((left, right) => timestamp(right.created_at) - timestamp(left.created_at))[0]

  return activeExecution || tasks[0]?.latest_execution || null
}
