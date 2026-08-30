import { compareByLatestExecution, preferredExecutedItem } from './executionSelection.js'
import { isExecutionActive } from '../services/executionState.js'

export function guardDutyTaskOptions(tasks, robotId) {
  const matchingTasks = (Array.isArray(tasks) ? tasks : [])
    .filter((task) => (
      task
      && (task.enabled || isExecutionActive(task.latest_execution?.state))
      && (!robotId || String(task.robot) === String(robotId))
    ))

  return matchingTasks.sort(compareByLatestExecution).slice(0, 10)
}

export function initialGuardDutyExecution(tasks) {
  return preferredExecutedItem(tasks)?.latest_execution || null
}
