import { isExecutionActive } from '../services/executionState.js'

export function executionTimestamp(value) {
  const parsed = new Date(value || 0).getTime()
  return Number.isFinite(parsed) ? parsed : 0
}

export function compareByLatestExecution(left, right) {
  const leftActive = Boolean(left?.latest_execution && isExecutionActive(left.latest_execution.state))
  const rightActive = Boolean(right?.latest_execution && isExecutionActive(right.latest_execution.state))
  if (leftActive !== rightActive) return rightActive ? 1 : -1
  const executionDelta = executionTimestamp(right?.latest_execution?.created_at)
    - executionTimestamp(left?.latest_execution?.created_at)
  if (executionDelta) return executionDelta
  return executionTimestamp(right?.scheduled_start || right?.created_at)
    - executionTimestamp(left?.scheduled_start || left?.created_at)
}

export function preferredExecutedItem(items) {
  const executed = (Array.isArray(items) ? items : []).filter(item => item?.latest_execution)
  if (!executed.length) return null
  return [...executed].sort(compareByLatestExecution)[0]
}
