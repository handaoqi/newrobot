export function guardDutyLoopCleanupExecutionId(currentLoopExecutionId, visibleExecution) {
  return String(currentLoopExecutionId || visibleExecution?.id || '')
}

export async function clearGuardDutyLoopExecution(sendAction, executionId) {
  if (!executionId) return null
  return sendAction(executionId, 'force-exit')
}
