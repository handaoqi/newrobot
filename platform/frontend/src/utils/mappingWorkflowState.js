const ACTIVE_MAPPING_STATES = new Set([
  'command_created',
  'command_published',
  'command_accepted',
  'starting',
  'origin_starting',
  'origin_waiting',
  'origin_locked',
  'slam_starting',
  'slam_warmup',
  'imu_initializing',
  'waiting_first_keyframe',
  'ready_to_map',
  'mapping',
  'recovering',
  'saving',
  'optimizing',
  'packaging',
  'uploading',
  'stopping',
])

const ACTIVE_COMMAND_STATUSES = new Set(['created', 'published', 'accepted', 'executing'])
const ACTIVE_ORIGIN_STATES = new Set([
  'starting',
  'waiting_fix',
  'quality_holding',
  'ready',
  'locked',
  'failed',
])

export function isActiveMappingState(mappingState) {
  return ACTIVE_MAPPING_STATES.has(mappingState)
}

export function canRetryFailedMappingSave({
  commandType = '',
  commandStatus = 'idle',
  uploadedMapId = '',
  failureStepKey = '',
} = {}) {
  if (uploadedMapId) return false
  if (['mapping', 'saving', 'optimizing', 'packaging', 'uploading'].includes(failureStepKey)) {
    return true
  }
  return commandType === 'mapping.save' && ['failed', 'rejected', 'timed_out'].includes(commandStatus)
}

export function hasActiveMappingWorkflow({
  mappingState = 'idle',
  commandStatus = 'idle',
  processAlive = false,
  originState = 'idle',
} = {}) {
  return Boolean(
    processAlive
    || ACTIVE_COMMAND_STATUSES.has(commandStatus)
    || isActiveMappingState(mappingState)
    || ACTIVE_ORIGIN_STATES.has(originState)
  )
}
