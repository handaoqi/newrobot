export const MAP_ZOOM_MIN = 0.5
export const MAP_ZOOM_MAX = 3
export const MAP_ZOOM_STEP = 0.25
export const KEYFRAME_PAGE_SIZE = 50

const RTK_QUALITY_ALIASES = {
  fixed: 'fixed',
  rtk_fixed: 'fixed',
  float: 'float',
  rtk_float: 'float',
  standalone: 'standalone',
  single: 'standalone',
  invalid: 'invalid',
  no_fix: 'invalid',
}

const RTK_QUALITY_LABELS = {
  fixed: '固定解',
  float: '浮点解',
  standalone: '单点解',
  invalid: '无效',
}

/** Normalize RTK solution quality at the route-planner UI boundary. */
export function normalizeRtkQuality(value) {
  if (value === null || value === undefined || value === '') return null
  const normalized = String(value).trim().toLowerCase()
  return RTK_QUALITY_ALIASES[normalized] || 'invalid'
}

export function rtkQualityLabel(value) {
  const quality = normalizeRtkQuality(value)
  return quality ? RTK_QUALITY_LABELS[quality] : '无数据'
}

/**
 * Decode the RTK solution status exposed by UniBestNav.p_sol_status.
 * The controller only treats 0 as a successful solution; other protocol
 * values are intentionally kept as unknown failure codes.
 */
export function rtkSolutionStatusLabel(value) {
  const code = Number(value)
  if (!Number.isFinite(code)) return '未上报'
  return code === 0 ? `解算成功（码 ${code}）` : `解算未通过（码 ${code}）`
}

/** Decode sensor_msgs/NavSatFix.status.status used by /fix. */
export function rtkFixStatusLabel(value) {
  const code = Number(value)
  const labels = {
    '-1': '无定位',
    0: '有效单点定位',
    1: 'SBAS增强定位（RTK浮点映射）',
    2: 'GBAS差分定位（RTK固定映射）',
  }
  if (!Number.isFinite(code)) return '未上报'
  return `${labels[code] || '未知状态'}（码 ${code}）`
}

/**
 * Decode the NovAtel-compatible position-type groups configured by the RTK
 * bridge. Keep the numeric code visible because the protocol does not expose
 * a complete enum description in the ROS message.
 */
export function rtkPositionTypeLabel(value) {
  const code = Number(value)
  if (!Number.isFinite(code)) return '未上报'
  if (code === 0) return `无定位（码 ${code}）`
  if ([48, 49, 50].includes(code)) return `RTK固定类型（码 ${code}）`
  if ([17, 18, 32, 33, 34].includes(code)) return `RTK浮点类型（码 ${code}）`
  return `未知类型（码 ${code}）`
}

export function normalizeRoutePlannerTelemetry(status = {}) {
  const root = status?.status && typeof status.status === 'object' ? status.status : status
  const localization = root?.localization && typeof root.localization === 'object'
    ? root.localization
    : {}
  const rootSensors = root?.sensors && typeof root.sensors === 'object' ? root.sensors : {}
  const localizationSensors = localization?.sensors && typeof localization.sensors === 'object'
    ? localization.sensors
    : {}
  const sensorRtk = rootSensors.rtk || localizationSensors.rtk || {}
  const rawSource = root?.raw_rtk || localization?.raw_rtk || sensorRtk?.raw_rtk || {}
  const rawRtk = { ...sensorRtk, ...(rawSource || {}) }
  const sensors = { ...localizationSensors, ...rootSensors }
  if (Object.keys(rawRtk).length > 0 && !sensors.rtk) sensors.rtk = rawRtk
  return {
    sensors,
    rtk: sensors.rtk || rawRtk,
    rawRtk,
    timeDiagnostics: root?.time_diagnostics || localization?.time_diagnostics || {},
  }
}

export function normalizeHeadingDegrees(value) {
  const numeric = Number(value)
  if (!Number.isFinite(numeric)) return null
  return ((numeric % 360) + 360) % 360
}

export function headingDegreesToRadians(value) {
  const degrees = normalizeHeadingDegrees(value)
  if (degrees === null) return null
  return Number((degrees * Math.PI / 180).toFixed(5))
}

export function clampMapZoom(value) {
  const numeric = Number(value)
  if (!Number.isFinite(numeric)) return 1
  return Math.min(MAP_ZOOM_MAX, Math.max(MAP_ZOOM_MIN, Number(numeric.toFixed(2))))
}

export function paginateKeyframes(samples, page, pageSize = KEYFRAME_PAGE_SIZE) {
  const source = Array.isArray(samples) ? samples : []
  const size = Math.max(1, Number(pageSize) || KEYFRAME_PAGE_SIZE)
  const pageCount = Math.max(1, Math.ceil(source.length / size))
  const currentPage = Math.min(pageCount, Math.max(1, Number(page) || 1))
  const start = (currentPage - 1) * size
  return {
    page: currentPage,
    pageCount,
    start,
    rows: source.slice(start, start + size),
  }
}

export function resolveMapClickAction(mode, initialPoseMode = false) {
  if (initialPoseMode) return 'initial_pose'
  return mode === 'inspect' ? 'inspect' : 'waypoint'
}

/** A retained `normal` label is reusable only while the live stack and samples agree. */
export function localizationReadyForReuse({
  mapMatches,
  navReady,
  localizationStatus,
  initializationVerified,
  localizationSampleStale,
  localizationQualityStale,
} = {}) {
  return mapMatches === true
    && navReady === true
    && localizationStatus === 'normal'
    && initializationVerified === true
    && localizationSampleStale === false
    && localizationQualityStale === false
}

function finitePose(value) {
  if (!value || typeof value !== 'object') return null
  const numeric = field => field === null || field === undefined || field === ''
    ? Number.NaN
    : Number(field)
  const x = numeric(value.x)
  const y = numeric(value.y)
  const yaw = numeric(value.yaw)
  if (![x, y, yaw].every(Number.isFinite)) return null
  const z = numeric(value.z)
  return {
    x,
    y,
    yaw,
    ...(Number.isFinite(z) ? { z } : {}),
  }
}

/** Extract the pose actually accepted/committed by NDT from a command result. */
export function initialPoseCommandOutcome(command, submittedPose = null) {
  const result = command?.result_payload && typeof command.result_payload === 'object'
    ? command.result_payload
    : {}
  const candidate = result.best_ndt_candidate && typeof result.best_ndt_candidate === 'object'
    ? result.best_ndt_candidate
    : null
  const localizedPose = finitePose(result.localized_pose)
  const matchedPose = finitePose(candidate?.matched_pose)
  const bestMatchPose = finitePose(result.best_match_pose)
  return {
    pose: localizedPose || bestMatchPose || matchedPose || finitePose(submittedPose),
    localizedPose,
    matchedPose: bestMatchPose || matchedPose,
    bestNdtCommitted: result.best_ndt_committed === true,
    handoffPending: result.handoff_pending === true,
    matchingError: candidate?.matching_error !== null
      && candidate?.matching_error !== undefined
      && candidate?.matching_error !== ''
      && Number.isFinite(Number(candidate.matching_error))
      ? Number(candidate.matching_error)
      : null,
    inlierFraction: candidate?.inlier_fraction !== null
      && candidate?.inlier_fraction !== undefined
      && candidate?.inlier_fraction !== ''
      && Number.isFinite(Number(candidate.inlier_fraction))
      ? Number(candidate.inlier_fraction)
      : null,
  }
}

export function appendConfirmedInspectionPoint(points, draft, id) {
  const source = Array.isArray(points) ? points : []
  if (!draft?.point || id === null || id === undefined || id === '') return source
  return [
    ...source,
    {
      id,
      point: { ...draft.point },
      sample: draft.sample ? { ...draft.sample } : null,
    },
  ]
}

export function removeConfirmedInspectionPoint(points, id) {
  return (Array.isArray(points) ? points : []).filter((item) => item.id !== id)
}

export function headingBetweenMapPoints(from, to, minimumDistance = 0.05) {
  const fromX = Number(from?.x)
  const fromY = Number(from?.y)
  const toX = Number(to?.x)
  const toY = Number(to?.y)
  if (![fromX, fromY, toX, toY].every(Number.isFinite)) return null
  const dx = toX - fromX
  const dy = toY - fromY
  if (Math.hypot(dx, dy) < minimumDistance) return null
  return Number(Math.atan2(dy, dx).toFixed(5))
}
