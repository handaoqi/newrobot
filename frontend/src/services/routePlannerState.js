export const MAP_ZOOM_MIN = 0.5
export const MAP_ZOOM_MAX = 3
export const MAP_ZOOM_STEP = 0.25
export const KEYFRAME_PAGE_SIZE = 50

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
