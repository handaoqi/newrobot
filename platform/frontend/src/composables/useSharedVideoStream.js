import { computed, readonly, ref } from 'vue'

const playUrls = ref({})
const robotId = ref(null)
const available = ref(false)
const loading = ref(true)
const streamUnavailable = ref(false)
const objectFit = ref('cover')

const sourceKey = computed(() => `${robotId.value || ''}\n${playUrls.value.flv || ''}\n${playUrls.value.hls || ''}`)
const hasSource = computed(() => Boolean(playUrls.value.flv || playUrls.value.hls))

function normalizePlayUrls(value = {}) {
  return {
    flv: value?.flv || '',
    hls: value?.hls || '',
  }
}

function setSharedVideoSource({ playUrls: nextPlayUrls = {}, robotId: nextRobotId = null, available: nextAvailable, loading: nextLoading, objectFit: nextObjectFit = 'cover' } = {}) {
  const normalizedUrls = normalizePlayUrls(nextPlayUrls)
  const nextKey = `${nextRobotId || ''}\n${normalizedUrls.flv}\n${normalizedUrls.hls}`
  if (nextKey !== sourceKey.value) streamUnavailable.value = false
  playUrls.value = normalizedUrls
  robotId.value = nextRobotId
  available.value = nextAvailable ?? Boolean(normalizedUrls.flv || normalizedUrls.hls)
  loading.value = nextLoading ?? !Boolean(normalizedUrls.flv || normalizedUrls.hls)
  objectFit.value = nextObjectFit
}

function markSharedVideoUnavailable() {
  streamUnavailable.value = true
  available.value = false
  loading.value = false
}

function clearSharedVideoSource() {
  playUrls.value = {}
  robotId.value = null
  available.value = false
  loading.value = true
  streamUnavailable.value = false
  objectFit.value = 'cover'
}

export function useSharedVideoStream() {
  return {
    playUrls: readonly(playUrls),
    robotId: readonly(robotId),
    available: readonly(available),
    loading: readonly(loading),
    objectFit: readonly(objectFit),
    streamUnavailable: readonly(streamUnavailable),
    sourceKey: readonly(sourceKey),
    hasSource: readonly(hasSource),
    setSharedVideoSource,
    markSharedVideoUnavailable,
    clearSharedVideoSource,
  }
}
