export const ASSET_LOAD_RETRY_KEY = 'roamerx:asset-load-retry'

const ASSET_LOAD_ERROR_PATTERNS = [
  /failed to fetch dynamically imported module/i,
  /importing a module script failed/i,
  /unable to preload css/i,
  /loading (?:chunk|css chunk)\b/i,
  /chunkloaderror/i,
]

export function isAssetLoadError(error) {
  const message = String(error?.message || error || '')
  return ASSET_LOAD_ERROR_PATTERNS.some((pattern) => pattern.test(message))
}

function resolveStorage(storage) {
  if (storage !== undefined) return storage
  try {
    return globalThis.sessionStorage
  } catch {
    return null
  }
}

export function claimAssetLoadRetry(storage) {
  const target = resolveStorage(storage)
  if (!target) return false
  try {
    if (target.getItem(ASSET_LOAD_RETRY_KEY)) return false
    target.setItem(ASSET_LOAD_RETRY_KEY, '1')
    return true
  } catch {
    return false
  }
}

export function clearAssetLoadRetry(storage) {
  const target = resolveStorage(storage)
  if (!target) return
  try {
    target.removeItem(ASSET_LOAD_RETRY_KEY)
  } catch {
    // A restricted storage implementation must not break normal navigation.
  }
}
