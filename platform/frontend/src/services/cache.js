export function createTTLCache({ ttlMs = 30_000, now = () => Date.now() } = {}) {
  const entries = new Map()
  const inFlight = new Map()
  const versions = new Map()

  function get(key, loader) {
    const cached = entries.get(key)
    if (cached && cached.expiresAt > now()) return Promise.resolve(cached.value)
    if (cached) entries.delete(key)
    if (inFlight.has(key)) return inFlight.get(key)

    const version = versions.get(key) || 0
    const pending = Promise.resolve().then(loader).then((value) => {
      if ((versions.get(key) || 0) === version) {
        entries.set(key, { value, expiresAt: now() + ttlMs })
      }
      return value
    }).finally(() => {
      if (inFlight.get(key) === pending) inFlight.delete(key)
    })
    inFlight.set(key, pending)
    return pending
  }

  function invalidate(key) {
    if (key === undefined) {
      const keys = new Set([
        ...entries.keys(),
        ...inFlight.keys(),
        ...versions.keys(),
      ])
      entries.clear()
      inFlight.clear()
      for (const cacheKey of keys) versions.set(cacheKey, (versions.get(cacheKey) || 0) + 1)
      return
    }
    entries.delete(key)
    versions.set(key, (versions.get(key) || 0) + 1)
    inFlight.delete(key)
  }

  return {
    get,
    invalidate,
    clear: () => invalidate(),
    get size() { return entries.size },
    get inFlightSize() { return inFlight.size },
  }
}
