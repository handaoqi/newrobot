export const GUARD_DUTY_LOOP_LEASE_TTL_MS = 10000

function readLease(storage, key) {
  try {
    const lease = JSON.parse(storage.getItem(key) || '{}')
    return {
      ownerId: String(lease.ownerId || ''),
      expiresAt: Number(lease.expiresAt || 0),
    }
  } catch {
    return { ownerId: '', expiresAt: 0 }
  }
}

export function acquireGuardDutyLoopLease(
  storage,
  key,
  ownerId,
  now = Date.now(),
  ttlMs = GUARD_DUTY_LOOP_LEASE_TTL_MS,
) {
  const current = readLease(storage, key)
  if (current.ownerId && current.ownerId !== ownerId && current.expiresAt > now) return false
  storage.setItem(key, JSON.stringify({ ownerId, expiresAt: now + ttlMs }))
  return readLease(storage, key).ownerId === ownerId
}

export function renewGuardDutyLoopLease(
  storage,
  key,
  ownerId,
  now = Date.now(),
  ttlMs = GUARD_DUTY_LOOP_LEASE_TTL_MS,
) {
  const current = readLease(storage, key)
  if (current.ownerId !== ownerId || current.expiresAt <= now) return false
  storage.setItem(key, JSON.stringify({ ownerId, expiresAt: now + ttlMs }))
  return true
}

export function releaseGuardDutyLoopLease(storage, key, ownerId) {
  if (readLease(storage, key).ownerId !== ownerId) return false
  storage.removeItem(key)
  return true
}
