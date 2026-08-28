export function parseMapDescription(description) {
  if (description && typeof description === 'object' && !Array.isArray(description)) {
    return description
  }
  try {
    if (typeof description === 'string') {
      const value = description.trim()
      if (value.startsWith('{') || value.startsWith('[')) return JSON.parse(value)
    }
  } catch {}
  return { raw: typeof description === 'string' ? description : '' }
}

export function hasRescueMetadata(description) {
  const rescue = parseMapDescription(description).rescue
  if (rescue === true) return true
  return Boolean(
    rescue
    && typeof rescue === 'object'
    && !Array.isArray(rescue)
    && Object.keys(rescue).length > 0,
  )
}
