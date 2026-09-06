/**
 * Keep live video requests on the operator UI origin when the API returns a
 * public absolute URL for the same host.  The cloud API historically returns
 * https://<host>/live/... while the legacy operator UI is also served from
 * http://<host>:8088.  Treating those as cross-origin makes HLS.js/mpegts.js
 * fail before the stream player gets a chance to attach.
 */
export function normalizeLiveVideoUrl(value, origin = '') {
  if (!value) return ''
  try {
    const target = new URL(value, origin || undefined)
    if (origin) {
      const current = new URL(origin)
      if (target.hostname === current.hostname) {
        target.protocol = current.protocol
        target.host = current.host
      }
    }
    return target.toString()
  } catch {
    return String(value)
  }
}

export function normalizeLivePlayUrls(urls, origin = '') {
  const input = urls || {}
  return {
    ...input,
    flv: normalizeLiveVideoUrl(input.flv, origin),
    hls: normalizeLiveVideoUrl(input.hls, origin),
  }
}

