export const SNAPSHOT_CLIP_DURATION_MS = 3000

export function captureFileStamp(date = new Date()) {
  const pad = (value, size = 2) => String(value).padStart(size, '0')
  return `${date.getFullYear()}${pad(date.getMonth() + 1)}${pad(date.getDate())}-${pad(date.getHours())}${pad(date.getMinutes())}${pad(date.getSeconds())}`
}

export function captureExtensionForMime(mimeType, fallback = 'webm') {
  const type = String(mimeType || '')
  if (type.includes('mp4')) return 'mp4'
  if (type.includes('webm')) return 'webm'
  if (type.includes('jpeg')) return 'jpg'
  if (type.includes('png')) return 'png'
  return fallback
}

export function pickMediaRecorderMimeType(MediaRecorderCtor = globalThis.MediaRecorder) {
  const candidates = [
    'video/webm;codecs=vp9',
    'video/webm;codecs=vp8',
    'video/webm',
    'video/mp4',
  ]
  if (typeof MediaRecorderCtor?.isTypeSupported !== 'function') return ''
  return candidates.find((type) => MediaRecorderCtor.isTypeSupported(type)) || ''
}

export async function captureVideoFrameBlob(video, {
  type = 'image/jpeg',
  quality = 0.92,
  createCanvas = () => globalThis.document.createElement('canvas'),
} = {}) {
  if (!video || video.readyState < 2 || !video.videoWidth || !video.videoHeight) {
    throw new Error('当前没有可截图的画面')
  }
  const canvas = createCanvas()
  canvas.width = video.videoWidth
  canvas.height = video.videoHeight
  const context = canvas.getContext('2d')
  if (!context) throw new Error('截图生成失败')
  try {
    context.drawImage(video, 0, 0, canvas.width, canvas.height)
  } catch {
    throw new Error('当前视频不允许截图')
  }
  return new Promise((resolve, reject) => {
    canvas.toBlob((blob) => {
      if (!blob) reject(new Error('截图生成失败'))
      else resolve(blob)
    }, type, quality)
  })
}

function captureElementStream(video) {
  if (typeof video?.captureStream === 'function') return video.captureStream()
  if (typeof video?.mozCaptureStream === 'function') return video.mozCaptureStream()
  throw new Error('当前浏览器不支持视频片段截取')
}

export async function recordVideoElementClip(video, {
  durationMs = SNAPSHOT_CLIP_DURATION_MS,
  MediaRecorderCtor = globalThis.MediaRecorder,
  wait = (ms) => new Promise((resolve) => setTimeout(resolve, ms)),
  onRecorder = null,
} = {}) {
  if (!video) throw new Error('当前没有可录制的画面')
  if (typeof MediaRecorderCtor !== 'function') throw new Error('当前浏览器不支持视频片段截取')
  const stream = captureElementStream(video)
  if (!stream) throw new Error('无法从当前画面截取视频')
  const mimeType = pickMediaRecorderMimeType(MediaRecorderCtor)
  const recorder = mimeType
    ? new MediaRecorderCtor(stream, { mimeType })
    : new MediaRecorderCtor(stream)
  onRecorder?.(recorder)
  const chunks = []
  recorder.ondataavailable = (event) => {
    if (event.data?.size) chunks.push(event.data)
  }
  const stopped = new Promise((resolve, reject) => {
    recorder.onerror = () => reject(new Error('视频片段录制失败'))
    recorder.onstop = () => {
      const type = recorder.mimeType || mimeType || 'video/webm'
      if (!chunks.length) {
        reject(new Error('视频片段为空'))
        return
      }
      resolve(new Blob(chunks, { type }))
    }
  })
  recorder.start(250)
  try {
    await wait(durationMs)
  } finally {
    if (recorder.state !== 'inactive') recorder.stop()
  }
  return stopped
}

export function downloadBlob(blob, filename, {
  document: doc = globalThis.document,
  URLCtor = globalThis.URL,
} = {}) {
  const url = URLCtor.createObjectURL(blob)
  const link = doc.createElement('a')
  link.href = url
  link.download = filename
  link.rel = 'noopener'
  doc.body.appendChild(link)
  link.click()
  link.remove()
  URLCtor.revokeObjectURL(url)
}
