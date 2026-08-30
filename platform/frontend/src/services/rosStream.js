/**
 * Decoding for zones 3 and 4 of the replay console.
 *
 * The 3D scene is rendered by the embedded host, but the charts and the status
 * panel are ours, so we decode the data ourselves. Both sources end up in the
 * same place -- CDR bytes plus a ROS 2 message definition in text form -- so
 * they share one decoder and one series store, and the panels above never learn
 * which source they are looking at.
 *
 *   本地 Bag: @mcap/core reads the operator's File directly. File.slice() is a
 *             random-access read, which is what the indexed reader needs, so
 *             there is no upload, no Range server and no second file picker.
 *   实时:     @foxglove/ws-protocol against the NX foxglove_bridge, which
 *             advertises the same schema text in its channel list.
 *
 * Custom messages (robots_dog_msgs/UniRtkPvh, localization/ScanMatchingStatus,
 * robots_dog_msgs/Localization) decode because both transports carry the message
 * definition with the data. That is exactly why the errata insists on MCAP over
 * .db3: a bare .db3 carries no definitions and these three topics -- the ones
 * that matter for drift diagnosis -- cannot be read out of it at all.
 */
import { parse as parseRosDefinition } from '@foxglove/rosmsg'
import { MessageReader } from '@foxglove/rosmsg2-serialization'
import { FoxgloveClient } from '@foxglove/ws-protocol'
import { McapIndexedReader } from '@mcap/core'

/** Samples kept per signal in live mode, ~10 minutes of a 20 Hz topic. */
const LIVE_SAMPLE_CAP = 12_000

/**
 * foxglove_bridge 3.4.3 negotiates `foxglove.sdk.v1`; most documentation (and
 * the constant in @foxglove/ws-protocol) still says `foxglove.websocket.v1`, and
 * offering only that gets a 400 back. Offer both and let the server choose.
 */
const SUBPROTOCOLS = ['foxglove.sdk.v1', FoxgloveClient.SUPPORTED_SUBPROTOCOL]

/** Read a browser File through the interface @mcap/core expects. */
class FileReadable {
  constructor(file) {
    this.file = file
  }

  async size() {
    return BigInt(this.file.size)
  }

  async read(offset, length) {
    const start = Number(offset)
    return new Uint8Array(await this.file.slice(start, start + Number(length)).arrayBuffer())
  }
}

/**
 * A decoder for one channel.
 *
 * Returns undefined for anything that is not CDR with a ROS 2 definition --
 * JSON channels, protobuf channels, a schema-less channel -- rather than
 * throwing, so one odd topic cannot take the whole recording down with it.
 */
function makeDecoder({ messageEncoding, schema }) {
  if (messageEncoding !== 'cdr' || !schema) return undefined
  const encoding = schema.encoding ?? 'ros2msg'
  if (encoding !== 'ros2msg' && encoding !== 'ros2idl') return undefined
  const text =
    typeof schema.data === 'string' ? schema.data : new TextDecoder().decode(schema.data)
  const definitions = parseRosDefinition(text, {
    ros2: true,
    skipTypeFixup: encoding === 'ros2idl',
  })
  const reader = new MessageReader(definitions)
  return (bytes) => reader.readMessage(bytes)
}

/** Walk a dotted path, e.g. "pose.pose.position.x". */
function read(message, path) {
  let value = message
  for (const key of path) {
    if (value == null) return undefined
    value = value[key]
  }
  return value
}

/**
 * Turn a signal definition into an extractor.
 *
 * `pick` may be a dotted string or a function; the function form is what makes
 * derived signals possible -- yaw out of a quaternion, or the |cmd_vel| the
 * safety layer actually let through -- without this module knowing what they
 * mean.
 */
function extractor(signal) {
  if (typeof signal.pick === 'function') return signal.pick
  const path = signal.pick.split('.')
  return (message) => read(message, path)
}

function appendSample(series, time, value) {
  // Non-finite samples would silently break both the chart's axis scaling and
  // any min/max readout, and they carry no information, so drop them here.
  if (typeof value !== 'number' || !Number.isFinite(value)) return
  series.t.push(time)
  series.v.push(value)
}

/**
 * Last sample at or before `time`.
 *
 * Zone 4 reads discrete state (indoor/outdoor, fix quality) this way, so
 * interpolating would be wrong: it must report the value that was actually
 * published, held until the next message.
 */
export function valueAt(series, time) {
  const { t, v } = series
  if (!t.length || time < t[0]) return undefined
  let low = 0
  let high = t.length - 1
  while (low < high) {
    const mid = (low + high + 1) >> 1
    if (t[mid] <= time) low = mid
    else high = mid - 1
  }
  return v[low]
}

/** Yaw in degrees from a ROS quaternion, for heading traces. */
export function yawDegrees(q) {
  if (!q) return undefined
  const siny = 2 * (q.w * q.z + q.x * q.y)
  const cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
  return (Math.atan2(siny, cosy) * 180) / Math.PI
}

function emptyStore(signals) {
  const series = new Map()
  for (const signal of signals) series.set(signal.key, { t: [], v: [] })
  return series
}

/**
 * Read an entire MCAP file into per-signal series.
 *
 * Everything is read up front rather than streamed alongside playback. A patrol
 * bag is tens of megabytes and the signals we chart are scalars, so the whole
 * set costs a few seconds once and then every seek is an array lookup -- which
 * is what keeps our charts locked to the host's playhead instead of chasing it.
 */
export async function openBagSource(file, signals, { onProgress } = {}) {
  const reader = await McapIndexedReader.Initialize({ readable: new FileReadable(file) })

  const wanted = new Map()
  for (const signal of signals) {
    if (!wanted.has(signal.topic)) wanted.set(signal.topic, [])
    wanted.get(signal.topic).push({ ...signal, read: extractor(signal) })
  }

  const series = emptyStore(signals)
  const decoders = new Map()
  const counts = new Map()
  const present = new Set()
  for (const channel of reader.channelsById.values()) {
    if (wanted.has(channel.topic)) present.add(channel.topic)
  }

  const startNs = reader.statistics?.messageStartTime ?? 0n
  const endNs = reader.statistics?.messageEndTime ?? 0n
  const start = Number(startNs) / 1e9
  const end = Number(endNs) / 1e9
  const span = Math.max(1e-9, end - start)

  let seen = 0
  for await (const message of reader.readMessages({ topics: [...present] })) {
    const channel = reader.channelsById.get(message.channelId)
    if (!channel) continue
    if (!decoders.has(message.channelId)) {
      const schema = channel.schemaId ? reader.schemasById.get(channel.schemaId) : undefined
      decoders.set(message.channelId, makeDecoder({ ...channel, schema }))
    }
    const decode = decoders.get(message.channelId)
    if (!decode) continue

    let decoded
    try {
      decoded = decode(message.data)
    } catch {
      // A single malformed message must not abort the read; the rest of the
      // recording is still worth charting.
      continue
    }
    const time = Number(message.logTime) / 1e9 - start
    for (const signal of wanted.get(channel.topic)) {
      appendSample(series.get(signal.key), time, signal.read(decoded))
    }
    counts.set(channel.topic, (counts.get(channel.topic) ?? 0) + 1)
    if (onProgress && ++seen % 2000 === 0) {
      onProgress(Math.min(1, (Number(message.logTime) / 1e9 - start) / span))
    }
  }
  onProgress?.(1)

  return {
    kind: 'bag',
    /** Seconds from the start of the recording; the host playhead maps onto it. */
    duration: span,
    startWallClock: start,
    series,
    counts,
    /** Topics a signal asked for that the recording does not contain. */
    missing: [...wanted.keys()].filter((topic) => !present.has(topic)),
    close() {},
  }
}

/**
 * Subscribe to the live bridge and append to the same series shape.
 *
 * Read-only by construction: this opens no advertisement and sends no client
 * publish, so nothing here can put a message on the dog's network.
 */
export function openLiveSource(url, signals, { onUpdate, onError } = {}) {
  const wanted = new Map()
  for (const signal of signals) {
    if (!wanted.has(signal.topic)) wanted.set(signal.topic, [])
    wanted.get(signal.topic).push({ ...signal, read: extractor(signal) })
  }

  const series = emptyStore(signals)
  const counts = new Map()
  const decoders = new Map()
  const topicBySubscription = new Map()
  let started

  const socket = new WebSocket(url, SUBPROTOCOLS)
  const client = new FoxgloveClient({ ws: socket })

  client.on('error', (error) => onError?.(error))
  client.on('advertise', (channels) => {
    for (const channel of channels) {
      if (!wanted.has(channel.topic)) continue
      const id = client.subscribe(channel.id)
      topicBySubscription.set(id, channel.topic)
      decoders.set(id, makeDecoder(channel))
    }
  })
  client.on('unadvertise', (ids) => {
    for (const [subscription, topic] of topicBySubscription) {
      if (!ids.includes(topic)) continue
      client.unsubscribe(subscription)
      topicBySubscription.delete(subscription)
    }
  })
  client.on('message', ({ subscriptionId, timestamp, data }) => {
    const topic = topicBySubscription.get(subscriptionId)
    const decode = decoders.get(subscriptionId)
    if (!topic || !decode) return
    let decoded
    try {
      decoded = decode(new Uint8Array(data.buffer, data.byteOffset, data.byteLength))
    } catch {
      return
    }
    const at = Number(timestamp) / 1e9
    started ??= at
    for (const signal of wanted.get(topic)) {
      const target = series.get(signal.key)
      appendSample(target, at - started, signal.read(decoded))
      if (target.t.length > LIVE_SAMPLE_CAP) {
        target.t.shift()
        target.v.shift()
      }
    }
    counts.set(topic, (counts.get(topic) ?? 0) + 1)
    onUpdate?.(at - started)
  })

  return {
    kind: 'live',
    series,
    counts,
    missing: [],
    close() {
      try {
        client.close()
      } catch {
        // Already closed, or never opened -- nothing to unwind.
      }
    },
  }
}
