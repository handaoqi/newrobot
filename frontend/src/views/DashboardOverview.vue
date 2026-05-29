<script setup>
import Hls from 'hls.js'
import mpegts from 'mpegts.js'
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'

import AppToast from '../components/AppToast.vue'
import { useToast } from '../composables/useToast'
import { fetchOverview, fetchRobots } from '../services/api'

const overview = ref(null)
const robots = ref([])
const loading = ref(true)
const speakerText = ref('您好，这里禁止自行车长时间停放，请尽快驶离指定区域，感谢配合。')
const videoRef = ref(null)
let flvPlayer = null
let hlsPlayer = null
const { toastMessage, visible, showToast } = useToast()

const quickTexts = [
  {
    label: '重点路段',
    text: '您好，当前区域为巡检重点路段，请勿长时间占道停留。',
  },
  {
    label: '驶离提醒',
    text: '您好，请将车辆停放至指定区域，共同保持通道顺畅。',
  },
  {
    label: '注意避让',
    text: '您好，系统检测到现场存在安全风险，请注意避让并配合引导。',
  },
]

const eventImages = ['/images/event-1.jpg', '/images/event-2.jpg', '/images/event-3.jpg']
const latestRobot = computed(() => overview.value?.latest_robot || null)
const liveEvent = computed(() => overview.value?.live_event || null)
const liveImage = computed(() => liveEvent.value?.snapshot_url || '/images/live-feed.jpg')
const livePlayUrls = computed(() => latestRobot.value?.play_urls || {})
const hasLiveStream = computed(() => Boolean(livePlayUrls.value.flv || livePlayUrls.value.hls))

function eventThumbStyle(index) {
  const event = latestRobot.value?.recent_events?.[index]
  const image = event?.snapshot_url || eventImages[index % eventImages.length]
  return {
    backgroundImage: `linear-gradient(rgba(6, 16, 28, 0.08), rgba(6, 16, 28, 0.18)), url(${image})`,
  }
}

function formatEventTime(value) {
  if (!value) return '--'
  const date = new Date(value)
  if (Number.isNaN(date.getTime())) return value
  return date.toLocaleString('zh-CN', {
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
  })
}

function setSpeakerText(text) {
  speakerText.value = text
  showToast('已切换喊话模板')
}

function beginSpeak() {
  showToast('演示状态：现场喊话已开始播放')
}

function previewVoice() {
  showToast('演示状态：语音预览已生成')
}

function emergencyStop() {
  showToast('演示状态：已触发紧急停止指令')
}

function destroyVideoPlayers() {
  if (flvPlayer) {
    flvPlayer.destroy()
    flvPlayer = null
  }
  if (hlsPlayer) {
    hlsPlayer.destroy()
    hlsPlayer = null
  }
}

async function setupLivePlayer() {
  await nextTick()
  destroyVideoPlayers()
  const element = videoRef.value
  if (!element || !hasLiveStream.value) return

  const { flv, hls } = livePlayUrls.value
  if (flv && mpegts.getFeatureList().mseLivePlayback) {
    flvPlayer = mpegts.createPlayer({
      type: 'flv',
      isLive: true,
      url: flv,
    })
    flvPlayer.attachMediaElement(element)
    flvPlayer.load()
    flvPlayer.play().catch(() => {})
    return
  }

  if (hls && Hls.isSupported()) {
    hlsPlayer = new Hls({ lowLatencyMode: true })
    hlsPlayer.loadSource(hls)
    hlsPlayer.attachMedia(element)
    hlsPlayer.on(Hls.Events.MANIFEST_PARSED, () => {
      element.play().catch(() => {})
    })
    return
  }

  if (hls) {
    element.src = hls
    element.play().catch(() => {})
  }
}

onMounted(async () => {
  try {
    const [overviewData, robotData] = await Promise.all([fetchOverview(), fetchRobots()])
    overview.value = overviewData
    robots.value = robotData
  } finally {
    loading.value = false
  }

  setupLivePlayer()
})

onBeforeUnmount(() => {
  destroyVideoPlayers()
})

watch(livePlayUrls, setupLivePlayer)
</script>

<template>
  <section v-if="!loading" class="page-grid">
    <div class="content-column">
      <section class="top-summary">
        <article class="status-pill online">
          <span class="dot"></span>
          设备在线 {{ overview.header.device_code }}
        </article>
        <article class="status-pill warning">今日告警 {{ overview.header.today_alerts }} 条</article>
        <article class="status-pill">当前区域 {{ overview.header.current_location }}</article>
      </section>

      <section class="panel video-panel">
        <div class="panel-head">
          <div>
            <h3>实时视频流监控</h3>
            <p>支持平台方查看机器人前端画面、检测框与事件识别状态</p>
          </div>
          <span class="panel-badge">Live Stream</span>
        </div>

        <div class="video-stage">
          <video
            v-if="hasLiveStream"
            ref="videoRef"
            class="video-source"
            muted
            playsinline
            autoplay
            controls
          ></video>
          <img v-else class="video-source" :src="liveImage" alt="实时视频画面" />

          <div class="video-overlay">
            <div class="overlay-card">
              <strong>巡检位置</strong>
              <span>{{ latestRobot?.location }}</span>
            </div>
            <div class="overlay-card" v-if="latestRobot?.stream_id">
              <strong>视频流</strong>
              <span>{{ latestRobot.stream_id }}</span>
            </div>
          </div>

        </div>

        <div class="video-footer">
          <div class="footer-card">
            <strong>今日巡检时长</strong>
            <span>{{ latestRobot?.patrol_duration_minutes || 0 }} 分钟</span>
          </div>
          <div class="footer-card wide">
            <strong>当前巡检区域</strong>
            <span>{{ latestRobot?.area }}</span>
          </div>
          <div class="footer-card">
            <strong>设备电量</strong>
            <span>{{ latestRobot?.battery_level }}%</span>
          </div>
          <button class="danger-btn" @click="emergencyStop">紧急停止</button>
        </div>
      </section>

      <section class="board-grid">
        <article class="panel compact-panel">
          <div class="panel-head">
            <div>
              <h3>运行概况</h3>
              <p>今日巡检与告警统计</p>
            </div>
          </div>
          <div class="metrics-grid">
            <div class="metric-card">
              <strong>{{ overview.summary.online_robot_count }}</strong>
              <span>在线机器人</span>
            </div>
            <div class="metric-card">
              <strong>{{ overview.summary.pending_event_count }}</strong>
              <span>待处理事件</span>
            </div>
            <div class="metric-card">
              <strong>{{ overview.summary.resolved_event_count }}</strong>
              <span>已处理事件</span>
            </div>
            <div class="metric-card">
              <strong>{{ overview.summary.today_alert_count }}</strong>
              <span>今日告警</span>
            </div>
          </div>
        </article>

        <article class="panel compact-panel">
          <div class="panel-head">
            <div>
              <h3>设备列表</h3>
              <p>支持在多台机器人之间快速切换与查看状态</p>
            </div>
          </div>
          <div class="robot-list">
            <div v-for="robot in robots" :key="robot.id" class="robot-item">
              <div>
                <strong>{{ robot.name }}</strong>
                <span>{{ robot.location }}</span>
              </div>
              <div class="robot-side">
                <span :class="['robot-status', robot.status]">{{ robot.status_label }}</span>
                <span>{{ robot.battery_level }}%</span>
              </div>
            </div>
          </div>
        </article>
      </section>
    </div>

    <aside class="side-column">
      <section class="panel side-panel">
        <div class="panel-head">
          <div>
            <h3>远程控制台</h3>
            <p>支持快速切换播报模板，并同步查看现场联动状态</p>
          </div>
          <span class="panel-badge">Control</span>
        </div>
        <div class="speaker-box">
          <textarea v-model="speakerText"></textarea>
          <div class="quick-actions">
            <button
              v-for="item in quickTexts"
              :key="item.label"
              class="chip"
              :class="{ active: speakerText === item.text }"
              @click="setSpeakerText(item.text)"
            >
              {{ item.label }}
            </button>
          </div>
          <div class="action-row">
            <button class="primary-btn" @click="beginSpeak">开始喊话</button>
            <button class="ghost-btn" @click="previewVoice">语音预览</button>
          </div>
          <div class="mini-row">
            <div class="mini-card">当前音量 {{ latestRobot?.speaker_volume }}%</div>
            <div class="mini-card">喊话链路状态 正常</div>
          </div>
        </div>
      </section>

      <section class="panel side-panel">
        <div class="panel-head">
          <div>
            <h3>历史事件识别</h3>
          </div>
          <span class="panel-badge">History</span>
        </div>
        <div class="event-list">
          <article v-for="(event, index) in latestRobot?.recent_events || []" :key="event.id" class="event-card">
            <div class="event-thumb" :style="eventThumbStyle(index)"></div>
            <div class="event-main">
              <strong>{{ event.title }}</strong>
              <span>{{ event.location }}</span>
              <small>{{ formatEventTime(event.detected_at) }}</small>
            </div>
            <div class="event-side">
              <span :class="['risk-chip', event.status === 'pending' ? event.status : 'review-chip']">
                {{ event.status === 'pending' ? event.status_label : event.review_result_label || '确认违规' }}
              </span>
            </div>
          </article>
        </div>
      </section>
    </aside>

    <AppToast :show="visible" :message="toastMessage" />
  </section>
</template>
