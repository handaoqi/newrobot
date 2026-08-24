<script setup>
import { computed, onBeforeUnmount } from 'vue'

import LiveVideoPlayer from './LiveVideoPlayer.vue'
import { useSharedVideoStream } from '../composables/useSharedVideoStream'
import { useToast } from '../composables/useToast'

const {
  playUrls,
  robotId,
  available,
  objectFit,
  streamUnavailable,
  markSharedVideoUnavailable,
  clearSharedVideoSource,
} = useSharedVideoStream()
const { showToast } = useToast()

const playerAvailable = computed(() => available.value && !streamUnavailable.value)

function handleNotice({ message, variant }) {
  showToast(message, variant ? { variant } : undefined)
}

onBeforeUnmount(clearSharedVideoSource)
</script>

<template>
  <Teleport to="#shared-video-slot">
    <LiveVideoPlayer
      :play-urls="playUrls"
      :robot-id="robotId"
      :available="playerAvailable"
      :object-fit="objectFit"
      @notice="handleNotice"
      @stream-error="markSharedVideoUnavailable"
    >
      <template #empty>
        <div class="shared-video-empty">
          <strong>视频暂不可用</strong>
          <span>当前机器狗未提供可用视频流</span>
        </div>
      </template>
    </LiveVideoPlayer>
  </Teleport>
</template>
