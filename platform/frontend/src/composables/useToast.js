import { ref } from 'vue'

const toastMessage = ref('')
const toastVariant = ref('default')
const visible = ref(false)
let timer = null

export function useToast() {
  function showToast(message, options = {}) {
    toastMessage.value = message
    toastVariant.value = options.variant || 'default'
    visible.value = true
    window.clearTimeout(timer)
    timer = window.setTimeout(() => {
      visible.value = false
    }, options.duration || 2200)
  }

  return {
    toastMessage,
    toastVariant,
    visible,
    showToast,
  }
}
