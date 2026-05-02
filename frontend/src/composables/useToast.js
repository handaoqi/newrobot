import { ref } from 'vue'

const toastMessage = ref('')
const visible = ref(false)
let timer = null

export function useToast() {
  function showToast(message) {
    toastMessage.value = message
    visible.value = true
    window.clearTimeout(timer)
    timer = window.setTimeout(() => {
      visible.value = false
    }, 2200)
  }

  return {
    toastMessage,
    visible,
    showToast,
  }
}
