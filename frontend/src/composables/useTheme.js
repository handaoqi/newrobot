import { computed, ref } from 'vue'

const STORAGE_KEY = 'inspection_theme'
const theme = ref('light')

function applyTheme(nextTheme) {
  theme.value = nextTheme
  document.documentElement.dataset.theme = nextTheme
  localStorage.setItem(STORAGE_KEY, nextTheme)
}

export function initTheme() {
  const saved = localStorage.getItem(STORAGE_KEY)
  applyTheme(saved === 'dark' ? 'dark' : 'light')
}

export function useTheme() {
  const isDark = computed(() => theme.value === 'dark')
  const toggleLabel = computed(() => (isDark.value ? '切换浅色' : '切换深色'))

  function toggleTheme() {
    applyTheme(isDark.value ? 'light' : 'dark')
  }

  return {
    theme,
    isDark,
    toggleLabel,
    toggleTheme,
  }
}
