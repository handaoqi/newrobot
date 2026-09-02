import { defaultHomePath, defaultModuleIds, getModule } from '../modules/index.js'

export const CONFIG_SCHEMA_VERSION = 1
export const RELEASE_VERSION = 'v1.0.0'
export const RUNTIME_CONFIG_URL = '/modules.json'

export function validateRuntimeConfig(value) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    return { ok: false, message: '模块配置必须是 JSON 对象。' }
  }
  if (value.schema_version !== CONFIG_SCHEMA_VERSION) {
    return { ok: false, message: `模块配置 schema_version 不匹配（需要 ${CONFIG_SCHEMA_VERSION}）。` }
  }
  if (value.release_version !== RELEASE_VERSION) {
    return { ok: false, message: `模块配置版本不匹配（需要 ${RELEASE_VERSION}）。` }
  }
  if (!Array.isArray(value.enabled_modules) || value.enabled_modules.length === 0) {
    return { ok: false, message: '至少需要启用一个业务模块。' }
  }
  const ids = value.enabled_modules
  if (new Set(ids).size !== ids.length || ids.some((id) => typeof id !== 'string' || !getModule(id))) {
    const unknown = ids.find((id) => typeof id !== 'string' || !getModule(id))
    return { ok: false, message: `模块配置包含未知模块：${String(unknown)}` }
  }
  return { ok: true, value: { schema_version: CONFIG_SCHEMA_VERSION, release_version: RELEASE_VERSION, enabled_modules: [...ids] } }
}

export function defaultRuntimeConfig() {
  return { schema_version: CONFIG_SCHEMA_VERSION, release_version: RELEASE_VERSION, enabled_modules: defaultModuleIds() }
}

export async function loadRuntimeConfig({ fetchImpl = globalThis.fetch, url = RUNTIME_CONFIG_URL } = {}) {
  if (typeof fetchImpl !== 'function') throw new Error('当前环境不支持读取模块配置。')
  let response
  try {
    response = await fetchImpl(url, { cache: 'no-store', headers: { Accept: 'application/json' } })
  } catch (error) {
    throw new Error(`模块配置读取失败：${error?.message || '网络错误'}`)
  }
  if (!response.ok) throw new Error(`模块配置读取失败：HTTP ${response.status}`)
  const result = validateRuntimeConfig(await response.json())
  if (!result.ok) throw new Error(result.message)
  return result.value
}

export function homePathForConfig(config) {
  return defaultHomePath(config?.enabled_modules || [])
}

