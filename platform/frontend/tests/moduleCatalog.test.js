import assert from 'node:assert/strict'
import fs from 'node:fs'
import test from 'node:test'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { MODULES, STABLE_MODULE_IDS, defaultHomePath, navigationItems } from '../src/modules/index.js'
import { defaultRuntimeConfig, validateRuntimeConfig } from '../src/services/runtimeConfig.js'

const frontendRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..')

test('stable module catalog has unique IDs and routes', () => {
  assert.equal(new Set(MODULES.map((item) => item.id)).size, MODULES.length)
  assert.equal(new Set(MODULES.map((item) => item.path)).size, MODULES.length)
  assert.deepEqual(STABLE_MODULE_IDS, defaultRuntimeConfig().enabled_modules)
  assert.equal(defaultHomePath(STABLE_MODULE_IDS), '/dashboard/overview')
  for (const module of MODULES) for (const recommendation of module.recommended) assert.ok(STABLE_MODULE_IDS.includes(recommendation))
})

test('catalog contains no hard dependencies and navigation follows enabled modules', () => {
  assert.ok(MODULES.every((module) => !module.dependencies || module.dependencies.length === 0))
  const items = navigationItems(['overview', 'tasks', 'maps'])
  assert.deepEqual(items.map((item) => item.moduleId), ['overview', 'tasks'])
  assert.deepEqual(items[1].children.map((item) => item.moduleId), ['tasks', 'maps'])
})

test('runtime config rejects unknown, mismatched, and empty installations', () => {
  assert.equal(validateRuntimeConfig(defaultRuntimeConfig()).ok, true)
  assert.equal(validateRuntimeConfig({ schema_version: 1, release_version: 'v9', enabled_modules: ['overview'] }).ok, false)
  assert.equal(validateRuntimeConfig({ schema_version: 1, release_version: 'v1.0.0', enabled_modules: ['nope'] }).ok, false)
  assert.equal(validateRuntimeConfig({ schema_version: 1, release_version: 'v1.0.0', enabled_modules: [] }).ok, false)
})

test('server keeps replay pages while the container package catalog excludes them', () => {
  const replayIds = ['validation', 'replay-debug', 'tracks']
  assert.deepEqual(MODULES.filter((module) => replayIds.includes(module.id)).map((module) => module.id), replayIds)

  const serverConfig = JSON.parse(fs.readFileSync(path.join(frontendRoot, 'public/modules.json'), 'utf8'))
  assert.deepEqual(replayIds.every((id) => serverConfig.enabled_modules.includes(id)), true)

  const containerConfig = JSON.parse(fs.readFileSync(path.join(frontendRoot, 'modules.json'), 'utf8'))
  assert.equal(replayIds.some((id) => containerConfig.enabled_modules.includes(id)), false)
})
