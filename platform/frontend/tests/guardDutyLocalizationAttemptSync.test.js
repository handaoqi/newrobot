import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { fileURLToPath } from 'node:url'

const source = readFileSync(fileURLToPath(
  new URL('../src/views/GuardDutyPage.vue', import.meta.url),
), 'utf8')
const routePlannerSource = readFileSync(fileURLToPath(
  new URL('../src/views/RoutePlannerPage.vue', import.meta.url),
), 'utf8')

test('guard-duty manual initialization refreshes route-planner localization attempts', () => {
  const handler = source.match(
    /async function initializeLocalization\(\) \{([\s\S]*?)\n\}\n\nasync function refreshExecution/,
  )?.[1] || ''

  assert.match(handler, /beginStoredAttemptSession\(robot\.id, \{ phase: 'transfer', commandType: 'map\.activate' \}\)/)
  assert.match(handler, /onCommand: event => updateStoredAttemptSession\(robot\.id, event\.command, event\)/)
  assert.match(handler, /if \(error\?\.command\) \{[\s\S]*updateStoredAttemptSession\(robot\.id, error\.command/)
})

test('guard-duty loop relocalization refreshes route-planner localization attempts', () => {
  const handler = source.match(
    /async function ensureLoopNavigationReady\(onProgress = \(\) => \{\}\) \{([\s\S]*?)\n\}\n\nasync function startLoopRestNavigationRepair/,
  )?.[1] || ''

  assert.match(handler, /beginStoredAttemptSession\(robot\.id, \{ phase: 'transfer', commandType: 'map\.activate' \}\)/)
  assert.match(handler, /onCommand: event => updateStoredAttemptSession\(robot\.id, event\.command, event\)/)
  assert.match(handler, /if \(error\?\.command\) \{[\s\S]*updateStoredAttemptSession\(robot\.id, error\.command/)
})

test('guard-duty refreshes attempts for task self-healing and externally started relocalization', () => {
  const handler = source.match(
    /async function refreshLocalizationStatus\(\{ sync = true \} = \{\}\) \{([\s\S]*?)\n\}\n\nasync function refreshGuardState/,
  )?.[1] || ''

  assert.match(handler, /LOCALIZATION_ATTEMPT_COMMAND_TYPES\.has\(command\?\.command_type\)/)
  assert.match(handler, /const detailed = await fetchRobotNavigationStatus\(robotId\)/)
  assert.match(handler, /updateStoredAttemptSession\(robotId, detailed\.localization_command/)
})

test('route planner retains stages synchronized by guard duty when detailed status loads', () => {
  const handler = routePlannerSource.match(
    /function applyLocalizationAttemptCommand\(command, extras = \{\}\) \{([\s\S]*?)\n\}\n\nfunction localizationAttemptProgressText/,
  )?.[1] || ''

  assert.match(handler, /localizationAttemptSession\.value \|\| readStoredAttemptSession\(robotId\)/)
  assert.match(handler, /const timelineHistory = previousSession/)
  assert.match(handler, /timelineHistory,/)
})

test('route planner keeps task-start localization evidence when stable localization is reused', () => {
  const handler = routePlannerSource.match(
    /function restoreAttemptSessionFromTaskExecution\(execution\) \{([\s\S]*?)\n\}\n\nfunction relocalizationHeadingStyle/,
  )?.[1] || ''

  assert.match(handler, /const isTaskStartup = command\.command_type === 'task\.start'/)
  assert.match(handler, /hasStartupLocalizationEvidence/)
  assert.match(handler, /result\.initial_ndt_commit/)
})
