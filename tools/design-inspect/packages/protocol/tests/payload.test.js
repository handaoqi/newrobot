import test from 'node:test'
import assert from 'node:assert/strict'

import {
  CHARS_PER_TOKEN,
  EDITS_SCHEMA,
  MAX_PAYLOAD_TOKENS,
  PAYLOAD_VERSION,
  budgetPayload,
  collapseHtmlChildren,
  createPayload,
  describeNode,
  estimateTokens,
  findUniqueMatch,
  formatSourceLocation,
  parseSourceLocation,
  renderContext,
  validateEditsResult,
} from '../payload.js'
import { CONTAINER_PROPS } from '../styles.js'

test('parseSourceLocation 解析 data-v-inspector 的三段式取值', () => {
  assert.deepEqual(parseSourceLocation('src/views/RoutePlannerPage.vue:142:7'), {
    file: 'src/views/RoutePlannerPage.vue',
    line: 142,
    column: 7,
  })
})

test('parseSourceLocation 从右侧匹配，路径里的冒号不会被当成分隔符', () => {
  assert.deepEqual(parseSourceLocation('src/odd:name/X.vue:3:1'), {
    file: 'src/odd:name/X.vue',
    line: 3,
    column: 1,
  })
})

test('parseSourceLocation 对残缺输入返回 null 而不是半个位置', () => {
  for (const bad of ['', 'src/X.vue', 'src/X.vue:12', ':1:2', null, undefined, 42]) {
    assert.equal(parseSourceLocation(bad), null, `应拒绝 ${JSON.stringify(bad)}`)
  }
})

test('formatSourceLocation 在没有位置时给出可读占位而不是 undefined', () => {
  assert.equal(formatSourceLocation(null), '未知')
  assert.equal(formatSourceLocation({ file: 'a.vue', line: 1, column: 2 }), 'a.vue:1:2')
})

test('describeNode 拼出 CSS 选择器风格的标签，class 最多带 6 个', () => {
  assert.equal(describeNode({ tag: 'DIV', id: 'app', classList: ['a', 'b'] }), 'div#app.a.b')
  const many = describeNode({ tag: 'div', classList: ['c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'c7'] })
  assert.equal(many, 'div.c1.c2.c3.c4.c5.c6')
})

test('collapseHtmlChildren 折叠子节点但保留最外层标签与属性', () => {
  const html = '<div class="row"><span>a</span><span>b</span></div>'
  assert.equal(collapseHtmlChildren(html, 2), '<div class="row"><!-- 2 children --> </div>')
})

test('collapseHtmlChildren 对自闭合与空元素原样返回', () => {
  assert.equal(collapseHtmlChildren('<img src="a.png">', 0), '<img src="a.png">')
  assert.equal(collapseHtmlChildren('<div></div>', 0), '<div></div>')
})

function smallPayload() {
  return createPayload({
    source: { file: 'src/views/RoutePlannerPage.vue', line: 142, column: 7 },
    ancestors: [{ tag: 'div', classList: ['route-list'], source: { file: 'src/views/RoutePlannerPage.vue', line: 120, column: 5 } }],
    tag: 'div',
    classList: ['route-card'],
    outerHTML: '<div class="route-card">卡片</div>',
    childCount: 1,
    styles: { display: 'flex', gap: '0px', color: 'rgb(230, 237, 243)' },
    rect: { x: 32, y: 210, width: 380, height: 164 },
    viewport: { width: 1440, height: 900 },
    route: '/dashboard/tasks/routes',
  })
}

test('createPayload 补齐所有字段并打上协议版本', () => {
  const payload = createPayload({})
  assert.equal(payload.v, PAYLOAD_VERSION)
  assert.deepEqual(payload.ancestors, [])
  assert.deepEqual(payload.styles, {})
  assert.equal(payload.outerHTML, '')
})

test('createPayload 把祖先链截到 3 层', () => {
  const payload = createPayload({ ancestors: [1, 2, 3, 4, 5].map((n) => ({ tag: `d${n}` })) })
  assert.equal(payload.ancestors.length, 3)
  assert.equal(payload.ancestors[0].tag, 'd1')
})

test('renderContext 把源码位置放在第一屏，Agent 一眼能拿到锚点', () => {
  const text = renderContext(smallPayload())
  assert.match(text, /源码: src\/views\/RoutePlannerPage\.vue:142:7/)
  assert.match(text, /路由: \/dashboard\/tasks\/routes/)
  assert.match(text, /节点: div\.route-card/)
  assert.match(text, /祖先链\(由内向外\): div\.route-list \(src\/views\/RoutePlannerPage\.vue:120:5\)/)
  assert.match(text, /```html\n<div class="route-card">卡片<\/div>\n```/)
  assert.match(text, /display: flex/)
})

test('小 Payload 原样通过，不做任何裁剪', () => {
  const result = budgetPayload(smallPayload(), { containerProps: CONTAINER_PROPS })
  assert.deepEqual(result.budget.trimmed, [])
  assert.equal(result.budget.withinBudget, true)
  assert.ok(result.budget.estTokens <= MAX_PAYLOAD_TOKENS)
  assert.equal(result.outerHTML, smallPayload().outerHTML)
  assert.equal(result.ancestors.length, 1)
})

test('预算按渲染后的上下文文本算，而不是 Payload 的 JSON 体积', () => {
  const payload = smallPayload()
  const result = budgetPayload(payload, { containerProps: CONTAINER_PROPS })
  assert.equal(result.budget.estTokens, estimateTokens(renderContext(payload)))
})

test('超预算时先丢非容器样式，容器样式与祖先链保住', () => {
  const payload = smallPayload()
  // 只让样式超预算：500 条非容器属性，outerHTML 保持很短。
  payload.styles = { display: 'flex', gap: '16px' }
  for (let i = 0; i < 500; i += 1) payload.styles[`noise-prop-${i}`] = `value-${i}-xxxxxxxxxxxxxxxx`
  const result = budgetPayload(payload, { containerProps: CONTAINER_PROPS })

  assert.deepEqual(result.budget.trimmed, ['styles:container-only'])
  assert.deepEqual(result.styles, { display: 'flex', gap: '16px' })
  assert.equal(result.ancestors.length, 1, '祖先链是最后才动的，不该被牵连')
  assert.equal(result.outerHTML, payload.outerHTML)
  assert.ok(result.budget.estTokens <= MAX_PAYLOAD_TOKENS)
})

test('样式全丢仍超预算时才折叠 outerHTML，且顺序固定', () => {
  const payload = smallPayload()
  const children = Array.from({ length: 400 }, (_, i) => `<span class="cell">单元格 ${i}</span>`).join('')
  payload.outerHTML = `<div class="route-card">${children}</div>`
  payload.childCount = 400
  const result = budgetPayload(payload, { containerProps: CONTAINER_PROPS })

  assert.deepEqual(result.budget.trimmed.slice(0, 1), ['styles:container-only'])
  assert.ok(result.budget.trimmed.includes('html:collapse-children'))
  assert.equal(result.outerHTML, '<div class="route-card"><!-- 400 children --> </div>')
  assert.ok(result.budget.estTokens <= MAX_PAYLOAD_TOKENS)
})

test('折叠后依然超预算就硬截，最后才丢祖先链', () => {
  const payload = smallPayload()
  // 单一巨型开标签：折叠帮不上忙，只能截断。
  payload.outerHTML = `<div class="${'x'.repeat(40000)}">a</div>`
  payload.childCount = 1
  const result = budgetPayload(payload, { containerProps: CONTAINER_PROPS })

  assert.ok(result.budget.trimmed.includes('html:truncate'))
  assert.ok(result.outerHTML.endsWith('-->'), '截断处要留下可见标记')
  assert.ok(result.budget.estTokens <= MAX_PAYLOAD_TOKENS)
  assert.equal(result.budget.withinBudget, true)
})

test('极端情况下祖先链被逐层丢弃，最终仍落在预算内', () => {
  const payload = smallPayload()
  payload.ancestors = Array.from({ length: 3 }, (_, i) => ({
    tag: 'div',
    classList: [`ancestor-${i}-${'y'.repeat(3000)}`],
    source: { file: 'src/views/RoutePlannerPage.vue', line: 100 + i, column: 1 },
  }))
  const result = budgetPayload(payload, { containerProps: CONTAINER_PROPS })

  assert.ok(result.budget.trimmed.includes('ancestors:drop'))
  assert.ok(result.ancestors.length < 3)
  assert.ok(result.budget.estTokens <= MAX_PAYLOAD_TOKENS)
})

test('自定义预算上限同样生效', () => {
  const result = budgetPayload(smallPayload(), { containerProps: CONTAINER_PROPS, maxTokens: 20 })
  assert.equal(result.budget.maxTokens, 20)
  assert.ok(result.budget.trimmed.length > 0)
})

test('estimateTokens 保守取整，不会低估', () => {
  assert.equal(estimateTokens('a'.repeat(CHARS_PER_TOKEN)), 1)
  assert.equal(estimateTokens('a'.repeat(CHARS_PER_TOKEN + 1)), 2)
  assert.equal(estimateTokens(''), 0)
})

test('findUniqueMatch 区分未命中 / 唯一命中 / 多次命中', () => {
  const content = 'alpha\nbeta\nalpha\n'
  assert.deepEqual(findUniqueMatch(content, 'beta'), { ok: true, index: 6, count: 1 })
  assert.deepEqual(findUniqueMatch(content, 'gamma'), { ok: false, reason: 'not-found', count: 0 })
  assert.deepEqual(findUniqueMatch(content, 'alpha'), { ok: false, reason: 'ambiguous', count: 2 })
  assert.deepEqual(findUniqueMatch(content, ''), { ok: false, reason: 'empty', count: 0 })
})

test('findUniqueMatch 把重叠出现也算作多次命中（判定偏保守）', () => {
  assert.deepEqual(findUniqueMatch('aaaa', 'aa'), { ok: false, reason: 'ambiguous', count: 3 })
})

test('EDITS_SCHEMA 关死 additionalProperties，模型不能夹带额外字段', () => {
  assert.equal(EDITS_SCHEMA.additionalProperties, false)
  assert.equal(EDITS_SCHEMA.properties.edits.items.additionalProperties, false)
  assert.deepEqual(EDITS_SCHEMA.required, ['summary', 'edits'])
  assert.deepEqual(EDITS_SCHEMA.properties.edits.items.required, ['file', 'oldText', 'newText', 'reason'])
})

test('validateEditsResult 接受合法输出，拒绝残缺输出', () => {
  const good = { summary: '居中', edits: [{ file: 'a.vue', oldText: 'x', newText: 'y', reason: 'z' }] }
  assert.deepEqual(validateEditsResult(good), { ok: true, errors: [] })

  assert.equal(validateEditsResult({ summary: '无事可做', edits: [] }).ok, true)
  assert.equal(validateEditsResult(null).ok, false)
  assert.equal(validateEditsResult({ summary: 'a' }).ok, false)
  assert.equal(validateEditsResult({ summary: 'a', edits: [{ file: 'a.vue', oldText: '', newText: 'y' }] }).ok, false)
})
