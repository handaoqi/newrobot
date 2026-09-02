import test from 'node:test'
import assert from 'node:assert/strict'

import {
  CONTAINER_PROPS,
  isLayoutContext,
  isReportableProp,
  selectStyles,
  snapshotComputedStyle,
} from '../styles.js'

/** 造一个和 CSSStyleDeclaration 同形的假对象：数字下标给属性名，取值走方法。 */
function fakeDeclaration(entries) {
  const props = Object.keys(entries)
  const declaration = {
    length: props.length,
    getPropertyValue: (prop) => entries[prop] ?? '',
  }
  props.forEach((prop, index) => {
    declaration[index] = prop
  })
  return declaration
}

test('snapshotComputedStyle 读的是属性值，不是被下标坑成属性名', () => {
  const declaration = fakeDeclaration({ display: 'flex', gap: '16px' })
  assert.deepEqual(snapshotComputedStyle(declaration), { display: 'flex', gap: '16px' })
})

test('snapshotComputedStyle 对空输入返回空对象', () => {
  assert.deepEqual(snapshotComputedStyle(null), {})
})

test('isReportableProp 丢掉厂商前缀与自定义属性', () => {
  assert.equal(isReportableProp('-webkit-box-align'), false)
  assert.equal(isReportableProp('-moz-osx-font-smoothing'), false)
  assert.equal(isReportableProp('--cyan'), false)
})

test('isReportableProp 丢掉与物理属性重复的逻辑属性', () => {
  for (const prop of [
    'inline-size',
    'block-size',
    'min-inline-size',
    'max-block-size',
    'margin-inline-start',
    'padding-block-end',
    'inset-inline-start',
    'border-inline-start-width',
  ]) {
    assert.equal(isReportableProp(prop), false, `${prop} 应被视为重复`)
  }
})

test('isReportableProp 丢掉整页排版与尺寸推导属性', () => {
  assert.equal(isReportableProp('font-family'), false)
  assert.equal(isReportableProp('tab-size'), false)
  assert.equal(isReportableProp('perspective-origin'), false)
  assert.equal(isReportableProp('transform-origin'), false)
})

test('isReportableProp 保留真正会被改的属性', () => {
  for (const prop of ['display', 'gap', 'justify-content', 'margin-top', 'color', 'font-size', 'width']) {
    assert.equal(isReportableProp(prop), true, `${prop} 应保留`)
  }
})

test('裸元素：computed 与 UA 默认完全一致时返回空集', () => {
  const defaults = { display: 'block', color: 'rgb(0, 0, 0)', 'margin-top': '0px' }
  const computed = { ...defaults }
  assert.deepEqual(selectStyles(computed, defaults, { container: false }), {})
})

test('只报与 UA 默认不同的属性', () => {
  const defaults = { display: 'block', color: 'rgb(0, 0, 0)', 'margin-top': '0px' }
  const computed = { display: 'block', color: 'rgb(230, 237, 243)', 'margin-top': '0px' }
  assert.deepEqual(selectStyles(computed, defaults, { container: false }), { color: 'rgb(230, 237, 243)' })
})

test('flex 容器：容器属性即使等于默认值也强制带上', () => {
  const defaults = { display: 'block', gap: 'normal', 'align-items': 'normal', color: 'rgb(0, 0, 0)' }
  const computed = { display: 'flex', gap: 'normal', 'align-items': 'normal', color: 'rgb(0, 0, 0)' }
  const picked = selectStyles(computed, defaults, { container: true })

  // 「gap 现在是 normal」正是「加 16px 间距」这类诉求必须知道的事实。
  assert.deepEqual(picked, { display: 'flex', 'align-items': 'normal', gap: 'normal' })
  assert.equal('color' in picked, false, '非容器属性等于默认值时仍应剔除')
})

test('容器属性排在前面，其余按字母序 —— 输出稳定且截断时先掉次要项', () => {
  const defaults = {}
  const computed = { zoom: '1', color: 'red', gap: '16px', display: 'flex', 'align-items': 'center' }
  const keys = Object.keys(selectStyles(computed, defaults, { container: true }))
  assert.deepEqual(keys, ['display', 'align-items', 'gap', 'color', 'zoom'])
})

test('拿不到 UA 默认基线时退化为只输出容器属性，输出量仍然有界', () => {
  const computed = { display: 'flex', gap: '16px', color: 'red', 'font-size': '14px' }
  assert.deepEqual(selectStyles(computed, null, { container: true }), { display: 'flex', gap: '16px' })
  assert.deepEqual(selectStyles(computed, null, { container: false }), {})
})

test('不可上报的属性即使与默认值不同也不会混进来', () => {
  const computed = { '-webkit-box-align': 'center', '--cyan': '#0ff', 'margin-inline-start': '8px', display: 'flex' }
  assert.deepEqual(selectStyles(computed, {}, { container: false }), { display: 'flex' })
})

test('isLayoutContext 认自身或父级的 flex/grid', () => {
  assert.equal(isLayoutContext('flex', 'block'), true)
  assert.equal(isLayoutContext('block', 'grid'), true)
  assert.equal(isLayoutContext('inline-flex', ''), true)
  assert.equal(isLayoutContext('block', 'block'), false)
  assert.equal(isLayoutContext('inline', undefined), false)
})

test('容器白名单覆盖 flex/grid/间距/定位/盒模型这几类', () => {
  const set = new Set(CONTAINER_PROPS)
  for (const prop of [
    'display',
    'flex-direction',
    'justify-content',
    'align-items',
    'gap',
    'grid-template-columns',
    'place-items',
    'position',
    'margin-left',
    'padding-top',
    'width',
    'overflow-x',
  ]) {
    assert.equal(set.has(prop), true, `容器白名单应包含 ${prop}`)
  }
})
