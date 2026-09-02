/**
 * Computed style 过滤器。
 *
 * 浏览器对一个元素报出 340+ 条 computed property，其中绝大多数是 UA 默认值，
 * 塞进 Prompt 只会挤掉真正有用的上下文。这里做两件事：
 *
 *   1. 拿一份「同 tag 裸元素」的 UA 默认值做基线，只保留与基线不同的属性；
 *   2. 当元素自身或父级是 flex/grid 时，额外强制带上一份容器属性白名单 ——
 *      「gap 是 0」「align-items 是 normal」这种等于默认值的事实，恰恰是
 *      「让这一排卡片居中并加 16px 间距」这类诉求必须知道的。
 *
 * 基线元素放在一个 about:blank iframe 里，拿到的是真正的 UA 初始值，不受本页
 * CSS reset 影响；iframe 本身挂在 overlay 的 shadow root 内，不进入页面的
 * light DOM，因此不会打乱任何 :nth-child / :last-child 选择器。
 *
 * 纯函数（selectStyles / snapshotComputedStyle）与浏览器胶水（createStyleFilter）
 * 分开，前者可以在 node:test 里直接喂普通对象测试。
 */

/**
 * 容器属性白名单：元素自身或父级为 flex/grid 时无条件带上。
 * 顺序即输出顺序 —— 越靠前越重要，超预算截断时后面的先被丢掉。
 */
export const CONTAINER_PROPS = [
  'display',
  'flex-direction',
  'flex-wrap',
  'justify-content',
  'align-items',
  'align-content',
  'align-self',
  'place-content',
  'place-items',
  'place-self',
  'gap',
  'row-gap',
  'column-gap',
  'flex-grow',
  'flex-shrink',
  'flex-basis',
  'order',
  'grid-template-columns',
  'grid-template-rows',
  'grid-template-areas',
  'grid-auto-flow',
  'grid-auto-columns',
  'grid-auto-rows',
  'grid-column-start',
  'grid-column-end',
  'grid-row-start',
  'grid-row-end',
  'position',
  'top',
  'right',
  'bottom',
  'left',
  'box-sizing',
  'width',
  'height',
  'min-width',
  'min-height',
  'max-width',
  'max-height',
  'margin-top',
  'margin-right',
  'margin-bottom',
  'margin-left',
  'padding-top',
  'padding-right',
  'padding-bottom',
  'padding-left',
  'overflow-x',
  'overflow-y',
]

const CONTAINER_PROP_SET = new Set(CONTAINER_PROPS)
const CONTAINER_PROP_ORDER = new Map(CONTAINER_PROPS.map((prop, index) => [prop, index]))

/** display 取值里意味着「我在给孩子排版」的那几个。 */
const LAYOUT_DISPLAYS = new Set(['flex', 'inline-flex', 'grid', 'inline-grid'])

/**
 * Chrome 会把每条物理属性再以逻辑属性名报一遍（margin-block-start 之类），
 * 内容与物理属性完全重复，纯粹是 token 浪费。
 */
const LOGICAL_PROP_RE = /(^|-)(inline|block)-(size|start|end)\b/

/**
 * 这几条是「整页排版」而非「这个元素」的属性。about:blank 基线的字体栈和宿主
 * 页面必然不同，不排掉的话它们会出现在每一个元素的样式里。color / font-size
 * 常常正是要改的目标，所以不在此列。
 */
const PAGE_WIDE_PROPS = new Set(['font-family', 'text-size-adjust', 'tab-size', 'color-scheme'])

/**
 * 值由盒子尺寸推导而来，和作者写了什么无关；基线元素尺寸为 0，一比必然「不同」。
 */
const GEOMETRY_DERIVED_PROPS = new Set(['perspective-origin', 'transform-origin'])

/** 该属性是否值得报给 LLM。 */
export function isReportableProp(prop) {
  if (typeof prop !== 'string' || prop === '') return false
  // 厂商前缀（-webkit- / -moz- / -ms-）与自定义属性（--foo）一并丢掉。
  if (prop.startsWith('-')) return false
  if (LOGICAL_PROP_RE.test(prop)) return false
  if (PAGE_WIDE_PROPS.has(prop)) return false
  if (GEOMETRY_DERIVED_PROPS.has(prop)) return false
  return true
}

/**
 * CSSStyleDeclaration 转普通对象。
 *
 * 必须走 length + 下标 + getPropertyValue：CSSStyleDeclaration 的自有可枚举属性
 * 是数字下标，其值是属性「名」而不是属性「值」，Object.entries 会拿到一堆
 * ['0', 'display'] 这种垃圾。
 */
export function snapshotComputedStyle(declaration) {
  const out = {}
  if (!declaration) return out
  for (let i = 0; i < declaration.length; i += 1) {
    const prop = declaration[i]
    if (typeof prop !== 'string') continue
    out[prop] = declaration.getPropertyValue(prop)
  }
  return out
}

/** 容器属性排前面（按白名单顺序），其余按字母序，保证输出稳定可比对。 */
function sortProps(props) {
  return props.sort((a, b) => {
    const rankA = CONTAINER_PROP_ORDER.has(a) ? CONTAINER_PROP_ORDER.get(a) : Number.MAX_SAFE_INTEGER
    const rankB = CONTAINER_PROP_ORDER.has(b) ? CONTAINER_PROP_ORDER.get(b) : Number.MAX_SAFE_INTEGER
    if (rankA !== rankB) return rankA - rankB
    return a < b ? -1 : a > b ? 1 : 0
  })
}

/**
 * 纯函数版过滤：computed 与 defaults 都是 prop -> value 的普通对象。
 *
 * @param {Record<string,string>} computed 元素的 computed style 快照
 * @param {Record<string,string>|null} defaults 同 tag 裸元素的 UA 默认值快照；
 *        拿不到时退化为「只输出容器属性白名单」，输出量仍然有界。
 * @param {{container?: boolean}} [options] container 为真时强制带上容器属性
 */
export function selectStyles(computed, defaults, options = {}) {
  const container = Boolean(options.container)
  const picked = []
  for (const prop of Object.keys(computed ?? {})) {
    if (!isReportableProp(prop)) continue
    const value = computed[prop]
    if (value === '' || value == null) continue
    if (container && CONTAINER_PROP_SET.has(prop)) {
      picked.push(prop)
      continue
    }
    if (!defaults) continue
    if (defaults[prop] !== value) picked.push(prop)
  }
  const out = {}
  for (const prop of sortProps(picked)) out[prop] = computed[prop]
  return out
}

/** 元素自身或父级在给孩子做 flex/grid 排版？ */
export function isLayoutContext(ownDisplay, parentDisplay) {
  return LAYOUT_DISPLAYS.has(ownDisplay) || LAYOUT_DISPLAYS.has(parentDisplay)
}

/**
 * 浏览器端过滤器。
 *
 * @param {Node} container 基线 iframe 的挂载点。必须传 overlay 的 shadow root，
 *        绝不能是 document.body —— 往 light DOM 里插节点会改变兄弟节点的
 *        :nth-child / :last-child 匹配，属于篡改页面。
 */
export function createStyleFilter(container) {
  const doc = container.ownerDocument ?? container
  const cache = new Map()
  let frame = null

  function baselineDocument() {
    if (!frame) {
      frame = doc.createElement('iframe')
      frame.setAttribute('aria-hidden', 'true')
      frame.setAttribute('tabindex', '-1')
      frame.setAttribute('src', 'about:blank')
      frame.style.cssText = 'position:absolute;width:0;height:0;border:0;visibility:hidden;pointer-events:none;'
      container.appendChild(frame)
    }
    // about:blank 的 contentDocument 在 append 之后即刻可用，无需等 load。
    return frame.contentDocument ?? null
  }

  function defaultsFor(tagName) {
    const key = String(tagName || 'div').toLowerCase()
    if (cache.has(key)) return cache.get(key)
    const baseline = baselineDocument()
    if (!baseline?.body) return null // 还没就绪：这一次退化，不写缓存，下次再试
    const probe = baseline.createElement(key)
    baseline.body.appendChild(probe)
    const snapshot = snapshotComputedStyle(baseline.defaultView.getComputedStyle(probe))
    probe.remove()
    cache.set(key, snapshot)
    return snapshot
  }

  return {
    /** @returns {Record<string,string>} 该元素值得报给 LLM 的样式 */
    stylesFor(element) {
      const view = element.ownerDocument?.defaultView
      if (!view) return {}
      const computed = snapshotComputedStyle(view.getComputedStyle(element))
      const parent = element.parentElement
      const parentDisplay = parent ? view.getComputedStyle(parent).display : ''
      return selectStyles(computed, defaultsFor(element.tagName), {
        container: isLayoutContext(computed.display, parentDisplay),
      })
    },
    dispose() {
      frame?.remove()
      frame = null
      cache.clear()
    },
  }
}
