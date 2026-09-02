/**
 * Design Inspect Payload 契约。
 *
 * 一次「点中元素」产生的全部上下文都装在这一个对象里，从浏览器 overlay 经
 * webview 传到扩展宿主。三端共用这份定义，字段含义不会各写各的。
 *
 * 关于 2000 token 上限：Payload 是**锚点**，不是全部上下文。Agent 拿到源码
 * 路径行号后会自己用 Read/Grep 把文件读全，所以这里裁掉的是冗余而不是信息量。
 */

export const PAYLOAD_VERSION = 1

/** postMessage 信封上的固定标识，用来把我们的消息和页面里其他消息区分开。 */
export const MESSAGE_SOURCE = 'roamerx-design-inspect'

/** 消息类型。页面 -> 宿主 与 宿主 -> 页面 共用一张表。 */
export const MESSAGE_TYPES = {
  /** 页面 -> 宿主：overlay 已就绪，可以接收指令 */
  READY: 'ready',
  /** 宿主 -> 页面：开/关 Design 模式 */
  SET_MODE: 'set-mode',
  /** 页面 -> 宿主：Design 模式实际状态（含激活耗时） */
  MODE: 'mode',
  /** 页面 -> 宿主：悬停命中的元素（轻量，只有源码位置和标签） */
  HOVER: 'hover',
  /** 页面 -> 宿主：锁定选中，携带完整 Payload */
  SELECT: 'select',
  /** 宿主 -> 页面：沿祖先链上/下走一层（delta: -1 子 / +1 父） */
  WALK: 'walk',
  /** 页面 -> 宿主：选中被清除 */
  CLEAR: 'clear',
}

/** token 预算。保守按 4 字符 ≈ 1 token 估。 */
export const MAX_PAYLOAD_TOKENS = 2000
export const CHARS_PER_TOKEN = 4

/** 祖先链最多带几层。再多对定位没有帮助，只是占预算。 */
export const MAX_ANCESTORS = 3

/** outerHTML 截断后至少保留多少字符 —— 少于这个数就没法看出结构了。 */
const MIN_HTML_CHARS = 200
const TRUNCATE_MARKER = '\n<!-- …truncated… -->'

/** 保守 token 估算。宁可高估，不能让 Prompt 超预算。 */
export function estimateTokens(text) {
  if (!text) return 0
  return Math.ceil(String(text).length / CHARS_PER_TOKEN)
}

/**
 * 解析 vite-plugin-vue-inspector 注入的 data-v-inspector 值。
 *
 * 格式是 `<相对 dev server cwd 的路径>:<line>:<column>`。路径本身可能含冒号
 * （少见但合法），所以从右边开始匹配。
 */
export function parseSourceLocation(raw) {
  if (typeof raw !== 'string') return null
  const match = /^(.*):(\d+):(\d+)$/.exec(raw.trim())
  if (!match || !match[1]) return null
  return { file: match[1], line: Number(match[2]), column: Number(match[3]) }
}

export function formatSourceLocation(source) {
  if (!source?.file) return '未知'
  const line = Number.isFinite(source.line) ? source.line : 0
  const column = Number.isFinite(source.column) ? source.column : 0
  return `${source.file}:${line}:${column}`
}

/** `div#app.route-card.is-active` —— 面包屑和 Prompt 里都用这个写法。 */
export function describeNode(node) {
  if (!node) return '?'
  let text = node.tag ? String(node.tag).toLowerCase() : '?'
  if (node.id) text += `#${node.id}`
  const classes = Array.isArray(node.classList) ? node.classList.slice(0, 6) : []
  for (const name of classes) text += `.${name}`
  return text
}

/**
 * 把 Payload 渲染成注入 Prompt 的上下文段。
 *
 * 预算是按**这段文本**算的，不是按 Payload 的 JSON 体积 —— 只有这段文本会真的
 * 进入 Prompt，拿它算 KPI 才不是自欺欺人。
 */
export function renderContext(payload) {
  const lines = []
  lines.push('## 选中元素')
  lines.push(`源码: ${formatSourceLocation(payload.source)}`)
  if (payload.route) lines.push(`路由: ${payload.route}`)
  lines.push(`节点: ${describeNode(payload)}`)

  const ancestors = Array.isArray(payload.ancestors) ? payload.ancestors : []
  if (ancestors.length) {
    const chain = ancestors
      .map((node) => `${describeNode(node)} (${formatSourceLocation(node.source)})`)
      .join(' < ')
    lines.push(`祖先链(由内向外): ${chain}`)
  }

  const attributes = payload.attributes && Object.keys(payload.attributes)
  if (attributes?.length) {
    lines.push(`属性: ${attributes.map((key) => `${key}="${payload.attributes[key]}"`).join(' ')}`)
  }

  if (payload.rect) {
    const { x = 0, y = 0, width = 0, height = 0 } = payload.rect
    const viewport = payload.viewport
      ? `，视口 ${Math.round(payload.viewport.width)}x${Math.round(payload.viewport.height)}`
      : ''
    lines.push(
      `盒子: ${Math.round(width)}x${Math.round(height)} @ (${Math.round(x)}, ${Math.round(y)})${viewport}`,
    )
  }

  if (payload.outerHTML) {
    lines.push('', '### Outer HTML', '```html', payload.outerHTML, '```')
  }

  const styleKeys = payload.styles ? Object.keys(payload.styles) : []
  if (styleKeys.length) {
    lines.push('', '### 计算样式（已剔除 UA 默认值）')
    for (const key of styleKeys) lines.push(`${key}: ${payload.styles[key]}`)
  }

  return lines.join('\n')
}

/**
 * 把 outerHTML 的子节点折叠成一条注释，保留最外层标签和它的属性。
 * 空元素 / 自闭合标签原样返回。
 */
export function collapseHtmlChildren(html, childCount = 0) {
  if (typeof html !== 'string' || html === '') return html
  const openEnd = html.indexOf('>')
  const closeStart = html.lastIndexOf('<')
  if (openEnd < 0 || closeStart <= openEnd) return html
  const inner = html.slice(openEnd + 1, closeStart)
  if (inner === '') return html
  const label = childCount > 0 ? `${childCount} children` : 'content'
  return `${html.slice(0, openEnd + 1)}<!-- ${label} --> ${html.slice(closeStart)}`
}

function hasNonContainerStyles(styles, containerProps) {
  if (!styles) return false
  return Object.keys(styles).some((prop) => !containerProps.has(prop))
}

/**
 * 按固定优先级裁剪 Payload 到 token 预算内，并把裁剪动作记进 `budget.trimmed`。
 *
 * 顺序 —— styles → outerHTML → ancestors —— 是按「丢掉后损失的信息量」排的：
 * 样式 Agent 可以自己重算，HTML 结构它能从源码读到，祖先链一旦丢了就只能靠猜。
 *
 * @param {object} payload
 * @param {{maxTokens?: number, containerProps?: Iterable<string>}} [options]
 */
export function budgetPayload(payload, options = {}) {
  const maxTokens = options.maxTokens ?? MAX_PAYLOAD_TOKENS
  const maxChars = maxTokens * CHARS_PER_TOKEN
  const containerProps = new Set(options.containerProps ?? [])
  const trimmed = []
  const draft = { ...payload }

  const rendered = () => renderContext(draft)
  const over = () => rendered().length > maxChars

  // 1. 样式：先只留容器属性，再整体丢掉。
  if (over() && hasNonContainerStyles(draft.styles, containerProps)) {
    const kept = {}
    for (const prop of Object.keys(draft.styles)) {
      if (containerProps.has(prop)) kept[prop] = draft.styles[prop]
    }
    draft.styles = kept
    trimmed.push('styles:container-only')
  }
  if (over() && draft.styles && Object.keys(draft.styles).length > 0) {
    draft.styles = {}
    trimmed.push('styles:drop')
  }

  // 2. outerHTML：先折叠子节点，还不够再硬截。
  if (over() && draft.outerHTML) {
    const collapsed = collapseHtmlChildren(draft.outerHTML, draft.childCount ?? 0)
    if (collapsed !== draft.outerHTML) {
      draft.outerHTML = collapsed
      trimmed.push('html:collapse-children')
    }
  }
  if (over() && draft.outerHTML && draft.outerHTML.length > MIN_HTML_CHARS) {
    let truncated = false
    while (draft.outerHTML.length > MIN_HTML_CHARS) {
      const overflow = rendered().length - maxChars
      if (overflow <= 0) break
      const keep = Math.max(MIN_HTML_CHARS, draft.outerHTML.length - overflow - TRUNCATE_MARKER.length)
      if (keep >= draft.outerHTML.length) break
      draft.outerHTML = draft.outerHTML.slice(0, keep) + TRUNCATE_MARKER
      truncated = true
    }
    if (truncated) trimmed.push('html:truncate')
  }

  // 3. 祖先链：从最外层开始一层层丢（列表是由内向外排的）。
  while (over() && Array.isArray(draft.ancestors) && draft.ancestors.length > 0) {
    draft.ancestors = draft.ancestors.slice(0, -1)
    trimmed.push('ancestors:drop')
  }

  const estTokens = estimateTokens(rendered())
  draft.budget = { estTokens, maxTokens, trimmed, withinBudget: estTokens <= maxTokens }
  return draft
}

/**
 * 组装一个字段齐全的 Payload。overlay 只管把原始数据丢进来，缺省与归一化在这里
 * 统一处理，免得两端对「没有祖先时 ancestors 是 undefined 还是 []」各有理解。
 */
export function createPayload(input) {
  return {
    v: PAYLOAD_VERSION,
    source: input.source ?? null,
    ancestors: (input.ancestors ?? []).slice(0, MAX_ANCESTORS),
    tag: input.tag ?? '',
    id: input.id ?? '',
    classList: input.classList ?? [],
    attributes: input.attributes ?? {},
    outerHTML: input.outerHTML ?? '',
    childCount: input.childCount ?? 0,
    styles: input.styles ?? {},
    rect: input.rect ?? null,
    viewport: input.viewport ?? null,
    route: input.route ?? '',
    marks: input.marks ?? {},
  }
}

/**
 * 喂给 `claude --json-schema` 的输出契约。
 *
 * Agent 全程只读，它唯一的产出就是这个结构；写文件的是扩展。`oldText` 必须在
 * 目标文件里**恰好命中一次** —— 这是把「模型给的一段文本」变成「可应用的
 * Range」的唯一锚点，也是 Diff 预览与一键回退能成立的前提。
 */
export const EDITS_SCHEMA = {
  type: 'object',
  additionalProperties: false,
  required: ['summary', 'edits'],
  properties: {
    summary: {
      type: 'string',
      description: '一句话说明这次改了什么，中文。',
    },
    edits: {
      type: 'array',
      description: '要应用的编辑列表。没有需要改的地方时返回空数组。',
      items: {
        type: 'object',
        additionalProperties: false,
        required: ['file', 'oldText', 'newText', 'reason'],
        properties: {
          file: {
            type: 'string',
            description: '相对工程根的文件路径，例如 src/views/RoutePlannerPage.vue',
          },
          oldText: {
            type: 'string',
            description:
              '被替换的原文，必须与文件内容逐字符一致（含缩进），且在该文件中恰好出现一次。'
              + '如果目标片段不唯一，请向上多带几行上下文直到唯一。',
          },
          newText: {
            type: 'string',
            description: '替换后的新内容。',
          },
          reason: {
            type: 'string',
            description: '这一处为什么这么改，一句话。',
          },
        },
      },
    },
  },
}

/**
 * 在文件内容里定位 oldText，要求恰好命中一次。
 *
 * @returns {{ok: true, index: number, count: 1}
 *          |{ok: false, reason: 'empty'|'not-found'|'ambiguous', count: number}}
 */
export function findUniqueMatch(content, needle) {
  if (typeof needle !== 'string' || needle === '') {
    return { ok: false, reason: 'empty', count: 0 }
  }
  let count = 0
  let first = -1
  let from = 0
  for (;;) {
    const index = content.indexOf(needle, from)
    if (index < 0) break
    if (count === 0) first = index
    count += 1
    // +1 而不是 +needle.length：重叠出现也算多次命中，判定偏保守。
    from = index + 1
  }
  if (count === 0) return { ok: false, reason: 'not-found', count: 0 }
  if (count > 1) return { ok: false, reason: 'ambiguous', count }
  return { ok: true, index: first, count: 1 }
}

/**
 * 结构化输出的形状校验。`--json-schema` 已经在 CLI 侧约束过一遍，这里再挡一次，
 * 因为下一步就要拿这些字段去改用户的文件。
 */
export function validateEditsResult(value) {
  const errors = []
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    return { ok: false, errors: ['输出不是一个对象'] }
  }
  if (typeof value.summary !== 'string') errors.push('缺少 summary')
  if (!Array.isArray(value.edits)) {
    errors.push('缺少 edits 数组')
    return { ok: false, errors }
  }
  value.edits.forEach((edit, index) => {
    if (!edit || typeof edit !== 'object') {
      errors.push(`edits[${index}] 不是对象`)
      return
    }
    for (const key of ['file', 'oldText', 'newText']) {
      if (typeof edit[key] !== 'string' || edit[key] === '') {
        errors.push(`edits[${index}].${key} 缺失或为空`)
      }
    }
  })
  return { ok: errors.length === 0, errors }
}
