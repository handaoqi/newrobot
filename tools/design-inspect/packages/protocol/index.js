/**
 * Design Inspect 协议层。
 *
 * 这个包被三方共享：浏览器 overlay 组装 Payload、VS Code webview 转发 Payload、
 * 扩展宿主把 Payload 渲染成 Prompt 并校验 Agent 的返回。三方对 Payload 的理解
 * 必须完全一致，所以定义只有这一份，且不依赖任何运行时（纯 ESM，无 npm 依赖，
 * 顶层不碰 DOM）。
 */
export * from './payload.js'
export * from './styles.js'
