# Lichtblick Web Bundle (调试用宿主)

Foxglove 调试 MVP 的网页宿主。Lichtblick 是 Foxglove Studio 在 MPL-2.0 下的开源延续版，
上游发布的 `lichtblick-web.tar.gz` 是**纯静态 SPA**（403 个文件，`index.html` + 5 个 `.wasm`，
无子目录），解压即可由任意静态服务器托管。

本目录只是**第三方原始安装包**的存放点，与 `vendor-archives/` 同性质。运行代码在
`robot/script/robot/`，不在这里。

## 内容

| 路径 | 是否提交 Git | 说明 |
|---|---|---|
| `lichtblick-web.lock` | 是 | version / url / sha256 三元组，唯一的事实来源 |
| `README.md` | 是 | 本文件 |
| `lichtblick-web-<version>.tar.gz` | 否 | 上游原始包，约 40 MB |
| `dist/` | 否 | 解压后的静态站点，170 MB |

载荷不提交是刻意的：`.gitignore` 里 `!runtime/*/install/**` 会反向强制跟踪整个 install
目录，所以在其**之后**追加了两条窄规则专门排除 `dist/` 和 `*.tar.gz`。改动 `.gitignore`
时注意顺序，规则靠后才生效。

## 当前固定版本

    version  1.28.1                                  (upstream tag v1.28.1, 2026-08-06)
    sha256   08a954bf95676180282d6d66c0d95ce82ff06c7e65090dd77dfbf2b6a10d13a8

已在本机实测：解压得到 403 个文件、5 个 wasm 模块；两个布局在真实 Chromium 中全部面板渲染通过。

## 安装 / 重装

    robot/script/robot/fetch_lichtblick_web.sh            # 缺什么补什么，已存在且校验通过则跳过
    robot/script/robot/fetch_lichtblick_web.sh --verify   # 只校验，不下载
    robot/script/robot/fetch_lichtblick_web.sh --force    # 忽略现有文件重新下载

脚本**先校验 sha256 再解压**；不匹配时把下载物移到 `*.rejected` 并失败退出，绝不解压未经
校验的内容。下载先落 `*.partial-<pid>`，所以中断不会留下一个看起来完整的坏包。

离线重装：把 `lichtblick-web-<version>.tar.gz` 用任意方式拷进本目录，然后跑
`fetch_lichtblick_web.sh` —— 它发现文件已存在且 sha256 匹配就直接解压，不联网。

## 升级版本

三个字段必须同时改，否则校验会失败（这正是期望行为）：

1. 取新版 sha256：`curl -sL <url> | sha256sum`
2. 同时更新 `lichtblick-web.lock` 的 `version` / `url` / `sha256`
3. `fetch_lichtblick_web.sh --force`
4. `robot/script/robot/foxglove_demo.sh verify` —— **必须跑**。panel 配置键在宿主版本间会漂移，
   布局能不能被新版本接受只有实测才知道。

`lichtblick-web.lock` 是纯数据，`fetch_lichtblick_web.sh` 用 `awk` 读它而**不 source**：
source 会让 vendored 文件里的任意一行当成 shell 执行。

## 为什么不是 `.deb` 桌面版

同一 release 里有 `lichtblick-1.28.1-linux-arm64.deb`（131.7 MB），但 NX 上唯一的显示是远程
X11，Electron + WebGL 的 3D 面板经 X11 转发基本渲染不出来；而且 `.deb` 要 `sudo` 往生产机器狗
上装系统依赖。静态包解压即用、零系统改动，并且正好满足需求文档「方案 2：私有化 Web 部署」。

操作员想在自己电脑上用桌面版，可以自行下载该 `.deb`，与本目录无关。
