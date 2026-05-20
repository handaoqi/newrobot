const statusOutput = document.querySelector("#statusOutput");
const actionOutput = document.querySelector("#actionOutput");
const statusPill = document.querySelector("#statusPill");
const remoteModePill = document.querySelector("#remoteModePill");
const remoteOutput = document.querySelector("#remoteOutput");
const remoteRefreshBtn = document.querySelector("#remoteRefreshBtn");
const remoteConnectBtn = document.querySelector("#remoteConnectBtn");
const remoteDisconnectBtn = document.querySelector("#remoteDisconnectBtn");
const robotStateText = document.querySelector("#robotStateText");
const robotStateHint = document.querySelector("#robotStateHint");
const navStateText = document.querySelector("#navStateText");
const navStateHint = document.querySelector("#navStateHint");
const motionStateText = document.querySelector("#motionStateText");
const motionStateHint = document.querySelector("#motionStateHint");
const locStateText = document.querySelector("#locStateText");
const locStateHint = document.querySelector("#locStateHint");
const slamStateText = document.querySelector("#slamStateText");
const slamStateHint = document.querySelector("#slamStateHint");
const timeoutInfo = document.querySelector("#timeoutInfo");
const refreshBtn = document.querySelector("#refreshBtn");
const startMappingBtn = document.querySelector("#startMappingBtn");
const saveMapBtn = document.querySelector("#saveMapBtn");
const resetMappingBtn = document.querySelector("#resetMappingBtn");
const stopBtn = document.querySelector("#stopBtn");
const motionLinear = document.querySelector("#motionLinear");
const motionAngular = document.querySelector("#motionAngular");
const motionDuration = document.querySelector("#motionDuration");
const motionBadge = document.querySelector("#motionBadge");
const standUpBtn = document.querySelector("#standUpBtn");
const enableMotionBtn = document.querySelector("#enableMotionBtn");
const turnLeftBtn = document.querySelector("#turnLeftBtn");
const forwardBtn = document.querySelector("#forwardBtn");
const turnRightBtn = document.querySelector("#turnRightBtn");
const backwardBtn = document.querySelector("#backwardBtn");
const motionStopBtn = document.querySelector("#motionStopBtn");
const releaseSdkBtn = document.querySelector("#releaseSdkBtn");
const motionOutput = document.querySelector("#motionOutput");
const loadMapBtn = document.querySelector("#loadMapBtn");
const refreshMapsBtn = document.querySelector("#refreshMapsBtn");
const mapList = document.querySelector("#mapList");
const mapCanvas = document.querySelector("#mapCanvas");
const mapInfo = document.querySelector("#mapInfo");
const goalBadge = document.querySelector("#goalBadge");
const goalText = document.querySelector("#goalText");
const navSpeed = document.querySelector("#navSpeed");
const navTolerance = document.querySelector("#navTolerance");
const initYaw = document.querySelector("#initYaw");
const initPoseBtn = document.querySelector("#initPoseBtn");
const loadNavMapBtn = document.querySelector("#loadNavMapBtn");
const sendGoalBtn = document.querySelector("#sendGoalBtn");
const cancelNavBtn = document.querySelector("#cancelNavBtn");
const navOutput = document.querySelector("#navOutput");
const patrolBadge = document.querySelector("#patrolBadge");
const addPatrolPointBtn = document.querySelector("#addPatrolPointBtn");
const clearPatrolBtn = document.querySelector("#clearPatrolBtn");
const startPatrolBtn = document.querySelector("#startPatrolBtn");
const patrolList = document.querySelector("#patrolList");
const patrolOutput = document.querySelector("#patrolOutput");
const snapshotBtn = document.querySelector("#snapshotBtn");
const liveBtn = document.querySelector("#liveBtn");
const videoInfoBtn = document.querySelector("#videoInfoBtn");
const cameraImage = document.querySelector("#cameraImage");
const videoInfo = document.querySelector("#videoInfo");
let liveTimer = null;
let actionInFlight = false;
let selectedMapDir = "";
let currentMap = null;
let selectedGoal = null;
let patrolPoints = [];
let latestSummary = {};
let navPollTimer = null;
let motionHold = {
  active: false,
  timer: null,
  direction: "",
  inFlight: false,
};

function setRemotePill(label, state = "") {
  remoteModePill.textContent = label;
  remoteModePill.className = state ? `pill ${state}` : "pill";
}

function setBusy(label) {
  statusPill.textContent = label;
  statusPill.className = "pill busy";
}

function setOk(label) {
  statusPill.textContent = label;
  statusPill.className = "pill ok";
}

function setPlain(label) {
  statusPill.textContent = label;
  statusPill.className = "pill";
}

function formatResult(data) {
  const lines = [];
  lines.push(data.ok ? "OK" : "FAILED");
  if (data.stdout) {
    lines.push("");
    lines.push(data.stdout);
  }
  if (data.stderr) {
    lines.push("");
    lines.push("[stderr]");
    lines.push(data.stderr);
  }
  return lines.join("\n");
}

async function callApi(path, options = {}) {
  const res = await fetch(path, options);
  return res.json();
}

function postJson(path, payload = {}) {
  return callApi(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
}

function formatBytes(size) {
  if (!size) {
    return "0 B";
  }
  const units = ["B", "KB", "MB", "GB"];
  let value = size;
  let unit = 0;
  while (value >= 1024 && unit < units.length - 1) {
    value /= 1024;
    unit += 1;
  }
  return `${value.toFixed(unit === 0 ? 0 : 1)} ${units[unit]}`;
}

function formatRemoteStatus(data) {
  const portSummary = [
    `jump:${data.ports?.jump ? "up" : "down"}`,
    `orin:${data.ports?.orin ? "up" : "down"}`,
  ].join("  ");
  return [
    data.mode === "remote" && data.ok ? "云端中转已启用，并且已经接到狗" :
      data.mode === "remote" ? "本地已经连到云服务器，但狗侧回流没有接通" : "当前走局域网直连",
    `server: ${data.server || "-"}`,
    `local ports: ${data.local_ports?.jump || "-"} / ${data.local_ports?.orin || "-"}`,
    `ports: ${portSummary}`,
    `jump banner: ${data.banners?.jump || "-"}`,
    `orin banner: ${data.banners?.orin || "-"}`,
    `pid: ${data.pid || "-"}`,
    `key: ${data.key_ready ? "ready" : "missing"}`,
    data.stderr || data.stdout || "",
  ].filter(Boolean).join("\n");
}

async function refreshRemoteStatus() {
  remoteRefreshBtn.disabled = true;
  try {
    const data = await callApi("/api/remote/status");
    remoteOutput.textContent = formatRemoteStatus(data);
    if (data.mode === "remote" && data.ok) {
      setRemotePill("云端中转", "ok");
    } else if (data.mode === "remote") {
      setRemotePill("云端异常");
    } else {
      setRemotePill("局域网");
    }
    remoteConnectBtn.disabled = data.mode === "remote" && data.ok;
    remoteDisconnectBtn.disabled = !data.running && data.mode !== "remote";
  } catch (error) {
    remoteOutput.textContent = String(error);
    setRemotePill("失败");
  } finally {
    remoteRefreshBtn.disabled = false;
  }
}

async function connectRemoteTunnel() {
  remoteConnectBtn.disabled = true;
  remoteOutput.textContent = "正在连接云端通道...";
  try {
    const data = await callApi("/api/remote/connect", { method: "POST" });
    remoteOutput.textContent = formatRemoteStatus(data);
    await refreshRemoteStatus();
  } catch (error) {
    remoteOutput.textContent = String(error);
  }
}

async function disconnectRemoteTunnel() {
  remoteDisconnectBtn.disabled = true;
  remoteOutput.textContent = "正在断开云端通道...";
  try {
    const data = await callApi("/api/remote/disconnect", { method: "POST" });
    remoteOutput.textContent = formatRemoteStatus(data);
    await refreshRemoteStatus();
  } catch (error) {
    remoteOutput.textContent = String(error);
  }
}

async function refreshStatus() {
  refreshBtn.disabled = true;
  setBusy("读取中");
  statusOutput.textContent = "正在读取机器狗状态...";
  try {
    const data = await callApi("/api/status");
    statusOutput.textContent = formatResult(data);
    latestSummary = data.summary || {};
    renderSummary(data.summary || {});
    if (data.summary?.overall === "在线") {
      setOk("在线");
    } else if (data.summary?.overall) {
      setPlain(data.summary.overall);
    } else {
      setPlain(data.ok ? "在线" : "异常");
    }
  } catch (error) {
    statusOutput.textContent = String(error);
    latestSummary = {};
    statusPill.textContent = "失败";
    statusPill.className = "pill";
    renderSummary({});
  } finally {
    refreshBtn.disabled = false;
  }
}

function renderSummary(summary) {
  robotStateText.textContent = summary.robot || "状态读取失败";
  robotStateHint.textContent = summary.pose
    ? `${summary.robot || ""} 当前位置 ${summary.pose}`.trim()
    : summary.robot || "请稍后重试。";
  navStateText.textContent = summary.navigation || "未知";
  navStateHint.textContent = summary.navigation_hint || "这里会说明当前是否正在执行导航。";
  motionStateText.textContent = summary.motion || "未知";
  motionStateHint.textContent = summary.motion_hint || "这里会说明底盘是否允许速度控制。";
  locStateText.textContent = summary.localization || "未知";
  locStateHint.textContent = summary.localization_hint || "这里会说明地图和定位是否准备好。";
  slamStateText.textContent = summary.mapping || "未知";
  slamStateHint.textContent = summary.mapping_hint || "这里会说明当前是不是在建图或已保存。";

  const timeouts = summary.timeouts || {};
  if (Object.keys(timeouts).length) {
    timeoutInfo.textContent = [
      `当前超时设置：状态 ${timeouts.status}s，开始建图 ${timeouts.slam_start}s，保存地图 ${timeouts.slam_save}s，重置建图 ${timeouts.slam_reset}s，停止指令 ${timeouts.stop_cmd}s。`,
      summary.recommendation || "",
    ].filter(Boolean).join(" ");
  } else {
    timeoutInfo.textContent = "超时配置读取中...";
  }

  applyActionState(summary);
}

function applyActionState(summary = {}) {
  if (actionInFlight) {
    return;
  }
  const mapping = summary.mapping || "未知";
  const knownState = Boolean(summary.mapping);

  startMappingBtn.disabled = knownState ? !summary.can_start_mapping : true;
  saveMapBtn.disabled = knownState ? !summary.can_save_map : true;
  resetMappingBtn.disabled = knownState ? !summary.can_reset_mapping : true;
  stopBtn.disabled = false;

  startMappingBtn.title = "";
  saveMapBtn.title = "";
  resetMappingBtn.title = "";

  if (mapping === "建图中") {
    startMappingBtn.title = "当前已经在建图，不需要重复开始。";
    resetMappingBtn.title = "正在建图时不要重置，先保存地图或停止操作。";
  } else if (mapping === "已保存") {
    startMappingBtn.title = "地图已保存完成，需要先重置建图状态。";
    saveMapBtn.title = "当前没有正在建图的任务可保存。";
  } else if (mapping === "待机") {
    saveMapBtn.title = "还没有开始建图，不能保存。";
    resetMappingBtn.title = "当前不需要重置。";
  } else {
    startMappingBtn.title = "先刷新状态，确认建图模块可用。";
    saveMapBtn.title = "先刷新状态，确认是否正在建图。";
    resetMappingBtn.title = "先刷新状态，确认是否需要重置。";
  }
}

async function postAction(path, label) {
  if (!confirm(label)) {
    return;
  }
  actionInFlight = true;
  for (const btn of [startMappingBtn, saveMapBtn, resetMappingBtn, stopBtn]) {
    btn.disabled = true;
  }
  actionOutput.textContent = "正在执行...";
  try {
    const data = await callApi(path, { method: "POST" });
    actionOutput.textContent = formatResult(data);
  } catch (error) {
    actionOutput.textContent = String(error);
  } finally {
    actionInFlight = false;
    await refreshStatus();
  }
}

function motionValues(direction) {
  const linear = Number(motionLinear.value || 0.12);
  const angular = Number(motionAngular.value || 0.35);
  const duration = Number(motionDuration.value || 0.6);
  if (direction === "forward") {
    return { linear, angular: 0, duration };
  }
  if (direction === "backward") {
    return { linear: -linear, angular: 0, duration };
  }
  if (direction === "left") {
    return { linear: 0, angular, duration };
  }
  if (direction === "right") {
    return { linear: 0, angular: -angular, duration };
  }
  return { linear: 0, angular: 0, duration: 0.2 };
}

async function pulseMotion(direction) {
  const payload = motionValues(direction);
  const buttons = [turnLeftBtn, forwardBtn, turnRightBtn, backwardBtn, motionStopBtn];
  for (const btn of buttons) {
    btn.disabled = true;
  }
  motionOutput.textContent = "正在发送点动指令...";
  try {
    const data = await postJson("/api/motion/pulse", payload);
    motionOutput.textContent = formatResult(data);
  } catch (error) {
    motionOutput.textContent = String(error);
  } finally {
    for (const btn of buttons) {
      btn.disabled = false;
    }
  }
}

function setMotionBusy(active, label = "按住移动") {
  motionBadge.textContent = label;
  motionBadge.className = active ? "pill busy" : "pill";
}

function bindHoldMotion(button, direction) {
  const start = (event) => {
    event.preventDefault();
    startHoldMotion(direction);
  };
  const end = (event) => {
    event.preventDefault();
    stopHoldMotion();
  };
  button.addEventListener("pointerdown", start);
  button.addEventListener("pointerup", end);
  button.addEventListener("pointercancel", end);
  button.addEventListener("pointerleave", () => {
    if (motionHold.active && motionHold.direction === direction) {
      stopHoldMotion();
    }
  });
  button.addEventListener("contextmenu", (event) => event.preventDefault());
}

function startHoldMotion(direction) {
  if (motionHold.active) {
    return;
  }
  motionHold.active = true;
  motionHold.direction = direction;
  setMotionBusy(true, "运动中");
  motionOutput.textContent = "按住中，正在连续发送低速指令...";
  sendHoldTick();
  motionHold.timer = setInterval(sendHoldTick, 700);
}

async function sendHoldTick() {
  if (!motionHold.active || motionHold.inFlight) {
    return;
  }
  motionHold.inFlight = true;
  const payload = motionValues(motionHold.direction);
  payload.duration = Math.min(Number(motionDuration.value || 0.45), 0.45);
  try {
    const data = await postJson("/api/motion/stream", payload);
    if (!data.ok) {
      motionOutput.textContent = formatResult(data);
    }
  } catch (error) {
    motionOutput.textContent = String(error);
  } finally {
    motionHold.inFlight = false;
  }
}

async function stopHoldMotion() {
  if (!motionHold.active) {
    return;
  }
  motionHold.active = false;
  motionHold.direction = "";
  motionHold.inFlight = false;
  if (motionHold.timer) {
    clearInterval(motionHold.timer);
    motionHold.timer = null;
  }
  setMotionBusy(false);
  await stopMotionNow();
}

async function stopMotionNow() {
  motionStopBtn.disabled = true;
  motionOutput.textContent = "正在发送停止指令...";
  try {
    const data = await callApi("/api/stop", { method: "POST" });
    motionOutput.textContent = formatResult(data);
    await refreshStatus();
  } catch (error) {
    motionOutput.textContent = String(error);
  } finally {
    motionStopBtn.disabled = false;
  }
}

async function releaseSdkControl() {
  if (!confirm("确认释放遥控器？页面会停止 SDK 接管，遥控器应恢复控制。")) {
    return;
  }
  releaseSdkBtn.disabled = true;
  motionOutput.textContent = "正在释放遥控器控制权...";
  try {
    const data = await callApi("/api/motion/release", { method: "POST" });
    motionOutput.textContent = formatResult(data);
    await refreshStatus();
  } catch (error) {
    motionOutput.textContent = String(error);
  } finally {
    releaseSdkBtn.disabled = false;
  }
}

async function enableMotionMode() {
  if (!confirm("确认进入运动模式？请确保机器狗已经站稳，周围安全。")) {
    return;
  }
  enableMotionBtn.disabled = true;
  motionOutput.textContent = "正在请求进入运动模式...";
  try {
    const data = await postJson("/api/motion/mode", { mode: 1 });
    motionOutput.textContent = formatResult(data);
    await refreshStatus();
  } catch (error) {
    motionOutput.textContent = String(error);
  } finally {
    enableMotionBtn.disabled = false;
  }
}

async function standUp() {
  if (!confirm("确认发送起立命令？请确保机器狗周围安全。")) {
    return;
  }
  standUpBtn.disabled = true;
  motionOutput.textContent = "正在发送起立命令...";
  try {
    const data = await postJson("/api/motion/stand");
    motionOutput.textContent = formatResult(data);
    await refreshStatus();
  } catch (error) {
    motionOutput.textContent = String(error);
  } finally {
    standUpBtn.disabled = false;
  }
}

function parsePgm(bytes) {
  let offset = 0;
  function readToken() {
    while (offset < bytes.length) {
      const ch = String.fromCharCode(bytes[offset]);
      if (/\s/.test(ch)) {
        offset += 1;
        continue;
      }
      if (ch === "#") {
        while (offset < bytes.length && String.fromCharCode(bytes[offset]) !== "\n") {
          offset += 1;
        }
        continue;
      }
      break;
    }
    let token = "";
    while (offset < bytes.length) {
      const ch = String.fromCharCode(bytes[offset]);
      if (/\s/.test(ch)) {
        offset += 1;
        break;
      }
      token += ch;
      offset += 1;
    }
    return token;
  }

  const magic = readToken();
  const width = Number(readToken());
  const height = Number(readToken());
  const max = Number(readToken());
  if (magic !== "P5" || !width || !height || max <= 0 || max > 255) {
    throw new Error("不支持的 PGM 地图格式");
  }
  return { width, height, pixels: bytes.slice(offset, offset + width * height) };
}

function drawMap(pgm, goal = null) {
  mapCanvas.width = pgm.width;
  mapCanvas.height = pgm.height;
  const ctx = mapCanvas.getContext("2d");
  const image = ctx.createImageData(pgm.width, pgm.height);
  for (let i = 0; i < pgm.pixels.length; i += 1) {
    const value = pgm.pixels[i];
    image.data[i * 4] = value;
    image.data[i * 4 + 1] = value;
    image.data[i * 4 + 2] = value;
    image.data[i * 4 + 3] = 255;
  }
  ctx.putImageData(image, 0, 0);
  if (goal) {
    ctx.save();
    ctx.lineWidth = Math.max(2, Math.round(pgm.width / 240));
    ctx.strokeStyle = "#e03b2f";
    ctx.fillStyle = "rgba(224, 59, 47, 0.18)";
    ctx.beginPath();
    ctx.arc(goal.pixelX, goal.pixelY, Math.max(6, pgm.width / 90), 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(goal.pixelX - 10, goal.pixelY);
    ctx.lineTo(goal.pixelX + 10, goal.pixelY);
    ctx.moveTo(goal.pixelX, goal.pixelY - 10);
    ctx.lineTo(goal.pixelX, goal.pixelY + 10);
    ctx.stroke();
    ctx.restore();
  }
}

function parseMapMetadata(yaml, fallback = {}) {
  const resolution = fallback.resolution || Number((yaml.match(/resolution:\s*([0-9.]+)/) || [])[1]);
  const originText = (yaml.match(/origin:\s*\[([^\]]+)\]/) || [])[1] || "";
  const origin = originText
    .split(",")
    .map((item) => Number(item.trim()))
    .filter((item) => Number.isFinite(item));
  return {
    resolution,
    origin: origin.length >= 2 ? origin : fallback.origin || [0, 0, 0],
  };
}

function pixelToMap(pixelX, pixelY) {
  if (!currentMap?.metadata?.resolution || !currentMap?.pgm) {
    return null;
  }
  const { resolution, origin } = currentMap.metadata;
  const x = origin[0] + pixelX * resolution;
  const y = origin[1] + (currentMap.pgm.height - pixelY) * resolution;
  return { x, y };
}

function localizationLooksReady(summary = latestSummary) {
  const text = [
    summary.localization,
    summary.localization_hint,
    summary.overall,
  ].filter(Boolean).join(" ").toLowerCase();
  if (!text) {
    return false;
  }
  const badWords = [
    "failed",
    "failure",
    "error",
    "未加载",
    "等待",
    "standby",
    "waiting",
    "not loaded",
    "reached max",
  ];
  return !badWords.some((word) => text.includes(word));
}

function startNavPolling() {
  if (navPollTimer) {
    clearInterval(navPollTimer);
  }
  navPollTimer = setInterval(refreshStatus, 3500);
}

function stopNavPolling() {
  if (navPollTimer) {
    clearInterval(navPollTimer);
    navPollTimer = null;
  }
}

function updateGoal(goal) {
  selectedGoal = goal;
  if (currentMap?.pgm) {
    drawMap(currentMap.pgm, selectedGoal);
  }
  if (!goal) {
    goalBadge.textContent = "未选点";
    goalBadge.className = "pill";
    goalText.textContent = "请在地图上点击目标位置";
    sendGoalBtn.disabled = true;
    initPoseBtn.disabled = true;
    return;
  }
  goalBadge.textContent = "已选点";
  goalBadge.className = "pill ok";
  goalText.textContent = `x=${goal.x.toFixed(3)}, y=${goal.y.toFixed(3)}`;
  sendGoalBtn.disabled = false;
  initPoseBtn.disabled = false;
}

async function loadMaps() {
  refreshMapsBtn.disabled = true;
  mapList.textContent = "正在读取地图列表...";
  try {
    const data = await callApi("/api/maps");
    if (!data.ok) {
      mapList.textContent = formatResult(data);
      return;
    }
    renderMapList(data.maps || []);
  } catch (error) {
    mapList.textContent = String(error);
  } finally {
    refreshMapsBtn.disabled = false;
  }
}

function renderMapList(maps) {
  mapList.innerHTML = "";
  if (!maps.length) {
    mapList.textContent = "还没有找到可用地图。保存地图后这里会出现记录。";
    selectedMapDir = "";
    return;
  }
  if (!selectedMapDir) {
    selectedMapDir = maps[0].directory;
  }
  for (const item of maps) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "map-item";
    if (item.directory === selectedMapDir) {
      button.classList.add("selected");
    }
    const title = document.createElement("strong");
    title.textContent = item.label;
    const detail = document.createElement("span");
    detail.textContent = `${item.updated_at} · ${formatBytes(item.total_size)} · ${item.has_pcd ? "含点云" : "无点云"}`;
    const path = document.createElement("small");
    path.textContent = item.directory;
    button.append(title, detail, path);
    button.addEventListener("click", async () => {
      selectedMapDir = item.directory;
      renderMapList(maps);
      await loadMap();
    });
    mapList.appendChild(button);
  }
}

async function loadMap() {
  loadMapBtn.disabled = true;
  mapInfo.textContent = "正在读取地图...";
  try {
    const query = selectedMapDir ? `?dir=${encodeURIComponent(selectedMapDir)}` : "";
    const data = await callApi(`/api/map${query}`);
    if (!data.ok || !data.exists) {
      mapInfo.textContent = formatResult(data);
      return;
    }
    const raw = atob(data.pgm_b64);
    const bytes = new Uint8Array(raw.length);
    for (let i = 0; i < raw.length; i += 1) {
      bytes[i] = raw.charCodeAt(i);
    }
    const pgm = parsePgm(bytes);
    const metadata = parseMapMetadata(data.yaml, data.metadata || {});
    currentMap = { pgm, metadata, directory: data.directory, name: data.name };
    updateGoal(null);
    drawMap(pgm);
    mapInfo.textContent = [
      `地图：${data.name}`,
      `目录：${data.directory}`,
      `尺寸：${pgm.width} x ${pgm.height}`,
      `分辨率：${metadata.resolution || "未知"} m/px`,
      `原点：${(metadata.origin || []).join(", ")}`,
      `文件：${data.files.pgm}`,
      `PCD：${data.files.pcd} (${data.sizes.pcd} bytes)`,
      "",
      data.yaml,
    ].join("\n");
  } catch (error) {
    mapInfo.textContent = String(error);
  } finally {
    loadMapBtn.disabled = false;
  }
}

async function setInitialPoseFromSelection() {
  if (!selectedGoal) {
    navOutput.textContent = "请先在地图上点击机器狗当前所在位置。";
    return;
  }
  if (!confirm(`确认把选点设为机器狗当前位姿 x=${selectedGoal.x.toFixed(2)}, y=${selectedGoal.y.toFixed(2)}？`)) {
    return;
  }
  initPoseBtn.disabled = true;
  navOutput.textContent = "正在发送初始位姿...";
  try {
    const data = await postJson("/api/localization/initial_pose", {
      x: selectedGoal.x,
      y: selectedGoal.y,
      yaw: Number(initYaw.value || 0),
    });
    navOutput.textContent = formatResult(data);
    await new Promise((resolve) => setTimeout(resolve, 2500));
    await refreshStatus();
  } catch (error) {
    navOutput.textContent = String(error);
  } finally {
    initPoseBtn.disabled = false;
  }
}

function selectGoalFromEvent(event) {
  if (!currentMap?.pgm) {
    navOutput.textContent = "请先读取一张地图。";
    return;
  }
  const rect = mapCanvas.getBoundingClientRect();
  const pixelX = (event.clientX - rect.left) * (mapCanvas.width / rect.width);
  const pixelY = (event.clientY - rect.top) * (mapCanvas.height / rect.height);
  const mapPoint = pixelToMap(pixelX, pixelY);
  if (!mapPoint) {
    navOutput.textContent = "地图元数据不完整，无法转换坐标。";
    return;
  }
  updateGoal({ ...mapPoint, pixelX, pixelY });
  navOutput.textContent = `已选目标点：x=${mapPoint.x.toFixed(3)}, y=${mapPoint.y.toFixed(3)}。`;
}

async function loadNavMap() {
  if (!selectedMapDir) {
    navOutput.textContent = "请先在地图列表里选择一张地图。";
    return;
  }
  loadNavMapBtn.disabled = true;
  navOutput.textContent = "正在加载地图到定位模块...";
  try {
    const data = await postJson("/api/nav/load_map", { dir: selectedMapDir });
    navOutput.textContent = formatResult(data);
    await refreshStatus();
  } catch (error) {
    navOutput.textContent = String(error);
  } finally {
    loadNavMapBtn.disabled = false;
  }
}

async function sendGoal() {
  if (!selectedGoal) {
    navOutput.textContent = "请先在地图上点击目标点。";
    return;
  }
  if (!confirm(`确认导航到 x=${selectedGoal.x.toFixed(2)}, y=${selectedGoal.y.toFixed(2)}？请确保机器狗周围安全。`)) {
    return;
  }
  sendGoalBtn.disabled = true;
  navOutput.textContent = "正在发送导航目标...";
  try {
    const data = await postJson("/api/nav/goal", {
      x: selectedGoal.x,
      y: selectedGoal.y,
      speed: Number(navSpeed.value || 0.25),
      tolerance: Number(navTolerance.value || 0.35),
    });
    navOutput.textContent = formatResult(data);
    await refreshStatus();
  } catch (error) {
    navOutput.textContent = String(error);
  } finally {
    sendGoalBtn.disabled = false;
  }
}

async function sendGoalGuarded() {
  if (!selectedGoal) {
    navOutput.textContent = "请先在地图上点击目标点。";
    return;
  }
  if (!localizationLooksReady()) {
    navOutput.textContent = [
      "定位还没有准备好，先不要发送导航目标。",
      `当前定位：${latestSummary.localization || "未知"}`,
      "请先加载地图，然后在地图上点击机器狗当前所在位置，发送初始位姿。",
    ].join("\n");
    return;
  }
  if (!confirm(`确认导航到 x=${selectedGoal.x.toFixed(2)}, y=${selectedGoal.y.toFixed(2)}？请确保机器狗周围安全。`)) {
    return;
  }
  sendGoalBtn.disabled = true;
  navOutput.textContent = "正在发送导航目标...";
  try {
    const data = await postJson("/api/nav/goal", {
      x: selectedGoal.x,
      y: selectedGoal.y,
      speed: Number(navSpeed.value || 0.25),
      tolerance: Number(navTolerance.value || 0.35),
    });
    navOutput.textContent = formatResult(data);
    await refreshStatus();
    if (data.ok) {
      startNavPolling();
    }
  } catch (error) {
    navOutput.textContent = String(error);
  } finally {
    sendGoalBtn.disabled = false;
  }
}

async function cancelNavigation() {
  if (!confirm("确认取消当前导航并发送停止指令？")) {
    return;
  }
  cancelNavBtn.disabled = true;
  stopNavPolling();
  navOutput.textContent = "正在取消导航...";
  try {
    const data = await postJson("/api/nav/cancel");
    navOutput.textContent = formatResult(data);
    await refreshStatus();
  } catch (error) {
    navOutput.textContent = String(error);
  } finally {
    cancelNavBtn.disabled = false;
  }
}

function refreshSnapshot() {
  cameraImage.src = `/api/video/snapshot?t=${Date.now()}`;
}

async function loadVideoInfo() {
  videoInfoBtn.disabled = true;
  videoInfo.textContent = "正在读取视频状态...";
  try {
    const data = await callApi("/api/video/info");
    videoInfo.textContent = formatResult(data);
  } catch (error) {
    videoInfo.textContent = String(error);
  } finally {
    videoInfoBtn.disabled = false;
  }
}

function toggleLive() {
  if (liveTimer) {
    clearInterval(liveTimer);
    liveTimer = null;
    liveBtn.textContent = "开始预览";
    return;
  }
  refreshSnapshot();
  liveTimer = setInterval(refreshSnapshot, 1500);
  liveBtn.textContent = "停止预览";
}

refreshBtn.addEventListener("click", refreshStatus);
startMappingBtn.addEventListener("click", () =>
  postAction("/api/slam/start", "确认开始建图？请确保机器狗在安全区域，并且有人看护。")
);
saveMapBtn.addEventListener("click", () =>
  postAction("/api/slam/save", "确认保存当前地图？")
);
resetMappingBtn.addEventListener("click", () =>
  postAction("/api/slam/reset", "确认重置建图状态？这会重启算法管理器，机器狗应保持原地、安全、有人看护。")
);
stopBtn.addEventListener("click", () =>
  postAction("/api/stop", "确认发送停止速度指令？")
);
standUpBtn.addEventListener("click", standUp);
enableMotionBtn.addEventListener("click", enableMotionMode);
bindHoldMotion(turnLeftBtn, "left");
bindHoldMotion(forwardBtn, "forward");
bindHoldMotion(turnRightBtn, "right");
bindHoldMotion(backwardBtn, "backward");
motionStopBtn.addEventListener("click", stopMotionNow);
releaseSdkBtn.addEventListener("click", releaseSdkControl);
loadMapBtn.addEventListener("click", loadMap);
refreshMapsBtn.addEventListener("click", loadMaps);
mapCanvas.addEventListener("click", selectGoalFromEvent);
initPoseBtn.addEventListener("click", setInitialPoseFromSelection);
loadNavMapBtn.addEventListener("click", loadNavMap);
sendGoalBtn.addEventListener("click", sendGoalGuarded);
cancelNavBtn.addEventListener("click", cancelNavigation);
snapshotBtn.addEventListener("click", refreshSnapshot);
liveBtn.addEventListener("click", toggleLive);
videoInfoBtn.addEventListener("click", loadVideoInfo);
remoteRefreshBtn.addEventListener("click", refreshRemoteStatus);
remoteConnectBtn.addEventListener("click", connectRemoteTunnel);
remoteDisconnectBtn.addEventListener("click", disconnectRemoteTunnel);

refreshStatus();
refreshRemoteStatus();
updateGoal(null);
loadMaps().then(loadMap);
loadVideoInfo();
