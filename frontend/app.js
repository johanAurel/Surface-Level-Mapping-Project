const state = {
  sessions: [],
  session: null,
  selected: new Set(),
  filterSelected: false,
  search: "",
  status: "connecting",
  lastUpdated: null,
  saveState: "idle",
  lastError: "",
  pendingEdits: new Map(),
  eventSource: null,
  eventRetryTimer: null,
  pollTimer: null,
  renderTimer: null,
  drag: null,
  hover: null,
  config: {
    x_length_m: 120,
    y_length_m: 56,
    block_size_m: 8,
    points_per_block: 4,
    reflector_height_m: 1.5,
    wheel_circumference_m: 0.5,
    start_x_m: 0,
    start_y_m: 0,
    start_yaw_deg: 0,
    tilt_mount_offset_deg: 0
  },
  metrics: { field: null, overview: null, depth: null },
  renderCache: { version: "", stats: new Map(), depthRange: null, visibleKey: "", visible: null }
};

const dom = {};
const COLORS = ["#48cae4", "#f4a261", "#4ade80", "#c084fc", "#f472b6", "#facc15", "#60a5fa", "#fb7185"];
const MAX_DEMO_POINTS = 1000000;
const MAX_RENDERED_FIELD_POINTS = 5000;
const MAX_RENDERED_CHART_POINTS = 2000;
const MAX_RENDERED_TABLE_ROWS = 500;
const NUMERIC_FIELDS = new Set([
  "scan_index", "x_m", "y_m", "depth_mm", "lidar_distance_m", "theta_v_deg", "theta_h_deg",
  "imu_pitch_deg", "yaw_deg", "platform_x_m", "platform_y_m"
]);
const PATCHABLE_FIELDS = new Set([
  "timestamp", "x_m", "y_m", "depth_mm", "lidar_distance_m", "theta_v_deg",
  "theta_h_deg", "imu_pitch_deg", "yaw_deg", "platform_x_m", "platform_y_m", "accepted", "source"
]);
const EDITABLE_COLUMNS = [
  "timestamp", "x_m", "y_m", "depth_mm", "lidar_distance_m", "theta_v_deg",
  "theta_h_deg", "imu_pitch_deg", "yaw_deg", "platform_x_m", "platform_y_m", "accepted", "source"
];

function init() {
  cacheDom();
  bindEvents();
  render();
  loadSessions();
  window.addEventListener("resize", () => requestRender());
}

function cacheDom() {
  dom.sessionSelect = document.getElementById("sessionSelect");
  dom.sessionName = document.getElementById("sessionName");
  dom.statusChip = document.getElementById("statusChip");
  dom.sessionId = document.getElementById("sessionId");
  dom.newSessionBtn = document.getElementById("newSessionBtn");
  dom.saveDraftBtn = document.getElementById("saveDraftBtn");
  dom.refreshBtn = document.getElementById("refreshBtn");
  dom.connection = document.getElementById("connection");
  dom.connectionText = document.getElementById("connectionText");
  dom.configForm = document.getElementById("configForm");
  dom.gridInfo = document.getElementById("gridInfo");
  dom.configErrors = document.getElementById("configErrors");
  dom.applyConfigBtn = document.getElementById("applyConfigBtn");
  dom.startWithConfigBtn = document.getElementById("startWithConfigBtn");
  dom.fieldWrap = document.getElementById("fieldWrap");
  dom.fieldCanvas = document.getElementById("fieldCanvas");
  dom.fieldTooltip = document.getElementById("fieldTooltip");
  dom.fieldSummary = document.getElementById("fieldSummary");
  dom.overviewCanvas = document.getElementById("overviewCanvas");
  dom.depthCanvas = document.getElementById("depthCanvas");
  dom.depthLegend = document.getElementById("depthLegend");
  dom.selectedCount = document.getElementById("selectedCount");
  dom.selectedList = document.getElementById("selectedList");
  dom.demoCount = document.getElementById("demoCount");
  dom.rescanBtn = document.getElementById("rescanBtn");
  dom.clearBlockBtn = document.getElementById("clearBlockBtn");
  dom.demoBtn = document.getElementById("demoBtn");
  dom.completeBtn = document.getElementById("completeBtn");
  dom.clearSelectionBtn = document.getElementById("clearSelectionBtn");
  dom.selectedOnly = document.getElementById("selectedOnly");
  dom.pointSearch = document.getElementById("pointSearch");
  dom.tableCount = document.getElementById("tableCount");
  dom.pointsBody = document.getElementById("pointsBody");
  dom.statusPoints = document.getElementById("statusPoints");
  dom.statusBlocks = document.getElementById("statusBlocks");
  dom.statusUpdated = document.getElementById("statusUpdated");
  dom.statusSave = document.getElementById("statusSave");
  dom.errorText = document.getElementById("errorText");
  dom.statusSession = document.getElementById("statusSession");
}

function bindEvents() {
  dom.sessionSelect.addEventListener("change", onSessionChange);
  dom.sessionName.addEventListener("change", onSessionNameChange);
  dom.newSessionBtn.addEventListener("click", onNewScan);
  dom.saveDraftBtn.addEventListener("click", onSaveDraft);
  dom.refreshBtn.addEventListener("click", () => { loadSessions(); if (state.session) loadSession(state.session.id); });
  dom.applyConfigBtn.addEventListener("click", onApplyConfig);
  dom.startWithConfigBtn.addEventListener("click", onStartWithConfig);
  dom.configForm.addEventListener("input", () => renderConfigValidation());
  dom.rescanBtn.addEventListener("click", onRescanSelected);
  dom.clearBlockBtn.addEventListener("click", onClearBlockPoints);
  dom.demoBtn.addEventListener("click", onAddDemo);
  dom.completeBtn.addEventListener("click", onMarkComplete);
  dom.clearSelectionBtn.addEventListener("click", () => { state.selected.clear(); requestRender(); });
  dom.selectedOnly.addEventListener("change", (e) => { state.filterSelected = e.target.checked; requestRender(); });
  dom.pointSearch.addEventListener("input", (e) => { state.search = e.target.value || ""; requestRender(); });
  dom.selectedList.addEventListener("click", onSelectedListClick);
  dom.pointsBody.addEventListener("change", onTableChange);
  dom.pointsBody.addEventListener("keydown", onTableKeydown);
  dom.pointsBody.addEventListener("click", onTableClick);
  bindFieldEvents();
  bindOverviewEvents();
}

function bindFieldEvents() {
  const canvas = dom.fieldCanvas;
  canvas.addEventListener("mousedown", onFieldMouseDown);
  canvas.addEventListener("mousemove", onFieldMouseMove);
  canvas.addEventListener("mouseleave", () => { state.hover = null; dom.fieldTooltip.hidden = true; requestRender(); });
  window.addEventListener("mousemove", onFieldDragMove);
  window.addEventListener("mouseup", onFieldMouseUp);
}

function bindOverviewEvents() {
  dom.overviewCanvas.addEventListener("click", (e) => {
    const hit = overviewHit(e);
    if (!hit) return;
    const key = blockKey(hit.row, hit.col);
    state.selected.clear();
    state.selected.add(key);
    requestRender();
  });
}

async function api(path, options = {}) {
  const method = options.method || "GET";
  const initOpts = { method, headers: { "Accept": "application/json" } };
  if (method !== "GET") {
    initOpts.headers["Content-Type"] = "application/json";
  }
  if (options.body !== undefined) {
    initOpts.body = JSON.stringify(options.body);
  }
  const res = await fetch(path, initOpts);
  const text = await res.text();
  let data = null;
  if (text) {
    try { data = JSON.parse(text); } catch { data = text; }
  }
  if (!res.ok) {
    const message = data && data.error ? data.error : (typeof data === "string" && data ? data : `${res.status} ${res.statusText}`);
    throw new Error(message);
  }
  return data;
}

function setConnection(status) {
  state.status = status;
  dom.connection.className = `connection ${status}`;
  dom.connectionText.textContent = status;
}

function setError(message) {
  state.lastError = message || "";
  dom.errorText.textContent = state.lastError;
  if (message) console.error(message);
}

function setSaveState(value) {
  state.saveState = value;
  dom.statusSave.textContent = value;
  dom.statusSave.className = value;
}

function requestRender() {
  if (state.renderTimer) cancelAnimationFrame(state.renderTimer);
  state.renderTimer = requestAnimationFrame(() => {
    state.renderTimer = null;
    render();
  });
}

function render() {
  buildRenderContext();
  renderHeader();
  renderConfig();
  renderField();
  renderSelectedPanel();
  renderDepth();
  renderTable();
  renderStatus();
}

function renderHeader() {
  const session = state.session;
  const sessions = state.sessions.slice().sort((a, b) => String(b.updated_at || "").localeCompare(String(a.updated_at || "")));
  dom.sessionSelect.innerHTML = "";
  const placeholder = document.createElement("option");
  placeholder.value = "";
  placeholder.textContent = sessions.length ? "Select session" : "No sessions";
  dom.sessionSelect.appendChild(placeholder);
  for (const item of sessions) {
    const opt = document.createElement("option");
    opt.value = item.id;
    const name = item.name || item.id;
    opt.textContent = `${name} (${item.status || "unknown"})`;
    if (session && item.id === session.id) opt.selected = true;
    dom.sessionSelect.appendChild(opt);
  }
  if (session) dom.sessionSelect.value = session.id;
  if (document.activeElement !== dom.sessionName) dom.sessionName.value = session ? (session.name || "") : "";
  const status = session ? (session.status || "draft") : "no session";
  dom.statusChip.textContent = status;
  dom.statusChip.className = `chip${session ? ` status-${status}` : ""}`;
  dom.sessionId.textContent = session ? session.id : "-";
  dom.sessionId.title = session ? `Session id: ${session.id}` : "Session id";
  dom.saveDraftBtn.disabled = !session;
  dom.applyConfigBtn.disabled = !session;
}

function renderConfig() {
  const source = state.session ? state.session.config : state.config;
  for (const key of Object.keys(state.config)) {
    const input = dom.configForm.elements.namedItem(key);
    if (!input || document.activeElement === input) continue;
    const value = source ? source[key] : state.config[key];
    input.value = formatConfigValue(key, value);
  }
  renderConfigValidation();
}

function formatConfigValue(key, value) {
  if (key === "points_per_block") return formatPointsPerBlock(value);
  if (value === null || value === undefined) return "";
  return String(value);
}

function formatPointsPerBlock(value) {
  if (Array.isArray(value)) return value.map((row) => Array.isArray(row) ? row.join(" ") : String(row)).join("; ");
  if (value && typeof value === "object" && Array.isArray(value.data)) {
    return formatPointsPerBlock(value.data);
  }
  return value === undefined || value === null ? "" : String(value);
}

function readConfigForm() {
  const errors = [];
  const config = {};
  const numericKeys = ["x_length_m", "y_length_m", "block_size_m", "reflector_height_m", "wheel_circumference_m", "start_x_m", "start_y_m", "start_yaw_deg", "tilt_mount_offset_deg"];
  for (const key of numericKeys) {
    const raw = String(dom.configForm.elements.namedItem(key).value).trim();
    const num = Number(raw);
    if (!raw || !Number.isFinite(num)) errors.push(`${key} must be a number`);
    config[key] = num;
  }
  const ppbRaw = String(dom.configForm.elements.namedItem("points_per_block").value).trim();
  config.points_per_block = parsePointsPerBlock(ppbRaw, errors);
  validateConfig(config, errors);
  return { config, errors };
}

function parsePointsPerBlock(raw, errors) {
  if (!raw) {
    errors.push("points_per_block is required");
    return null;
  }
  if (!/[;\s]/.test(raw)) {
    const n = Number(raw);
    if (!Number.isInteger(n)) errors.push("points_per_block must be an integer or 2D array");
    return n;
  }
  const rows = raw.split(";").map((r) => r.trim()).filter(Boolean).map((row) => row.split(/[\s,]+/).filter(Boolean).map(Number));
  if (!rows.length || rows.some((r) => !r.length || r.some((v) => !Number.isInteger(v)))) {
    errors.push("points_per_block 2D array must contain integers");
    return rows;
  }
  const width = rows[0].length;
  if (rows.some((r) => r.length !== width)) errors.push("points_per_block 2D array rows must have same length");
  return rows;
}

function validateConfig(config, errors) {
  if (!(config.x_length_m > 0)) errors.push("x_length_m must be > 0");
  if (!(config.y_length_m > 0)) errors.push("y_length_m must be > 0");
  if (!(config.block_size_m > 0)) errors.push("block_size_m must be > 0");
  if (!(config.reflector_height_m > 0)) errors.push("reflector_height_m must be > 0");
  if (!(config.wheel_circumference_m > 0)) errors.push("wheel_circumference_m must be > 0");
  if (config.x_length_m > 0 && config.block_size_m > 0) {
    const cols = config.x_length_m / config.block_size_m;
    if (Math.abs(cols - Math.round(cols)) > 1e-9) errors.push("block_size_m must divide x_length_m exactly");
  }
  if (config.y_length_m > 0 && config.block_size_m > 0) {
    const rows = config.y_length_m / config.block_size_m;
    if (Math.abs(rows - Math.round(rows)) > 1e-9) errors.push("block_size_m must divide y_length_m exactly");
  }
  if (Array.isArray(config.points_per_block)) {
    const rows = config.points_per_block;
    const expectedRows = Math.round(config.y_length_m / config.block_size_m);
    const expectedCols = Math.round(config.x_length_m / config.block_size_m);
    if (rows.length !== expectedRows || rows[0]?.length !== expectedCols) {
      errors.push(`points_per_block array must be ${expectedRows}x${expectedCols}`);
    }
    if (rows.flat().some((v) => v < 0)) errors.push("points_per_block values cannot be negative");
  } else if (Number.isInteger(config.points_per_block)) {
    if (config.points_per_block < 1) errors.push("points_per_block must be at least 1");
  }
}

function renderConfigValidation() {
  const { config, errors } = readConfigForm();
  dom.configErrors.innerHTML = "";
  for (const err of [...new Set(errors)]) {
    const div = document.createElement("div");
    div.className = "error-line";
    div.textContent = err;
    dom.configErrors.appendChild(div);
  }
  const rows = Math.round(config.y_length_m / config.block_size_m);
  const cols = Math.round(config.x_length_m / config.block_size_m);
  dom.gridInfo.textContent = Number.isFinite(rows) && Number.isFinite(cols) && rows > 0 && cols > 0 ? `${rows} x ${cols}` : "-";
  return { config, errors: [...new Set(errors)] };
}

function blockKey(row, col) { return `${row},${col}`; }
function parseBlockKey(key) { const [r, c] = key.split(",").map(Number); return { row: r, col: c }; }

function sessionBlocks() { return Array.isArray(state.session?.blocks) ? state.session.blocks : []; }
function sessionPoints() { return Array.isArray(state.session?.points) ? state.session.points : []; }

function getBlock(row, col) {
  return sessionBlocks().find((b) => Number(b.row) === Number(row) && Number(b.col) === Number(col));
}

function renderVersion() {
  return `${state.session?.id || ""}|${state.session?.updated_at || ""}`;
}

function buildRenderContext() {
  const version = renderVersion();
  if (state.renderCache.version === version) return state.renderCache;
  const stats = new Map();
  for (const block of sessionBlocks()) {
    const simulated = Math.max(0, Number(block.demo_simulated_count) || 0);
    stats.set(blockKey(block.row, block.col), {
      count: simulated,
      representedCount: 0,
      simulated,
      target: Number(block.target_count) || 0,
      completion: 0,
      depthSum: 0,
      depthCount: 0,
      avgDepth: null,
      points: []
    });
  }
  let minDepth = null;
  let maxDepth = null;
  for (const point of sessionPoints()) {
    const depth = Number(point.depth_mm);
    if (Number.isFinite(depth)) {
      if (minDepth === null || depth < minDepth) minDepth = depth;
      if (maxDepth === null || depth > maxDepth) maxDepth = depth;
    }
    if (point.accepted === false) continue;
    const stat = stats.get(blockKey(point.block_row, point.block_col));
    if (!stat) continue;
    stat.representedCount += 1;
    stat.count += 1;
    stat.points.push(point);
    if (Number.isFinite(depth)) {
      stat.depthSum += depth;
      stat.depthCount += 1;
    }
  }
  for (const stat of stats.values()) {
    stat.completion = stat.target > 0 ? Math.min(1, stat.count / stat.target) : (stat.count > 0 ? 1 : 0);
    stat.avgDepth = stat.depthCount ? stat.depthSum / stat.depthCount : null;
    delete stat.depthSum;
    delete stat.depthCount;
  }
  state.renderCache = {
    version,
    stats,
    depthRange: minDepth === null ? null : { min: minDepth, max: maxDepth },
    visibleKey: "",
    visible: null
  };
  return state.renderCache;
}

function blockStats(block) {
  const stats = buildRenderContext().stats.get(blockKey(block.row, block.col));
  return stats || { count: 0, representedCount: 0, simulated: 0, target: 0, completion: 0, avgDepth: null, points: [] };
}

function blockColor(block) {
  const stats = blockStats(block);
  if (stats.avgDepth !== null) return depthToColor(stats.avgDepth);
  const pct = stats.completion;
  if (pct <= 0) return "#111a24";
  const hue = 190 - pct * 35;
  const light = 18 + pct * 18;
  return `hsl(${hue} 58% ${light}%)`;
}

function depthToColor(depth) {
  const range = depthRange();
  if (!range) return "#1d5f76";
  const t = range.max === range.min ? 0.5 : (depth - range.min) / (range.max - range.min);
  return lerpColor("#2ec4b6", "#f4a261", "#e63946", clamp(t, 0, 1));
}

function depthRange() {
  return buildRenderContext().depthRange;
}

function lerpColor(a, b, c, t) {
  if (t <= 0.5) return mixHex(a, b, t / 0.5);
  return mixHex(b, c, (t - 0.5) / 0.5);
}

function mixHex(a, b, t) {
  const pa = hexToRgb(a); const pb = hexToRgb(b);
  const r = Math.round(pa.r + (pb.r - pa.r) * t);
  const g = Math.round(pa.g + (pb.g - pa.g) * t);
  const bl = Math.round(pa.b + (pb.b - pa.b) * t);
  return `rgb(${r},${g},${bl})`;
}

function hexToRgb(hex) {
  const v = hex.replace("#", "");
  return { r: parseInt(v.slice(0, 2), 16), g: parseInt(v.slice(2, 4), 16), b: parseInt(v.slice(4, 6), 16) };
}

function clamp(v, min, max) { return Math.max(min, Math.min(max, v)); }

function samplePoints(points, limit) {
  if (!Array.isArray(points) || points.length <= limit) return points || [];
  if (limit <= 1) return points.length ? [points[points.length - 1]] : [];
  const sampled = [];
  for (let i = 0; i < limit; i++) {
    const index = Math.round(i * (points.length - 1) / (limit - 1));
    if (!sampled.length || index > sampled[sampled.length - 1].index) {
      sampled.push({ index, point: points[index] });
    }
  }
  return sampled.map((item) => item.point);
}

function sampleSeries(series, limit) {
  const tagged = [];
  series.forEach((s, seriesIndex) => {
    s.points.forEach((point) => tagged.push({ seriesIndex, point }));
  });
  const chosen = samplePoints(tagged, limit);
  const sampled = series.map((s) => ({ ...s, points: [] }));
  chosen.forEach((item) => sampled[item.seriesIndex].points.push(item.point));
  return sampled;
}

function canvasSize(canvas, height) {
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(100, Math.floor(rect.width));
  const h = Math.max(100, Math.floor(height || rect.height));
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.floor(width * dpr) || canvas.height !== Math.floor(h * dpr)) {
    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(h * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, h);
  return { ctx, width, height: h, dpr };
}

function getFieldMetrics() {
  const session = state.session;
  const rect = dom.fieldCanvas.getBoundingClientRect();
  const width = Math.max(100, Math.floor(rect.width));
  const height = Math.max(100, Math.floor(rect.height));
  if (!session?.config) return null;
  const cfg = session.config;
  const xLen = Number(cfg.x_length_m) || 1;
  const yLen = Number(cfg.y_length_m) || 1;
  const pad = 28;
  const innerW = width - pad * 2;
  const innerH = height - pad * 2;
  const scale = Math.min(innerW / xLen, innerH / yLen);
  const drawW = xLen * scale;
  const drawH = yLen * scale;
  const ox = (width - drawW) / 2;
  const oy = (height - drawH) / 2;
  return { width, height, xLen, yLen, scale, drawW, drawH, ox, oy, pad };
}

function worldToCanvas(x, y, m) {
  return { x: m.ox + x * m.scale, y: m.oy + m.drawH - y * m.scale };
}

function canvasToWorld(x, y, m) {
  return { x: (x - m.ox) / m.scale, y: (m.oy + m.drawH - y) / m.scale };
}

function renderField() {
  const { ctx, width, height } = canvasSize(dom.fieldCanvas);
  const session = state.session;
  state.metrics.field = getFieldMetrics();
  ctx.fillStyle = "#0a1118";
  ctx.fillRect(0, 0, width, height);
  if (!session || !state.metrics.field) {
    ctx.fillStyle = "#6f8396";
    ctx.font = "13px sans-serif";
    ctx.fillText("Load or create a session to view the field.", 20, 30);
    dom.fieldSummary.textContent = "No session loaded";
    return;
  }
  const m = state.metrics.field;
  const blocks = sessionBlocks();
  ctx.strokeStyle = "#182533";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 10; i++) {
    const gx = m.ox + (m.drawW / 10) * i;
    const gy = m.oy + (m.drawH / 10) * i;
    ctx.beginPath(); ctx.moveTo(gx, m.oy); ctx.lineTo(gx, m.oy + m.drawH); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(m.ox, gy); ctx.lineTo(m.ox + m.drawW, gy); ctx.stroke();
  }
  for (const block of blocks) {
    const x0 = Number(block.col) * Number(session.config.block_size_m);
    const y0 = Number(block.row) * Number(session.config.block_size_m);
    const p0 = worldToCanvas(x0, y0, m);
    const p1 = worldToCanvas(x0 + Number(session.config.block_size_m), y0 + Number(session.config.block_size_m), m);
    const bx = p0.x;
    const by = p1.y;
    const bw = p1.x - p0.x;
    const bh = p0.y - p1.y;
    const key = blockKey(block.row, block.col);
    const selected = state.selected.has(key);
    const hovered = state.hover && state.hover.row === block.row && state.hover.col === block.col;
    ctx.fillStyle = blockColor(block);
    ctx.fillRect(bx, by, bw, bh);
    if (selected) {
      ctx.fillStyle = "rgba(250, 204, 21, 0.13)";
      ctx.fillRect(bx, by, bw, bh);
    }
    if (hovered) {
      ctx.fillStyle = "rgba(72, 202, 228, 0.08)";
      ctx.fillRect(bx, by, bw, bh);
    }
    ctx.strokeStyle = selected ? "#facc15" : "#2a4053";
    ctx.lineWidth = selected ? 2 : 1;
    ctx.strokeRect(bx + 0.5, by + 0.5, bw - 1, bh - 1);
    const stats = blockStats(block);
    if (bw > 34 && bh > 24) {
      ctx.fillStyle = selected ? "#fff3b0" : "#8ea7bc";
      ctx.font = "10px sans-serif";
      ctx.fillText(`${block.row},${block.col}`, bx + 4, by + 12);
      ctx.fillStyle = "#bfd0dd";
      ctx.fillText(`${stats.count}/${stats.target || 0}`, bx + 4, by + 24);
    }
  }

  const allPoints = sessionPoints();
  const points = samplePoints(allPoints, MAX_RENDERED_FIELD_POINTS);
  for (const p of points) {
    const x = Number(p.x_m);
    const y = Number(p.y_m);
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    const pos = worldToCanvas(x, y, m);
    const key = blockKey(p.block_row, p.block_col);
    const selected = state.selected.has(key);
    ctx.fillStyle = selected ? "#fff0a8" : "rgba(225, 245, 255, 0.82)";
    ctx.beginPath();
    ctx.arc(pos.x, pos.y, selected ? 2.7 : 2.1, 0, Math.PI * 2);
    ctx.fill();
  }

  if (state.drag && state.drag.current) {
    const a = state.drag.startPx;
    const b = state.drag.current;
    const x = Math.min(a.x, b.x);
    const y = Math.min(a.y, b.y);
    const w = Math.abs(a.x - b.x);
    const h = Math.abs(a.y - b.y);
    ctx.strokeStyle = "#48cae4";
    ctx.fillStyle = "rgba(72, 202, 228, 0.1)";
    ctx.lineWidth = 1;
    ctx.fillRect(x, y, w, h);
    ctx.strokeRect(x + 0.5, y + 0.5, w - 1, h - 1);
  }

  ctx.strokeStyle = "#6a8499";
  ctx.lineWidth = 1.2;
  ctx.strokeRect(m.ox, m.oy, m.drawW, m.drawH);
  ctx.fillStyle = "#7f95a8";
  ctx.font = "11px sans-serif";
  ctx.fillText("0", m.ox - 8, m.oy + m.drawH + 14);
  ctx.fillText(`${m.xLen.toFixed(1)} m`, m.ox + m.drawW - 24, m.oy + m.drawH + 14);
  ctx.save();
  ctx.translate(m.ox - 18, m.oy + 24);
  ctx.rotate(-Math.PI / 2);
  ctx.fillText(`${m.yLen.toFixed(1)} m`, 0, 0);
  ctx.restore();

  const complete = blocks.filter((b) => b.status === "complete").length;
  const simulated = blocks.reduce((total, b) => total + Math.max(0, Number(b.demo_simulated_count) || 0), 0);
  const logicalPoints = allPoints.length + simulated;
  const shown = points.length < allPoints.length ? `, ${points.length} shown` : "";
  dom.fieldSummary.textContent = `${blocks.length} blocks, ${complete} complete, ${logicalPoints} points${shown}`;
}

function fieldBlockAtEvent(e) {
  const m = state.metrics.field;
  if (!m || !state.session) return null;
  const rect = dom.fieldCanvas.getBoundingClientRect();
  const px = e.clientX - rect.left;
  const py = e.clientY - rect.top;
  const world = canvasToWorld(px, py, m);
  const bs = Number(state.session.config.block_size_m);
  const col = Math.floor(world.x / bs);
  const row = Math.floor(world.y / bs);
  const within = world.x >= 0 && world.y >= 0 && world.x <= m.xLen && world.y <= m.yLen;
  if (!within) return { px, py, world, row: null, col: null, within: false };
  const maxRow = Math.round(m.yLen / bs) - 1;
  const maxCol = Math.round(m.xLen / bs) - 1;
  return {
    px, py, world,
    row: clamp(row, 0, maxRow),
    col: clamp(col, 0, maxCol),
    within: true
  };
}

function onFieldMouseDown(e) {
  if (e.button !== 0) return;
  const hit = fieldBlockAtEvent(e);
  if (!hit || !hit.within) return;
  state.drag = {
    startPx: { x: hit.px, y: hit.py },
    startBlock: { row: hit.row, col: hit.col },
    current: { x: hit.px, y: hit.py },
    additive: e.shiftKey || e.ctrlKey || e.metaKey,
    toggle: e.ctrlKey || e.metaKey,
    moved: false
  };
}

function onFieldDragMove(e) {
  if (!state.drag) return;
  const rect = dom.fieldCanvas.getBoundingClientRect();
  const px = clamp(e.clientX - rect.left, 0, rect.width);
  const py = clamp(e.clientY - rect.top, 0, rect.height);
  const dx = px - state.drag.startPx.x;
  const dy = py - state.drag.startPx.y;
  if (Math.abs(dx) > 3 || Math.abs(dy) > 3) state.drag.moved = true;
  state.drag.current = { x: px, y: py };
  requestRender();
}

function onFieldMouseUp(e) {
  if (!state.drag) return;
  const drag = state.drag;
  state.drag = null;
  const hit = fieldBlockAtEvent(e);
  if (!hit) { requestRender(); return; }
  if (!drag.moved) {
    applyBlockSelection(drag.startBlock.row, drag.startBlock.col, { shift: e.shiftKey, toggle: e.ctrlKey || e.metaKey });
  } else {
    applyRangeSelection(drag.startBlock, hit.within ? { row: hit.row, col: hit.col } : drag.startBlock, drag.additive);
  }
  requestRender();
}

function applyBlockSelection(row, col, mods) {
  const key = blockKey(row, col);
  if (mods.shift && state.selected.size) {
    const anchor = parseBlockKey([...state.selected].at(-1));
    applyRangeSelection(anchor, { row, col }, true);
    return;
  }
  if (mods.toggle) {
    if (state.selected.has(key)) state.selected.delete(key);
    else state.selected.add(key);
    return;
  }
  state.selected.clear();
  state.selected.add(key);
}

function applyRangeSelection(a, b, additive) {
  if (!additive) state.selected.clear();
  const r0 = Math.min(a.row, b.row); const r1 = Math.max(a.row, b.row);
  const c0 = Math.min(a.col, b.col); const c1 = Math.max(a.col, b.col);
  for (let r = r0; r <= r1; r++) for (let c = c0; c <= c1; c++) state.selected.add(blockKey(r, c));
}

function onFieldMouseMove(e) {
  const hit = fieldBlockAtEvent(e);
  if (!hit || !hit.within || state.drag) {
    if (state.drag) {
      dom.fieldTooltip.hidden = true;
      return;
    }
    state.hover = null;
    dom.fieldTooltip.hidden = true;
    requestRender();
    return;
  }
  state.hover = { row: hit.row, col: hit.col };
  const block = getBlock(hit.row, hit.col);
  if (!block) { dom.fieldTooltip.hidden = true; return; }
  const stats = blockStats(block);
  const avg = stats.avgDepth === null ? "-" : `${stats.avgDepth.toFixed(1)} mm`;
  dom.fieldTooltip.textContent = `Block ${hit.row},${hit.col}\nStatus: ${block.status || "-"}\nPoints: ${stats.count}/${stats.target || 0}\nAvg depth: ${avg}`;
  dom.fieldTooltip.hidden = false;
  const wrapRect = dom.fieldWrap.getBoundingClientRect();
  const left = clamp(hit.px + 14, 8, wrapRect.width - 170);
  const top = clamp(hit.py + 12, 8, wrapRect.height - 70);
  dom.fieldTooltip.style.left = `${left}px`;
  dom.fieldTooltip.style.top = `${top}px`;
  requestRender();
}

function renderSelectedPanel() {
  const selected = [...state.selected].map(parseBlockKey).sort((a, b) => a.row - b.row || a.col - b.col);
  dom.selectedCount.textContent = String(selected.length);
  dom.selectedList.innerHTML = "";
  if (!selected.length) {
    const empty = document.createElement("div");
    empty.className = "selected-empty";
    empty.textContent = "No blocks selected";
    dom.selectedList.appendChild(empty);
  } else {
    for (const sel of selected) {
      const block = getBlock(sel.row, sel.col);
      const stats = block ? blockStats(block) : { count: 0, target: 0 };
      const item = document.createElement("div");
      item.className = "selected-item";
      item.innerHTML = `<span class="block-label">${sel.row},${sel.col}</span><span class="block-count">${stats.count}/${stats.target || 0} ${block?.status || ""}</span><button type="button" class="link-btn" data-remove="${sel.row},${sel.col}">remove</button>`;
      dom.selectedList.appendChild(item);
    }
  }
  const hasSession = !!state.session;
  const blocks = sessionBlocks();
  const allComplete = blocks.length > 0 && blocks.every((block) => block.status === "complete");
  dom.rescanBtn.disabled = !hasSession || !selected.length;
  dom.clearBlockBtn.disabled = !hasSession || !selected.length;
  dom.demoBtn.disabled = !hasSession || !selected.length;
  dom.clearSelectionBtn.disabled = !selected.length;
  dom.completeBtn.disabled = !hasSession || !allComplete;
}

function onSelectedListClick(e) {
  const btn = e.target.closest("[data-remove]");
  if (!btn) return;
  state.selected.delete(btn.dataset.remove);
  requestRender();
}

function renderDepth() {
  renderOverview();
  renderDepthChart();
}

function overviewMetrics() {
  const blocks = sessionBlocks();
  const canvas = dom.overviewCanvas;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(100, Math.floor(rect.width));
  const height = Math.max(100, Math.floor(rect.height));
  if (!state.session || !blocks.length) return null;
  const cfg = state.session.config;
  const rows = Math.round(cfg.y_length_m / cfg.block_size_m);
  const cols = Math.round(cfg.x_length_m / cfg.block_size_m);
  const pad = 18;
  const innerW = width - pad * 2;
  const innerH = height - pad * 2;
  const cellW = innerW / cols;
  const cellH = innerH / rows;
  return { width, height, rows, cols, pad, cellW, cellH };
}

function renderOverview() {
  const { ctx, width, height } = canvasSize(dom.overviewCanvas);
  const m = overviewMetrics();
  state.metrics.overview = m;
  ctx.fillStyle = "#0a1118";
  ctx.fillRect(0, 0, width, height);
  if (!m) {
    ctx.fillStyle = "#6f8396";
    ctx.font = "12px sans-serif";
    ctx.fillText("No block data.", 14, 24);
    return;
  }
  for (const block of sessionBlocks()) {
    const r = Number(block.row);
    const c = Number(block.col);
    const x = m.pad + c * m.cellW;
    const y = m.pad + (m.rows - 1 - r) * m.cellH;
    const key = blockKey(r, c);
    const selected = state.selected.has(key);
    const stats = blockStats(block);
    ctx.fillStyle = blockColor(block);
    ctx.fillRect(x + 1, y + 1, m.cellW - 2, m.cellH - 2);
    if (stats.completion > 0 && stats.completion < 1) {
      ctx.fillStyle = "rgba(255,255,255,0.12)";
      ctx.fillRect(x + 1, y + m.cellH - 4, (m.cellW - 2) * stats.completion, 3);
    }
    ctx.strokeStyle = selected ? "#facc15" : "rgba(50, 77, 97, 0.8)";
    ctx.lineWidth = selected ? 2 : 1;
    ctx.strokeRect(x + 0.5, y + 0.5, m.cellW - 1, m.cellH - 1);
    if (m.cellW > 38 && m.cellH > 28) {
      ctx.fillStyle = "#d6e5f1";
      ctx.font = "10px sans-serif";
      const label = stats.avgDepth === null ? "-" : Math.round(stats.avgDepth).toString();
      ctx.fillText(label, x + 4, y + 13);
    }
  }
  ctx.fillStyle = "#7f95a8";
  ctx.font = "10px sans-serif";
  ctx.fillText("avg depth mm / completion", m.pad, 12);
}

function overviewHit(e) {
  const m = state.metrics.overview || overviewMetrics();
  if (!m) return null;
  const rect = dom.overviewCanvas.getBoundingClientRect();
  const px = e.clientX - rect.left;
  const py = e.clientY - rect.top;
  const c = Math.floor((px - m.pad) / m.cellW);
  const rr = Math.floor((py - m.pad) / m.cellH);
  const row = m.rows - 1 - rr;
  if (c < 0 || c >= m.cols || row < 0 || row >= m.rows) return null;
  return { row, col: c };
}

function renderDepthChart() {
  const { ctx, width, height } = canvasSize(dom.depthCanvas);
  ctx.fillStyle = "#0a1118";
  ctx.fillRect(0, 0, width, height);
  const selectedKeys = [...state.selected];
  dom.depthLegend.innerHTML = "";
  if (!selectedKeys.length) {
    ctx.fillStyle = "#6f8396";
    ctx.font = "12px sans-serif";
    ctx.fillText("Select blocks to plot depth vs scan index.", 14, 24);
    return;
  }
  const series = selectedKeys.map((key, i) => {
    const { row, col } = parseBlockKey(key);
    const points = sessionPoints()
      .filter((p) => Number(p.block_row) === row && Number(p.block_col) === col && Number.isFinite(Number(p.depth_mm)))
      .map((p) => ({ x: Number(p.scan_index), y: Number(p.depth_mm), accepted: p.accepted }))
      .sort((a, b) => (a.x - b.x) || 0);
    return { key, row, col, color: COLORS[i % COLORS.length], points };
  });
  series.forEach((s) => {
    const item = document.createElement("span");
    item.innerHTML = `<i style="background:${s.color}"></i>${s.row},${s.col}`;
    dom.depthLegend.appendChild(item);
  });

  const plotted = sampleSeries(series, MAX_RENDERED_CHART_POINTS);
  const all = plotted.flatMap((s) => s.points);
  if (!all.length) {
    ctx.fillStyle = "#6f8396";
    ctx.font = "12px sans-serif";
    ctx.fillText("Selected blocks have no numeric depth points.", 14, 24);
    return;
  }
  const pad = { l: 44, r: 12, t: 12, b: 28 };
  const innerW = width - pad.l - pad.r;
  const innerH = height - pad.t - pad.b;
  let xMin = Math.min(...all.map((p) => p.x));
  let xMax = Math.max(...all.map((p) => p.x));
  let yMin = Math.min(...all.map((p) => p.y));
  let yMax = Math.max(...all.map((p) => p.y));
  if (xMin === xMax) { xMin -= 1; xMax += 1; }
  if (yMin === yMax) { yMin -= 1; yMax += 1; }
  const yPad = (yMax - yMin) * 0.08;
  yMin -= yPad; yMax += yPad;
  const sx = (x) => pad.l + ((x - xMin) / (xMax - xMin)) * innerW;
  const sy = (y) => pad.t + innerH - ((y - yMin) / (yMax - yMin)) * innerH;

  ctx.strokeStyle = "#1a2a38";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = pad.t + (innerH / 4) * i;
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(pad.l + innerW, y); ctx.stroke();
    const val = yMax - ((yMax - yMin) / 4) * i;
    ctx.fillStyle = "#71879a";
    ctx.font = "10px sans-serif";
    ctx.fillText(val.toFixed(0), 4, y + 3);
  }
  ctx.strokeStyle = "#355066";
  ctx.strokeRect(pad.l, pad.t, innerW, innerH);
  ctx.fillStyle = "#71879a";
  ctx.fillText(String(Math.round(xMin)), pad.l, height - 8);
  ctx.fillText(String(Math.round(xMax)), pad.l + innerW - 14, height - 8);
  ctx.fillText("scan_index", pad.l + innerW / 2 - 24, height - 8);

  for (const s of plotted) {
    if (!s.points.length) continue;
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    s.points.forEach((p, i) => {
      const x = sx(p.x); const y = sy(p.y);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
    for (const p of s.points) {
      const x = sx(p.x); const y = sy(p.y);
      ctx.fillStyle = p.accepted === false ? "#6b7f90" : s.color;
      ctx.beginPath(); ctx.arc(x, y, 3, 0, Math.PI * 2); ctx.fill();
      ctx.strokeStyle = "#081018"; ctx.lineWidth = 1; ctx.stroke();
    }
  }
}

function visiblePoints() {
  const context = buildRenderContext();
  const key = `${state.filterSelected}|${[...state.selected].sort().join(";")}|${state.search}`;
  if (context.visibleKey === key && Array.isArray(context.visible)) return context.visible;
  let points = sessionPoints().slice();
  if (state.filterSelected && state.selected.size) {
    points = points.filter((p) => state.selected.has(blockKey(p.block_row, p.block_col)));
  }
  const q = state.search.trim().toLowerCase();
  if (q) {
    points = points.filter((p) => pointMatchesSearch(p, q));
  }
  points.sort((a, b) => Number(a.block_row) - Number(b.block_row)
    || Number(a.block_col) - Number(b.block_col)
    || Number(a.scan_index) - Number(b.scan_index)
    || String(a.id).localeCompare(String(b.id)));
  context.visibleKey = key;
  context.visible = points;
  return points;
}

function pointMatchesSearch(point, q) {
  const values = [
    point.id, point.block_row, point.block_col, point.scan_index, point.timestamp, point.source,
    point.accepted, point.x_m, point.y_m, point.depth_mm
  ];
  if (values.some((v) => String(v ?? "").toLowerCase().includes(q))) return true;
  try { return JSON.stringify(point.raw || {}).toLowerCase().includes(q); } catch { return false; }
}

function renderTable() {
  const all = sessionPoints();
  const rows = visiblePoints();
  const shown = samplePoints(rows, MAX_RENDERED_TABLE_ROWS);
  const truncated = shown.length < rows.length ? `, showing ${shown.length}` : "";
  dom.tableCount.textContent = `${rows.length}/${all.length} points${truncated}`;
  dom.pointsBody.innerHTML = "";
  if (!shown.length) {
    const tr = document.createElement("tr");
    tr.className = "empty-row";
    const td = document.createElement("td");
    td.colSpan = 18;
    td.textContent = state.session ? "No points match the current filters." : "No session loaded.";
    tr.appendChild(td);
    dom.pointsBody.appendChild(tr);
    return;
  }
  for (const point of shown) {
    dom.pointsBody.appendChild(renderPointRow(point));
  }
}

function renderPointRow(point) {
  const tr = document.createElement("tr");
  tr.dataset.pointId = point.id;
  tr.appendChild(textCell(shortId(point.id)));
  tr.appendChild(textCell(`${point.block_row},${point.block_col}`));
  tr.appendChild(textCell(point.scan_index ?? ""));
  for (const field of EDITABLE_COLUMNS) {
    tr.appendChild(editCell(point, field));
  }
  tr.appendChild(rawCell(point.raw));
  tr.appendChild(actionCell(point.id));
  return tr;
}

function shortId(id) {
  const s = String(id ?? "");
  return s.length > 10 ? `${s.slice(0, 8)}...` : s;
}

function textCell(value) {
  const td = document.createElement("td");
  td.textContent = value == null ? "" : String(value);
  return td;
}

function editCell(point, field) {
  const td = document.createElement("td");
  const input = document.createElement("input");
  input.className = "cell-input";
  input.dataset.field = field;
  input.dataset.pointId = point.id;
  input.value = formatCellValue(field, point[field]);
  if (NUMERIC_FIELDS.has(field)) {
    input.type = "number";
    input.step = "any";
  } else if (field === "accepted") {
    input.type = "text";
    input.placeholder = "true/false";
  } else {
    input.type = "text";
  }
  const pending = state.pendingEdits.get(editKey(point.id, field));
  if (pending !== undefined) {
    input.classList.add("dirty");
    input.value = pending;
  }
  td.appendChild(input);
  return td;
}

function rawCell(raw) {
  const td = document.createElement("td");
  const details = document.createElement("details");
  const summary = document.createElement("summary");
  summary.textContent = "raw";
  const pre = document.createElement("pre");
  try { pre.textContent = JSON.stringify(raw ?? {}, null, 2); } catch { pre.textContent = String(raw); }
  details.appendChild(summary);
  details.appendChild(pre);
  td.appendChild(details);
  return td;
}

function actionCell(pointId) {
  const td = document.createElement("td");
  const btn = document.createElement("button");
  btn.type = "button";
  btn.className = "btn tiny danger";
  btn.dataset.action = "delete-point";
  btn.dataset.pointId = pointId;
  btn.textContent = "Delete";
  td.appendChild(btn);
  return td;
}

function formatCellValue(field, value) {
  if (value === null || value === undefined) return "";
  if (field === "accepted") return String(value);
  return String(value);
}

function editKey(pointId, field) { return `${pointId}::${field}`; }

function onTableKeydown(e) {
  if (e.key !== "Enter") return;
  const input = e.target.closest(".cell-input");
  if (!input) return;
  e.preventDefault();
  input.blur();
}

function onTableChange(e) {
  const input = e.target.closest(".cell-input");
  if (!input) return;
  commitCellEdit(input);
}

async function commitCellEdit(input) {
  const pointId = input.dataset.pointId;
  const field = input.dataset.field;
  if (!pointId || !PATCHABLE_FIELDS.has(field) || !state.session) return;
  const point = sessionPoints().find((p) => String(p.id) === String(pointId));
  if (!point) return;
  const rawValue = input.value;
  const parsed = parseCellValue(field, rawValue);
  if (!parsed.ok) {
    setError(parsed.error);
    input.value = formatCellValue(field, point[field]);
    input.classList.remove("dirty");
    return;
  }
  if (Object.is(parsed.value, point[field]) || String(parsed.value) === String(point[field])) {
    input.classList.remove("dirty");
    state.pendingEdits.delete(editKey(pointId, field));
    return;
  }
  const key = editKey(pointId, field);
  state.pendingEdits.set(key, rawValue);
  input.classList.add("dirty");
  setSaveState("saving");
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(state.session.id)}/points/${encodeURIComponent(pointId)}`, {
      method: "PATCH",
      body: { [field]: parsed.value }
    });
    state.pendingEdits.delete(key);
    setSaveState("saved");
    setError("");
    applySession(session);
  } catch (err) {
    setSaveState("error");
    setError(err.message || String(err));
  }
}

function parseCellValue(field, raw) {
  const value = String(raw).trim();
  if (field === "accepted") {
    if (value.toLowerCase() === "true") return { ok: true, value: true };
    if (value.toLowerCase() === "false") return { ok: true, value: false };
    return { ok: false, error: "accepted must be true or false" };
  }
  if (NUMERIC_FIELDS.has(field)) {
    if (value === "") return { ok: true, value: null };
    const num = Number(value);
    if (!Number.isFinite(num)) return { ok: false, error: `${field} must be numeric` };
    if (field === "scan_index") return { ok: true, value: Math.trunc(num) };
    return { ok: true, value: num };
  }
  return { ok: true, value };
}

async function onTableClick(e) {
  const btn = e.target.closest('[data-action="delete-point"]');
  if (!btn || !state.session) return;
  const pointId = btn.dataset.pointId;
  if (!pointId) return;
  btn.disabled = true;
  setSaveState("saving");
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(state.session.id)}/points/${encodeURIComponent(pointId)}`, { method: "DELETE" });
    setSaveState("saved");
    setError("");
    applySession(session);
  } catch (err) {
    btn.disabled = false;
    setSaveState("error");
    setError(err.message || String(err));
  }
}

async function loadSessions() {
  try {
    const data = await api("/api/sessions");
    state.sessions = Array.isArray(data?.sessions) ? data.sessions : [];
    if (state.session) {
      const found = state.sessions.find((s) => s.id === state.session.id);
      if (!found) state.session = null;
    }
    if (!state.session && state.sessions.length) {
      const latest = state.sessions.slice().sort((a, b) => String(b.updated_at || "").localeCompare(String(a.updated_at || "")))[0];
      await loadSession(latest.id);
      return;
    }
    setConnection("live");
    setError("");
    requestRender();
  } catch (err) {
    setError(err.message || String(err));
    setConnection("offline");
  }
}

async function loadSession(id) {
  if (!id) return;
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(id)}`);
    applySession(session, true);
    connectEvents(session.id);
    setError("");
  } catch (err) {
    setError(err.message || String(err));
  }
}

function applySession(session, preserveSelection) {
  const prevId = state.session?.id;
  if (session?.config) state.config = { ...state.config, ...session.config };
  state.session = session;
  state.lastUpdated = session?.updated_at || new Date().toISOString();
  if (!preserveSelection && prevId !== session?.id) {
    state.selected.clear();
  } else {
    pruneSelection();
  }
  upsertSessionSummary(session);
  requestRender();
}

function upsertSessionSummary(session) {
  if (!session) return;
  const summary = {
    id: session.id,
    name: session.name,
    updated_at: session.updated_at,
    status: session.status,
    config: session.config,
    completion: session.completion
  };
  const idx = state.sessions.findIndex((s) => s.id === session.id);
  if (idx >= 0) state.sessions[idx] = { ...state.sessions[idx], ...summary };
  else state.sessions.unshift(summary);
}

function pruneSelection() {
  const valid = new Set(sessionBlocks().map((b) => blockKey(b.row, b.col)));
  for (const key of [...state.selected]) if (!valid.has(key)) state.selected.delete(key);
}

async function onSessionChange(e) {
  const id = e.target.value;
  if (!id) return;
  await loadSession(id);
}

async function onSessionNameChange(e) {
  if (!state.session) return;
  const name = e.target.value.trim();
  if (!name || name === state.session.name) return;
  setSaveState("saving");
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(state.session.id)}`, {
      method: "PATCH",
      body: { name }
    });
    setSaveState("saved");
    setError("");
    applySession(session);
  } catch (err) {
    setSaveState("error");
    setError(err.message || String(err));
  }
}

async function onNewScan() {
  const { config, errors } = renderConfigValidation();
  if (errors.length) { setError(errors[0]); return; }
  const name = dom.sessionName.value.trim() || defaultSessionName();
  setSaveState("saving");
  try {
    const session = await api("/api/sessions", { method: "POST", body: { name, config } });
    state.selected.clear();
    setSaveState("saved");
    setError("");
    applySession(session);
    connectEvents(session.id);
    await loadSessions();
  } catch (err) {
    setSaveState("error");
    setError(err.message || String(err));
  }
}

async function onSaveDraft() {
  if (!state.session) return;
  setSaveState("saving");
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(state.session.id)}`, {
      method: "PATCH",
      body: { status: "draft" }
    });
    setSaveState("saved");
    setError("");
    applySession(session);
  } catch (err) {
    setSaveState("error");
    setError(err.message || String(err));
  }
}

function defaultSessionName() {
  const d = new Date();
  const pad = (n) => String(n).padStart(2, "0");
  return `Scan ${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ${pad(d.getHours())}:${pad(d.getMinutes())}`;
}

async function onApplyConfig() {
  if (!state.session) return;
  const { config, errors } = renderConfigValidation();
  if (errors.length) { setError(errors[0]); return; }
  setSaveState("saving");
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(state.session.id)}/reconfigure`, {
      method: "POST",
      body: { config }
    });
    setSaveState("saved");
    setError("");
    applySession(session);
  } catch (err) {
    setSaveState("error");
    setError(err.message || String(err));
  }
}

async function onStartWithConfig() {
  await onNewScan();
}

async function postBlockAction(action) {
  if (!state.session || !state.selected.size) return;
  const blocks = [...state.selected].map(parseBlockKey);
  setSaveState("saving");
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(state.session.id)}/blocks/${action}`, {
      method: "POST",
      body: { blocks }
    });
    setSaveState("saved");
    setError("");
    applySession(session);
  } catch (err) {
    setSaveState("error");
    setError(err.message || String(err));
  }
}

async function onRescanSelected() {
  await postBlockAction("rescan");
}

async function onClearBlockPoints() {
  await postBlockAction("clear");
}

async function onAddDemo() {
  if (!state.session || !state.selected.size) return;
  const selected = [...state.selected].map(parseBlockKey);
  const rawCount = dom.demoCount.value.trim();
  let explicitCount = null;
  if (rawCount) {
    explicitCount = Number(rawCount);
    if (!Number.isInteger(explicitCount) || explicitCount < 1 || explicitCount > MAX_DEMO_POINTS) {
      setError(`Demo scans per block must be an integer from 1 to ${MAX_DEMO_POINTS.toLocaleString()}`);
      return;
    }
  }
  const blocks = selected.map((block) => {
    const target = Number(getBlock(block.row, block.col)?.target_count);
    const fallback = Number.isInteger(target) && target > 0 ? target : 1;
    const count = explicitCount === null ? fallback : explicitCount;
    return { row: block.row, col: block.col, count };
  });
  if (blocks.some((block) => block.count > MAX_DEMO_POINTS)) {
    setError(`Demo scans per block cannot exceed ${MAX_DEMO_POINTS.toLocaleString()}`);
    return;
  }
  setSaveState("saving");
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(state.session.id)}/demo`, {
      method: "POST",
      body: { blocks }
    });
    setSaveState("saved");
    setError("");
    applySession(session);
  } catch (err) {
    setSaveState("error");
    setError(err.message || String(err));
  }
}

async function onMarkComplete() {
  if (!state.session) return;
  setSaveState("saving");
  try {
    const session = await api(`/api/sessions/${encodeURIComponent(state.session.id)}`, {
      method: "PATCH",
      body: { status: "complete" }
    });
    setSaveState("saved");
    setError("");
    applySession(session);
  } catch (err) {
    setSaveState("error");
    setError(err.message || String(err));
  }
}

function connectEvents(sessionId, attempt = 0) {
  if (state.eventSource) {
    state.eventSource.close();
    state.eventSource = null;
  }
  if (state.eventRetryTimer) {
    clearTimeout(state.eventRetryTimer);
    state.eventRetryTimer = null;
  }
  if (state.pollTimer) {
    clearInterval(state.pollTimer);
    state.pollTimer = null;
  }
  if (!sessionId) return;
  if (typeof EventSource === "undefined") {
    startPolling(sessionId);
    return;
  }
  setConnection("connecting");
  const es = new EventSource(`/api/sessions/${encodeURIComponent(sessionId)}/events`);
  state.eventSource = es;
  es.addEventListener("session", (evt) => {
    try {
      const session = JSON.parse(evt.data);
      applySession(session, true);
      setConnection("live");
    } catch (err) {
      setError(`SSE parse error: ${err.message || err}`);
    }
  });
  es.onopen = () => { setConnection("live"); setError(""); };
  es.onerror = () => {
    if (state.eventSource === es) {
      es.close();
      state.eventSource = null;
    }
    if (attempt < 4) {
      const delay = Math.min(8000, 400 * (2 ** attempt));
      setConnection("connecting");
      state.eventRetryTimer = setTimeout(() => connectEvents(sessionId, attempt + 1), delay);
      return;
    }
    startPolling(sessionId);
  };
}

function startPolling(sessionId) {
  if (state.pollTimer) clearInterval(state.pollTimer);
  setConnection("polling");
  const poll = async () => {
    if (!state.session || state.session.id !== sessionId) return;
    try {
      const session = await api(`/api/sessions/${encodeURIComponent(sessionId)}`);
      applySession(session, true);
      setConnection("polling");
    } catch (err) {
      setConnection("offline");
      setError(err.message || String(err));
    }
  };
  poll();
  state.pollTimer = setInterval(poll, 2000);
}

function renderStatus() {
  const session = state.session;
  const points = sessionPoints();
  const blocks = sessionBlocks();
  const complete = blocks.filter((b) => b.status === "complete").length;
  const simulated = blocks.reduce((total, b) => total + Math.max(0, Number(b.demo_simulated_count) || 0), 0);
  dom.statusPoints.textContent = String(points.length + simulated);
  dom.statusBlocks.textContent = `${complete}/${blocks.length}`;
  dom.statusUpdated.textContent = formatTime(state.lastUpdated || session?.updated_at);
  dom.statusSession.textContent = session ? `${session.name || session.id} [${session.status}]` : "No session";
}

function formatTime(value) {
  if (!value) return "-";
  const d = new Date(value);
  if (Number.isNaN(d.getTime())) return String(value);
  return d.toLocaleString();
}

window.addEventListener("DOMContentLoaded", init);
