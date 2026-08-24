"use strict";

const NS = "http://www.w3.org/2000/svg";
const ROBOT_COLORS = ["#155eef", "#df4f7c", "#07883f", "#d76a00", "#7646d8", "#087da4", "#b33a3a", "#53657b"];

const state = {
  document: null,
  catalog: [],
  selectedNodes: new Set(),
  selectedLane: null,
  view: {scale: 70, tx: 400, ty: 280},
  draggingNode: null,
  panning: null,
  runId: null,
  runProfile: null,
  stream: null,
  events: [],
  rawLines: [],
  runtime: "",
  analysis: null,
  comparison: null,
  penaltyDirected: {},
  trajectory: {byRobot: new Map(), min: 0, max: 0, current: 0},
  playing: false,
  animationStarted: 0,
  animationOrigin: 0,
  yamlText: "",
  yamlFileName: "",
  yamlMetadata: null,
};

const $ = (id) => document.getElementById(id);
const deepCopy = (value) => JSON.parse(JSON.stringify(value));
const number = (value, fallback = 0) => Number.isFinite(Number(value)) ? Number(value) : fallback;
const format = (value, digits = 3) => {
  if (value === null || value === undefined || value === "") return "-";
  if (typeof value === "object") return JSON.stringify(value);
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed.toFixed(digits).replace(/\.?0+$/, "") : String(value);
};
const escapeHtml = (value) => String(value ?? "").replace(/[&<>"']/g, (character) => ({"&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;", "'": "&#39;"})[character]);

function token() {
  return localStorage.getItem("rmf_lab_token") || "";
}

async function api(path, options = {}) {
  const headers = Object.assign({}, options.headers || {});
  if (token()) headers.Authorization = `Bearer ${token()}`;
  if (options.body && !headers["Content-Type"]) headers["Content-Type"] = "application/json";
  const response = await fetch(path, Object.assign({}, options, {headers}));
  const contentType = response.headers.get("content-type") || "";
  const payload = contentType.includes("json") ? await response.json() : await response.text();
  if (!response.ok) throw new Error(payload.error || payload || `HTTP ${response.status}`);
  return payload;
}

function setStatus(message, kind = "neutral") {
  $("status-bar").textContent = message;
  const badge = $("run-status");
  badge.textContent = message;
  badge.className = `badge ${kind}`;
}

function syncChromeHeight() {
  const height = document.querySelector(".app-header").offsetHeight + $("status-bar").offsetHeight;
  document.documentElement.style.setProperty("--dynamic-chrome", `${height}px`);
}

function svgElement(tag, attributes = {}, text = "") {
  const element = document.createElementNS(NS, tag);
  Object.entries(attributes).forEach(([key, value]) => {
    if (value !== null && value !== undefined) element.setAttribute(key, String(value));
  });
  if (text) element.textContent = text;
  return element;
}

function toScreen(x, y) {
  return {x: state.view.tx + number(x) * state.view.scale, y: state.view.ty - number(y) * state.view.scale};
}

function toWorld(x, y) {
  return {x: (x - state.view.tx) / state.view.scale, y: (state.view.ty - y) / state.view.scale};
}

function svgPoint(event) {
  const rect = $("map-svg").getBoundingClientRect();
  return {x: event.clientX - rect.left, y: event.clientY - rect.top};
}

function directedLaneIds(sourceIndex) {
  let cursor = 0;
  for (let index = 0; index < state.document.lanes.length; index += 1) {
    const count = state.document.lanes[index].bidirectional === false ? 1 : 2;
    if (index === sourceIndex) return Array.from({length: count}, (_, offset) => cursor + offset);
    cursor += count;
  }
  return [];
}

function renderMap() {
  if (!state.document) return;
  const laneLayer = $("lane-layer");
  const corridorLayer = $("corridor-layer");
  const nodeLayer = $("node-layer");
  laneLayer.replaceChildren();
  corridorLayer.replaceChildren();
  nodeLayer.replaceChildren();
  const nodes = state.document.nodes;

  state.document.lanes.forEach((lane, index) => {
    const startNode = nodes[lane.from];
    const endNode = nodes[lane.to];
    if (!startNode || !endNode) return;
    const start = toScreen(startNode.x, startNode.y);
    const end = toScreen(endNode.x, endNode.y);
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const length = Math.hypot(dx, dy) || 1;
    const inset = Math.min(22, length * .22);
    const x1 = start.x + dx / length * inset;
    const y1 = start.y + dy / length * inset;
    const x2 = end.x - dx / length * inset;
    const y2 = end.y - dy / length * inset;
    const corridorLeft = Math.max(0, number(lane.corridor_left_width));
    const corridorRight = Math.max(0, number(lane.corridor_right_width));
    if (corridorLeft + corridorRight > 0) {
      corridorLayer.append(svgElement("line", {
        x1: start.x, y1: start.y, x2: end.x, y2: end.y,
        class: "corridor-band",
        "stroke-width": Math.max(8, Math.min(160, (corridorLeft + corridorRight) * state.view.scale)),
      }));
    }
    const directedIds = directedLaneIds(index);
    const runtimePenalty = directedIds.some((id) => number(state.penaltyDirected[id]) > 0);
    const classes = ["lane-line"];
    if (state.selectedLane === index) classes.push("selected");
    if (lane.closed || (state.document.closed_lanes || []).some((id) => directedIds.includes(Number(id)))) classes.push("closed");
    else if (runtimePenalty) classes.push("runtime-penalty");
    else if (number(lane.after_penalty) > 0) classes.push("manual-penalty");
    else if (lane.mutex_group) classes.push("mutex");
    const lineAttributes = {x1, y1, x2, y2, class: classes.join(" "), "data-lane": index, "marker-end": "url(#arrow-end)"};
    if (lane.bidirectional !== false) lineAttributes["marker-start"] = "url(#arrow-start)";
    const line = svgElement("line", lineAttributes);
    const hit = svgElement("line", {x1, y1, x2, y2, class: "lane-hit", "data-lane": index});
    [line, hit].forEach((element) => element.addEventListener("click", (event) => {
      event.stopPropagation();
      state.selectedLane = index;
      state.selectedNodes.clear();
      renderMap();
      showSelection();
    }));
    laneLayer.append(line, hit);
    const penalty = directedIds.map((id) => state.penaltyDirected[id]).filter((value) => number(value) > 0);
    const parts = [`L${index}`];
    if (lane.speed_limit !== undefined) parts.push(`${format(lane.speed_limit)}m/s`);
    if (penalty.length) parts.push(`+${format(Math.max(...penalty))}`);
    else if (number(lane.after_penalty) > 0) parts.push(`수동+${format(lane.after_penalty)}`);
    if (lane.mutex_group) parts.push(lane.mutex_group);
    if (corridorLeft + corridorRight > 0) parts.push(`폭 ${format(corridorLeft + corridorRight)}m`);
    laneLayer.append(svgElement("text", {x: (x1 + x2) / 2, y: (y1 + y2) / 2 - 7, class: "lane-label"}, parts.join(" · ")));
  });

  nodes.forEach((node, index) => {
    const point = toScreen(node.x, node.y);
    const group = svgElement("g", {"data-node": index});
    const classes = ["node-dot"];
    if (node.holding) classes.push("holding");
    if (node.parking) classes.push("parking");
    if (node.passthrough) classes.push("passthrough");
    if (state.selectedNodes.has(index)) classes.push("selected");
    const circle = svgElement("circle", {cx: point.x, cy: point.y, r: 15, class: classes.join(" ")});
    circle.addEventListener("pointerdown", (event) => {
      if (event.button !== 0) return;
      event.stopPropagation();
      circle.setPointerCapture(event.pointerId);
      state.draggingNode = {index, pointerId: event.pointerId};
      if (event.ctrlKey || event.metaKey) {
        if (state.selectedNodes.has(index)) state.selectedNodes.delete(index);
        else state.selectedNodes.add(index);
      } else {
        state.selectedNodes = new Set([index]);
      }
      state.selectedLane = null;
      renderMap();
      showSelection();
    });
    circle.addEventListener("pointermove", (event) => {
      if (!state.draggingNode || state.draggingNode.pointerId !== event.pointerId) return;
      const world = toWorld(svgPoint(event).x, svgPoint(event).y);
      state.document.nodes[index].x = Number(world.x.toFixed(3));
      state.document.nodes[index].y = Number(world.y.toFixed(3));
      renderMap();
      showSelection();
    });
    circle.addEventListener("pointerup", () => { state.draggingNode = null; });
    group.append(circle);
    group.append(svgElement("circle", {cx: point.x, cy: point.y, r: 8, fill: state.selectedNodes.has(index) ? "#155eef" : "#35475b", "pointer-events": "none"}));
    group.append(svgElement("text", {x: point.x, y: point.y + .5, class: "node-index"}, String(index)));
    group.append(svgElement("text", {x: point.x, y: point.y - 23, class: "node-label"}, node.name || `N${index}`));
    nodeLayer.append(group);
  });
  renderRobots(state.trajectory.current);
  updateGraphSummary();
}

function renderRobots(timeValue) {
  const layer = $("robot-layer");
  const trajectoryLayer = $("trajectory-layer");
  layer.replaceChildren();
  trajectoryLayer.replaceChildren();
  if (!state.document) return;
  const hasRunTrajectory = state.trajectory.byRobot.size > 0;
  const robots = state.document.robots || [];
  robots.forEach((robot, index) => {
    const points = state.trajectory.byRobot.get(robot.name) || [];
    let pose;
    let robotState = "대기";
    if (points.length) {
      pose = interpolatePose(points, timeValue);
      robotState = pose.state;
      const pathData = points.map((point, pointIndex) => {
        const screen = toScreen(point.x, point.y);
        return `${pointIndex ? "L" : "M"}${screen.x.toFixed(2)},${screen.y.toFixed(2)}`;
      }).join(" ");
      trajectoryLayer.append(svgElement("path", {d: pathData, class: "trajectory-path", stroke: ROBOT_COLORS[index % ROBOT_COLORS.length]}));
      const goal = toScreen(points[points.length - 1].x, points[points.length - 1].y);
      trajectoryLayer.append(svgElement("circle", {cx: goal.x, cy: goal.y, r: 20, class: "trajectory-goal", stroke: ROBOT_COLORS[index % ROBOT_COLORS.length]}));
    } else {
      const node = state.document.nodes[robot.start];
      if (!node) return;
      pose = {x: node.x, y: node.y, yaw: number(robot.yaw), state: hasRunTrajectory ? "계획 없음" : `출발 t=${format(robot.start_time_s)}s`};
      robotState = pose.state;
    }
    const point = toScreen(pose.x, pose.y);
    const color = ROBOT_COLORS[index % ROBOT_COLORS.length];
    const group = svgElement("g", {transform: `translate(${point.x} ${point.y})`, "aria-label": `${robot.name} ${robotState}`});
    const body = svgElement("g", {transform: `rotate(${-number(pose.yaw) * 180 / Math.PI})`});
    body.append(svgElement("rect", {x: -18, y: -14, width: 36, height: 28, rx: 8, class: "robot-body", fill: color}));
    body.append(svgElement("rect", {x: -17, y: -17, width: 9, height: 5, rx: 2, class: "robot-wheel"}));
    body.append(svgElement("rect", {x: 8, y: -17, width: 9, height: 5, rx: 2, class: "robot-wheel"}));
    body.append(svgElement("rect", {x: -17, y: 12, width: 9, height: 5, rx: 2, class: "robot-wheel"}));
    body.append(svgElement("rect", {x: 8, y: 12, width: 9, height: 5, rx: 2, class: "robot-wheel"}));
    body.append(svgElement("path", {d: "M18,-8 L30,0 L18,8 Z", class: "robot-nose"}));
    body.append(svgElement("circle", {cx: -7, cy: 0, r: 4, fill: "rgba(255,255,255,.75)"}));
    group.append(body);
    group.append(svgElement("text", {x: 0, y: -25, class: "robot-label"}, robot.name));
    group.append(svgElement("text", {x: 0, y: 29, class: "robot-state"}, robotState));
    layer.append(group);
  });
}

function interpolatePose(points, timeValue) {
  if (timeValue <= points[0].time) return {...points[0], state: timeValue + 1e-6 < points[0].time ? "출발 대기" : "출발"};
  if (timeValue >= points[points.length - 1].time) return {...points[points.length - 1], state: "도착"};
  let high = points.findIndex((point) => point.time >= timeValue);
  if (high <= 0) high = 1;
  const a = points[high - 1];
  const b = points[high];
  const ratio = (timeValue - a.time) / Math.max(b.time - a.time, 1e-9);
  const yawDelta = Math.atan2(Math.sin(b.yaw - a.yaw), Math.cos(b.yaw - a.yaw));
  const distance = Math.hypot(b.x - a.x, b.y - a.y);
  const stateText = distance < 1e-5 ? Math.abs(yawDelta) > 1e-4 ? "제자리 회전" : "협상 대기" : "전진 이동";
  return {x: a.x + (b.x - a.x) * ratio, y: a.y + (b.y - a.y) * ratio, yaw: a.yaw + yawDelta * ratio, state: stateText};
}

function updateGraphSummary() {
  const document = state.document;
  if (!document) return;
  $("graph-summary").textContent = `노드 ${document.nodes.length} · Lane ${document.lanes.length} · 로봇 ${document.robots.length}`;
}

function fitMap() {
  if (!state.document || !state.document.nodes.length) return;
  const rect = $("map-svg").getBoundingClientRect();
  const xs = state.document.nodes.map((node) => number(node.x));
  const ys = state.document.nodes.map((node) => number(node.y));
  const width = Math.max(...xs) - Math.min(...xs) || 2;
  const height = Math.max(...ys) - Math.min(...ys) || 2;
  state.view.scale = Math.max(12, Math.min(140, Math.min((rect.width - 120) / width, (rect.height - 120) / height)));
  state.view.tx = rect.width / 2 - ((Math.max(...xs) + Math.min(...xs)) / 2) * state.view.scale;
  state.view.ty = rect.height / 2 + ((Math.max(...ys) + Math.min(...ys)) / 2) * state.view.scale;
  updateZoomLabel();
  renderMap();
}

function zoomAt(factor, center = null) {
  const rect = $("map-svg").getBoundingClientRect();
  const point = center || {x: rect.width / 2, y: rect.height / 2};
  const world = toWorld(point.x, point.y);
  state.view.scale = Math.max(10, Math.min(240, state.view.scale * factor));
  state.view.tx = point.x - world.x * state.view.scale;
  state.view.ty = point.y + world.y * state.view.scale;
  updateZoomLabel();
  renderMap();
}

function updateZoomLabel() {
  $("zoom-label").textContent = `${Math.round(state.view.scale / 70 * 100)}%`;
}

function showSelection() {
  const nodeIndex = state.selectedNodes.size === 1 ? [...state.selectedNodes][0] : null;
  const laneIndex = state.selectedLane;
  const fields = ["prop-name", "prop-x", "prop-y", "prop-mutex", "prop-holding", "prop-parking", "prop-passthrough", "prop-bidirectional", "prop-speed", "prop-penalty", "prop-closed", "prop-yaml-orientation", "prop-yaml-rotation", "prop-corridor-left", "prop-corridor-right", "prop-corridor-ref"];
  fields.forEach((id) => $(id).disabled = nodeIndex === null && laneIndex === null);
  $("yaml-lane-properties").hidden = true;
  if (nodeIndex !== null) {
    const node = state.document.nodes[nodeIndex];
    $("selection-title").textContent = `노드 ${nodeIndex} · ${node.name}`;
    $("prop-name").value = node.name || "";
    $("prop-x").value = node.x;
    $("prop-y").value = node.y;
    $("prop-mutex").value = node.mutex_group || "";
    $("prop-holding").checked = Boolean(node.holding);
    $("prop-parking").checked = Boolean(node.parking);
    $("prop-passthrough").checked = Boolean(node.passthrough);
    $("prop-bidirectional").checked = false;
    $("prop-speed").value = "";
    $("prop-penalty").value = 0;
    $("prop-closed").checked = false;
  } else if (laneIndex !== null) {
    const lane = state.document.lanes[laneIndex];
    $("selection-title").textContent = `Lane ${laneIndex} · ${lane.from} → ${lane.to}`;
    $("prop-name").value = `Lane ${laneIndex}`;
    $("prop-x").value = "";
    $("prop-y").value = "";
    $("prop-mutex").value = lane.mutex_group || "";
    $("prop-holding").checked = false;
    $("prop-parking").checked = false;
    $("prop-passthrough").checked = false;
    $("prop-bidirectional").checked = lane.bidirectional !== false;
    $("prop-speed").value = lane.speed_limit ?? "";
    $("prop-penalty").value = lane.after_penalty ?? 0;
    $("prop-closed").checked = Boolean(lane.closed);
    const hasYamlProperties = ["yaml_orientation_rad", "yaml_rotation_allowed", "yaml_corridor", "corridor_left_width", "corridor_right_width", "corridor_ref_point"].some((key) => key in lane);
    $("yaml-lane-properties").hidden = !hasYamlProperties;
    $("prop-yaml-orientation").value = lane.yaml_orientation_rad ?? "";
    $("prop-yaml-rotation").checked = lane.yaml_rotation_allowed !== false;
    $("prop-corridor-left").value = lane.corridor_left_width ?? "";
    $("prop-corridor-right").value = lane.corridor_right_width ?? "";
    $("prop-corridor-ref").value = lane.corridor_ref_point ?? "";
  } else {
    $("selection-title").textContent = state.selectedNodes.size > 1 ? `노드 ${[...state.selectedNodes].join(", ")} 선택됨` : "노드 또는 Lane을 선택하세요.";
  }
}

function applyProperties() {
  if (state.selectedNodes.size === 1) {
    const index = [...state.selectedNodes][0];
    const node = state.document.nodes[index];
    node.name = $("prop-name").value.trim() || `N${index}`;
    node.x = number($("prop-x").value, node.x);
    node.y = number($("prop-y").value, node.y);
    node.mutex_group = $("prop-mutex").value.trim();
    node.holding = $("prop-holding").checked;
    node.parking = $("prop-parking").checked;
    node.passthrough = $("prop-passthrough").checked;
  } else if (state.selectedLane !== null) {
    const lane = state.document.lanes[state.selectedLane];
    lane.mutex_group = $("prop-mutex").value.trim();
    lane.bidirectional = $("prop-bidirectional").checked;
    lane.closed = $("prop-closed").checked;
    lane.after_penalty = Math.max(0, number($("prop-penalty").value));
    if ($("prop-speed").value === "") delete lane.speed_limit;
    else lane.speed_limit = Math.max(.001, number($("prop-speed").value));
    if (!$("yaml-lane-properties").hidden) {
      if ($("prop-yaml-orientation").value === "") delete lane.yaml_orientation_rad;
      else lane.yaml_orientation_rad = number($("prop-yaml-orientation").value);
      lane.yaml_rotation_allowed = $("prop-yaml-rotation").checked;
      if ($("prop-corridor-left").value === "") delete lane.corridor_left_width;
      else lane.corridor_left_width = Math.max(0, number($("prop-corridor-left").value));
      if ($("prop-corridor-right").value === "") delete lane.corridor_right_width;
      else lane.corridor_right_width = Math.max(0, number($("prop-corridor-right").value));
      if ($("prop-corridor-ref").value.trim()) lane.corridor_ref_point = $("prop-corridor-ref").value.trim();
      else delete lane.corridor_ref_point;
    }
  }
  renderMap();
  renderRobotTable();
  showSelection();
}

function addNode() {
  const rect = $("map-svg").getBoundingClientRect();
  const point = toWorld(rect.width / 2, rect.height / 2);
  const index = state.document.nodes.length;
  state.document.nodes.push({name: `N${index}`, x: Number(point.x.toFixed(3)), y: Number(point.y.toFixed(3)), holding: true, parking: false, passthrough: false});
  state.selectedNodes = new Set([index]);
  state.selectedLane = null;
  renderMap();
  showSelection();
}

function addLane() {
  if (state.selectedNodes.size !== 2) {
    setStatus("Ctrl+클릭으로 서로 다른 노드 2개를 먼저 선택하세요", "failed");
    return;
  }
  const [from, to] = [...state.selectedNodes];
  if (state.document.lanes.some((lane) => lane.from === from && lane.to === to && lane.bidirectional !== false)) {
    setStatus("두 노드 사이에 이미 양방향 Lane이 있습니다", "failed");
    return;
  }
  state.document.lanes.push({from, to, bidirectional: true});
  state.selectedLane = state.document.lanes.length - 1;
  state.selectedNodes.clear();
  renderMap();
  showSelection();
}

function deleteSelection() {
  if (state.selectedLane !== null) {
    state.document.lanes.splice(state.selectedLane, 1);
    state.selectedLane = null;
  } else if (state.selectedNodes.size) {
    const deleted = new Set(state.selectedNodes);
    const mapping = new Map();
    const nodes = [];
    state.document.nodes.forEach((node, oldIndex) => {
      if (!deleted.has(oldIndex)) { mapping.set(oldIndex, nodes.length); nodes.push(node); }
    });
    state.document.nodes = nodes;
    state.document.lanes = state.document.lanes.filter((lane) => mapping.has(lane.from) && mapping.has(lane.to)).map((lane) => ({...lane, from: mapping.get(lane.from), to: mapping.get(lane.to)}));
    state.document.robots = state.document.robots.filter((robot) => mapping.has(robot.start) && mapping.has(robot.goal)).map((robot) => ({...robot, start: mapping.get(robot.start), goal: mapping.get(robot.goal)}));
    state.selectedNodes.clear();
  }
  renderMap();
  renderRobotTable();
  showSelection();
}

function renderRobotTable() {
  const tbody = $("robot-table").querySelector("tbody");
  tbody.replaceChildren();
  (state.document?.robots || []).forEach((robot, index) => {
    const row = document.createElement("tr");
    const definitions = [
      ["name", "text", robot.name], ["start", "number", robot.start], ["goal", "number", robot.goal],
      ["yaw", "number", robot.yaw ?? 0], ["start_time_s", "number", robot.start_time_s ?? 0],
    ];
    definitions.forEach(([field, type, value]) => {
      const cell = document.createElement("td");
      const input = document.createElement("input");
      input.type = type;
      input.value = value;
      if (type === "number") input.step = field === "start" || field === "goal" ? "1" : ".001";
      input.addEventListener("change", () => {
        robot[field] = type === "number" ? number(input.value) : input.value.trim();
        renderMap();
      });
      cell.append(input);
      row.append(cell);
    });
    const action = document.createElement("td");
    const remove = document.createElement("button");
    remove.type = "button";
    remove.textContent = "삭제";
    remove.addEventListener("click", () => {
      state.document.robots.splice(index, 1);
      renderRobotTable();
      renderMap();
    });
    action.append(remove);
    row.append(action);
    tbody.append(row);
  });
}

function addRobot() {
  const index = state.document.robots.length;
  state.document.robots.push({name: `R${index}`, start: 0, goal: Math.max(0, state.document.nodes.length - 1), yaw: 0, start_time_s: 0});
  renderRobotTable();
  renderMap();
}

function syncDocumentFromRobotInputs() {
  $("robot-table").querySelectorAll("tbody tr").forEach((row, index) => {
    const inputs = [...row.querySelectorAll("input")];
    if (!state.document.robots[index]) return;
    state.document.robots[index] = {
      name: inputs[0].value.trim() || `R${index}`,
      start: Math.trunc(number(inputs[1].value)),
      goal: Math.trunc(number(inputs[2].value)),
      yaw: number(inputs[3].value),
      start_time_s: Math.max(0, number(inputs[4].value)),
    };
  });
}

async function loadCatalog() {
  state.catalog = await api("/api/scenarios");
  const select = $("scenario-select");
  select.replaceChildren();
  state.catalog.forEach((item) => {
    const option = document.createElement("option");
    option.value = item.key;
    option.textContent = `${item.name} · N${item.nodes}/L${item.lanes}/R${item.robots}`;
    select.append(option);
  });
  const preferred = state.catalog.find((item) => item.key === "single_lane_bidirectional") || state.catalog[0];
  if (preferred) {
    select.value = preferred.key;
    await loadScenario(preferred.key);
  }
}

async function loadScenario(key) {
  clearYamlSource();
  setDocument(await api(`/api/scenarios/${encodeURIComponent(key)}`));
  setStatus(`시나리오 ${state.document.name}을 불러왔습니다`, "neutral");
}

function clearYamlSource() {
  state.yamlText = "";
  state.yamlFileName = "";
  state.yamlMetadata = null;
  $("yaml-level-label").hidden = true;
  $("yaml-level-select").replaceChildren();
}

async function importYamlMap(level = null) {
  if (!state.yamlText) return;
  const payload = {yaml_text: state.yamlText};
  if (level) payload.level = level;
  const response = await api("/api/scenarios/import-yaml", {
    method: "POST",
    body: JSON.stringify(payload),
  });
  state.yamlMetadata = response.metadata;
  setDocument(response.document);
  const levels = response.metadata.available_levels || [];
  const select = $("yaml-level-select");
  select.replaceChildren();
  levels.forEach((name) => {
    const option = document.createElement("option");
    option.value = name;
    option.textContent = name;
    select.append(option);
  });
  select.value = response.metadata.selected_level;
  $("yaml-level-label").hidden = levels.length <= 1;
  const warnings = response.metadata.warnings || [];
  const robotNote = response.document.robots.length
    ? ""
    : " · 로봇 0대: 오른쪽 로봇 탭에서 로봇을 추가해야 실행할 수 있습니다";
  setStatus(
    `YAML ${response.metadata.building_name}/${response.metadata.selected_level}: ` +
    `노드 ${response.metadata.node_count}, 원본 Lane ${response.metadata.source_lane_count}, ` +
    `방향 Lane ${response.metadata.directed_lane_count}${robotNote}`,
    warnings.length ? "running" : "completed",
  );
  $("map-hint").textContent = warnings.length
    ? `YAML 가져오기 주의: ${warnings.join(" | ")}`
    : "YAML 맵을 가져왔습니다. 노드 드래그 · Ctrl+클릭으로 Lane 추가";
  if (!response.document.robots.length) {
    document.querySelector('[data-right-tab="robots"]').click();
  }
}

function setDocument(documentValue) {
  state.document = deepCopy(documentValue);
  state.document.closed_lanes ||= [];
  state.document.robots ||= [];
  state.selectedNodes.clear();
  state.selectedLane = null;
  resetRunOutput();
  renderRobotTable();
  showSelection();
  requestAnimationFrame(fitMap);
}

function downloadScenario() {
  syncDocumentFromRobotInputs();
  const blob = new Blob([JSON.stringify(state.document, null, 2) + "\n"], {type: "application/json"});
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = `${state.document.name || "scenario"}.json`;
  link.click();
  URL.revokeObjectURL(link.href);
}

async function saveScenario() {
  syncDocumentFromRobotInputs();
  const response = await api("/api/scenarios/save", {method: "POST", body: JSON.stringify({filename: state.document.name, document: state.document})});
  await loadCatalog();
  $("scenario-select").value = response.key;
  await loadScenario(response.key);
  setStatus("서버 web_data/scenarios에 저장했습니다", "completed");
}

function coreOptions() {
  return {
    setup: $("setup-path").value,
    before_setup: $("before-setup").value,
    before_source: $("before-source").value,
    after_workspace: $("after-workspace").value,
    after_setup: $("after-setup").value,
    after_source: $("after-source").value,
    base_ros_setup: $("base-ros-setup").value,
    after_label: $("after-label").value,
    rebuild_lab: $("rebuild-lab").checked,
    rebuild_after: $("rebuild-after").checked,
    timeout: number($("timeout-input").value, 60),
    lane_penalty_mode: $("penalty-mode").value,
    lane_penalty_value: number($("penalty-value").value, 60),
  };
}

function updateCoreProfile() {
  const profile = $("core-profile").value;
  const badge = $("core-badge");
  if (profile === "after") {
    $("setup-path").value = $("after-setup").value;
    badge.className = "badge after";
    badge.textContent = "AFTER · 수정 코어 + 우회 비용";
  } else {
    $("setup-path").value = $("before-setup").value;
    badge.className = "badge before";
    badge.textContent = "BEFORE · 기본 코어";
  }
}

function resetRunOutput() {
  if (state.stream) state.stream.close();
  state.runId = null;
  state.events = [];
  state.rawLines = [];
  state.runtime = "";
  state.analysis = null;
  state.penaltyDirected = {};
  state.trajectory = {byRobot: new Map(), min: 0, max: 0, current: 0};
  $("runtime-log").textContent = "";
  $("runtime-summary").textContent = "실행 전입니다.";
  $("raw-jsonl").textContent = "";
  $("jsonl-summary").textContent = "실행 전입니다.";
  $("diagnosis-summary").textContent = "실행 전입니다.";
  $("diagnosis-raw").textContent = "";
  $("jsonl-download").classList.add("disabled");
  $("jsonl-download").removeAttribute("href");
  renderAnalysisTables();
}

async function startRun() {
  if (!state.document || state.runId) return;
  syncDocumentFromRobotInputs();
  if (!state.document.robots.length) {
    document.querySelector('[data-right-tab="robots"]').click();
    setStatus("YAML 맵에 로봇이 없습니다. 로봇을 한 대 이상 추가하고 start/goal을 지정하세요", "failed");
    return;
  }
  resetRunOutput();
  const profile = $("core-profile").value;
  try {
    const request = Object.assign({profile, document: state.document}, coreOptions());
    const response = await api("/api/runs", {method: "POST", body: JSON.stringify(request)});
    state.runId = response.run_id;
    state.runProfile = profile;
    $("run-button").disabled = true;
    $("stop-button").disabled = false;
    setStatus("실행 대기열에 등록됨", "running");
    openStream(state.runId);
  } catch (error) {
    state.runId = null;
    setStatus(error.message, "failed");
    $("runtime-log").textContent = error.stack || error.message;
  }
}

function openStream(runId) {
  const tokenQuery = token() ? `?token=${encodeURIComponent(token())}` : "";
  const stream = new EventSource(`/api/runs/${runId}/stream${tokenQuery}`);
  state.stream = stream;
  stream.onmessage = (message) => {
    const payload = JSON.parse(message.data);
    consumeStream(payload);
    if (["completed", "failed", "timeout", "cancelled"].includes(payload.state.status)) stream.close();
  };
  stream.onerror = () => {
    if (state.runId) setStatus("실시간 연결이 끊겼습니다. 서버 실행 상태를 확인하세요", "failed");
  };
}

function consumeStream(payload) {
  const runState = payload.state;
  (payload.logs || []).forEach((line) => { state.runtime += line; });
  (payload.events || []).forEach((event) => {
    state.events.push(event);
    state.rawLines.push(JSON.stringify(event));
    if (event.event === "occupancy_penalty_configuration") state.penaltyDirected = event.directed_lane_penalties || {};
    if (event.event === "runner_core_profile" && event.directed_lane_penalties) state.penaltyDirected = event.directed_lane_penalties;
  });
  $("runtime-log").textContent = state.runtime;
  $("raw-jsonl").textContent = state.rawLines.join("\n");
  $("runtime-log").scrollTop = $("runtime-log").scrollHeight;
  $("raw-jsonl").scrollTop = $("raw-jsonl").scrollHeight;
  setStatus(statusLabel(runState.status), runState.status);
  renderMap();
  if (payload.analysis) {
    state.analysis = payload.analysis;
    state.comparison = payload.comparison;
    applyAnalysis();
  }
  if (["completed", "failed", "timeout", "cancelled"].includes(runState.status)) {
    state.runId = null;
    $("run-button").disabled = false;
    $("stop-button").disabled = true;
    if (runState.run_id) {
      $("jsonl-download").href = `/api/runs/${runState.run_id}/jsonl${token() ? `?token=${encodeURIComponent(token())}` : ""}`;
      $("jsonl-download").classList.remove("disabled");
    }
  }
}

function statusLabel(status) {
  return ({queued: "대기열", running: "실제 RMF 실행 중", completed: "완료", failed: "실패", timeout: "시간 초과", cancelled: "취소됨"})[status] || status;
}

async function stopRun() {
  if (!state.runId) return;
  try {
    await api(`/api/runs/${state.runId}/stop`, {method: "POST", body: "{}"});
    setStatus("중지 요청을 보냈습니다", "cancelled");
  } catch (error) { setStatus(error.message, "failed"); }
}

function applyAnalysis() {
  const analysis = state.analysis || {};
  $("runtime-summary").textContent = analysis.runtime_summary || "요약 없음";
  $("jsonl-summary").textContent = analysis.jsonl_summary || "요약 없음";
  $("diagnosis-summary").textContent = analysis.diagnosis_summary || "진단 없음";
  $("diagnosis-raw").textContent = JSON.stringify(analysis.diagnosis_raw || [], null, 2);
  state.penaltyDirected = analysis.summary?.penalty_lanes || state.penaltyDirected;
  buildTrajectory();
  renderAnalysisTables();
  renderMap();
  setPlaybackTime(state.trajectory.min);
}

function selectedTrajectoryPhase() {
  const phases = new Set(state.events.filter((event) => event.event === "trajectory_point").map((event) => event.phase));
  if (phases.has("negotiated")) return "negotiated";
  if (phases.has("free_flow")) return "free_flow";
  if (phases.has("free_flow_baseline")) return "free_flow_baseline";
  return "";
}

function buildTrajectory() {
  const phase = selectedTrajectoryPhase();
  const byRobot = new Map();
  state.events.filter((event) => event.event === "trajectory_point" && (!phase || event.phase === phase)).forEach((event) => {
    if (!byRobot.has(event.robot)) byRobot.set(event.robot, []);
    byRobot.get(event.robot).push({time: number(event.time_s), x: number(event.x), y: number(event.y), yaw: number(event.yaw_rad)});
  });
  byRobot.forEach((points) => points.sort((a, b) => a.time - b.time));
  const allPoints = [...byRobot.values()].flat();
  const min = allPoints.length ? Math.min(...allPoints.map((point) => point.time)) : 0;
  const max = allPoints.length ? Math.max(...allPoints.map((point) => point.time)) : 0;
  state.trajectory = {byRobot, min, max, current: min};
  $("time-slider").value = 0;
  $("time-label").textContent = `0.00 / ${Math.max(0, max - min).toFixed(2)} s`;
}

function setPlaybackTime(absoluteTime) {
  state.trajectory.current = absoluteTime;
  const span = Math.max(state.trajectory.max - state.trajectory.min, 1e-9);
  const progress = (absoluteTime - state.trajectory.min) / span;
  $("time-slider").value = Math.max(0, Math.min(1000, progress * 1000));
  $("time-label").textContent = `${Math.max(0, absoluteTime - state.trajectory.min).toFixed(2)} / ${Math.max(0, state.trajectory.max - state.trajectory.min).toFixed(2)} s`;
  renderRobots(absoluteTime);
  updateLiveDecision(absoluteTime);
}

function togglePlayback() {
  if (!state.trajectory.byRobot.size) { setStatus("재생할 RMF trajectory가 없습니다", "failed"); return; }
  state.playing = !state.playing;
  $("play-button").textContent = state.playing ? "❚❚ 일시정지" : "▶ 계획 재생";
  if (state.playing) {
    if (state.trajectory.current >= state.trajectory.max - 1e-6) state.trajectory.current = state.trajectory.min;
    state.animationStarted = performance.now();
    state.animationOrigin = state.trajectory.current;
    requestAnimationFrame(playbackFrame);
  }
}

function playbackFrame(now) {
  if (!state.playing) return;
  const next = state.animationOrigin + (now - state.animationStarted) / 1000;
  if (next >= state.trajectory.max) {
    setPlaybackTime(state.trajectory.max);
    state.playing = false;
    $("play-button").textContent = "▶ 계획 재생";
    return;
  }
  setPlaybackTime(next);
  requestAnimationFrame(playbackFrame);
}

function updateLiveDecision(timeValue) {
  const phase = selectedTrajectoryPhase();
  const waypoints = state.events.filter((event) => event.event === "plan_waypoint" && event.phase === phase).sort((a, b) => number(a.time_s) - number(b.time_s));
  const current = waypoints.filter((event) => number(event.time_s) <= timeValue + 1e-6).slice(-1)[0];
  const upcoming = waypoints.find((event) => number(event.time_s) > timeValue + 1e-6);
  const negotiation = state.events.filter((event) => event.event === "negotiation_summary").slice(-1)[0];
  const schedule = state.events.filter((event) => event.event === "schedule_database_state").slice(-1)[0];
  const routeChoice = state.events.filter((event) => event.event === "route_choice_explanation" && (!current || event.robot === current.robot)).slice(-1)[0];
  const lines = [`현재 RMF 계획 시각: ${Math.max(0, timeValue - state.trajectory.min).toFixed(2)} s`];
  if (current) {
    const movement = ({start: "출발 자세", rotate_in_place: "후진 대신 제자리 회전", wait: "시간·협상 조건을 위한 정지", forward_traverse: "Lane을 따라 전진"})[current.movement_type] || current.movement_type;
    lines.push(`로봇/현재 판단: ${current.robot} · ${movement}`);
    lines.push(`좌표/방향: (${format(current.x)}, ${format(current.y)}) · yaw ${format(current.yaw_rad)} rad`);
    lines.push(`접근 Lane: ${JSON.stringify(current.approach_lanes || [])} · graph node ${current.graph_index ?? "연속 궤적점"}`);
    lines.push(`이동 근거: ${current.movement_reason || "최종 RMF plan waypoint"}`);
  }
  if (upcoming) lines.push(`다음 목표: ${upcoming.robot} → (${format(upcoming.x)}, ${format(upcoming.y)}) at +${format(number(upcoming.time_s) - state.trajectory.min)}s`);
  if (routeChoice) lines.push(`경로 근거: 선택 비용 ${format(routeChoice.selected_cost)}, 차선 후보 대비 margin ${format(routeChoice.cost_margin)}`);
  if (negotiation) lines.push(`협상: success=${negotiation.success}, executable=${negotiation.executable_plan}, safety=${negotiation.safety_verified}`);
  if (schedule) lines.push(`Schedule DB: ${schedule.phase} · version ${schedule.latest_version} · route ${schedule.stored_route_count}`);
  $("live-decision").textContent = lines.join("\n");
}

function renderAnalysisTables() {
  renderScheduleTable();
  renderAstarTable();
  renderDecisionTable();
  renderCompareTable();
}

function renderTable(table, rows, columns, onSelect) {
  table.replaceChildren();
  const thead = document.createElement("thead");
  const header = document.createElement("tr");
  columns.forEach(([key, label]) => { const th = document.createElement("th"); th.textContent = label; header.append(th); });
  thead.append(header);
  const tbody = document.createElement("tbody");
  rows.forEach((row, index) => {
    const tr = document.createElement("tr");
    columns.forEach(([key]) => { const td = document.createElement("td"); td.textContent = format(row[key]); tr.append(td); });
    tr.addEventListener("click", () => {
      tbody.querySelectorAll("tr").forEach((candidate) => candidate.classList.remove("selected"));
      tr.classList.add("selected");
      if (onSelect) onSelect(row, index);
    });
    tbody.append(tr);
  });
  table.append(thead, tbody);
}

const scheduleKinds = {
  state: {events: ["schedule_database_state"], columns: [["seq", "seq"], ["phase", "phase"], ["latest_version", "DB version"], ["participant_count", "participants"], ["stored_route_count", "routes"], ["storage", "storage"]]},
  operation: {events: ["schedule_database_operation", "schedule_commit"], columns: [["seq", "seq"], ["action", "action"], ["api", "실제 API"], ["name", "robot"], ["version_before", "ver 전"], ["version_after", "ver 후"], ["accepted", "accepted"], ["result", "result"]]},
  participant: {events: ["schedule_participant"], columns: [["seq", "seq"], ["phase", "phase"], ["participant_id", "id"], ["name", "name"], ["owner", "owner"], ["current_plan_id", "plan"], ["itinerary_version", "itinerary ver"], ["route_count", "routes"], ["trajectory_point_count", "points"]]},
  route: {events: ["schedule_database_route"], columns: [["seq", "seq"], ["phase", "phase"], ["participant_id", "participant"], ["name", "name"], ["plan_id", "plan"], ["route_id", "route"], ["map", "map"], ["start_time_s", "start"], ["finish_time_s", "finish"], ["duration_s", "duration"], ["trajectory_point_count", "points"]]},
  point: {events: ["schedule_database_trajectory_point"], columns: [["seq", "seq"], ["phase", "phase"], ["name", "name"], ["plan_id", "plan"], ["route_id", "route"], ["sequence", "point"], ["time_s", "time"], ["x", "x"], ["y", "y"], ["yaw_rad", "yaw"], ["vx", "vx"], ["vy", "vy"], ["vyaw", "vyaw"]]},
};

function renderScheduleTable() {
  const definition = scheduleKinds[$("schedule-kind").value] || scheduleKinds.state;
  const rows = state.events.filter((event) => definition.events.includes(event.event));
  renderTable($("schedule-table"), rows, definition.columns, (row) => {
    const meaning = row.event === "schedule_database_state"
      ? "이 행은 그 순간 실제 in-memory schedule::Database에 query_all을 실행해 얻은 스냅샷입니다. Navigation Graph와는 별도입니다."
      : row.event === "schedule_database_operation"
        ? "이 행은 Database/Participant에 실제로 수행한 쓰기 또는 읽기 API와 version 변화를 기록합니다."
        : row.event === "schedule_database_route"
          ? "이 Route는 Planner 결과를 복사해 꾸민 값이 아니라 Database::query(query_all)이 반환한 itinerary입니다."
          : row.event === "schedule_database_trajectory_point"
            ? "저장된 Route의 원본 시간좌표점입니다. 같은 participant/plan/route 안에서 sequence 순서로 읽으세요."
            : "등록된 Schedule participant의 설명과 itinerary version입니다.";
    $("schedule-explanation").textContent = `${meaning}\n\n${JSON.stringify(row, null, 2)}`;
  });
  if (!rows.length) $("schedule-explanation").textContent = state.analysis?.schedule_guide || "실행 후 실제 Schedule Database 기록이 표시됩니다.";
}

const astarColumns = [["seq", "seq"], ["event", "event"], ["robot", "robot"], ["step", "step"], ["selected_node_id", "selected node"], ["node_id", "node"], ["waypoint", "waypoint"], ["selected_waypoint", "selected wp"], ["g", "g"], ["h", "h"], ["f", "f"], ["selected_g", "selected g"], ["selected_h", "selected h"], ["selected_f", "selected f"], ["next_best_f", "차순위 f"], ["f_margin_to_next", "f margin"], ["queue_size", "queue"], ["frontier_size_after", "frontier 후"]];

function renderAstarTable() {
  const rows = state.events.filter((event) => ["astar_step_decision", "astar_expand", "astar_generated", "astar_step_summary", "astar_frontier_best", "astar_trace_summary"].includes(event.event));
  renderTable($("astar-table"), rows, astarColumns, (row) => {
    const g = row.selected_g ?? row.g;
    const h = row.selected_h ?? row.h;
    const f = row.selected_f ?? row.f;
    $("astar-explanation").textContent = [
      `실제 이벤트: ${row.event} · seq ${row.seq ?? "-"} · ${row.robot || ""}`,
      `선택 비용: g=${format(g)}, h=${format(h)}, f=${format(f)} (기록상 g+h=${format(number(g) + number(h))})`,
      row.next_best_f !== undefined ? `차순위 f=${format(row.next_best_f)}, margin=${format(row.f_margin_to_next)}. margin이 0이면 기본 Debug API 밖의 tie-break가 작동할 수 있습니다.` : "",
      row.delta_g_from_parent !== undefined ? `부모 대비 Δg=${format(row.delta_g_from_parent)}, Δh=${format(row.delta_h_from_parent)}, Δf=${format(row.delta_f_from_parent)}` : "",
      "선택 근거: 실제 Planner::Debug frontier 우선순위 큐의 top이었습니다. 기본 API는 각 탈락 분기의 완전한 reason code와 g 세부분해를 제공하지 않습니다.",
      "",
      JSON.stringify(row, null, 2),
    ].filter(Boolean).join("\n");
  });
  if (!rows.length) $("astar-explanation").textContent = state.analysis?.astar_guide || "실행 후 실제 Planner::Debug frontier가 표시됩니다.";
}

function renderDecisionTable() {
  const rows = state.analysis?.decisions || [];
  const columns = [["seq", "seq"], ["phase", "단계"], ["robot", "로봇"], ["decision", "무엇을 판단했나"], ["reason", "왜 그렇게 했나"], ["evidence", "실제 근거값"], ["result", "결과"]];
  renderTable($("decision-table"), rows, columns, (row) => { $("decision-explanation").textContent = `${row.detail || ""}\n\n원본 이벤트\n${JSON.stringify(row.event || {}, null, 2)}`; });
}

function renderCompareTable() {
  const comparison = state.comparison || {rows: [], explanation: "Before와 After를 같은 시나리오로 각각 실행하면 비교 결과가 표시됩니다."};
  renderTable($("compare-table"), comparison.rows || [], [["label", "항목"], ["before", "BEFORE"], ["after", "AFTER"], ["change", "변화/판정"]], (row) => {
    $("compare-explanation").textContent = `${comparison.explanation || ""}\n\n선택 항목\n${JSON.stringify(row, null, 2)}`;
  });
  $("compare-explanation").textContent = comparison.explanation || "Before와 After를 같은 시나리오로 각각 실행하면 비교 결과가 표시됩니다.";
}

function tableToTsv(table) {
  return [...table.querySelectorAll("tr")].map((row) => [...row.children].map((cell) => cell.textContent.replace(/\t|\r?\n/g, " ")).join("\t")).join("\n");
}

async function copyTable(tableId) {
  try { await navigator.clipboard.writeText(tableToTsv($(tableId))); setStatus("표 전체를 TSV로 복사했습니다", "completed"); }
  catch (error) { setStatus(`복사 실패: ${error.message}`, "failed"); }
}

async function refreshComparison() {
  try { state.comparison = await api("/api/compare"); renderCompareTable(); setStatus("Before/After 최근 결과를 읽었습니다", "completed"); }
  catch (error) { setStatus(error.message, "failed"); }
}

async function prepareAfter() {
  if (!confirm("Before rmf_traffic 소스를 After workspace로 복사하고 실험용 A* lane penalty 패치를 적용합니다. 계속할까요?")) return;
  try {
    setStatus("AFTER 코어를 준비하는 중입니다", "running");
    const result = await api("/api/after-core/prepare", {method: "POST", body: JSON.stringify({before_source: $("before-source").value, after_workspace: $("after-workspace").value})});
    $("after-source").value = result.after_source || $("after-source").value;
    setStatus(`AFTER 코어 준비 완료: ${result.after_source || "완료"}`, "completed");
  } catch (error) { setStatus(error.message, "failed"); }
}

function activateTabs(buttonSelector, targetPrefix, dataField) {
  document.querySelectorAll(buttonSelector).forEach((button) => button.addEventListener("click", () => {
    document.querySelectorAll(buttonSelector).forEach((candidate) => candidate.classList.remove("active"));
    button.classList.add("active");
    const value = button.dataset[dataField];
    document.querySelectorAll(`[id^="${targetPrefix}"]`).forEach((page) => page.classList.remove("active"));
    $(`${targetPrefix}${value}`).classList.add("active");
  }));
}

function installSplitters() {
  const horizontal = $("horizontal-splitter");
  horizontal.addEventListener("pointerdown", (event) => {
    horizontal.setPointerCapture(event.pointerId);
    horizontal.classList.add("dragging");
    const move = (next) => {
      const rect = $("top-workspace").getBoundingClientRect();
      const width = Math.max(330, Math.min(rect.width - 420, rect.right - next.clientX));
      document.documentElement.style.setProperty("--editor-width", `${width}px`);
    };
    const up = () => { horizontal.classList.remove("dragging"); horizontal.removeEventListener("pointermove", move); horizontal.removeEventListener("pointerup", up); requestAnimationFrame(renderMap); };
    horizontal.addEventListener("pointermove", move);
    horizontal.addEventListener("pointerup", up);
  });
  const vertical = $("vertical-splitter");
  vertical.addEventListener("pointerdown", (event) => {
    vertical.setPointerCapture(event.pointerId);
    vertical.classList.add("dragging");
    const move = (next) => {
      const workspace = $("workspace").getBoundingClientRect();
      const height = Math.max(180, Math.min(workspace.height - 260, workspace.bottom - next.clientY));
      document.documentElement.style.setProperty("--output-height", `${height}px`);
    };
    const up = () => { vertical.classList.remove("dragging"); vertical.removeEventListener("pointermove", move); vertical.removeEventListener("pointerup", up); requestAnimationFrame(renderMap); };
    vertical.addEventListener("pointermove", move);
    vertical.addEventListener("pointerup", up);
  });
}

function installMapInteraction() {
  const svg = $("map-svg");
  svg.addEventListener("click", () => { state.selectedNodes.clear(); state.selectedLane = null; renderMap(); showSelection(); });
  svg.addEventListener("wheel", (event) => { event.preventDefault(); zoomAt(event.deltaY < 0 ? 1.12 : 1 / 1.12, svgPoint(event)); }, {passive: false});
  svg.addEventListener("pointerdown", (event) => {
    if (![1, 2].includes(event.button)) return;
    event.preventDefault();
    svg.setPointerCapture(event.pointerId);
    state.panning = {pointerId: event.pointerId, x: event.clientX, y: event.clientY, tx: state.view.tx, ty: state.view.ty};
  });
  svg.addEventListener("pointermove", (event) => {
    if (!state.panning || state.panning.pointerId !== event.pointerId) return;
    state.view.tx = state.panning.tx + event.clientX - state.panning.x;
    state.view.ty = state.panning.ty + event.clientY - state.panning.y;
    renderMap();
  });
  svg.addEventListener("pointerup", () => { state.panning = null; });
  svg.addEventListener("contextmenu", (event) => event.preventDefault());
}

async function initialize() {
  syncChromeHeight();
  window.addEventListener("resize", () => { syncChromeHeight(); renderMap(); });
  activateTabs(".sub-tab", "right-", "rightTab");
  activateTabs(".output-tab", "output-", "outputTab");
  document.querySelectorAll("[data-schedule-side]").forEach((button) => button.addEventListener("click", () => {
    button.parentElement.querySelectorAll("button").forEach((candidate) => candidate.classList.remove("active")); button.classList.add("active");
    if (button.dataset.scheduleSide === "guide") $("schedule-explanation").textContent = state.analysis?.schedule_guide || "실행 후 가이드가 표시됩니다.";
    else renderScheduleTable();
  }));
  document.querySelectorAll("[data-astar-side]").forEach((button) => button.addEventListener("click", () => {
    button.parentElement.querySelectorAll("button").forEach((candidate) => candidate.classList.remove("active")); button.classList.add("active");
    if (button.dataset.astarSide === "guide") $("astar-explanation").textContent = state.analysis?.astar_guide || "실행 후 가이드가 표시됩니다.";
    else renderAstarTable();
  }));
  installMapInteraction();
  installSplitters();
  $("scenario-select").addEventListener("change", () => loadScenario($("scenario-select").value).catch((error) => setStatus(error.message, "failed")));
  $("load-json-button").addEventListener("click", () => $("load-json-input").click());
  $("load-json-input").addEventListener("change", async (event) => {
    try { clearYamlSource(); setDocument(JSON.parse(await event.target.files[0].text())); setStatus("로컬 JSON을 열었습니다", "completed"); }
    catch (error) { setStatus(`JSON 열기 실패: ${error.message}`, "failed"); }
    event.target.value = "";
  });
  $("load-yaml-button").addEventListener("click", () => $("load-yaml-input").click());
  $("load-yaml-input").addEventListener("change", async (event) => {
    const file = event.target.files[0];
    if (!file) return;
    try {
      state.yamlText = await file.text();
      state.yamlFileName = file.name;
      await importYamlMap();
    } catch (error) {
      clearYamlSource();
      setStatus(`YAML 맵 열기 실패: ${error.message}`, "failed");
    }
    event.target.value = "";
  });
  $("yaml-level-select").addEventListener("change", () => {
    importYamlMap($("yaml-level-select").value).catch((error) => setStatus(`YAML 레벨 열기 실패: ${error.message}`, "failed"));
  });
  $("download-json-button").addEventListener("click", downloadScenario);
  $("save-server-button").addEventListener("click", () => saveScenario().catch((error) => setStatus(error.message, "failed")));
  $("core-profile").addEventListener("change", updateCoreProfile);
  $("before-setup").addEventListener("change", () => { if ($("core-profile").value === "before") updateCoreProfile(); });
  $("after-setup").addEventListener("change", () => { if ($("core-profile").value === "after") updateCoreProfile(); });
  $("run-button").addEventListener("click", startRun);
  $("stop-button").addEventListener("click", stopRun);
  $("add-node-button").addEventListener("click", addNode);
  $("add-lane-button").addEventListener("click", addLane);
  $("delete-selection-button").addEventListener("click", deleteSelection);
  $("apply-properties-button").addEventListener("click", applyProperties);
  $("add-robot-button").addEventListener("click", addRobot);
  $("zoom-out-button").addEventListener("click", () => zoomAt(1 / 1.2));
  $("zoom-in-button").addEventListener("click", () => zoomAt(1.2));
  $("zoom-reset-button").addEventListener("click", () => { state.view.scale = 70; updateZoomLabel(); renderMap(); });
  $("fit-button").addEventListener("click", fitMap);
  $("focus-map-button").addEventListener("click", () => { document.body.classList.toggle("map-focused"); requestAnimationFrame(fitMap); });
  $("toggle-output-button").addEventListener("click", () => { document.body.classList.toggle("output-collapsed"); requestAnimationFrame(renderMap); });
  $("reset-layout-button").addEventListener("click", () => {
    document.body.classList.remove("map-focused", "output-collapsed");
    document.documentElement.style.setProperty("--editor-width", "430px");
    document.documentElement.style.setProperty("--output-height", "390px");
    requestAnimationFrame(fitMap);
  });
  $("play-button").addEventListener("click", togglePlayback);
  $("time-slider").addEventListener("input", () => {
    state.playing = false; $("play-button").textContent = "▶ 계획 재생";
    setPlaybackTime(state.trajectory.min + number($("time-slider").value) / 1000 * (state.trajectory.max - state.trajectory.min));
  });
  $("schedule-kind").addEventListener("change", renderScheduleTable);
  $("copy-schedule-button").addEventListener("click", () => copyTable("schedule-table"));
  $("copy-astar-button").addEventListener("click", () => copyTable("astar-table"));
  $("copy-decisions-button").addEventListener("click", () => copyTable("decision-table"));
  $("copy-compare-button").addEventListener("click", () => copyTable("compare-table"));
  $("refresh-compare-button").addEventListener("click", refreshComparison);
  $("prepare-after-button").addEventListener("click", prepareAfter);
  $("help-button").addEventListener("click", () => $("help-dialog").showModal());
  $("close-help-button").addEventListener("click", () => $("help-dialog").close());
  $("save-token-button").addEventListener("click", () => { localStorage.setItem("rmf_lab_token", $("access-token").value); $("help-dialog").close(); location.reload(); });
  $("access-token").value = token();

  try {
    const [health, config] = await Promise.all([api("/api/health"), api("/api/config")]);
    $("connection-badge").textContent = `웹 서버 ${health.version}`;
    $("connection-badge").className = "badge completed";
    $("before-setup").value = config.before_setup;
    $("before-source").value = config.before_source;
    $("after-workspace").value = config.after_workspace;
    $("after-setup").value = config.after_setup;
    $("after-source").value = config.after_source;
    $("base-ros-setup").value = config.base_ros_setup;
    $("after-label").value = config.after_label;
    $("setup-path").value = config.before_setup;
    if (!config.allow_core_patch) $("prepare-after-button").disabled = true;
    await loadCatalog();
    await refreshComparison();
  } catch (error) {
    $("connection-badge").textContent = "서버 연결 실패";
    $("connection-badge").className = "badge failed";
    setStatus(error.message, "failed");
  }
}

initialize();
