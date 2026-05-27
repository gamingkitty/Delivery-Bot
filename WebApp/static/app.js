const elements = {
  adminLoginButton: document.getElementById("adminLoginButton"),
  adminPin: document.getElementById("adminPin"),
  cameraFeed: document.getElementById("cameraFeed"),
  campusMap: document.getElementById("campusMap"),
  connectionState: document.getElementById("connectionState"),
  debugPanel: document.getElementById("debugPanel"),
  debugError: document.getElementById("debugError"),
  debugStatus: document.getElementById("debugStatus"),
  deepsceneFeed: document.getElementById("deepsceneFeed"),
  destinationMarker: document.getElementById("destinationMarker"),
  destinationSelect: document.getElementById("destinationSelect"),
  dispatchForm: document.getElementById("dispatchForm"),
  headingText: document.getElementById("headingText"),
  manualButtons: Array.from(document.querySelectorAll("[data-manual-direction]")),
  mapEditorClearButton: document.getElementById("mapEditorClearButton"),
  mapEditorDownloadButton: document.getElementById("mapEditorDownloadButton"),
  mapEditorDrawButton: document.getElementById("mapEditorDrawButton"),
  mapEditorLines: document.getElementById("mapEditorLines"),
  mapEditorNewLineButton: document.getElementById("mapEditorNewLineButton"),
  mapEditorPointCount: document.getElementById("mapEditorPointCount"),
  mapEditorPoints: document.getElementById("mapEditorPoints"),
  mapEditorPreview: document.getElementById("mapEditorPreview"),
  mapEditorSnapRadius: document.getElementById("mapEditorSnapRadius"),
  mapEditorSpacing: document.getElementById("mapEditorSpacing"),
  mapFrame: document.getElementById("mapFrame"),
  plannedRoute: document.getElementById("plannedRoute"),
  progressText: document.getElementById("progressText"),
  robotMarker: document.getElementById("robotMarker"),
  statePill: document.getElementById("statePill"),
  statusText: document.getElementById("statusText"),
  stopButton: document.getElementById("stopButton"),
  userPin: document.getElementById("userPin"),
  waypointText: document.getElementById("waypointText"),
};

const EARTH_RADIUS_M = 6378137.0;
const MAP_EDITOR_IDS = {
  clearButton: "mapEditorClearButton",
  downloadButton: "mapEditorDownloadButton",
  drawButton: "mapEditorDrawButton",
  lines: "mapEditorLines",
  newLineButton: "mapEditorNewLineButton",
  pointCount: "mapEditorPointCount",
  points: "mapEditorPoints",
  preview: "mapEditorPreview",
  snapRadius: "mapEditorSnapRadius",
  spacing: "mapEditorSpacing",
  frame: "mapFrame",
};

const state = {
  config: null,
  debugUnlocked: false,
  mapEditor: {
    active: false,
    draftStart: null,
    lines: [],
    pointer: null,
    wired: false,
  },
  manualControlClientId: `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`,
  manualControlDirection: null,
  manualControlSequence: 0,
  manualControlTimer: null,
  manualControlRepeatMs: 220,
  pollIntervalMs: 750,
};

async function init() {
  const response = await fetch("/api/config/public");
  state.config = await response.json();
  state.pollIntervalMs = Number(state.config.ui.poll_interval_ms) || 750;

  elements.campusMap.src = state.config.map.image_url;
  populateStops(state.config.stops);
  wireEvents();
  await refreshStatus();
  setInterval(refreshStatus, state.pollIntervalMs);
}

function populateStops(stops) {
  elements.destinationSelect.innerHTML = "";

  for (const stop of stops) {
    const option = document.createElement("option");
    option.value = stop.id;
    option.textContent = stop.name;
    elements.destinationSelect.appendChild(option);
  }

  updateDestinationMarker();
}

function wireEvents() {
  elements.dispatchForm.addEventListener("submit", async (event) => {
    event.preventDefault();
    await postJson("/api/jobs", {
      destination: elements.destinationSelect.value,
      pin: elements.userPin.value,
    });
    await refreshStatus();
  });

  elements.stopButton.addEventListener("click", async () => {
    await postJson("/api/stop", {
      pin: elements.userPin.value,
    });
    await refreshStatus();
  });

  elements.destinationSelect.addEventListener("change", updateDestinationMarker);

  elements.adminLoginButton.addEventListener("click", async () => {
    await postJson("/api/admin/login", {
      pin: elements.adminPin.value,
    });
    state.debugUnlocked = true;
    elements.debugPanel.hidden = false;
    if (state.config.debug.enabled) {
      elements.cameraFeed.src = `/api/debug/camera?ts=${Date.now()}`;
      elements.deepsceneFeed.src = `/api/debug/deepscene?ts=${Date.now()}`;
    }
    safeRenderMapEditor();
    await refreshDebug();
  });

  for (const button of elements.manualButtons) {
    const direction = button.dataset.manualDirection;

    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      startManualControl(direction);
    });

    button.addEventListener("keydown", (event) => {
      if ((event.key === "Enter" || event.key === " ") && !event.repeat) {
        event.preventDefault();
        startManualControl(direction);
      }
    });

    button.addEventListener("keyup", (event) => {
      if (event.key === "Enter" || event.key === " ") {
        event.preventDefault();
        stopManualControl();
      }
    });
  }

  window.addEventListener("pointerup", stopManualControl);
  window.addEventListener("pointercancel", stopManualControl);
  window.addEventListener("blur", stopManualControl);

  wireMapEditorEvents();
  safeRenderMapEditor();
}

function wireMapEditorEvents() {
  if (state.mapEditor.wired) {
    return;
  }

  const editor = mapEditorElements();
  if (!hasMapEditorElements(editor)) {
    return;
  }

  editor.drawButton.addEventListener("click", () => {
    setMapEditorActive(!state.mapEditor.active);
  });

  editor.newLineButton.addEventListener("click", () => {
    cancelMapEditorDraft();
  });

  editor.undoButton.addEventListener("click", () => {
    const removedLine = state.mapEditor.lines.pop();
    if (removedLine) {
      state.mapEditor.draftStart = cloneMapPoint(removedLine.start);
      state.mapEditor.pointer = null;
    }
    safeRenderMapEditor();
  });

  editor.clearButton.addEventListener("click", () => {
    state.mapEditor.lines = [];
    state.mapEditor.draftStart = null;
    state.mapEditor.pointer = null;
    safeRenderMapEditor();
  });

  editor.downloadButton.addEventListener("click", downloadMapEditorPointCloud);
  editor.spacing.addEventListener("input", safeRenderMapEditor);
  editor.snapRadius.addEventListener("input", renderMapEditorPreview);

  editor.frame.addEventListener("pointerdown", handleMapEditorPointerDown);
  editor.frame.addEventListener("pointermove", handleMapEditorPointerMove);
  editor.frame.addEventListener("pointerleave", () => {
    state.mapEditor.pointer = null;
    renderMapEditorPreview();
  });
  editor.frame.addEventListener("contextmenu", (event) => {
    if (!canEditMap()) {
      return;
    }

    event.preventDefault();
    cancelMapEditorDraft();
  });

  window.addEventListener("keydown", (event) => {
    if (event.key === "Escape" && canEditMap()) {
      cancelMapEditorDraft();
    }
  });

  state.mapEditor.wired = true;
}

function setMapEditorActive(active) {
  if (!hasMapEditorElements(mapEditorElements())) {
    state.mapEditor.active = false;
    return;
  }

  state.mapEditor.active = Boolean(active) && state.debugUnlocked;

  if (!state.mapEditor.active) {
    state.mapEditor.draftStart = null;
    state.mapEditor.pointer = null;
  }

  safeRenderMapEditor();
}

function canEditMap() {
  return (
    state.debugUnlocked &&
    state.mapEditor.active &&
    hasMapEditorElements(mapEditorElements())
  );
}

function cancelMapEditorDraft() {
  state.mapEditor.draftStart = null;
  state.mapEditor.pointer = null;
  safeRenderMapEditor();
}

function handleMapEditorPointerDown(event) {
  if (!canEditMap() || event.button !== 0) {
    return;
  }

  const point = eventToMapPoint(event);
  if (!point) {
    return;
  }

  event.preventDefault();
  const snappedPoint = snapMapEditorPoint(point);

  if (!state.mapEditor.draftStart) {
    state.mapEditor.draftStart = snappedPoint;
    state.mapEditor.pointer = null;
    safeRenderMapEditor();
    return;
  }

  if (mapPointDistance(state.mapEditor.draftStart, snappedPoint) < 0.05) {
    return;
  }

  state.mapEditor.lines.push({
    start: cloneMapPoint(state.mapEditor.draftStart),
    end: cloneMapPoint(snappedPoint),
  });
  state.mapEditor.draftStart = cloneMapPoint(snappedPoint);
  state.mapEditor.pointer = null;
  safeRenderMapEditor();
}

function handleMapEditorPointerMove(event) {
  if (!canEditMap() || !state.mapEditor.draftStart) {
    return;
  }

  const point = eventToMapPoint(event);
  state.mapEditor.pointer = point ? snapMapEditorPoint(point) : null;
  renderMapEditorPreview();
}

function safeRenderMapEditor() {
  try {
    renderMapEditor();
  } catch (error) {
    state.mapEditor.active = false;
    showDebugError(`Map editor unavailable: ${error.message}`);
  }
}

function renderMapEditor() {
  const editor = mapEditorElements();
  if (!hasMapEditorElements(editor)) {
    return;
  }

  editor.frame.classList.toggle("map-editing", canEditMap());
  editor.drawButton.textContent = state.mapEditor.active ? "Done" : "Draw";
  editor.drawButton.setAttribute(
    "aria-pressed",
    String(state.mapEditor.active)
  );

  replaceElementChildren(
    editor.lines,
    state.mapEditor.lines.map(createMapEditorLineElement)
  );
  replaceElementChildren(
    editor.points,
    uniqueMapEditorEndpoints().map(createMapEditorPointElement)
  );

  renderMapEditorPreview();
  const pointCount = updateMapEditorSummary();
  const hasDraft = Boolean(state.mapEditor.draftStart);
  const hasLines = state.mapEditor.lines.length > 0;
  editor.newLineButton.disabled = !hasDraft;
  editor.undoButton.disabled = !hasLines;
  editor.clearButton.disabled = !hasDraft && !hasLines;
  editor.downloadButton.disabled = pointCount === 0;
}

function renderMapEditorPreview() {
  const editor = mapEditorElements();
  if (!hasMapEditorElements(editor)) {
    return;
  }

  const preview = editor.preview;

  if (!canEditMap() || !state.mapEditor.draftStart || !state.mapEditor.pointer) {
    preview.classList.remove("visible");
    return;
  }

  preview.setAttribute("x1", state.mapEditor.draftStart.x);
  preview.setAttribute("y1", state.mapEditor.draftStart.y);
  preview.setAttribute("x2", state.mapEditor.pointer.x);
  preview.setAttribute("y2", state.mapEditor.pointer.y);
  preview.classList.add("visible");
}

function mapEditorElements() {
  const editor = {};

  for (const [key, id] of Object.entries(MAP_EDITOR_IDS)) {
    editor[key] = document.getElementById(id);
  }

  editor.undoButton = document.getElementById("mapEditorUndoButton");
  return editor;
}

function hasMapEditorElements(editor) {
  return Boolean(
    editor.clearButton &&
      editor.downloadButton &&
      editor.drawButton &&
      editor.frame &&
      editor.lines &&
      editor.newLineButton &&
      editor.pointCount &&
      editor.points &&
      editor.preview &&
      editor.snapRadius &&
      editor.spacing &&
      editor.undoButton
  );
}

function replaceElementChildren(element, children) {
  while (element.firstChild) {
    element.removeChild(element.firstChild);
  }

  for (const child of children) {
    element.appendChild(child);
  }
}

function updateMapEditorSummary() {
  const editor = mapEditorElements();

  if (!hasMapEditorElements(editor)) {
    return 0;
  }

  let pointCount = 0;

  try {
    pointCount = sampleMapEditorPointCloudPoints().length;
  } catch (_error) {
    pointCount = 0;
  }

  editor.pointCount.textContent = `${pointCount} ${
    pointCount === 1 ? "point" : "points"
  }`;
  return pointCount;
}

function createMapEditorLineElement(line) {
  const element = createSvgElement("line");
  element.setAttribute("class", "map-editor-line");
  element.setAttribute("x1", line.start.x);
  element.setAttribute("y1", line.start.y);
  element.setAttribute("x2", line.end.x);
  element.setAttribute("y2", line.end.y);
  return element;
}

function createMapEditorPointElement(point) {
  const element = createSvgElement("circle");
  element.setAttribute("class", "map-editor-point");
  element.setAttribute("cx", point.x);
  element.setAttribute("cy", point.y);
  element.setAttribute("r", "0.75");
  return element;
}

function createSvgElement(tagName) {
  return document.createElementNS("http://www.w3.org/2000/svg", tagName);
}

function uniqueMapEditorEndpoints() {
  const points = [];
  const seen = new Set();

  for (const line of state.mapEditor.lines) {
    addUniqueMapPoint(points, seen, line.start);
    addUniqueMapPoint(points, seen, line.end);
  }

  if (state.mapEditor.draftStart) {
    addUniqueMapPoint(points, seen, state.mapEditor.draftStart);
  }

  return points;
}

function addUniqueMapPoint(points, seen, point) {
  const key = mapPointKey(point, 3);

  if (seen.has(key)) {
    return;
  }

  seen.add(key);
  points.push(cloneMapPoint(point));
}

function eventToMapPoint(event) {
  const editor = mapEditorElements();

  if (!editor.frame) {
    return null;
  }

  const rect = editor.frame.getBoundingClientRect();

  if (rect.width <= 0 || rect.height <= 0) {
    return null;
  }

  return normalizeMapPoint({
    x: ((event.clientX - rect.left) / rect.width) * 100,
    y: ((event.clientY - rect.top) / rect.height) * 100,
  });
}

function snapMapEditorPoint(point) {
  const nearest = nearestMapEditorEndpoint(point);
  return nearest || normalizeMapPoint(point);
}

function nearestMapEditorEndpoint(point) {
  const editor = mapEditorElements();

  if (!editor.frame) {
    return null;
  }

  const rect = editor.frame.getBoundingClientRect();
  const snapRadiusPx = mapEditorSnapRadiusPx();
  const target = mapPointToFramePixels(point, rect);
  let bestDistanceSq = snapRadiusPx * snapRadiusPx;
  let bestPoint = null;

  for (const endpoint of uniqueMapEditorEndpoints()) {
    const candidate = mapPointToFramePixels(endpoint, rect);
    const distanceSq =
      (target.x - candidate.x) ** 2 + (target.y - candidate.y) ** 2;

    if (distanceSq <= bestDistanceSq) {
      bestDistanceSq = distanceSq;
      bestPoint = endpoint;
    }
  }

  return bestPoint ? cloneMapPoint(bestPoint) : null;
}

function mapPointToFramePixels(point, rect) {
  return {
    x: (point.x / 100) * rect.width,
    y: (point.y / 100) * rect.height,
  };
}

function downloadMapEditorPointCloud() {
  try {
    const pointCloud = buildMapEditorPointCloud();

    if (!pointCloud.points.length) {
      showDebugError("Draw at least one map line before downloading");
      return;
    }

    const blob = new Blob([`${JSON.stringify(pointCloud, null, 2)}\n`], {
      type: "application/json",
    });
    const url = URL.createObjectURL(blob);
    const link = document.createElement("a");
    link.href = url;
    link.download = `point_cloud_map_${new Date()
      .toISOString()
      .replace(/[:.]/g, "-")}.json`;
    document.body.appendChild(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 0);
  } catch (error) {
    showDebugError(error.message);
  }
}

function buildMapEditorPointCloud() {
  const now = Date.now() / 1000;
  const points = sampleMapEditorPointCloudPoints();
  const map = state.config.map;

  return {
    version: 1,
    units: "meters",
    created_at: now,
    updated_at: now,
    point_count: points.length,
    generator: "webapp_admin_line_editor",
    point_spacing_m: mapEditorSpacingM(),
    line_count: state.mapEditor.lines.length,
    navigation_origin: publicLatLon(map.navigation_origin),
    map_bounds: {
      top_left: publicLatLon(map.top_left),
      bottom_right: publicLatLon(map.bottom_right),
    },
    points,
  };
}

function sampleMapEditorPointCloudPoints() {
  if (!state.config || !state.config.map || !state.mapEditor.lines.length) {
    return [];
  }

  const spacingM = mapEditorSpacingM();
  const points = [];
  const seen = new Set();

  for (const line of state.mapEditor.lines) {
    const start = mapPointToXY(line.start);
    const end = mapPointToXY(line.end);
    const distance = Math.hypot(end.x - start.x, end.y - start.y);
    const steps = Math.max(1, Math.ceil(distance / spacingM));

    for (let index = 0; index <= steps; index += 1) {
      const t = index / steps;
      const point = {
        x: roundNumber(start.x + (end.x - start.x) * t, 3),
        y: roundNumber(start.y + (end.y - start.y) * t, 3),
      };
      const key = `${point.x.toFixed(3)},${point.y.toFixed(3)}`;

      if (!seen.has(key)) {
        seen.add(key);
        points.push(point);
      }
    }
  }

  return points;
}

function mapPointToXY(point) {
  const latlon = mapPointToLatLon(point);
  return latLonToXY(latlon.lat, latlon.lon, navigationOrigin());
}

function mapPointToLatLon(point) {
  const map = state.config.map;
  const xRatio = clamp(point.x, 0, 100) / 100;
  const yRatio = clamp(point.y, 0, 100) / 100;

  return {
    lat: map.top_left.lat - yRatio * (map.top_left.lat - map.bottom_right.lat),
    lon: map.top_left.lon + xRatio * (map.bottom_right.lon - map.top_left.lon),
  };
}

function latLonToXY(lat, lon, origin) {
  const originLat = Number(origin.lat);
  const originLon = Number(origin.lon);
  const originLatRad = degreesToRadians(originLat);
  const dLat = degreesToRadians(Number(lat) - originLat);
  const dLon = degreesToRadians(Number(lon) - originLon);

  return {
    x: EARTH_RADIUS_M * dLon * Math.cos(originLatRad),
    y: EARTH_RADIUS_M * dLat,
  };
}

function navigationOrigin() {
  const origin = state.config?.map?.navigation_origin;

  if (
    !origin ||
    origin.lat === undefined ||
    origin.lon === undefined ||
    origin.lat === null ||
    origin.lon === null
  ) {
    throw new Error("Map navigation origin is missing");
  }

  return origin;
}

function publicLatLon(point) {
  return {
    lat: Number(point.lat),
    lon: Number(point.lon),
  };
}

function normalizeMapPoint(point) {
  return {
    x: roundNumber(clamp(point.x, 0, 100), 4),
    y: roundNumber(clamp(point.y, 0, 100), 4),
  };
}

function cloneMapPoint(point) {
  return {
    x: Number(point.x),
    y: Number(point.y),
  };
}

function mapPointDistance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function mapPointKey(point, digits) {
  return `${Number(point.x).toFixed(digits)},${Number(point.y).toFixed(digits)}`;
}

function mapEditorSpacingM() {
  const editor = mapEditorElements();
  const value = Number(editor.spacing?.value);
  return clamp(Number.isFinite(value) ? value : 0.25, 0.05, 5);
}

function mapEditorSnapRadiusPx() {
  const editor = mapEditorElements();
  const value = Number(editor.snapRadius?.value);
  return clamp(Number.isFinite(value) ? value : 18, 2, 80);
}

function degreesToRadians(value) {
  return (Number(value) * Math.PI) / 180;
}

function roundNumber(value, digits) {
  const factor = 10 ** digits;
  return Math.round(Number(value) * factor) / factor;
}

async function postJson(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(body),
  });
  const data = await response.json();

  if (!response.ok) {
    throw new Error(data.error || `Request failed: ${response.status}`);
  }

  return data;
}

async function refreshStatus() {
  try {
    const response = await fetch("/api/status");
    const data = await response.json();
    renderStatus(data);
    elements.connectionState.textContent = "Connected";

    if (state.debugUnlocked) {
      await refreshDebug();
    }
  } catch (error) {
    elements.connectionState.textContent = error.message;
  }
}

async function refreshDebug() {
  const response = await fetch("/api/debug/status");

  if (!response.ok) {
    state.debugUnlocked = false;
    setMapEditorActive(false);
    elements.debugPanel.hidden = true;
    return;
  }

  const data = await response.json();
  const error = data.last_error || data.debug_camera?.error || "";
  elements.debugError.textContent = error;
  elements.debugError.hidden = !error;
  elements.debugStatus.textContent = JSON.stringify(data, null, 2);
}

function startManualControl(direction) {
  if (!state.debugUnlocked) {
    return;
  }

  if (state.manualControlDirection === direction) {
    return;
  }

  stopManualControl({ sendStop: false });
  state.manualControlDirection = direction;
  setManualButtonActive(direction);
  sendManualControl(direction);
  state.manualControlTimer = setInterval(() => {
    sendManualControl(direction);
  }, state.manualControlRepeatMs);
}

function stopManualControl(options = {}) {
  if (!state.manualControlDirection) {
    return;
  }

  const sendStop = options.sendStop !== false;
  clearInterval(state.manualControlTimer);
  state.manualControlTimer = null;
  state.manualControlDirection = null;
  setManualButtonActive(null);

  if (sendStop && state.debugUnlocked) {
    sendManualControl("stop");
  }
}

async function sendManualControl(direction) {
  try {
    const status = await postJson("/api/admin/manual-control", {
      direction,
      client_id: state.manualControlClientId,
      duration_sec: direction === "stop" ? 0 : 0.45,
      sequence: ++state.manualControlSequence,
    });
    renderStatus(status);
  } catch (error) {
    showDebugError(error.message);
    stopManualControl({ sendStop: false });
  }
}

function setManualButtonActive(direction) {
  for (const button of elements.manualButtons) {
    button.classList.toggle(
      "active",
      Boolean(direction) && button.dataset.manualDirection === direction
    );
  }
}

function showDebugError(message) {
  elements.debugError.textContent = message;
  elements.debugError.hidden = !message;
}

function renderStatus(status) {
  const stateLabel = titleCase(status.state || "idle");
  elements.statePill.textContent = stateLabel;
  elements.statusText.textContent = status.message || stateLabel;
  elements.progressText.textContent = `${Math.round((status.progress || 0) * 100)}%`;
  elements.headingText.textContent = `${Math.round(status.robot?.heading_deg || 0)} deg`;
  elements.waypointText.textContent =
    status.active_waypoint_index === null || status.active_waypoint_index === undefined
      ? "-"
      : String(status.active_waypoint_index + 1);

  placeMarker(elements.robotMarker, status.robot);
  renderRoute(status.planned_path || []);

  if (status.destination) {
    placeMarker(elements.destinationMarker, status.destination);
  } else {
    updateDestinationMarker();
  }
}

function updateDestinationMarker() {
  const stop = selectedStop();

  if (!stop) {
    elements.destinationMarker.classList.remove("visible");
    return;
  }

  placeMarker(elements.destinationMarker, stop);
}

function selectedStop() {
  if (!state.config) {
    return null;
  }

  return state.config.stops.find((stop) => stop.id === elements.destinationSelect.value);
}

function renderRoute(path) {
  if (!path.length) {
    elements.plannedRoute.setAttribute("points", "");
    return;
  }

  const points = path
    .map((point) => {
      const [x, y] = latlonToPercent(point.lat, point.lon);
      return `${x.toFixed(3)},${y.toFixed(3)}`;
    })
    .join(" ");

  elements.plannedRoute.setAttribute("points", points);
}

function placeMarker(marker, point) {
  if (
    !point ||
    point.lat === undefined ||
    point.lon === undefined ||
    point.lat === null ||
    point.lon === null
  ) {
    marker.classList.remove("visible");
    return;
  }

  const [x, y] = latlonToPercent(point.lat, point.lon);
  marker.style.left = `${x}%`;
  marker.style.top = `${y}%`;
  marker.classList.add("visible");
}

function latlonToPercent(lat, lon) {
  const { top_left: topLeft, bottom_right: bottomRight } = state.config.map;
  const x = ((Number(lon) - topLeft.lon) / (bottomRight.lon - topLeft.lon)) * 100;
  const y = ((topLeft.lat - Number(lat)) / (topLeft.lat - bottomRight.lat)) * 100;
  return [clamp(x, 0, 100), clamp(y, 0, 100)];
}

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, Number(value)));
}

function titleCase(value) {
  return String(value)
    .replace(/_/g, " ")
    .replace(/\b\w/g, (char) => char.toUpperCase());
}

init().catch((error) => {
  elements.connectionState.textContent = error.message;
});
