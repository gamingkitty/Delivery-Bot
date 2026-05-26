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
  plannedRoute: document.getElementById("plannedRoute"),
  progressText: document.getElementById("progressText"),
  robotMarker: document.getElementById("robotMarker"),
  statePill: document.getElementById("statePill"),
  statusText: document.getElementById("statusText"),
  stopButton: document.getElementById("stopButton"),
  userPin: document.getElementById("userPin"),
  waypointText: document.getElementById("waypointText"),
};

const state = {
  config: null,
  debugUnlocked: false,
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
