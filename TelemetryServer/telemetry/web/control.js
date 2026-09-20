"use strict";

const ACTION_PARAMS = new Set(["pos_reset", "heading_reset", "heading_inc", "heading_dec"]);
const GEAR_WHEEL_PER_MOTOR = 3.0 / 16.0;

let controlWs = null;
let loadedValues = {};
let wheelRadiusM = 0.04;
// FIFO of in-flight {resolve, reject} pairs. The server replies strictly in
// order on a single /ws/control connection, so the Nth reply belongs to the
// Nth outstanding request.
let pendingRequests = [];

function connectControl() {
  const url = `ws://${window.location.host}/ws/control`;
  controlWs = new WebSocket(url);
  controlWs.onopen = () => refreshGains();
  controlWs.onclose = () => {
    const err = new Error("control socket closed");
    const stranded = pendingRequests;
    pendingRequests = [];
    for (const pending of stranded) pending.reject(err);
    setTimeout(connectControl, 1000);
  };
  controlWs.onmessage = (event) => {
    const pending = pendingRequests.shift();
    if (!pending) return;
    let resp;
    try {
      resp = JSON.parse(event.data);
    } catch (err) {
      pending.reject(err);
      return;
    }
    if (resp.ok) pending.resolve(resp);
    else pending.reject(new Error(resp.error));
  };
}

function sendControl(action, extra) {
  return new Promise((resolve, reject) => {
    if (!controlWs || controlWs.readyState !== WebSocket.OPEN) {
      reject(new Error("control socket not connected"));
      return;
    }
    const pending = { resolve, reject };
    pendingRequests.push(pending);
    try {
      controlWs.send(JSON.stringify(Object.assign({ action }, extra)));
    } catch (err) {
      // Nothing was sent, so no reply will come: drop this slot or the FIFO
      // would desync and every later reply would go to the wrong request.
      pendingRequests = pendingRequests.filter((p) => p !== pending);
      reject(err);
    }
  });
}

function setGainsStatus(text, isError) {
  const el = document.getElementById("gains-status");
  el.textContent = text;
  el.className = isError ? "status-bad" : "";
}

const DRIVE_SLIDERS = new Set(["vel_ref_turns_s", "heading_ref_rad"]);

function appendGainRow(container, name, value, tooltip) {
  const row = document.createElement("div");
  row.className = "gains-row";
  const label = document.createElement("label");
  label.textContent = name;
  label.htmlFor = `gain-${name}`;
  if (tooltip) label.title = tooltip;
  const input = document.createElement("input");
  input.type = "text";
  input.id = `gain-${name}`;
  input.dataset.name = name;
  if (tooltip) input.title = tooltip;
  input.value =
    typeof value === "number" && !Number.isInteger(value)
      ? value.toPrecision(6)
      : String(value);
  loadedValues[name] = input.value;
  row.appendChild(label);
  row.appendChild(input);
  container.appendChild(row);
}

function buildGainsForm(params, panels) {
  const form = document.getElementById("gains-form");
  form.innerHTML = "";
  loadedValues = {};
  const used = new Set();

  const shouldSkip = (name) =>
    ACTION_PARAMS.has(name) || DRIVE_SLIDERS.has(name) || !(name in params);

  const addFieldGrid = (parent, fields) => {
    const grid = document.createElement("div");
    grid.className = "gains-fields";
    let added = 0;
    for (const field of fields) {
      const name = typeof field === "string" ? field : field.name;
      const tooltip = typeof field === "string" ? "" : field.tooltip || "";
      if (shouldSkip(name) || used.has(name)) continue;
      appendGainRow(grid, name, params[name], tooltip);
      used.add(name);
      added += 1;
    }
    if (added) parent.appendChild(grid);
    return added;
  };

  const addPanel = (title, legend, fields, sections) => {
    const fieldset = document.createElement("fieldset");
    fieldset.className = "gains-panel";
    const legendEl = document.createElement("legend");
    legendEl.textContent = title;
    fieldset.appendChild(legendEl);
    if (legend) {
      const p = document.createElement("p");
      p.className = "gains-legend";
      p.textContent = legend;
      fieldset.appendChild(p);
    }
    let added = addFieldGrid(fieldset, fields || []);
    for (const section of sections || []) {
      const sub = document.createElement("div");
      sub.className = "gains-subsection";
      const heading = document.createElement("h3");
      heading.textContent = section.title || "";
      sub.appendChild(heading);
      if (section.legend) {
        const p = document.createElement("p");
        p.className = "gains-legend";
        p.textContent = section.legend;
        sub.appendChild(p);
      }
      added += addFieldGrid(sub, section.fields || []);
      fieldset.appendChild(sub);
    }
    if (!added) return;
    form.appendChild(fieldset);
  };

  if (Array.isArray(panels) && panels.length) {
    for (const panel of panels) {
      addPanel(panel.title, panel.legend, panel.fields || [], panel.sections || []);
    }
  }

  const leftovers = Object.keys(params)
    .filter((name) => !used.has(name) && !shouldSkip(name))
    .sort();
  if (leftovers.length) {
    addPanel("Autres", "", leftovers);
  }
}

async function refreshGains() {
  try {
    const resp = await sendControl("get_params");
    buildGainsForm(resp.params, resp.panels);
    if (typeof resp.params.wheel_radius_m === "number" && resp.params.wheel_radius_m > 1e-6) {
      wheelRadiusM = resp.params.wheel_radius_m;
    }
    updateDriveLabels();
    setGainsStatus(`Loaded snapshot version=${resp.version}`, false);
  } catch (err) {
    setGainsStatus(`Refresh failed: ${err.message}`, true);
  }
}

async function applyGains(changedOnly) {
  const inputs = document.querySelectorAll("#gains-form input");
  let applied = 0;
  for (const input of inputs) {
    const name = input.dataset.name;
    const text = input.value.trim();
    if (!text) continue;
    if (changedOnly && loadedValues[name] === text) continue;
    const value = Number(text);
    if (Number.isNaN(value)) {
      setGainsStatus(`Invalid number for ${name}: ${text}`, true);
      return false;
    }
    try {
      const resp = await sendControl("set_param", { name, value });
      input.value = String(resp.applied);
      loadedValues[name] = input.value;
      applied += 1;
    } catch (err) {
      setGainsStatus(`Apply stopped after ${applied}: ${err.message}`, true);
      return false;
    }
  }
  setGainsStatus(applied ? `Applied ${applied} param(s).` : "Nothing to apply.", false);
  return true;
}

async function pulseAction(action) {
  try {
    await sendControl(action);
    setGainsStatus(`${action} pulsed.`, false);
  } catch (err) {
    setGainsStatus(`${action} failed: ${err.message}`, true);
  }
}

document.getElementById("gains-refresh").addEventListener("click", refreshGains);
document.getElementById("gains-apply-changed").addEventListener("click", () => applyGains(true));
document.getElementById("gains-apply-all").addEventListener("click", () => applyGains(false));
document.getElementById("gains-pos-reset").addEventListener("click", () => pulseAction("pos_reset"));
document
  .getElementById("gains-heading-reset")
  .addEventListener("click", () => pulseAction("heading_reset"));

const presetOverlay = document.getElementById("preset-overlay");
const presetTitle = document.getElementById("preset-overlay-title");
const presetList = document.getElementById("preset-list");
const presetEmpty = document.getElementById("preset-empty");
const presetCreateRow = document.getElementById("preset-create-row");
const presetNameInput = document.getElementById("preset-name");
let presetMode = "save";

function collectFormValues() {
  const values = {};
  for (const input of document.querySelectorAll("#gains-form input")) {
    const name = input.dataset.name;
    const text = input.value.trim();
    if (!text) continue;
    const value = Number(text);
    if (Number.isNaN(value)) {
      throw new Error(`Invalid number for ${name}: ${text}`);
    }
    values[name] = value;
  }
  return values;
}

function closePresetOverlay() {
  presetOverlay.hidden = true;
}

async function openPresetOverlay(mode) {
  presetMode = mode;
  presetTitle.textContent = mode === "save" ? "Save preset" : "Load preset";
  presetCreateRow.hidden = mode !== "save";
  presetNameInput.value = "";
  presetList.innerHTML = "";
  presetEmpty.hidden = true;
  try {
    const resp = await sendControl("list_presets");
    const names = resp.presets || [];
    if (!names.length) {
      presetEmpty.hidden = false;
    }
    for (const name of names) {
      const li = document.createElement("li");
      const btn = document.createElement("button");
      btn.type = "button";
      btn.textContent = mode === "save" ? `Overwrite “${name}”` : name;
      btn.addEventListener("click", () => {
        if (mode === "save") savePreset(name, true);
        else loadPreset(name);
      });
      li.appendChild(btn);
      presetList.appendChild(li);
    }
    presetOverlay.hidden = false;
    if (mode === "save") presetNameInput.focus();
  } catch (err) {
    setGainsStatus(`Presets failed: ${err.message}`, true);
  }
}

async function savePreset(name, overwrite) {
  const trimmed = name.trim();
  if (!trimmed) {
    setGainsStatus("Enter a preset name.", true);
    return;
  }
  let values;
  try {
    values = collectFormValues();
  } catch (err) {
    setGainsStatus(err.message, true);
    return;
  }
  if (!Object.keys(values).length) {
    setGainsStatus("Refresh gains before saving a preset.", true);
    return;
  }
  try {
    const resp = await sendControl("save_preset", {
      name: trimmed,
      values,
      overwrite: Boolean(overwrite),
    });
    closePresetOverlay();
    setGainsStatus(`Saved preset “${resp.name}”.`, false);
  } catch (err) {
    setGainsStatus(`Save failed: ${err.message}`, true);
  }
}

async function loadPreset(name) {
  try {
    const resp = await sendControl("load_preset", { name });
    for (const [key, value] of Object.entries(resp.params || {})) {
      const input = document.getElementById(`gain-${key}`);
      if (input) input.value = String(value);
    }
    closePresetOverlay();
    const applied = await applyGains(false);
    if (applied) setGainsStatus(`Loaded preset “${resp.name}”.`, false);
  } catch (err) {
    setGainsStatus(`Load failed: ${err.message}`, true);
  }
}

document.getElementById("preset-save").addEventListener("click", () => openPresetOverlay("save"));
document.getElementById("preset-load").addEventListener("click", () => openPresetOverlay("load"));
document.getElementById("preset-close").addEventListener("click", closePresetOverlay);
document.getElementById("preset-create").addEventListener("click", () => {
  savePreset(presetNameInput.value, false);
});
presetNameInput.addEventListener("keydown", (event) => {
  if (event.key === "Enter") {
    event.preventDefault();
    savePreset(presetNameInput.value, false);
  }
});
presetOverlay.addEventListener("click", (event) => {
  if (event.target === presetOverlay) closePresetOverlay();
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape" && !presetOverlay.hidden) closePresetOverlay();
});

function motorTurnsPerSToMps(turnsPerS) {
  return turnsPerS * GEAR_WHEEL_PER_MOTOR * 2 * Math.PI * wheelRadiusM;
}

function mpsToMotorTurnsPerS(mps) {
  const wheelMPerMotorTurn = GEAR_WHEEL_PER_MOTOR * 2 * Math.PI * wheelRadiusM;
  return wheelMPerMotorTurn <= 1e-12 ? 0 : mps / wheelMPerMotorTurn;
}

function formatSpeedLabel(mps) {
  const sign = mps >= 0 ? "+" : "";
  return `${sign}${mps.toFixed(2)} m/s (${sign}${mpsToMotorTurnsPerS(mps).toFixed(3)} motor turn/s)`;
}

function formatYawLabel(radS) {
  const degS = (radS * 180) / Math.PI;
  const sign = radS >= 0 ? "+" : "";
  return `yaw ${sign}${radS.toFixed(2)} rad/s (${sign}${degS.toFixed(0)} °/s)`;
}

const ACCEL_TIME_S = 0.7;
const DECEL_TIME_S = 0.22;
const DRIVE_SEND_MS = 50;
const DRIVE_IDLE_EPS = 1e-4;

const speedLabel = document.getElementById("speed-label");
const headingLabel = document.getElementById("heading-label");
const maxMpsInput = document.getElementById("drive-max-mps");
const maxYawInput = document.getElementById("drive-max-yaw");

const heldKeys = { up: false, down: false, left: false, right: false };
let cmdMps = 0;
let cmdYawRadS = 0;
let driveRaf = 0;
let lastDriveTs = 0;
let lastSendTs = 0;
let lastSentMps = null;
let lastSentYaw = null;
let sendInFlight = false;
let pendingForceSend = false;

function analogStep(value, heldDir, maxAbs, dt) {
  if (!(maxAbs > 0) || !(dt > 0)) return 0;
  const dir = heldDir > 0 ? 1 : heldDir < 0 ? -1 : 0;
  if (dir !== 0) {
    value += dir * (maxAbs / ACCEL_TIME_S) * dt;
    return Math.max(-maxAbs, Math.min(maxAbs, value));
  }
  const rate = (maxAbs / DECEL_TIME_S) * dt;
  if (Math.abs(value) <= rate) return 0;
  return value - Math.sign(value) * rate;
}

function readMaxMps() {
  const n = Number(maxMpsInput.value);
  return Number.isFinite(n) && n > 0 ? n : 0.5;
}

function readMaxYawRadS() {
  const n = Number(maxYawInput.value);
  const deg = Number.isFinite(n) && n > 0 ? n : 45;
  return (deg * Math.PI) / 180;
}

function updateDriveLabels() {
  speedLabel.textContent = formatSpeedLabel(cmdMps);
  headingLabel.textContent = formatYawLabel(cmdYawRadS);
}

function isTypingTarget(el) {
  if (!el) return false;
  const tag = el.tagName;
  return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || el.isContentEditable;
}

function driveBlocked() {
  return !presetOverlay.hidden || isTypingTarget(document.activeElement);
}

function anyDriveHeld() {
  return heldKeys.up || heldKeys.down || heldKeys.left || heldKeys.right;
}

function driveIdle() {
  return !anyDriveHeld() && Math.abs(cmdMps) <= DRIVE_IDLE_EPS && Math.abs(cmdYawRadS) <= DRIVE_IDLE_EPS;
}

async function flushDrive(force) {
  if (sendInFlight) {
    if (force) pendingForceSend = true;
    return;
  }
  const now = performance.now();
  if (!force && now - lastSendTs < DRIVE_SEND_MS) return;
  const velChanged = lastSentMps === null || Math.abs(cmdMps - lastSentMps) > DRIVE_IDLE_EPS;
  const yawChanged = lastSentYaw === null || Math.abs(cmdYawRadS - lastSentYaw) > DRIVE_IDLE_EPS;
  if (!force && !velChanged && !yawChanged) return;
  sendInFlight = true;
  lastSendTs = now;
  try {
    if (velChanged || force) {
      await sendControl("set_param", {
        name: "vel_ref_turns_s",
        value: mpsToMotorTurnsPerS(cmdMps),
      });
      lastSentMps = cmdMps;
    }
    if (yawChanged || force) {
      await sendControl("set_param", {
        name: "heading_ref_rad",
        value: cmdYawRadS,
      });
      lastSentYaw = cmdYawRadS;
    }
    updateDriveLabels();
  } catch (err) {
    speedLabel.textContent = `SET failed: ${err.message}`;
  } finally {
    sendInFlight = false;
    if (pendingForceSend) {
      pendingForceSend = false;
      flushDrive(true);
    }
  }
}

function tickDrive(ts) {
  const dt = lastDriveTs ? Math.min(0.05, (ts - lastDriveTs) / 1000) : 0.016;
  lastDriveTs = ts;
  const maxMps = readMaxMps();
  const maxYaw = readMaxYawRadS();
  const dirV = (heldKeys.up ? 1 : 0) + (heldKeys.down ? -1 : 0);
  const dirW = (heldKeys.right ? 1 : 0) + (heldKeys.left ? -1 : 0);
  cmdMps = analogStep(cmdMps, dirV, maxMps, dt);
  cmdYawRadS = analogStep(cmdYawRadS, dirW, maxYaw, dt);
  if (Math.abs(cmdMps) < DRIVE_IDLE_EPS) cmdMps = 0;
  if (Math.abs(cmdYawRadS) < DRIVE_IDLE_EPS) cmdYawRadS = 0;
  updateDriveLabels();
  flushDrive(false);
  if (driveIdle()) {
    flushDrive(true);
    driveRaf = 0;
    lastDriveTs = 0;
    return;
  }
  driveRaf = requestAnimationFrame(tickDrive);
}

function ensureDriveLoop() {
  if (!driveRaf) {
    lastDriveTs = 0;
    driveRaf = requestAnimationFrame(tickDrive);
  }
}

function setHeld(key, down) {
  if (heldKeys[key] === down) return;
  heldKeys[key] = down;
  if (down) ensureDriveLoop();
}

function keyToDrive(key) {
  if (key === "ArrowUp") return "up";
  if (key === "ArrowDown") return "down";
  if (key === "ArrowLeft") return "left";
  if (key === "ArrowRight") return "right";
  return "";
}

document.addEventListener("keydown", (event) => {
  const drive = keyToDrive(event.key);
  if (!drive || driveBlocked() || event.repeat) return;
  event.preventDefault();
  setHeld(drive, true);
});

document.addEventListener("keyup", (event) => {
  const drive = keyToDrive(event.key);
  if (!drive) return;
  event.preventDefault();
  setHeld(drive, false);
  ensureDriveLoop();
});

window.addEventListener("blur", () => {
  heldKeys.up = heldKeys.down = heldKeys.left = heldKeys.right = false;
  ensureDriveLoop();
});

document.getElementById("drive-zero").addEventListener("click", () => {
  heldKeys.up = heldKeys.down = heldKeys.left = heldKeys.right = false;
  cmdMps = 0;
  cmdYawRadS = 0;
  updateDriveLabels();
  flushDrive(true);
});

connectControl();
