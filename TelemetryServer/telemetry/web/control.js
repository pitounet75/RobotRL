"use strict";

const ACTION_PARAMS = new Set(["pos_reset", "heading_reset", "heading_inc", "heading_dec"]);
const GEAR_WHEEL_PER_MOTOR = 3.0 / 16.0;

let controlWs = null;
let loadedValues = {};
let wheelRadiusM = 0.04;
let speedSendTimer = null;
let headingSendTimer = null;

function connectControl() {
  const url = `ws://${window.location.host}/ws/control`;
  controlWs = new WebSocket(url);
  controlWs.onopen = () => refreshGains();
  controlWs.onclose = () => setTimeout(connectControl, 1000);
}

function sendControl(action, extra) {
  return new Promise((resolve, reject) => {
    if (!controlWs || controlWs.readyState !== WebSocket.OPEN) {
      reject(new Error("control socket not connected"));
      return;
    }
    const handler = (event) => {
      controlWs.removeEventListener("message", handler);
      const resp = JSON.parse(event.data);
      if (resp.ok) resolve(resp);
      else reject(new Error(resp.error));
    };
    controlWs.addEventListener("message", handler);
    controlWs.send(JSON.stringify(Object.assign({ action }, extra)));
  });
}

function setGainsStatus(text, isError) {
  const el = document.getElementById("gains-status");
  el.textContent = text;
  el.className = isError ? "status-bad" : "";
}

function buildGainsForm(params) {
  const form = document.getElementById("gains-form");
  form.innerHTML = "";
  loadedValues = {};
  const names = Object.keys(params).sort();
  for (const name of names) {
    if (ACTION_PARAMS.has(name)) continue;
    const row = document.createElement("div");
    row.className = "gains-row";
    const label = document.createElement("label");
    label.textContent = name;
    label.htmlFor = `gain-${name}`;
    const input = document.createElement("input");
    input.type = "text";
    input.id = `gain-${name}`;
    input.dataset.name = name;
    const val = params[name];
    input.value =
      typeof val === "number" && !Number.isInteger(val) ? val.toPrecision(6) : String(val);
    loadedValues[name] = input.value;
    row.appendChild(label);
    row.appendChild(input);
    form.appendChild(row);
  }
}

async function refreshGains() {
  try {
    const resp = await sendControl("get_params");
    buildGainsForm(resp.params);
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
      return;
    }
    try {
      const resp = await sendControl("set_param", { name, value });
      input.value = String(resp.applied);
      loadedValues[name] = input.value;
      applied += 1;
    } catch (err) {
      setGainsStatus(`Apply stopped after ${applied}: ${err.message}`, true);
      return;
    }
  }
  setGainsStatus(applied ? `Applied ${applied} param(s).` : "Nothing to apply.", false);
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

function motorTurnsPerSToMps(turnsPerS) {
  return turnsPerS * GEAR_WHEEL_PER_MOTOR * 2 * Math.PI * wheelRadiusM;
}

function mpsToMotorTurnsPerS(mps) {
  const wheelMPerMotorTurn = GEAR_WHEEL_PER_MOTOR * 2 * Math.PI * wheelRadiusM;
  return wheelMPerMotorTurn <= 1e-12 ? 0 : mps / wheelMPerMotorTurn;
}

function formatSpeedLabel(mps) {
  return `${mps.toFixed(2)} m/s (${mpsToMotorTurnsPerS(mps).toFixed(3)} motor turn/s)`;
}

function formatHeadingLabel(deg) {
  return `${deg.toFixed(0)}° (${((deg * Math.PI) / 180).toFixed(3)} rad)`;
}

const speedSlider = document.getElementById("speed-slider");
const speedLabel = document.getElementById("speed-label");
speedSlider.addEventListener("input", () => {
  const mmS = Number(speedSlider.value);
  speedLabel.textContent = formatSpeedLabel(mmS / 1000);
  clearTimeout(speedSendTimer);
  speedSendTimer = setTimeout(() => flushSpeed(mmS), 80);
});
document.getElementById("speed-zero").addEventListener("click", () => {
  speedSlider.value = "0";
  speedSlider.dispatchEvent(new Event("input"));
});

async function flushSpeed(mmS) {
  try {
    const turnsPerS = mpsToMotorTurnsPerS(mmS / 1000);
    const resp = await sendControl("set_param", { name: "vel_ref_turns_s", value: turnsPerS });
    speedLabel.textContent = formatSpeedLabel(motorTurnsPerSToMps(resp.applied));
  } catch (err) {
    speedLabel.textContent = `SET failed: ${err.message}`;
  }
}

const headingSlider = document.getElementById("heading-slider");
const headingLabel = document.getElementById("heading-label");
headingSlider.addEventListener("input", () => {
  const deg = Number(headingSlider.value);
  headingLabel.textContent = formatHeadingLabel(deg);
  clearTimeout(headingSendTimer);
  headingSendTimer = setTimeout(() => flushHeading(deg), 80);
});
document.getElementById("heading-zero").addEventListener("click", () => {
  headingSlider.value = "0";
  headingSlider.dispatchEvent(new Event("input"));
});

async function flushHeading(deg) {
  try {
    const rad = (deg * Math.PI) / 180;
    const resp = await sendControl("set_param", { name: "heading_ref_rad", value: rad });
    const appliedDeg = (resp.applied * 180) / Math.PI;
    headingLabel.textContent = formatHeadingLabel(appliedDeg);
  } catch (err) {
    headingLabel.textContent = `SET failed: ${err.message}`;
  }
}

connectControl();
