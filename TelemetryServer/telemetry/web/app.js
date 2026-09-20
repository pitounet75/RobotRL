"use strict";

const WS_TELEMETRY_PATH = "/ws/telemetry";
const FRAME_BYTES_MIN = 56;
const FRAME_BYTES_V3 = 64;
const FRAME_BYTES_V4 = 68;
const FRAME_BYTES_V5 = 84;
const VBUS_EMA = 0.995;
const VBUS_PRESENT_V = 0.5;
const VBUS_WARN_V = 11.4;
const VBUS_CRIT_V = 11.0;
// Chart chrome colours; keep in sync with --muted / --border in style.css.
const AXIS_TEXT = "#9aa0a8";
const AXIS_GRID = "#2c3038";

// Time-window slider range (seconds). MAX_BUFFER_S must cover the slider's
// max so widening the window never needs data we've already discarded.
const MIN_VIEW_WINDOW_S = 2;
const MAX_VIEW_WINDOW_S = 30;
const DEFAULT_VIEW_WINDOW_S = 10;
const MAX_BUFFER_S = MAX_VIEW_WINDOW_S;

let viewWindowS = DEFAULT_VIEW_WINDOW_S;
let frozen = false;

function decodeBalanceFrame(buf) {
  const dv = new DataView(buf);
  return {
    frame_number: dv.getUint32(0, true),
    time_us: dv.getUint32(4, true),
    pitch_rad: dv.getFloat32(8, true),
    pitch_rate_rads: dv.getFloat32(12, true),
    vel_wheel_turns_s: dv.getFloat32(16, true),
    vel_wheel_l_turns_s: dv.getFloat32(20, true),
    vel_wheel_r_turns_s: dv.getFloat32(24, true),
    cmd_torque_nm: dv.getFloat32(28, true),
    cmd_torque_left_nm: dv.getFloat32(32, true),
    cmd_torque_right_nm: dv.getFloat32(36, true),
    u_meca_nm: dv.getFloat32(40, true),
    u_err_nm: dv.getFloat32(44, true),
    pitch_ref_rad: dv.getFloat32(48, true),
    imu_valid: dv.getUint8(52),
    estop: dv.getUint8(53),
    strategy_id: dv.getUint8(54),
    source_drop_count_mod256: dv.getUint8(55),
    vbus_l_v: buf.byteLength >= FRAME_BYTES_V3 ? dv.getFloat32(56, true) : 0,
    vbus_r_v: buf.byteLength >= FRAME_BYTES_V3 ? dv.getFloat32(60, true) : 0,
    wc_mode: buf.byteLength >= FRAME_BYTES_V4 ? dv.getUint8(64) : 0,
    sync_l: buf.byteLength >= FRAME_BYTES_V4 ? dv.getUint8(65) : 0,
    sync_r: buf.byteLength >= FRAME_BYTES_V4 ? dv.getUint8(66) : 0,
    // V5: order-2 fit beside the EMA above, to compare them live.
    vel_fit_l_turns_s: buf.byteLength >= FRAME_BYTES_V5 ? dv.getFloat32(68, true) : 0,
    vel_fit_r_turns_s: buf.byteLength >= FRAME_BYTES_V5 ? dv.getFloat32(72, true) : 0,
    acc_fit_l_turns_s2: buf.byteLength >= FRAME_BYTES_V5 ? dv.getFloat32(76, true) : 0,
    acc_fit_r_turns_s2: buf.byteLength >= FRAME_BYTES_V5 ? dv.getFloat32(80, true) : 0,
  };
}

const BUFFER_KEYS = [
  "pitch_rad", "pitch_deg", "pitch_rate",
  "cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_meca", "u_err",
  "vel_l", "vel_r", "vel_fit_l", "vel_fit_r",
  "acc_fit_l", "acc_fit_r",
  "estop", "imu_valid", "sync_l", "sync_r",
];

class ChannelBuffers {
  constructor(keys) {
    this.keys = keys;
    this.t = [];
    this.y = {};
    for (const k of keys) this.y[k] = [];
    this.t0 = null;
  }

  push(frame, tRecvSec) {
    if (this.t0 === null) this.t0 = tRecvSec;
    const t = tRecvSec - this.t0;
    this.t.push(t);
    this.y.pitch_rad.push(frame.pitch_rad);
    this.y.pitch_deg.push((frame.pitch_rad * 180) / Math.PI);
    this.y.pitch_rate.push(frame.pitch_rate_rads);
    this.y.cmd_torque.push(frame.cmd_torque_nm);
    this.y.cmd_torque_l.push(frame.cmd_torque_left_nm);
    this.y.cmd_torque_r.push(frame.cmd_torque_right_nm);
    this.y.u_meca.push(frame.u_meca_nm);
    this.y.u_err.push(frame.u_err_nm);
    this.y.vel_l.push(frame.vel_wheel_l_turns_s);
    this.y.vel_r.push(frame.vel_wheel_r_turns_s);
    this.y.vel_fit_l.push(frame.vel_fit_l_turns_s);
    this.y.vel_fit_r.push(frame.vel_fit_r_turns_s);
    this.y.acc_fit_l.push(frame.acc_fit_l_turns_s2);
    this.y.acc_fit_r.push(frame.acc_fit_r_turns_s2);
    this.y.estop.push(frame.estop);
    this.y.imu_valid.push(frame.imu_valid);
    this.y.sync_l.push(frame.sync_l ? 1 : 0);
    this.y.sync_r.push(frame.sync_r ? 1 : 0);
    // Evict anything older than MAX_BUFFER_S, so the buffer always covers
    // the widest window the slider can ask for.
    const cutoff = t - MAX_BUFFER_S;
    let dropCount = 0;
    while (dropCount < this.t.length && this.t[dropCount] < cutoff) dropCount++;
    if (dropCount > 0) {
      this.t.splice(0, dropCount);
      for (const k of this.keys) this.y[k].splice(0, dropCount);
    }
  }

  series(keys) {
    return [this.t, ...keys.map((k) => this.y[k])];
  }
}

const buffers = new ChannelBuffers(BUFFER_KEYS);

const CHART_HEIGHT = 220;

function makeChart(containerId, title, seriesKeys, labels, yRange) {
  const container = document.getElementById(containerId);
  const opts = {
    title,
    width: container.clientWidth,
    height: CHART_HEIGHT,
    series: [
      {},
      ...seriesKeys.map((k, i) => ({
        label: labels[i],
        stroke: `hsl(${(i * 67) % 360},70%,55%)`,
      })),
    ],
    // No `range` here: a static range array fights with setScale() below
    // (uPlot keeps reapplying it on every setData, freezing the window).
    // scheduleRedraw() is the sole authority over the x-domain via setScale.
    scales: {
      x: { time: false, auto: false },
      ...(yRange ? { y: { auto: false, range: yRange } } : {}),
    },
    // uPlot defaults to black axes/grid, which is invisible on the dark panel
    // background. These match --muted / --border in style.css.
    axes: [
      {
        label: "s",
        stroke: AXIS_TEXT,
        grid: { stroke: AXIS_GRID },
        ticks: { stroke: AXIS_GRID },
      },
      { stroke: AXIS_TEXT, grid: { stroke: AXIS_GRID }, ticks: { stroke: AXIS_GRID } },
    ],
  };
  const plot = new uPlot(opts, buffers.series(seriesKeys), container);
  // Set the initial window explicitly now that there's no static `range`
  // option to do it for us.
  plot.setScale("x", { min: 0, max: viewWindowS });
  return { plot, seriesKeys, container };
}

const charts = [
  makeChart("chart-pitch", "Pitch", ["pitch_rad", "pitch_rate"], ["pitch_rad", "pitch_rate"]),
  makeChart(
    "chart-torque",
    "Torque (Nm)",
    ["cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_meca", "u_err"],
    ["cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_meca", "u_err"]
  ),
  makeChart(
    "chart-velocity",
    "Wheel velocity (turn/s) — EMA vs order-2 fit",
    ["vel_l", "vel_r", "vel_fit_l", "vel_fit_r"],
    ["vel_l", "vel_r", "vel_fit_l", "vel_fit_r"]
  ),
  makeChart(
    "chart-accel",
    "Wheel acceleration from the fit (turn/s²)",
    ["acc_fit_l", "acc_fit_r"],
    ["acc_fit_l", "acc_fit_r"]
  ),
  makeChart("chart-flags", "Flags", ["estop", "imu_valid", "sync_l", "sync_r"], ["estop", "imu_valid", "sync_l", "sync_r"], [-0.1, 1.2]),
];

function resizeCharts() {
  for (const { plot, container } of charts) {
    const width = container.clientWidth;
    if (width > 0) {
      plot.setSize({ width, height: CHART_HEIGHT });
    }
  }
}

window.addEventListener("resize", resizeCharts);

let redrawScheduled = false;

function scheduleRedraw() {
  if (redrawScheduled) return;
  redrawScheduled = true;
  requestAnimationFrame(() => {
    redrawScheduled = false;
    const latestT = buffers.t.length ? buffers.t[buffers.t.length - 1] : 0;
    const maxT = Math.max(latestT, viewWindowS);
    const minT = maxT - viewWindowS;
    for (const { plot, seriesKeys } of charts) {
      plot.setData(buffers.series(seriesKeys));
      plot.setScale("x", { min: minT, max: maxT });
    }
  });
}

const vbusLabel = document.getElementById("vbus-label");
const vbusLamp = document.getElementById("vbus-lamp");
let vbusFiltL = null;
let vbusFiltR = null;

function vbusAlertLevel(leftV, rightV) {
  const rails = [leftV, rightV].filter((v) => v > VBUS_PRESENT_V);
  if (!rails.length) return "off";
  const lowest = Math.min(...rails);
  if (lowest < VBUS_CRIT_V) return "crit";
  if (lowest < VBUS_WARN_V) return "warn";
  return "ok";
}

function setVbusLamp(level) {
  if (!vbusLamp) return;
  vbusLamp.className = level === "warn" || level === "crit" ? `vbus-lamp ${level}` : "vbus-lamp";
}

const syncLampL = document.getElementById("sync-l-lamp");
const syncLampR = document.getElementById("sync-r-lamp");

function setSyncLamp(el, on) {
  if (!el) return;
  el.className = on ? "sync-lamp sync" : "sync-lamp normal";
}

function updateSyncLamps(frame) {
  setSyncLamp(syncLampL, frame.sync_l);
  setSyncLamp(syncLampR, frame.sync_r);
}

function updateVbusLabel(frame) {
  if (!vbusLabel) return;
  const rawL = frame.vbus_l_v;
  const rawR = frame.vbus_r_v;
  if (!(rawL > VBUS_PRESENT_V) && !(rawR > VBUS_PRESENT_V)) {
    vbusLabel.textContent = "Vbus L — V · R — V";
    setVbusLamp("off");
    return;
  }
  vbusFiltL = vbusFiltL == null ? rawL : VBUS_EMA * vbusFiltL + (1 - VBUS_EMA) * rawL;
  vbusFiltR = vbusFiltR == null ? rawR : VBUS_EMA * vbusFiltR + (1 - VBUS_EMA) * rawR;
  vbusLabel.textContent = `Vbus L ${vbusFiltL.toFixed(2)} V · R ${vbusFiltR.toFixed(2)} V`;
  setVbusLamp(vbusAlertLevel(vbusFiltL, vbusFiltR));
}

function setStatus(connected) {
  const el = document.getElementById("connection-status");
  if (!el) return;
  el.textContent = connected ? "connected" : "disconnected";
  el.className = connected ? "status-ok" : "status-bad";
}

function connectTelemetry() {
  const url = `ws://${window.location.host}${WS_TELEMETRY_PATH}`;
  const ws = new WebSocket(url);
  ws.binaryType = "arraybuffer";
  ws.onopen = () => setStatus(true);
  ws.onclose = () => {
    setStatus(false);
    setTimeout(connectTelemetry, 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (event) => {
    if (frozen) return;
    if (event.data.byteLength < FRAME_BYTES_MIN) return;
    const frame = decodeBalanceFrame(event.data);
    buffers.push(frame, performance.now() / 1000);
    updateVbusLabel(frame);
    updateSyncLamps(frame);
    scheduleRedraw();
  };
}

const windowSlider = document.getElementById("window-slider");
const windowLabel = document.getElementById("window-label");
windowSlider.addEventListener("input", () => {
  viewWindowS = Math.max(MIN_VIEW_WINDOW_S, Math.min(MAX_VIEW_WINDOW_S, Number(windowSlider.value)));
  windowLabel.textContent = `${viewWindowS} s`;
  scheduleRedraw();
});

const freezeToggle = document.getElementById("freeze-toggle");
freezeToggle.addEventListener("click", () => {
  frozen = !frozen;
  freezeToggle.textContent = frozen ? "Resume" : "Freeze";
  freezeToggle.classList.toggle("status-bad", frozen);
});

connectTelemetry();
