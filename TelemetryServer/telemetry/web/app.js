"use strict";

const WS_TELEMETRY_PATH = "/ws/telemetry";
const FRAME_BYTES = 56;
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
    u_ff_nm: dv.getFloat32(40, true),
    u_fb_nm: dv.getFloat32(44, true),
    pitch_ref_rad: dv.getFloat32(48, true),
    imu_valid: dv.getUint8(52),
    estop: dv.getUint8(53),
    strategy_id: dv.getUint8(54),
    source_drop_count_mod256: dv.getUint8(55),
  };
}

const BUFFER_KEYS = [
  "pitch_rad", "pitch_deg", "pitch_rate",
  "cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_ff", "u_fb",
  "vel_l", "vel_r", "estop", "imu_valid",
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
    this.y.u_ff.push(frame.u_ff_nm);
    this.y.u_fb.push(frame.u_fb_nm);
    this.y.vel_l.push(frame.vel_wheel_l_turns_s);
    this.y.vel_r.push(frame.vel_wheel_r_turns_s);
    this.y.estop.push(frame.estop);
    this.y.imu_valid.push(frame.imu_valid);
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

function makeChart(containerId, title, seriesKeys, labels) {
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
    scales: { x: { time: false, auto: false } },
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
    ["cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_ff", "u_fb"],
    ["cmd_torque", "cmd_torque_l", "cmd_torque_r", "u_ff", "u_fb"]
  ),
  makeChart("chart-velocity", "Wheel velocity (turn/s)", ["vel_l", "vel_r"], ["vel_l", "vel_r"]),
  makeChart("chart-flags", "Flags", ["estop", "imu_valid"], ["estop", "imu_valid"]),
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
    if (event.data.byteLength !== FRAME_BYTES) return;
    const frame = decodeBalanceFrame(event.data);
    buffers.push(frame, performance.now() / 1000);
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
