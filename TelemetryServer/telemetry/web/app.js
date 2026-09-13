"use strict";

const MAX_POINTS = 5000;
const WS_TELEMETRY_PATH = "/ws/telemetry";
const FRAME_BYTES = 56;

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
    this.t.push(tRecvSec - this.t0);
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
    if (this.t.length > MAX_POINTS) {
      this.t.shift();
      for (const k of this.keys) this.y[k].shift();
    }
  }

  series(keys) {
    return [this.t, ...keys.map((k) => this.y[k])];
  }
}

const buffers = new ChannelBuffers(BUFFER_KEYS);

function makeChart(containerId, title, seriesKeys, labels) {
  const opts = {
    title,
    width: 600,
    height: 220,
    series: [
      {},
      ...seriesKeys.map((k, i) => ({
        label: labels[i],
        stroke: `hsl(${(i * 67) % 360},70%,55%)`,
      })),
    ],
    scales: { x: { time: false } },
    axes: [{ label: "s" }, {}],
  };
  const plot = new uPlot(opts, buffers.series(seriesKeys), document.getElementById(containerId));
  return { plot, seriesKeys };
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

let redrawScheduled = false;

function scheduleRedraw() {
  if (redrawScheduled) return;
  redrawScheduled = true;
  requestAnimationFrame(() => {
    redrawScheduled = false;
    for (const { plot, seriesKeys } of charts) {
      plot.setData(buffers.series(seriesKeys));
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
    if (event.data.byteLength !== FRAME_BYTES) return;
    const frame = decodeBalanceFrame(event.data);
    buffers.push(frame, performance.now() / 1000);
    scheduleRedraw();
  };
}

connectTelemetry();
