# Telemetry web UI — design spec

Date: 2026-09-12
Status: proposed

## Problem

`TelemetryServer`'s live view (`--plot`) renders ~12 signal channels across
4 stacked subplots with Matplotlib, redrawn via `FuncAnimation` every 50ms.
Every redraw does a full `relim()`/`autoscale_view()` over up to 5000
buffered points per channel (10s history at the STM32's native 500Hz
`BalanceFrame` rate). Matplotlib was not built for this access pattern —
it's a static-plot library retrofitted for animation — and the UI has no
real visual design pass behind it (default Matplotlib chrome, ad hoc Qt
widget layout).

`pyqtgraph` is already listed in `requirements.txt` but unused anywhere in
the codebase, suggesting a prior, unfinished intent to fix this the
Qt-native way.

## Decision

Build a browser-based telemetry dashboard as a new, additive run mode
(`--web`), living alongside the existing `--plot` (PyQt5/Matplotlib) mode.
Once the web UI is validated against real hardware traffic, the Qt/
Matplotlib path is retired in a follow-up cleanup (out of scope for this
spec — tracked as a Non-Goal below).

Rejected alternative: swap Matplotlib for `pyqtgraph` inside the existing
Qt shell. This would fix the performance problem with much less new
surface area, but caps visual polish at Qt stylesheet (QSS) styling, and
the user specifically wants to use the `superdesign` visual-design tool,
which only applies to an HTML/CSS surface. Given the browser path was
independently confirmed to handle the real data rate (100Hz+, a dozen
channels) comfortably with a canvas-based charting library, the extra
build cost is justified by the polish ceiling it unlocks.

## Goals

- Live multi-channel telemetry view in a browser, at the STM32's native
  frame rate, with no visible lag or dropped-frame stutter under normal
  operation.
- Gains editing (GET/SET all control params) equivalent to today's
  **Gains** tab.
- Speed/heading control equivalent to today's sliders.
- Reuse all existing decode/RPC/CSV-recording logic unchanged.
- A visual design worth calling "polished" — delegated to the
  `superdesign` skill once the functional skeleton exists.

## Non-goals

- Retiring `app_window.py` / `plot_live.py` / `gains_panel.py` / the
  PyQt5 and Matplotlib dependencies. That happens in a later cleanup once
  the web UI has been run against real hardware and is preferred in
  practice.
- Remote access beyond the local network (no auth, no TLS, no exposure
  hardening) — this is a LAN-local diagnostics tool, same trust model as
  today's UDP telemetry link.
- Changing the wire protocol between ESP32 and `TelemetryServer`, or the
  `BalanceFrame`/`ControlParamsSnapshot` encodings.
- Multi-client conflict resolution for control (SET). If two browser tabs
  both edit gains, last-write-wins, same as today (nothing currently
  prevents two Qt instances from doing the same).

## Architecture

### Reused unchanged

`udp_receiver.py`, `protocol.py`, `udp_envelope.py`, `balance_frame.py`,
`ctrl_params.py`, `rpc_mux.py`, `recorder.py`. None of these are aware a
web server exists.

### New: `telemetry/web_server.py`

An `asyncio`-based server (library: `websockets`, added to
`requirements.txt`) that owns:

1. **Static file serving** for `telemetry/web/` (`index.html`, `app.js`,
   `style.css`; a vendored copy of `uPlot` — no build step, matching this
   repo's "just run a script" simplicity).
2. **`/ws/telemetry`** — a broadcast WebSocket. Every decoded
   `BalanceFrame` accepted by `run_server.py`'s `rx_loop` is pushed here
   in addition to being handed to the CSV recorder and (optionally) the
   Qt plotter — same decode, three consumers. Frames are sent as a
   compact binary payload (a `struct.pack` mirroring
   `BALANCE_FRAME_STRUCT_V2`) rather than JSON, to keep parse cost
   negligible on the browser side at high rate. The server does **not**
   throttle; the browser decides render cadence (see Frontend).
3. **`/ws/control`** — a request/response WebSocket (JSON) wrapping
   `SharedRpcClient`: `get_params`, `set_param(name, value)`,
   `pos_reset`, `heading_reset`. One coroutine per connection issues
   these as blocking calls via `loop.run_in_executor` (the existing
   `SharedRpcClient._transact` blocks on a `queue.Queue`).

### Threading model

`rx_loop` in `scripts/run_server.py` stays a plain background thread
(unchanged). It gains one more branch: after a `BalanceFrame` passes
sanity checks, it calls
`loop.call_soon_threadsafe(broadcast_queue.put_nowait, bf)` where
`broadcast_queue` is an `asyncio.Queue` owned by the web server's event
loop. A single coroutine drains that queue and writes to all connected
`/ws/telemetry` clients.

The web server's `asyncio` event loop runs on the main thread in place of
Qt's `app.exec_()` when `--web` is passed (mutually exclusive with
`--plot` at the CLI level — pick one UI per process, run two processes if
you want both).

### New CLI

```
scripts/run_server.py --esp32-host <ip> --web [--web-port 8765]
```

Mirrors `--plot`'s relationship to `--esp32-host` (Gains/control panel
needs the ESP32 host to talk RPC; without it, telemetry-only streaming
still works, matching today's `--plot` behavior with no `--esp32-host`).

### Frontend (`telemetry/web/`)

Plain HTML/JS, no build step, `uPlot` for charts:

- Four chart panels mirroring today's grouping (pitch rad/deg + rate,
  torque, wheel velocity, flags), each a `uPlot` instance.
- Incoming binary WS messages are decoded into per-channel
  `Float32Array` ring buffers immediately on receipt (matches ingest
  rate); a `requestAnimationFrame` loop redraws all four charts from the
  current buffer contents, decoupling ingest from paint rate. This is the
  same design used by Grafana-class dashboards.
- A **Gains** panel: grouped form fields mirroring
  `gains_panel.py`'s param groups and Refresh / Apply changed / Apply all
  / `pos_reset` / `heading_reset` buttons, calling `/ws/control`.
- Speed/heading sliders mirroring `app_window.py`'s conversion math
  (motor-turn/s ↔ m/s, degrees ↔ rad) and 80ms SET-debounce.
- Visual design (layout, color, typography, dark mode) is drafted with
  the `superdesign` skill once this functional skeleton is wired up and
  talking to live data — designing against real, moving data rather than
  a mockup avoids polishing a layout that turns out wrong once real
  numbers/label widths are in it.

### Error handling

- WebSocket disconnect: browser shows a "disconnected" badge and
  auto-retries with backoff; no data loss server-side since recording/
  sanity-checking happen independently of whether a browser is attached
  (matches today: Qt plotter being closed doesn't stop `--record`).
  socket.
- Control RPC timeout/error: surfaced as an inline status message in the
  Gains panel / slider label, same UX as today's Qt status line and
  `SET failed: {exc}` labels.
- Malformed/short binary telemetry frame: dropped client-side, logged to
  the browser console; does not stop the render loop (mirrors
  `plot_live.py`'s existing tolerance for length mismatches).

## Testing

- New `tests/test_web_server.py`: pure-Python, no browser — asserts the
  binary frame encoding round-trips through the same struct layout as
  `BalanceFrame.decode`, and that `/ws/control` JSON request/response
  shapes match `SharedRpcClient`'s existing return tuples.
- Existing protocol/decoder/receiver tests are untouched (no changes to
  the reused modules).
- No automated browser/E2E test in this repo's `pytest` setup; manual
  verification against real hardware (or a synthetic `BalanceFrame`
  generator, if one becomes convenient) is the acceptance check before
  retiring the Qt path.

## Follow-up (separate task, not this spec)

Once the web UI has been run against real hardware and is preferred in
practice: remove `app_window.py`, `plot_live.py`, `gains_panel.py`,
`mpl_backend.py`, the `--plot` CLI flag, and the `PyQt5`/`matplotlib`/
`pyqtgraph` entries from `requirements.txt`.
