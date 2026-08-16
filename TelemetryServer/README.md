# TelemetryServer setup (Windows)

```powershell
cd H:\Projects\RobotRL\TelemetryServer
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

For tests: `pip install -r requirements-dev.txt`.

## Run (live plot + CSV)

```powershell
.\venv\Scripts\Activate.ps1
python scripts/run_server.py --esp32-host 192.168.x.x --plot --record logs\session.csv
```

- `--esp32-host`: sends UDP `subscribe` so ESP32 learn-remote forwards STM32 UART data to your PC.
- With `--plot`: Qt window with **Graphs** (live) and **Gains** (edit all control params; needs `--esp32-host`).
- Omit `--plot` for headless record only.
- Omit `--record` for plot only.
- `--receive-buffer` requests the UDP socket receive buffer (default 1 MiB).
- Sanity limits are configurable with `--max-torque-nm`, `--max-pitch-rad`,
  `--max-wheel-turns-s`, and `--max-strategy-id`.
- `--verbose` separates UDP sequence gaps, envelope errors, TM CRC/length/version
  errors, STM32 error responses, decode errors, and physical-limit rejects.

## Remote control (mouse)

```powershell
python scripts/remote_control.py --esp32-host 192.168.x.x
```

Mouse stick pad only (no plots): up/down → speed, left/right → heading.
Sensitivity sliders set max speed (m/s) and max heading (deg) at pad edge.
Release mouse / STOP → zero commands. Do not run at the same time as `--plot`
(same UDP port 5000).

## Offline plot

```powershell
python scripts/plot_file.py logs\session.csv --channels pitch_rad,cmd_torque_nm,pitch_rate_rads
```

## Chain

```
STM32 UART4 (921600, BalanceFrame @ 500 Hz, HAL DMA TX/RX)
  -> ESP32Telemetry (TM parse/CRC + complete-frame queue)
  -> sequenced RT/UDP envelope :5000
  -> TelemetryServer (envelope CRC + TM CRC + plot/CSV)
```

The server still accepts legacy raw-TM UDP datagrams. New RT envelopes carry a
32-bit datagram sequence, complete-frame count, payload length, and CRC16 so
network loss is measured independently from STM32 `frame_number` gaps.

## Tests

```powershell
python -m pytest -q
```

Tests cover arbitrary TM fragmentation, malformed lengths/version/CRC, false
magic, parser recovery, configurable sanity checks, UDP draining, and RT
envelope truncation/corruption.

## STM32 firmware (UART4)

1. UART4 @ **921600** (CubeMX + `APP_TELEMETRY_BAUD`).
2. DMA **TX** (`DMA1_Stream2`) and **RX** (`DMA1_Stream3`) linked in CubeMX.
3. `APP_TELEMETRY_UART4_USE_DMA = 1` in `app_config.h` (default after HAL refactor).
4. `APP_TELEMETRY_WAIT_BRIDGE_READY = 1` — wait for ESP32 `READY\n` before streaming.

See [`ESP32Telemetry/README.md`](../ESP32Telemetry/README.md) for the boot handshake.

## Wiring

| STM32 | ESP32 |
|-------|-------|
| PD1 TX | GPIO16 RX |
| PD0 RX | GPIO17 TX |
| GND | GND |
