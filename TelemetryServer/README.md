# TelemetryServer setup (Windows)

```powershell
cd H:\Projects\RobotRL\TelemetryServer
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Run (live plot + CSV)

```powershell
.\venv\Scripts\Activate.ps1
python scripts/run_server.py --esp32-host 192.168.x.x --plot --record logs\session.csv
```

- `--esp32-host`: sends UDP `subscribe` so ESP32 learn-remote forwards STM32 UART data to your PC.
- Omit `--plot` for headless record only.
- Omit `--record` for plot only.

## Offline plot

```powershell
python scripts/plot_file.py logs\session.csv --channels pitch_rad,cmd_torque_nm,pitch_rate_rads
```

## Chain

```
STM32 UART4 (115200, BalanceFrame @ 100 Hz)
  -> ESP32Telemetry (transparent)
  -> UDP :5000
  -> TelemetryServer (CRC validate + plot/CSV)
```

## STM32 CubeMX (your step)

1. UART4: keep 115200.
2. Add **DMA TX** for UART4 (normal mode, interrupt enabled).
3. Set `APP_TELEMETRY_UART4_USE_DMA` to `1` in `app_config.h` after rebuild.

Until DMA is enabled, firmware uses blocking UART TX (OK for bring-up @ 115200).

## Wiring

| STM32 | ESP32 |
|-------|-------|
| PD1 TX | GPIO16 RX |
| PD0 RX | GPIO17 TX |
| GND | GND |
