# Balance telemetry bring-up (STM32 + ESP32 + PC)

## Firmware

- `task_telemetry` pushes **BalanceFrame** (type `0x0100`) @ **100 Hz** on **UART4**.
- Always streaming (even if no PC connected).
- Fields: pitch, pitch_rate, wheel velocities, cmd/u_ff/u_fb torque, estop, strategy.

## Quick test

1. Flash STM32 with telemetry build.
2. Flash `ESP32Telemetry` (edit WiFi in `include/config.h`).
3. Setup Python venv (`TelemetryServer/README.md`).
4. Run:
   ```powershell
   python scripts/run_server.py --esp32-host <ESP32_IP> --plot --record logs\test.csv
   ```

## Question 7 (old bridge)

`ESP32/serial_udp_bridge/` = earlier prototype. Use **`ESP32Telemetry/`** for this project; same transparent UDP idea.
