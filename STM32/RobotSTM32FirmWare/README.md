# RobotSTM32FirmWare

Real-time firmware for a **self-balancing robot**, running on an **STM32H743VIT6** (LQFP100). The MCU is the low-level controller: motor drives over CAN, IMU over SPI, telemetry to an ESP32, and (planned) supervisory commands from an **NVIDIA Jetson Orin Nano**.

## Role in the system

| Layer | Device | Role |
|-------|--------|------|
| **Real-time control** | STM32H743 (this project) | CAN → motors, SPI → IMU, safety, fast loops |
| **Telemetry / wireless** | ESP32 | UART binary telemetry (Wi‑Fi bridge planned) |
| **Planning / AI** (future) | Jetson Orin Nano | Commands and parameters over UART |
| **Motor drives** | 2× MKS XDrive Mini | ODrive-compatible, 1 channel each, CAN Simple |
| **IMU** | TDK **ICM-45686** | 6-axis, SPI1 (`task_imu` @ 1 kHz) |
| **PCB** | KiCad | [`../../Board/STM32GenericBoard`](../../Board/README.md) — H743 + ICM-45686 (WIP) |

The Jetson is **not** on the ODrive CAN bus. The STM32 owns CAN timing and motor I/O; the Jetson sends setpoints and receives state over **USART2**.

---

## Hardware summary

- **MCU:** STM32H743VIT6 @ 480 MHz (typ.), HSE + PLL
- **Motors:** 2× [MKS XDrive Mini](https://github.com/makerbase-mks) (ODrive firmware, CAN Simple)
- **IMU:** ICM-45686
- **Transceiver:** SN65HVD230 (or compatible) on **FDCAN2**
- **Logic levels:** 3.3 V on all UART/SPI/CAN interfaces

Reference ODrive saved config (node IDs, 250 kbit/s, anticogging):

- `ODrive/OdriveTool/Commands/Configs/2836_anticogging_calibrated.txt`
- USB workflow: `ODrive/OdriveTool/Commands/velocity_mode.txt`

---

## Communication map

```
                    ┌─────────────────┐
                    │ Jetson Orin Nano│  (future supervisor)
                    └────────┬────────┘
                             │ USART2  (commands / parameters)
                    ┌────────▼────────┐
                    │   STM32H743     │
                    │ RobotSTM32FirmWare│
                    └──┬───┬───┬───┬──┘
           USART1      │   │   │      USART3 (debug printf)
              │         │   │   │           │
         ┌────▼────┐    │   │   │      USB-UART adapter
         │  ESP32  │    │   │   │      (development)
         │Telemetry│    │   │   │
         └─────────┘    │   │   │
                         │   │   └── SPI1 ──► ICM-45686
                         │   │
                         └── FDCAN2 ──► CAN bus ──► XDrive node 0 & 1
```

### Peripheral assignment (CubeMX / PCB)

| Interface | Peripheral | Pins (MCU) | Partner | Status |
|-----------|------------|------------|---------|--------|
| **Motor CAN** | **FDCAN2** | PB12 RX, PB13 TX | 2× XDrive Mini | **Active** (`ODRIVE_CAN_HANDLE` = `hfdcan2`) |
| Motor CAN (alt.) | FDCAN1 | PA11 RX, PA12 TX | — | Configured in Cube; **not used** by application code |
| **IMU** | **SPI1** | PA5 SCK, PA6 MISO, PA7 MOSI, **PA4 CS** | ICM-45686 | `task_imu` + `imu_async` (see `APP_IMU_*` in `app_config.h`) |
| SPI2 | SPI2 | PA9 SCK, PC2 MISO, PC1 MOSI | — | Reserved / second SPI peripheral |
| **Telemetry** | **USART1** | PB14 TX, PB15 RX | ESP32 | 115200 8N1 (bring-up) |
| **Jetson** | **USART2** | PA2 TX, PA3 RX | Orin Nano 40-pin UART | 115200 8N1 (bring-up) |
| **Debug** | **USART3** | PB10 TX, PB11 RX | USB-serial adapter | 115200 8N1; target for `printf` |

**Wiring rule (UART):** MCU **TX** → partner **RX**, common **GND**. All UARTs: **8N1**, no hardware flow control initially.

### USART usage (project convention)

| USART | Handle | Baud (start → target) | Purpose |
|-------|--------|------------------------|---------|
| **USART1** | `huart1` | 115200 → **921600** | **ESP32 telemetry** — framed binary protocol ([`../Telemetry/README.md`](../Telemetry/README.md)) |
| **USART2** | `huart2` | 115200 → **921600** | **Jetson Orin Nano** — commands in, robot state / parameters out |
| **USART3** | `huart3` | **115200** | **Developer debug** — `printf` / log stream (retarget `_write()` → `huart3` in `syscalls.c`) |

Jetson side (typical 40-pin header): **`/dev/ttyTHS0`** at the same baud as `huart2`.

### CAN (ODrive / XDrive)

| Parameter | Value |
|-----------|--------|
| Active peripheral | **FDCAN2** only at runtime |
| Bit rate | **250 kbit/s** (matches ODrive config) |
| Node ID | Axis0 = **0**, Axis1 = **1** |
| Protocol | ODrive **CAN Simple** (11-bit standard IDs) |
| Transceiver | SN65HVD230 on PB12/PB13 |

Defined in `Core/Inc/fdcan.h`:

```c
#define ODRIVE_CAN_HANDLE hfdcan2
```

### SPI (ICM-45686)

| Parameter | Value |
|-----------|--------|
| Bus | **SPI1** (PA5/PA6/PA7) |
| Mode | Master, 8-bit, **Mode 3** (CPOL=1, CPHA=1; TDK ICM45686 reference) |
| Clock (current Cube) | **2.5 MHz** (prescaler ÷8, 20 MHz kernel) — safe for bring-up |
| Max (device) | 24 MHz @ 3.3 V IO |
| Driver | [`../sensor_drivers/icm45686`](../sensor_drivers/README.md) — software **CS** on a GPIO (e.g. PA4; assign in board config) |

`WHO_AM_I` must read **0xE9** before raising SPI speed.

---

## Software architecture

### Repository layout

```
RobotSTM32FirmWare/          ← CubeIDE / VS Code project (this folder)
  Core/
    Src/tasks/               FreeRTOS tasks (control, motor_tx, imu, …)
    Inc/app_config.h         Rates, PID gains, IMU selection
  docs/CONTROL_ARCHITECTURE.md
  .vscode/                   Build (make) + Cortex-Debug launch
  README.md                  This file

STM32/                       Shared + reference modules
  common/                    pid_controller (linked in Debug build)
  sensor_drivers/            ICM-45686, … (also under project `sensor_drivers/`)
  odrive_can/                CAN Simple protocol
  Telemetry/                 UART binary telemetry (ESP32, planned)

Board/STM32GenericBoard/     KiCad — see ../../Board/README.md
```

### Boot sequence

**In `main()` (before scheduler):**

1. Clock, GPIO, peripherals (`MX_*_Init`)
2. Unless `DEBUG_DESK_NO_ODRIVE` is defined:
   - **`odrive_can_fdcan_start(&ODRIVE_CAN_HANDLE)`** — start FDCAN2, fix Cube zero FIFO depth
   - **`odrive_velocity_mode_startup()`** — heartbeats, closed loop, velocity mode (nodes 0 & 1)

**In `app_drivers_rtos_init()` (before tasks run):**

3. **`app_odrive_boot_if_needed()`** — if startup was skipped (old binary), ensure FDCAN + run startup
4. **`odrive_can_async_init/start`**, **`odrive_can_dma_init`**, **`app_imu_init`**

Startup in `main` must finish **before** CAN RX IRQ is used for heartbeat polling during `odrive_velocity_mode_startup()`.

### RTOS rates (`app_config.h`)

| Task | Period | Rate |
|------|--------|------|
| control | 2 ms | 500 Hz |
| motor_tx | 2 ms | 500 Hz |
| imu | 1 ms | 1 kHz |
| encoder | 2 ms | 500 Hz |
| odrive | 10 ms | 100 Hz |

### Key source files

| File | Description |
|------|-------------|
| `Core/Src/main.c` | Entry, ODrive boot, `DEBUG_DESK_NO_ODRIVE` |
| `Core/Src/app_tasks.c` | Creates FreeRTOS tasks |
| `Core/Src/tasks/task_control.c` | 500 Hz PID / strategies |
| `Core/Src/tasks/task_motor_tx.c` | 500 Hz `SET_INPUT_VEL` |
| `Core/Src/tasks/task_imu.c` | 1 kHz IMU + fusion |
| `Core/Src/odrive_can_dma.c` | Mutex TX queue, encoder RX |
| `Core/Src/odrive_can_hal.c` | FDCAN + `odrive_can_fdcan_ensure_started()` |
| `Core/Src/app_drivers.c` | RTOS driver init + CAN fallback boot |
| `Core/Inc/app_halt.h` | `g_halt_*` on fatal errors |

### ODrive CAN notes

- **`SET_INPUT_VEL`** carries **velocity (turn/s)** + **torque feedforward (Nm)** in one frame. For velocity-only control, set torque FF to **0**.
- Heartbeat period on axis0: **100 ms** (`can_heartbeat_rate_ms` in config file).
- Do not force **IDLE** on the skip-calibration path if `startup_closed_loop_control=True` (avoids long startup waits).

---

## Feature status

- [x] FDCAN2 + ODrive startup (2× node 0/1, velocity mode)
- [x] FreeRTOS tasks @ 500 Hz control + motor CAN TX
- [x] ICM-45686 SPI read + complementary pitch fusion
- [x] TIM2 wheel encoder @ 500 Hz
- [x] Swappable control strategies (dual PID, linear, cascade)
- [x] Estop: IMU invalid / pitch failsafe → zero `SET_INPUT_VEL`
- [x] VS Code / CubeIDE build via GNU Make (`Debug/`)
- [ ] USART telemetry / Jetson / `printf` on USART3
- [ ] Bias task (dual-wheel trim) wired to control
- [ ] FreeRTOS FPU context (`configENABLE_FPU=1`)

---

## Build and flash

### STM32CubeIDE

1. Open `RobotSTM32FirmWare/` — **STM32Cube FW_H7 V1.12.1**
2. Build **Debug** or **Release**
3. Flash with ST-LINK (SWD)

### VS Code / Cursor

1. Open folder `STM32/RobotSTM32FirmWare` (or workspace root).
2. **Terminal → Run Build Task** (`make -j16` in `Debug/`) or F5 → **Prepare Debug** (build + flash via Cortex-Debug).
3. Toolchain: **GNU Tools for STM32** on `PATH` (`arm-none-eabi-gcc`), ST-LINK GDB server paths in `.vscode/launch.json`.

**Important:** ODrive boot runs **by default** in Debug. For bench **without** CAN hardware, add define **`DEBUG_DESK_NO_ODRIVE`** (Cube: MCU GCC Preprocessor, or edit `Debug/**/subdir.mk` `-D` flags). Do **not** rely on `.cproject` alone if you only build with `make` — the makefile must see the same `-D` flags.

After CubeMX regenerate: re-apply USER CODE in `main.c`, check `fdcan.c` USER block calls `odrive_can_fdcan_apply_global_accept_std`.

---

## Configuration defines

| Define | Location | Default | Meaning |
|--------|----------|---------|---------|
| `ODRIVE_CAN_HANDLE` | `fdcan.h` | `hfdcan2` | Active CAN peripheral |
| `ODRIVE_VELOCITY_MODE_SKIP_CALIBRATION` | `odrive_velocity_mode_startup.h` | `1` | Skip full cal; use saved XDrive config |
| `ODRIVE_VELOCITY_MODE_AXIS0_NODE_ID` | same | `0` | Left/right motor per your wiring |
| `ODRIVE_VELOCITY_MODE_AXIS1_NODE_ID` | same | `1` | Second XDrive |
| `ODRIVE_CAN_DIAG_SCOPE_PING` | `odrive_can_hal.h` | `0` | `1` = CAN scope test only (no ODrive startup) |
| `DEBUG_DESK_NO_ODRIVE` | build flags | undefined | Define to skip ODrive boot on desk (no CAN) |
| `APP_CTRL_STRATEGY_DEFAULT` | `app_config.h` | `0` | 0=dual_pid, 1=linear, 2=cascade |

---

## Debugging

On fatal errors, firmware calls `app_halt_record()` and spins in `Error_Handler()`.

| Variable | Meaning |
|----------|---------|
| `g_halt_magic` | `0xC0DEF00D` when halt info is valid |
| `g_odrive_startup_last_error` | **0** = OK; **255** = startup never ran |
| `g_odrive_drive_valid_mask` | Bits 0/1 = valid encoder per node |
| `g_odrive_can_tx_hal_fail` | Should stay ~0 (FDCAN not started if huge) |
| `g_app_odrive_rtos_fdcan_ok` | 1 if RTOS fallback started FDCAN |
| `g_motor_tx_vel_left/right` | Commands sent (turn/s) |
| `g_ctrl_pitch_rad`, `g_ctrl_cmd_turns_s` | Control outputs |
| `g_imu_init_err` | 0 OK; -1 SPI; -2 WHO_AM_I |
| `g_ctrl_strategy` | Live change strategy ID (debugger) |

**USART3:** PB10 TX, **115200 8N1** (future `printf`).

**CAN sniffer:** `STM32/DebugCan` (listen-only on FDCAN2).

---

## Related documentation

- **[Control & RTOS architecture](docs/CONTROL_ARCHITECTURE.md)** — task rates, strategies, bias (planned), debug globals
- **[KiCad board](../../Board/README.md)** — STM32H743 + ICM-45686 schematic
- [STM32 sensor drivers (ICM-45686)](../sensor_drivers/README.md)
- [Telemetry protocol (ESP32 / USART1)](../Telemetry/README.md)
- [ODrive velocity mode (USB reference)](../../ODrive/OdriveTool/Commands/velocity_mode.txt)
- [XDrive saved config](../../ODrive/OdriveTool/Commands/Configs/2836_anticogging_calibrated.txt)

---

## Revision notes

| Date | Notes |
|------|--------|
| 2026-05 | FreeRTOS 500 Hz control + motor TX; CAN boot / `ensure_started`; docs + KiCad board |
| 2026-05 | Control/RTOS design doc |
| 2026-05 | Initial README: FDCAN2 ODrive bring-up, USART/SPI pin map |
