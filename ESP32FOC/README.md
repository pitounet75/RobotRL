# ESP32FOC — small dual-gimbal balancer

One ESP32 runs **everything**: FOC for two **2804 gimbal** motors, two **MT6835**,
and an **ICM-45686**. No ODrive, no STM32.

Voltage-mode FOC (SimpleFOC). Gimbals are high-R / low-Kv; there is no current
sense on this board.

## Hardware

| Part | Role |
|------|------|
| ESP32-WROOM (no PSRAM) | FOC + IMU fusion + balance |
| 2× 2804 gimbal | wheels (default `pole_pairs = 7` for 12N14P) |
| 2× MT6835 | shaft angle, SPI Mode 3, 21-bit |
| ICM-45686 | pitch (same chip as the big RobotRL bot) |

Do not share an MT6835 with an ODrive at the same time.

## Pins (change in `include/config.h`)

Shared VSPI, software CS, all CSN high **before** `SPI.begin()`:

| Signal | GPIO |
|--------|------|
| SCK / MISO / MOSI | 18 / 19 / 23 |
| MT6835 CS left | 5 |
| MT6835 CS right | 4 |
| ICM45686 CS | 15 (idle high — OK for strapping) |
| Left UH/VH/WH | 32 / 33 / 25 |
| Right UH/VH/WH | 26 / 27 / 14 |

MT6835 framing is the RobotRL 24-clock `transferBytes()` (not SimpleFOC’s
3× `transfer()`). EEPROM PROG lives only in `ESP32MT6835Setup`.

ICM-45686: WHO_AM_I `0xE9`, try SPI Mode 3 then Mode 0 (same as the STM32 driver).
Accel 16 g / gyro 2000 dps / ODR 800 Hz. Pitch complementary filter uses the
same axis macros as `app_config.h` on the STM32 (`forward=1`, `up=0`, `gyro=2`).

## Motors

Default `FOC_VBUS = 8.4` (2S). Use `12.0` for 3S. `FOC_VOLTAGE_LIMIT = 3.0` until
`initFOC` is clean. If the 2804 is 24N22P, set `FOC_POLE_PAIRS 11`.

`FOC_CMD_SIGN_L = -1`, `FOC_CMD_SIGN_R = +1` so `+u` is chassis-forward on both
wheels. Flip a sign if a motor is mounted the other way.

## Build

Same PlatformIO core as `ESP32MT6835Setup` (`espressif32@7.0.1`, SimpleFOC **2.3.3**).

```powershell
cd H:\Projects\RobotRL\ESP32FOC

# Encoders + IMU printout, no PWM
pio run -e esp32dev -t upload
pio device monitor -e esp32dev

# Dual open-loop (wheels off the ground)
pio run -e openloop -t upload

# Dual voltage-torque, Commander
pio run -e foc -t upload

# Balance PD (after FOC works on both wheels)
pio run -e balance -t upload
```

Serial **115200**. Commander (`openloop` / `foc`): `T<val>` both axes (robot-frame),
`L` / `R` for one motor.

Balance is **computed torque** (STM32 `ff_cascade` idea): invert
`I·θ̈ − m g h·sin(θ)`, then a light PID that only commands **θ̈***.

`u = K_J·θ̈* − K_ff·sin(θ)`,  `θ̈* = Kp·e + Ki·∫e − Kd·θ̇`.

Identify `BAL_K_FF` / `BAL_K_INERTIA` from the robot; keep Kp/Kd small.
`|θ| > BAL_FAILSAFE_RAD` (~30°) zeros both targets.

## Bring-up order

1. `esp32dev` — both angles move, IMU WHO=0xE9, pitch ~0 when upright
2. One wheel at a time: `foc`, `T0.3`, confirm direction vs `FOC_CMD_SIGN_*`
3. `balance` on a tether, raise `BAL_K_PITCH` / `BAL_K_RATE` slowly
4. Set `BAL_K_FF` from `m·g·h / (2·Kt_voltage)` once mass/COM are known

## Safety

- Align PWM pins before any env other than `esp32dev`
- `initFOC` will twitch both motors — wheels off the ground
- GPIO 15/4/5 idle high at reset
