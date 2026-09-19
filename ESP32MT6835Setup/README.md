# ESP32 MT6835 Setup

One-shot PlatformIO firmware to set the MagnTek **MT6835** ABZ resolution over SPI
and burn it to EEPROM.

Default target: **16384 PPR** (`MT6835_ABZ_PPR` in `include/config.h`).

## Why

RobotRL wheel velocity uses STM32 TIM encoder mode on the MT6835 ABZ outputs.
Higher ABZ PPR reduces quantization noise into `cascade_vel_kd` (see growl ~37 Hz
A/B captures).

## Wiring (ESP32 ↔ MT6835)

Defaults in `include/config.h` (VSPI):

| Signal | ESP32 GPIO | MT6835 |
|--------|------------|--------|
| SCK    | 18         | SCK    |
| MISO   | 19         | MISO   |
| MOSI   | 23         | MOSI   |
| CS0    | 5          | CSN |
| CS1    | unused (`-1`) | second chip not wired |
| 3V3/GND| —          | VDD / GND (3.3 V) |

SPI **Mode 3** (CPOL=1, CPHA=1). Default **400 kHz**. One **24-clock** frame per access
(`transferBytes` × 3, CSN held). Do not use three separate 8-bit `SPI.transfer()` calls
on ESP32 — extra SCK edges while CSN=0 shift the frame; a following EEPROM PROG can
brick the chip (burns the whole map, including MagnTek-only bytes).

Change pins in `include/config.h` if your harness differs. You can temporarily
unplug the ESP32 telemetry UART bridge and use the same board for this tool.

## ESP32 SPI loopback (no encoder)

Unplug the MT6835. Jumper **GPIO23 (MOSI) ↔ GPIO19 (MISO)**.

```powershell
cd H:\Projects\RobotRL\ESP32MT6835Setup
pio run -e loopback -t upload
pio device monitor -e loopback
```

Every `TX=.. RX=..` line must be **OK**. If FAIL, the ESP32/harness is bad — stop before talking to a chip.

## Build / flash (default = EEPROM dump, read-only)

```powershell
cd H:\Projects\RobotRL\ESP32MT6835Setup
pio run -e esp32dev -t upload
pio device monitor -e esp32dev > mt6835_dump_right.txt
```

Watch serial @ **115200**. Default flow:

1. Short probe (USER_ID, angle, ABZ_PPR, bus alive)  
2. **Full EEPROM map dump** — lines `reg,0xADDR,0xVV` for `0x001`, `0x007..0x012`, `0x013..0x0D2` (NLC)  
3. Save the log to a file — compare **good vs suspect** chip before any write  

Short probe only (no full map):

```powershell
pio run -e probe -t upload
pio device monitor -e probe
```

## Constants

| Macro | Default | Meaning |
|-------|---------|---------|
| `MT6835_MODE_DUMP` | `1` | Full EEPROM dump on boot |
| `MT6835_ALLOW_WRITE` | `0` | ABZ volatile write (env `write`) |
| `MT6835_PROGRAM_EEPROM` | `0` | Never `1` until restore validated |
| `MT6835_ABZ_PPR` | `16384` | Target ABZ (write env only) |
| `MT6835_PIN_CS0` / `CS1` | `5` / `-1` | CS0 only; CS1 unused |

Example (volatile test only):

```ini
; platformio.ini build_flags
build_flags =
    -DMT6835_ABZ_PPR=8192
    -DMT6835_PROGRAM_EEPROM=0
```

## After programming — STM32

Your RobotRL firmware uses `TIM_ENCODERMODE_TI1` on TIM2/TIM4 (**×2**, not ×4):

```c
/* app_config.h — matches TI1 encoder mode */
#define WHEEL_ENCODER_CPR  (16384u * 2u)  /* 32768 counts / motor rev */
```

`TIM_ENCODERMODE_TI12` would be true ×4 (`PPR * 4`). Do not assume ×4 unless CubeMX/HAL says `TI12`.

## Safety / brick notes

- Default build is **read-only** (`MT6835_ALLOW_WRITE=0`, `MT6835_PROGRAM_EEPROM=0`).
- Boot runs an **SPI loopback self-test** first (jumper MOSI GPIO23 ↔ MISO GPIO19).
  If that fails, the ESP32/harness is bad — do not conclude the MT6835 is dead.
- Two encoders both reading `0x00` on the same ESP32 usually means **MISO stuck low
  or CS/power**, not two independent silicon deaths.
- `Program EEPROM` burns the **entire** register map. Only enable write/PROG after
  loopback OK and a healthy probe dump.

## Restore a bricked chip from a healthy dump

Unplug the **donor** before any `restore` / `restore_burn`. CS is the original
single pin: **GPIO5 (`CS0`)**. `CS1` stays unused (`-1`).

Dump the healthy encoder:

```powershell
cd H:\Projects\RobotRL\ESP32MT6835Setup
pio run -e esp32dev -t upload
pio device monitor -e esp32dev > mt6835_dump_healthy.txt
```

Sanity-check the log: `busAlive=yes`, data on the 3rd MISO byte. Then:

```powershell
python dump_to_restore_map.py mt6835_dump_healthy.txt
```

Unplug the donor. Wire the **bricked** chip on CS0 (GPIO5). Then volatile write:

```powershell
pio run -e restore -t upload
pio device monitor -e restore
```

Motor test. If it reaches ~120 turn/s, burn:

```powershell
pio run -e restore_burn -t upload
pio device monitor -e restore_burn
```

Keep power 7 s after `EEPROM wait done`. No extra SPI. Then power-cycle.

`restore` is volatile only. A power-cycle without `restore_burn` keeps the
old EEPROM. That is intentional.

ODrive: `FULL_CALIBRATION_SEQUENCE` (offset is now the donor's `ZERO_POS`).
Do not `save_configuration()` until that cal succeeds.

## Write ABZ only (not a full-map restore)

```powershell
pio run -e write -t upload
pio device monitor -e write
```

Volatile ABZ only (`PROGRAM_EEPROM=0`). Do **not** enable EEPROM PROG until a donor dump is verified and a restore path exists.
- At 16384 PPR, max mechanical speed is limited by AB edge rate (see MagnTek MT6835 DS)

## Safety

- Program **one axis at a time** if unsure of CS mapping  
- Keep motors disabled while SPI-programming  
- After STM32 CPR change, verify `vel_wheel` scale with a known spin before riding


```powershell
pio run -e write -t upload
pio device monitor -e write
```

Volatile ABZ only (`PROGRAM_EEPROM=0`). Do **not** enable EEPROM PROG until a donor dump is verified and a restore path exists.
- At 16384 PPR, max mechanical speed is limited by AB edge rate (see MagnTek MT6835 DS)

## Safety

- Program **one axis at a time** if unsure of CS mapping  
- Keep motors disabled while SPI-programming  
- After STM32 CPR change, verify `vel_wheel` scale with a known spin before riding
