# Robot PCB (KiCad)

KiCad projects for the RobotRL hardware.

## STM32GenericBoard

| Item | Value |
|------|--------|
| MCU (schematic) | **STM32H743VGTx** (LQFP100) — matches firmware `STM32H743VIT6` in [`RobotSTM32FirmWare.ioc`](../STM32/RobotSTM32FirmWare/RobotSTM32FirmWare.ioc) |
| IMU symbol | TDK **ICM-45686** (local lib under `libs/ICM_45686/`) |
| Status | Schematic in progress — **no routed PCB** yet (empty `.kicad_pcb`) |

### Open in KiCad

1. Open `STM32GenericBoard/STM32GenericBoard.kicad_pro`.
2. Assign footprints and complete power + bus wiring (FDCAN2, SPI1, USART, SWD).
3. Cross-check nets against CubeMX: [`STM32/RobotSTM32FirmWare/README.md`](../STM32/RobotSTM32FirmWare/README.md).

### Pin map (firmware reference)

Application CAN uses **FDCAN2** (PB12 RX, PB13 TX), not FDCAN1. See firmware README for full USART/SPI table.

### Notes

- Do not commit KiCad lock files (`~*.lck`) — listed in repo `.gitignore`.
- For symbol generation from CubeMX, tools such as [cubemx2kicad](https://github.com/alt-uranium/cubemx2kicad) can label pins from the `.ioc` file.
