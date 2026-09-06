# Shared STM32 modules

Host-testable and firmware-linked C modules used by `RobotSTM32FirmWare`.

| Module | Files | Used by |
|--------|--------|---------|
| **IMU fusion** | Also built from `RobotSTM32FirmWare/Core/Src/imu_fusion.c` | `task_imu` complementary pitch |

Host tests under [`../tests/host/`](../tests/host/) compile firmware sources directly where noted in that README.
