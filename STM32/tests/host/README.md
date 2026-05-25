# STM32 host tests

Run logic tests on your PC (no board required):

```cmd
cd STM32\tests\host
make
```

Requires **host gcc** on PATH (MinGW-w64, MSYS2, or Linux). CubeIDE's `arm-none-eabi-gcc` is for firmware only.

Windows (MSYS2):

```bash
pacman -S mingw-w64-x86_64-gcc make
# add C:\msys64\mingw64\bin to PATH
```

## What is tested

| Module | File under test |
|--------|-----------------|
| ICM45686 raw parsing | `sensor_drivers/icm45686_parse.c` |
| Complementary pitch fusion | `common/imu_fusion.c` |
| IMU snapshot seqlock | `RobotSTM32FirmWare/Core/Src/app_samples.c` |

## CI

```yaml
- run: make -C STM32/tests/host test
```
