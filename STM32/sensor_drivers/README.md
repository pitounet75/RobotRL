# STM32 sensor drivers

Drivers for sensors used by the STM32 balance loop. IMU drivers use **STM32 HAL** for SPI and I2C and share a **common data format** (`imu_data_t`) so the application can switch between IMUs via config or a single compile-time change.

## IMU common interface

- **Data**: `imu_driver.h` defines `imu_data_t` (accel m/s², gyro rad/s, temp °C).
- **Transport**: ICM45686 and BMI323 support SPI + I2C; MPU6050 is **I2C only** (`mpu6050_init_i2c`).
- **API**: After init, call `xxx_read(dev, data)` to fill `imu_data_t`.
- **Scale selection**: Init takes `accel_fs` and `gyro_fs` enums (e.g. `ICM45686_ACCEL_16G`, `ICM45686_GYRO_2000_DPS`). See each driver header for available scales.

Same read API and data layout for all IMUs.

## HAL platform

- **`imu_hal_platform.h`**: Includes your MCU HAL (STM32H7xx, F4xx, F7xx) or forward-declares types. Define `IMU_HAL_TIMEOUT_MS` (default 10) if needed.
- Ensure your project links the HAL (SPI, I2C, GPIO).

## Drivers

| Driver   | Part       | HAL interfaces | Notes |
|----------|------------|----------------|--------|
| ICM45686 | TDK/InvenSense 6-axis | SPI, I2C | SPI: software CS; MSB=1 for read. |
| BMI323   | Bosch Sensortec 6-axis | SPI, I2C | SPI: 1 dummy byte before read; I2C: 2 dummy bytes. Soft reset `0xDEAF` to CMD; feature engine enabled per datasheet. |
| MPU6050  | TDK/InvenSense 6-axis  | I2C only | Classic 0x3B burst; WHO_AM_I `0x68`; big-endian raw samples. |

## Usage

**SPI (e.g. ICM45686 on SPI1, CS on PA4):**

```c
#include "icm45686.h"

icm45686_t imu;
if (icm45686_init_spi(&imu, &hspi1, GPIOA, GPIO_PIN_4,
                      ICM45686_ACCEL_16G, ICM45686_GYRO_2000_DPS) != 0)
    /* error */;

imu_data_t data;
while (1) {
    if (icm45686_read(&imu, &data) == 0)
        /* use data.accel_mps2[], data.gyro_rads[], data.temp_celsius */;
}
```

**I2C (e.g. BMI323 at 7-bit addr 0x68):**

```c
#include "bmi323.h"

bmi323_t imu;
if (bmi323_init_i2c(&imu, &hi2c1, 0x68,
                    BMI323_ACCEL_4G, BMI323_GYRO_2000_DPS) != 0)
    /* error */;

imu_data_t data;
bmi323_read(&imu, &data);
```

**I2C (MPU6050):**

```c
#include "mpu6050.h"

mpu6050_t imu;
if (mpu6050_init_i2c(&imu, &hi2c1, 0x68,
                     MPU6050_ACCEL_16G, MPU6050_GYRO_2000_DPS) != 0)
    /* error */;

imu_data_t data;
mpu6050_read(&imu, &data);
```

**Compile-time IMU switch:**

```c
#include "imu_driver.h"
#if IMU_SELECT == 1
#include "icm45686.h"
#else
#include "bmi323.h"
#endif

imu_data_t data;
#if IMU_SELECT == 1
  icm45686_t imu;
  icm45686_init_spi(&imu, &hspi1, GPIOA, GPIO_PIN_4,
                    ICM45686_ACCEL_16G, ICM45686_GYRO_2000_DPS);
  icm45686_read(&imu, &data);
#else
  bmi323_t imu;
  bmi323_init_i2c(&imu, &hi2c1, 0x68,
                  BMI323_ACCEL_4G, BMI323_GYRO_2000_DPS);
  bmi323_read(&imu, &data);
#endif
```

## Noise and bias test

`imu_noise_test.h` / `imu_noise_test.c` characterize sensor noise and bias while the IMU is stationary.

```c
#include "imu_noise_test.h"
#include "icm45686.h"  // or bmi323.h / mpu6050.h

icm45686_t imu;
// ... init_spi or init_i2c ...

imu_noise_test_result_t result;
if (imu_noise_test_run(&imu, IMU_TEST_ICM45686, 10.0f, 100.0f,
                       HAL_Delay, &result) == 0) {
    // Per-axis: result.accel_bias[0..2], result.accel_stddev[0..2]
    // Combined: result.accel_combined_stddev, result.accel_combined_bias
    // Same for gyro. result.temp_mean, result.n_samples, result.duration_s
}
```

- **duration_s**: Test length (s). Place sensor flat and still.
- **samples_per_sec**: Sample rate (Hz). Higher = more samples, longer per-sample interval.
- **delay_ms**: Blocking delay (e.g. `HAL_Delay`).
- **Output**: Per-axis bias (mean) and stddev; combined stddev = √(var_x + var_y + var_z), combined bias = √(bias_x² + bias_y² + bias_z²).

## Adding to your build

Add `STM32/sensor_drivers/icm45686.c`, `bmi323.c`, `mpu6050.c`, `imu_noise_test.c` to your STM32 project and include path `STM32/sensor_drivers/`. Link `libm` (for `sqrt`). Include `imu_hal_platform.h` before or with the driver.
