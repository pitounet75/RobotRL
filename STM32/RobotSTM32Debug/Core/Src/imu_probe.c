#include "imu_probe.h"

#include "icm45686.h"
#include "main.h"
#include "spi.h"

#define IMU_POLL_MS 10u
#define G_MPS2 9.80665f

volatile int32_t g_mpu_init = -99;
volatile uint32_t g_mpu_who;
volatile uint32_t g_mpu_ok;
volatile int32_t g_mpu_ax_mg;
volatile int32_t g_mpu_ay_mg;
volatile int32_t g_mpu_az_mg;
volatile int32_t g_mpu_gx_mrads;
volatile int32_t g_mpu_gy_mrads;
volatile int32_t g_mpu_gz_mrads;
volatile int32_t g_mpu_temp_c;
volatile uint32_t g_mpu_read_ok;
volatile uint32_t g_mpu_read_fail;

static icm45686_t s_dev;
static uint8_t s_ready;
static uint32_t s_last_ms;

static int32_t to_mg(float mps2) {
  return (int32_t)(mps2 * (1000.0f / G_MPS2));
}

void imu_probe_init(void) {
  /* Cube leaves PE6 (CS) low; idle high before the first transfer. */
  HAL_GPIO_WritePin(PE6_SPI3_CS_GPIO_Port, PE6_SPI3_CS_Pin, GPIO_PIN_SET);

  g_mpu_init = icm45686_init_spi(&s_dev, &hspi3, PE6_SPI3_CS_GPIO_Port,
                                 PE6_SPI3_CS_Pin, ICM45686_ACCEL_16G,
                                 ICM45686_GYRO_2000_DPS);
  g_mpu_who = icm45686_last_who_am_i;
  s_ready = (g_mpu_init == 0) ? 1u : 0u;
  s_last_ms = HAL_GetTick();
}

void imu_probe_poll(void) {
  if (s_ready == 0u) {
    return;
  }

  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - s_last_ms) < IMU_POLL_MS) {
    return;
  }
  s_last_ms = now;

  imu_data_t d;
  if (icm45686_read(&s_dev, &d) != 0) {
    g_mpu_ok = 0u;
    g_mpu_read_fail++;
    return;
  }

  g_mpu_ax_mg = to_mg(d.accel_mps2[0]);
  g_mpu_ay_mg = to_mg(d.accel_mps2[1]);
  g_mpu_az_mg = to_mg(d.accel_mps2[2]);
  g_mpu_gx_mrads = (int32_t)(d.gyro_rads[0] * 1000.0f);
  g_mpu_gy_mrads = (int32_t)(d.gyro_rads[1] * 1000.0f);
  g_mpu_gz_mrads = (int32_t)(d.gyro_rads[2] * 1000.0f);
  g_mpu_temp_c = (int32_t)d.temp_celsius;
  g_mpu_ok = 1u;
  g_mpu_read_ok++;
}
