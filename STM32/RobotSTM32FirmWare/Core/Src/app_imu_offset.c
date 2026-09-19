/**
 * @file app_imu_offset.c
 */

#include "app_imu_offset.h"

#include "app_config.h"

#include "stm32h7xx_hal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define APP_IMU_OFFSET_MAGIC    0x314D5549u /* 'IMU1' */
#define APP_IMU_OFFSET_VERSION  1u
#define APP_IMU_OFFSET_REC_SIZE 64u

typedef struct {
    uint32_t magic;
    uint32_t version;
    float accel_bias_mps2[3];
    float gyro_bias_rads[3];
    uint32_t crc32;
    uint32_t reserved[7];
} app_imu_offset_record_t;

_Static_assert(sizeof(app_imu_offset_record_t) == APP_IMU_OFFSET_REC_SIZE,
               "IMU offset flash record must be 64 bytes");

static float s_accel_bias[3];
static float s_gyro_bias[3];
static volatile bool s_owns_motors;
static volatile float s_tq_left;
static volatile float s_tq_right;
static volatile bool s_fusion_reset;

volatile uint32_t g_imu_offset_state;
volatile uint32_t g_imu_offset_flash_ok;
volatile uint32_t g_imu_offset_owns;
volatile uint32_t g_imu_offset_sample_n;
volatile float g_imu_offset_a_up;
volatile float g_imu_offset_accel_mps2[3];
volatile float g_imu_offset_gyro_rads[3];

static uint32_t crc32_ieee(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    int b;

    for (i = 0u; i < len; i++) {
        crc ^= (uint32_t)p[i];
        for (b = 0; b < 8; b++) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static void publish_watch(void)
{
    uint32_t i;
    for (i = 0u; i < 3u; i++) {
        g_imu_offset_accel_mps2[i] = s_accel_bias[i];
        g_imu_offset_gyro_rads[i] = s_gyro_bias[i];
    }
}

static void cache_invalidate_record(void)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    SCB_InvalidateDCache_by_Addr((void *)APP_IMU_OFFSET_FLASH_ADDR,
                                 (int32_t)APP_IMU_OFFSET_REC_SIZE);
#endif
}

static bool record_valid(const app_imu_offset_record_t *rec)
{
    uint32_t crc;

    if (rec->magic != APP_IMU_OFFSET_MAGIC || rec->version != APP_IMU_OFFSET_VERSION) {
        return false;
    }
    crc = crc32_ieee(rec, offsetof(app_imu_offset_record_t, crc32));
    return crc == rec->crc32;
}

static bool offsets_plausible(const float accel_bias[3], const float gyro_bias[3])
{
    uint32_t i;
    for (i = 0u; i < 3u; i++) {
        if (accel_bias != NULL &&
            fabsf(accel_bias[i]) > APP_IMU_OFFSET_ACCEL_MAX_MPS2) {
            return false;
        }
        if (gyro_bias != NULL &&
            fabsf(gyro_bias[i]) > APP_IMU_OFFSET_GYRO_MAX_RADS) {
            return false;
        }
    }
    return true;
}

static void clear_runtime(void)
{
    memset(s_accel_bias, 0, sizeof(s_accel_bias));
    memset(s_gyro_bias, 0, sizeof(s_gyro_bias));
    publish_watch();
}

static void load_from_flash(void)
{
    app_imu_offset_record_t rec;

    cache_invalidate_record();
    memcpy(&rec, (const void *)APP_IMU_OFFSET_FLASH_ADDR, sizeof(rec));
    if (!record_valid(&rec)) {
        g_imu_offset_flash_ok = 0u;
        clear_runtime();
        return;
    }
    if (!offsets_plausible(rec.accel_bias_mps2, rec.gyro_bias_rads)) {
        /* e.g. accel ≈ g stored as “bias” → pitch ±90° → failsafe while upright. */
        g_imu_offset_flash_ok = 2u;
        clear_runtime();
        return;
    }
    memcpy(s_accel_bias, rec.accel_bias_mps2, sizeof(s_accel_bias));
    memcpy(s_gyro_bias, rec.gyro_bias_rads, sizeof(s_gyro_bias));
    g_imu_offset_flash_ok = 1u;
    publish_watch();
}

void app_imu_offset_init(void)
{
    s_owns_motors = false;
    g_imu_offset_owns = 0u;
    s_tq_left = 0.0f;
    s_tq_right = 0.0f;
    s_fusion_reset = false;
    g_imu_offset_state = (uint32_t)APP_IMU_OFFSET_ST_WAIT_IMU;
    g_imu_offset_sample_n = 0u;
    load_from_flash();
}

void app_imu_offset_apply(float accel_mps2[3], float gyro_rads[3])
{
    uint32_t i;
    if (accel_mps2 == NULL || gyro_rads == NULL) {
        return;
    }
    for (i = 0u; i < 3u; i++) {
        accel_mps2[i] -= s_accel_bias[i];
        gyro_rads[i] -= s_gyro_bias[i];
    }
}

void app_imu_offset_get(float accel_bias_mps2[3], float gyro_bias_rads[3])
{
    if (accel_bias_mps2 != NULL) {
        memcpy(accel_bias_mps2, s_accel_bias, sizeof(s_accel_bias));
    }
    if (gyro_bias_rads != NULL) {
        memcpy(gyro_bias_rads, s_gyro_bias, sizeof(s_gyro_bias));
    }
}

void app_imu_offset_set_runtime(const float accel_bias_mps2[3],
                                const float gyro_bias_rads[3])
{
    if (!offsets_plausible(accel_bias_mps2, gyro_bias_rads)) {
        clear_runtime();
        return;
    }
    if (accel_bias_mps2 != NULL) {
        memcpy(s_accel_bias, accel_bias_mps2, sizeof(s_accel_bias));
    }
    if (gyro_bias_rads != NULL) {
        memcpy(s_gyro_bias, gyro_bias_rads, sizeof(s_gyro_bias));
    }
    publish_watch();
}

static app_imu_offset_record_t s_prog_rec __attribute__((aligned(32)));

bool app_imu_offset_save_flash(void)
{
    app_imu_offset_record_t *const rec = &s_prog_rec;
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_err = 0u;
    HAL_StatusTypeDef st;
    bool ok = false;
    uint32_t i;

    memset(rec, 0, sizeof(*rec));
    rec->magic = APP_IMU_OFFSET_MAGIC;
    rec->version = APP_IMU_OFFSET_VERSION;
    memcpy(rec->accel_bias_mps2, s_accel_bias, sizeof(rec->accel_bias_mps2));
    memcpy(rec->gyro_bias_rads, s_gyro_bias, sizeof(rec->gyro_bias_rads));
    rec->crc32 = crc32_ieee(rec, offsetof(app_imu_offset_record_t, crc32));

    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);
    if (HAL_FLASH_Unlock() != HAL_OK) {
        g_imu_offset_flash_ok = 0u;
        return false;
    }

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_2;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1u;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    st = HAL_FLASHEx_Erase(&erase, &sector_err);
    if (st == HAL_OK) {
        ok = true;
        for (i = 0u; i < APP_IMU_OFFSET_REC_SIZE; i += 32u) {
            const uint32_t addr = APP_IMU_OFFSET_FLASH_ADDR + i;
            const uint32_t data = (uint32_t)((const uint8_t *)rec + i);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, data) != HAL_OK) {
                ok = false;
                break;
            }
        }
    }
    (void)HAL_FLASH_Lock();
    cache_invalidate_record();

    if (ok) {
        app_imu_offset_record_t check;
        memcpy(&check, (const void *)APP_IMU_OFFSET_FLASH_ADDR, sizeof(check));
        ok = record_valid(&check);
    }
    g_imu_offset_flash_ok = ok ? 1u : 0u;
    return ok;
}

bool app_imu_offset_owns_motors(void)
{
    return s_owns_motors;
}

void app_imu_offset_set_owns_motors(bool own)
{
    s_owns_motors = own;
    g_imu_offset_owns = own ? 1u : 0u;
    if (!own) {
        s_tq_left = 0.0f;
        s_tq_right = 0.0f;
    }
}

void app_imu_offset_set_motor_torque(float left_nm, float right_nm)
{
    s_tq_left = left_nm;
    s_tq_right = right_nm;
}

void app_imu_offset_motor_torque(float *left_nm, float *right_nm)
{
    if (left_nm != NULL) {
        *left_nm = s_tq_left;
    }
    if (right_nm != NULL) {
        *right_nm = s_tq_right;
    }
}

void app_imu_offset_request_fusion_reset(void)
{
    s_fusion_reset = true;
}

bool app_imu_offset_take_fusion_reset(void)
{
    const bool req = s_fusion_reset;
    s_fusion_reset = false;
    return req;
}
