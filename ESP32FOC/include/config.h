#pragma once

/**
 * Small dual-gimbal balancer: ESP32 + 2× 2804 + 2× MT6835 + ICM45686.
 *
 * FOC_MODE:
 *   0 = sensors only (encoders + IMU, no PWM)
 *   1 = dual voltage open-loop
 *   2 = dual closed-loop voltage torque (Commander)
 *   3 = balance (pitch PD + gravity FF → both wheels)
 */
#ifndef FOC_MODE
#define FOC_MODE 0
#endif

/* --- Shared VSPI: both MT6835 + ICM45686 (software CS) --- */
#ifndef PIN_SPI_SCK
#define PIN_SPI_SCK 18
#endif
#ifndef PIN_SPI_MISO
#define PIN_SPI_MISO 19
#endif
#ifndef PIN_SPI_MOSI
#define PIN_SPI_MOSI 23
#endif

#ifndef MT6835_PIN_CS_L
#define MT6835_PIN_CS_L 5
#endif
#ifndef MT6835_PIN_CS_R
#define MT6835_PIN_CS_R 4
#endif
#ifndef MT6835_SPI_HZ
#define MT6835_SPI_HZ 400000u
#endif
#ifndef MT6835_CPR
#define MT6835_CPR 2097152u
#endif

#ifndef ICM_PIN_CS
#define ICM_PIN_CS 15
#endif
#ifndef ICM_SPI_HZ
#define ICM_SPI_HZ 1000000u
#endif

/* 3PWM per motor. Avoid VSPI 18/19/23 and CS 5/4/15. WROOM (no PSRAM). */
#ifndef FOC_PIN_L_UH
#define FOC_PIN_L_UH 32
#endif
#ifndef FOC_PIN_L_VH
#define FOC_PIN_L_VH 33
#endif
#ifndef FOC_PIN_L_WH
#define FOC_PIN_L_WH 25
#endif
#ifndef FOC_PIN_R_UH
#define FOC_PIN_R_UH 26
#endif
#ifndef FOC_PIN_R_VH
#define FOC_PIN_R_VH 27
#endif
#ifndef FOC_PIN_R_WH
#define FOC_PIN_R_WH 14
#endif
#ifndef FOC_PIN_L_EN
#define FOC_PIN_L_EN (-1)
#endif
#ifndef FOC_PIN_R_EN
#define FOC_PIN_R_EN (-1)
#endif

/**
 * 2804 gimbal: iPower GBM2804 / similar 12N14P → 7 pole pairs.
 * 24N22P parts are 11 — measure or read the seller listing.
 */
#ifndef FOC_POLE_PAIRS
#define FOC_POLE_PAIRS 7
#endif

/** 2S pack default for a small bot. Set 12.0 for 3S. */
#ifndef FOC_VBUS
#define FOC_VBUS 8.4f
#endif

/** First-spin / balance voltage cap (phase volts). Gimbals are voltage-mode. */
#ifndef FOC_VOLTAGE_LIMIT
#define FOC_VOLTAGE_LIMIT 3.0f
#endif
#ifndef FOC_VOLTAGE_ALIGN
#define FOC_VOLTAGE_ALIGN 1.0f
#endif
#ifndef FOC_PWM_HZ
#define FOC_PWM_HZ 20000u
#endif
#ifndef FOC_OPENLOOP_VEL
#define FOC_OPENLOOP_VEL 4.0f
#endif

/** Robot-frame: +u = both wheels drive the chassis forward. */
#ifndef FOC_CMD_SIGN_L
#define FOC_CMD_SIGN_L (-1)
#endif
#ifndef FOC_CMD_SIGN_R
#define FOC_CMD_SIGN_R (1)
#endif

/* Pitch fusion (same convention as STM32 RobotRL). */
#ifndef IMU_PITCH_ACCEL_FORWARD_AXIS
#define IMU_PITCH_ACCEL_FORWARD_AXIS 1
#endif
#ifndef IMU_PITCH_ACCEL_UP_AXIS
#define IMU_PITCH_ACCEL_UP_AXIS 0
#endif
#ifndef IMU_PITCH_GYRO_AXIS
#define IMU_PITCH_GYRO_AXIS 2
#endif
#ifndef IMU_PITCH_GYRO_SIGN
#define IMU_PITCH_GYRO_SIGN 1.0f
#endif
#ifndef IMU_COMPLEMENTARY_ALPHA
#define IMU_COMPLEMENTARY_ALPHA 0.98f
#endif

/**
 * Computed torque (same idea as STM32 ff_cascade):
 *   invert the 1-DoF pitch EOM, then a *light* PID only on the residual.
 *
 *   k·u ≈ I·θ̈ − m g h·sin(θ)     (signs: +pitch = nose-down, +u catches)
 *   θ̈* = Kp·(θ_ref−θ) + Ki·∫e − Kd·θ̇
 *   u   = K_J·θ̈* − K_ff·sin(θ)
 *
 * K_ff, K_J come from the robot (mass, COM, I, V→torque). Kp/Kd/Ki stay small.
 * K_J=1 and K_ff=0 → same as a raw voltage PD (bring-up).
 */
#ifndef BAL_PITCH_REF_RAD
#define BAL_PITCH_REF_RAD 0.0f
#endif
#ifndef BAL_K_FF
#define BAL_K_FF 0.0f /* V ; m g h / k  — 0 until identified */
#endif
#ifndef BAL_K_INERTIA
#define BAL_K_INERTIA 1.0f /* V / (rad/s²) ; I / k */
#endif
#ifndef BAL_K_PITCH
#define BAL_K_PITCH 4.0f /* (rad/s²) / rad  →  θ̈* */
#endif
#ifndef BAL_K_RATE
#define BAL_K_RATE 0.4f /* (rad/s²) / (rad/s) */
#endif
#ifndef BAL_K_I
#define BAL_K_I 0.0f /* (rad/s²) / (rad·s) */
#endif
#ifndef BAL_I_LIMIT
#define BAL_I_LIMIT 2.0f /* clamp on ∫e [rad·s] */
#endif
#ifndef BAL_FAILSAFE_RAD
#define BAL_FAILSAFE_RAD 0.52f
#endif
