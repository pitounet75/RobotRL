/**
 * @file app_config.h
 * @brief Robot application rates, priorities, and hardware constants.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* --- FreeRTOS task periods (ms) --- */
#define APP_CONTROL_PERIOD_MS        2u    /* 500 Hz */
#define APP_MOTOR_TX_PERIOD_MS       2u
#define APP_IMU_PERIOD_MS            1u    /* 1 kHz target */
#define APP_ENCODER_PERIOD_MS        2u    /* 500 Hz */
#define APP_ODRIVE_PERIOD_MS         10u   /* 100 Hz */

/*
 * Two separate motor drives on the CAN bus (e.g. 2x MKS XDrive Mini).
 * Each board has a single active axis (axis0); CAN Simple uses one node_id per board.
 * This is NOT one ODrive with axis0 + axis1 on the same node.
 */
#define APP_ODRIVE_DRIVE_COUNT       2u
#define APP_ODRIVE_NODE_COUNT        APP_ODRIVE_DRIVE_COUNT
/** CAN node_id of the first XDrive (configure the other board as node 1). */
#define APP_ODRIVE_DRIVE0_NODE_ID    0u
#define APP_ODRIVE_DRIVE1_NODE_ID    1u
#define APP_ODRIVE_NODE0_ID          APP_ODRIVE_DRIVE0_NODE_ID
#define APP_ODRIVE_NODE1_ID          APP_ODRIVE_DRIVE1_NODE_ID

/** Stale encoder snapshot: re-issue GET_ENCODER_ESTIMATES RTR. */
#define APP_ODRIVE_ENCODER_STALE_MS  80u

#define APP_BIAS_PERIOD_MS           10u
#define APP_WATCHDOG_PERIOD_MS       50u
#define APP_JETSON_PERIOD_MS         20u
#define APP_TELEMETRY_PERIOD_MS      40u   /* 25 Hz */

/* --- Wheel encoder (TIM2 ABZ) --- */
#ifndef WHEEL_ENCODER_CPR
#define WHEEL_ENCODER_CPR            (2097152u) /* 21-bit counts/rev */
#endif

#ifndef APP_ODRIVE_ENCODER_CPR
#define APP_ODRIVE_ENCODER_CPR       WHEEL_ENCODER_CPR
#endif

/* --- Control strategy (see control_strategy.h) --- */
#ifndef APP_CTRL_STRATEGY_DEFAULT
#define APP_CTRL_STRATEGY_DEFAULT    0  /* CTRL_STRATEGY_DUAL_PID */
#endif

/* --- Control references --- */
#define APP_CTRL_PITCH_REF_RAD       0.0f
#define APP_CTRL_VEL_REF_TURNS_S     0.0f

/** Cut drive if |pitch| exceeds this (rad). ~45 deg default. */
#ifndef APP_CTRL_PITCH_FAILSAFE_RAD
#define APP_CTRL_PITCH_FAILSAFE_RAD  (0.785398163f)
#endif

/* Balance: u += Kp*(pitch_ref - pitch) - Kd*pitch_rate (see pid_controller) */
#ifndef APP_CTRL_PITCH_KP
#define APP_CTRL_PITCH_KP            3.0f
#endif
#ifndef APP_CTRL_PITCH_KI
#define APP_CTRL_PITCH_KI            0.0f
#endif
#ifndef APP_CTRL_PITCH_KD
#define APP_CTRL_PITCH_KD            0.08f
#endif

/* Forward speed on ABZ wheel velocity */
#ifndef APP_CTRL_VEL_KP
#define APP_CTRL_VEL_KP              1.0f
#endif
#ifndef APP_CTRL_VEL_KI
#define APP_CTRL_VEL_KI              0.2f
#endif
#ifndef APP_CTRL_VEL_KD
#define APP_CTRL_VEL_KD              0.0f
#endif

/** Max wheel command magnitude (turn/s) sent to ODrive. */
#ifndef APP_CTRL_CMD_MAX_TURNS_S
#define APP_CTRL_CMD_MAX_TURNS_S     8.0f
#endif

/* Linear strategy (F407-style state feedback, turn/s out). */
#ifndef APP_CTRL_LINEAR_K_PITCH
#define APP_CTRL_LINEAR_K_PITCH          12.0f
#endif
#ifndef APP_CTRL_LINEAR_K_PITCH_RATE
#define APP_CTRL_LINEAR_K_PITCH_RATE     0.5f
#endif
#ifndef APP_CTRL_LINEAR_K_VEL
#define APP_CTRL_LINEAR_K_VEL            2.0f
#endif
#ifndef APP_CTRL_LINEAR_OUTPUT_ALPHA
#define APP_CTRL_LINEAR_OUTPUT_ALPHA     0.8f
#endif

/* Cascade: outer vel PID output added to pitch_ref (rad). */
#ifndef APP_CTRL_CASCADE_VEL_KP
#define APP_CTRL_CASCADE_VEL_KP          0.15f
#endif
#ifndef APP_CTRL_CASCADE_VEL_KI
#define APP_CTRL_CASCADE_VEL_KI          0.02f
#endif
#ifndef APP_CTRL_CASCADE_VEL_KD
#define APP_CTRL_CASCADE_VEL_KD          0.0f
#endif
#ifndef APP_CTRL_CASCADE_PITCH_REF_MAX_RAD
#define APP_CTRL_CASCADE_PITCH_REF_MAX_RAD  (0.35f)
#endif

/* --- IMU selection (compile-time) --- */
typedef enum {
    APP_IMU_NONE = 0,
    APP_IMU_ICM45686,
    APP_IMU_BMI323,
    APP_IMU_MPU6050,
} app_imu_type_t;

#ifndef APP_IMU_SELECTED
#define APP_IMU_SELECTED             APP_IMU_ICM45686
#endif

/**
 * 1 when SPI1 TX/RX DMA is configured in CubeMX (recommended).
 * 0 = HAL SPI IT fallback until DMA is enabled.
 */
#ifndef APP_IMU_SPI_USE_DMA
#define APP_IMU_SPI_USE_DMA          0
#endif

/** Software CS for SPI1 IMU (configure PA4 or your board net in CubeMX). */
#ifndef APP_IMU_SPI_CS_PORT
#define APP_IMU_SPI_CS_PORT          GPIOA
#endif
#ifndef APP_IMU_SPI_CS_PIN
#define APP_IMU_SPI_CS_PIN           GPIO_PIN_4
#endif

#endif /* APP_CONFIG_H */
