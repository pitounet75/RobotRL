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

/**
 * 1 = send SET_INPUT_TORQUE to ODrives from the control loop.
 * 0 = always transmit 0 Nm (control + telemetry still run; use for telemetry-only tests).
 * Also toggle at runtime via g_app_odrive_torque_tx_enabled (debugger).
 */
#ifndef APP_ODRIVE_TORQUE_TX_ENABLED
#define APP_ODRIVE_TORQUE_TX_ENABLED 0
#endif

/**
 * 1 = skip ODrive CAN boot in main() (bench telemetry, motors not required).
 * FreeRTOS + UART4 BalanceFrame still run. Set 0 before on-robot balance tests.
 */
#ifndef APP_TELEMETRY_BENCH_MODE
#define APP_TELEMETRY_BENCH_MODE       1
#endif

/*
 * Two separate motor drives on the CAN bus (e.g. 2x MKS XDrive Mini).
 * Each board has a single active axis (axis0); CAN Simple uses one node_id per board.
 * This is NOT one ODrive with axis0 + axis1 on the same node.
 */
#define APP_ODRIVE_DRIVE_COUNT       2u
#define APP_ODRIVE_NODE_COUNT        APP_ODRIVE_DRIVE_COUNT
/** CAN node_id per wheel. Separate buses allow both ODrives to keep the same firmware/config. */
#define APP_ODRIVE_LEFT_NODE_ID      0u
#define APP_ODRIVE_RIGHT_NODE_ID     0u
#define APP_ODRIVE_DRIVE0_NODE_ID    APP_ODRIVE_LEFT_NODE_ID
#define APP_ODRIVE_DRIVE1_NODE_ID    APP_ODRIVE_RIGHT_NODE_ID
#define APP_ODRIVE_NODE0_ID          APP_ODRIVE_DRIVE0_NODE_ID
#define APP_ODRIVE_NODE1_ID          APP_ODRIVE_DRIVE1_NODE_ID

/** Stale encoder snapshot: re-issue GET_ENCODER_ESTIMATES RTR. */
#define APP_ODRIVE_ENCODER_STALE_MS  80u

/**
 * ODrive anticogging (cogging map applied inside the drive as torque offset in Nm).
 * Works in torque mode — see ODrive controller.cpp (independent of control_mode).
 *
 * 1 = enabled: ODrive flash must have anticogging_enabled=true, pre_calibrated=true,
 *     anticogging_valid=true (e.g. 283_anticogging_calibrated.txt + save_configuration).
 * 0 = disabled: set anticogging_enabled=false in odrivetool and save_configuration().
 *
 * Not toggled over CAN on stock ODrive firmware; this flag documents intent and is
 * exposed at startup as g_odrive_anticogging_enabled_cfg for the debugger.
 */
#ifndef APP_ODRIVE_ANTICOGGING_ENABLED
#define APP_ODRIVE_ANTICOGGING_ENABLED       1
#endif

#define APP_BIAS_PERIOD_MS           10u
#define APP_WATCHDOG_PERIOD_MS       50u
#define APP_JETSON_PERIOD_MS         20u
#define APP_TELEMETRY_PERIOD_MS      2u    /* 500 Hz BalanceFrame (matches control loop) */

/** Wait for ESP32 "READY\n" on UART4 RX before streaming (common PSU boot). */
#ifndef APP_TELEMETRY_WAIT_BRIDGE_READY
#define APP_TELEMETRY_WAIT_BRIDGE_READY 1
#endif

#ifndef APP_TELEMETRY_BRIDGE_READY_TIMEOUT_MS
#define APP_TELEMETRY_BRIDGE_READY_TIMEOUT_MS 5000u
#endif

/** Fallback fixed delay if bridge-ready handshake is disabled. */
#ifndef APP_TELEMETRY_STARTUP_DELAY_MS
#define APP_TELEMETRY_STARTUP_DELAY_MS 4000u
#endif

/** telemetry_send() uses ~512 B frame buffers; RX replies can use ~900 B. */
#ifndef APP_TELEMETRY_TASK_STACK_WORDS
#define APP_TELEMETRY_TASK_STACK_WORDS 768u
#endif

/* --- Local wheel encoders (TIM2 ABZ + TIM4 ABZ) --- */
#define APP_LOCAL_ENCODER_COUNT      2u

#ifndef WHEEL_ENCODER_CPR
#define WHEEL_ENCODER_CPR            (2500u) /* ABZ on wheel: counts per wheel revolution */
#endif

/** Exponential LPF on per-wheel vel_turns_s (0= off, 0.7-0.9 = light smoothing). */
#ifndef WHEEL_ENCODER_VEL_LPF_ALPHA
#define WHEEL_ENCODER_VEL_LPF_ALPHA  0.75f
#endif

/** ODrive axis encoder CPR (encoder.config.cpr), NOT the local ABZ CPR. */
#ifndef APP_ODRIVE_ENCODER_CPR
#define APP_ODRIVE_ENCODER_CPR       (2097152u) /* 2^21, typical ODrive/incr encoder */
#endif

/* --- Wheel hardware orientation (semantic +turn/s = robot forward) --- */
#ifndef APP_WHEEL_LEFT_ODRIVE_CMD_SIGN
#define APP_WHEEL_LEFT_ODRIVE_CMD_SIGN        -1
#endif
#ifndef APP_WHEEL_RIGHT_ODRIVE_CMD_SIGN
#define APP_WHEEL_RIGHT_ODRIVE_CMD_SIGN       1
#endif
#ifndef APP_WHEEL_LEFT_ODRIVE_FEEDBACK_SIGN
#define APP_WHEEL_LEFT_ODRIVE_FEEDBACK_SIGN   -1
#endif
#ifndef APP_WHEEL_RIGHT_ODRIVE_FEEDBACK_SIGN
#define APP_WHEEL_RIGHT_ODRIVE_FEEDBACK_SIGN   1
#endif
#ifndef APP_WHEEL_LEFT_ENCODER_SIGN
#define APP_WHEEL_LEFT_ENCODER_SIGN           -1
#endif
#ifndef APP_WHEEL_RIGHT_ENCODER_SIGN
#define APP_WHEEL_RIGHT_ENCODER_SIGN          1
#endif

/* --- Control strategy (see control_strategy.h) --- */
#ifndef APP_CTRL_STRATEGY_DEFAULT
#define APP_CTRL_STRATEGY_DEFAULT    3  /* CTRL_STRATEGY_FF_CASCADE */
#endif

/* --- Control references --- */
/** Level pitch target. Fine trim (e.g. 0.026f) optional; 1-2 deg bias is not enough to explain a fall alone. */
#ifndef APP_CTRL_PITCH_REF_RAD
#define APP_CTRL_PITCH_REF_RAD       0.0f
#endif
#define APP_CTRL_VEL_REF_TURNS_S     0.0f

/** Cut drive if |pitch| exceeds this (rad). ~45 deg default. */
#ifndef APP_CTRL_PITCH_FAILSAFE_RAD
#define APP_CTRL_PITCH_FAILSAFE_RAD  (0.785398163f)
#endif

/* Balance: u = Kp*(pitch_ref - pitch) - Kd*pitch_rate (see pid_controller).
 * Positive pitch = nose-down on this IMU mount; Kp/Kd > 0 for our wheel cmd signs. */
#ifndef APP_CTRL_PITCH_KP
#define APP_CTRL_PITCH_KP            0.01f
#endif
#ifndef APP_CTRL_PITCH_KI
#define APP_CTRL_PITCH_KI            0.0f
#endif
#ifndef APP_CTRL_PITCH_KD
#define APP_CTRL_PITCH_KD            0.0f
#endif

/* Forward speed on ABZ wheel velocity */
#ifndef APP_CTRL_VEL_KP
#define APP_CTRL_VEL_KP              0.0f
#endif
#ifndef APP_CTRL_VEL_KI
#define APP_CTRL_VEL_KI              0.0f
#endif
#ifndef APP_CTRL_VEL_KD
#define APP_CTRL_VEL_KD              0.0f
#endif

/** Max torque (Nm) at motor shaft sent to ODrive (~0.19 Nm at wheel, gear 15/80).
 * Align ODrive torque_lim to this value. */
#ifndef APP_CTRL_CMD_MAX_TORQUE_NM
#define APP_CTRL_CMD_MAX_TORQUE_NM     0.035f
#endif

/* Linear: u = Kθ·f(θ_ref−θ) − Kω·θ̇ + Kv·(v_ref−v), then output low-pass.
 * f(err): 0=err (default, torque mode), 1=atan(err). */
#ifndef APP_CTRL_LINEAR_THETA_FUNC
#define APP_CTRL_LINEAR_THETA_FUNC         0  /* 0=linear, 1=atan */
#endif
#ifndef APP_CTRL_LINEAR_K_PITCH
#define APP_CTRL_LINEAR_K_PITCH          0.01f
#endif
#ifndef APP_CTRL_LINEAR_K_PITCH_RATE
#define APP_CTRL_LINEAR_K_PITCH_RATE     0.0f
#endif
#ifndef APP_CTRL_LINEAR_K_VEL
#define APP_CTRL_LINEAR_K_VEL            0.0f
#endif
#ifndef APP_CTRL_LINEAR_OUTPUT_ALPHA
#define APP_CTRL_LINEAR_OUTPUT_ALPHA     0.0f
#endif

/* Cascade: outer vel PID output added to pitch_ref (rad). */
#ifndef APP_CTRL_CASCADE_VEL_KP
#define APP_CTRL_CASCADE_VEL_KP          0.0f
#endif
#ifndef APP_CTRL_CASCADE_VEL_KI
#define APP_CTRL_CASCADE_VEL_KI          0.00f
#endif
#ifndef APP_CTRL_CASCADE_VEL_KD
#define APP_CTRL_CASCADE_VEL_KD          0.0f
#endif
#ifndef APP_CTRL_CASCADE_PITCH_REF_MAX_RAD
#define APP_CTRL_CASCADE_PITCH_REF_MAX_RAD  (0.0f)
#endif

/* FF + cascade (strategy 3): catch early (|pitch| few deg), not at 15 deg then saturate.
 * v3: at |pitch|<5 deg + |rate|>0.3, mean cmd~0.001 Nm (P and D cancel). */
#ifndef APP_CTRL_FF_GRAV_K
#define APP_CTRL_FF_GRAV_K               (-0.06f)
#endif
#ifndef APP_CTRL_FF_FB_K_PITCH
#define APP_CTRL_FF_FB_K_PITCH           0.30f
#endif
#ifndef APP_CTRL_FF_FB_K_RATE
#define APP_CTRL_FF_FB_K_RATE            0.002f
#endif
/** 0 = no output lag; late catch was partly LPF + D term cancelling P at small angle. */
#ifndef APP_CTRL_FF_OUTPUT_ALPHA
#define APP_CTRL_FF_OUTPUT_ALPHA         0.15f
#endif

#ifndef APP_TELEMETRY_BAUD
#define APP_TELEMETRY_BAUD             921600u
#endif

/** 1 = UART4 TX/RX via HAL DMA (hdma_uart4_tx/rx in CubeMX, ReceiveToIdle_DMA + Transmit_DMA). */
#ifndef APP_TELEMETRY_UART4_USE_DMA
#define APP_TELEMETRY_UART4_USE_DMA    1
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
 * 1 when IMU SPI TX/RX DMA is configured in CubeMX (recommended).
 * 0 = blocking HAL SPI xfer from task context (same as icm45686 probe path).
 */
#ifndef APP_IMU_SPI_USE_DMA
#define APP_IMU_SPI_USE_DMA          1
#endif

/** Complementary filter gyro weight (1.0 = gyro only). Lower = faster accel correction, more lag. */
#ifndef APP_IMU_COMPLEMENTARY_ALPHA
#define APP_IMU_COMPLEMENTARY_ALPHA  0.92f
#endif

/** Retry interval when the IMU is not ready at first boot. */
#ifndef APP_IMU_INIT_RETRY_MS
#define APP_IMU_INIT_RETRY_MS        100u
#endif

/** Software CS for SPI3 IMU. */
#ifndef APP_IMU_SPI_CS_PORT
#define APP_IMU_SPI_CS_PORT          GPIOE
#endif
#ifndef APP_IMU_SPI_CS_PIN
#define APP_IMU_SPI_CS_PIN           GPIO_PIN_6
#endif

/** Optional debug read of the IMU SPI MISO idle state. */
#ifndef APP_IMU_SPI_MISO_PORT
#define APP_IMU_SPI_MISO_PORT        GPIOC
#endif
#ifndef APP_IMU_SPI_MISO_PIN
#define APP_IMU_SPI_MISO_PIN         GPIO_PIN_11
#endif

/* IMU axes/signs used to compute robot pitch. */
/* Chip axes aligned with robot frame; pitch = atan2f(-accel[forward], accel[up]). */
#ifndef APP_IMU_PITCH_ACCEL_FORWARD_AXIS
#define APP_IMU_PITCH_ACCEL_FORWARD_AXIS  1
#endif
#ifndef APP_IMU_PITCH_ACCEL_UP_AXIS
#define APP_IMU_PITCH_ACCEL_UP_AXIS       0
#endif
#ifndef APP_IMU_PITCH_GYRO_AXIS
#define APP_IMU_PITCH_GYRO_AXIS           2
#endif
#ifndef APP_IMU_PITCH_GYRO_SIGN
#define APP_IMU_PITCH_GYRO_SIGN           1.0f
#endif

#endif /* APP_CONFIG_H */
