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
#define APP_ODRIVE_TORQUE_TX_ENABLED 1
#endif

/**
 * 1 = skip ODrive CAN boot in main() (bench telemetry, motors not required).
 * FreeRTOS + UART4 BalanceFrame still run. Set 0 before on-robot balance tests.
 * When 1, SET_CONTROLLER_MODES(torque) is never sent — ODrive stays on flash mode
 * (often velocity), which matches "cmd saturates but wheels brake to stop".
 */
#ifndef APP_TELEMETRY_BENCH_MODE
#define APP_TELEMETRY_BENCH_MODE       0
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
#ifndef APP_IMU_G_MPS2
#define APP_IMU_G_MPS2               9.80665f
#endif
/**
 * Startup cal: |a_up| < sin(π/12)·g for 3 s (a_up = IMU X) and 8 < ‖a‖ < 11.
 * Then stand vertical for the 20 s average. A new cal overwrites flash.
 */
#ifndef APP_IMU_OFFSET_LIE_SIN
#define APP_IMU_OFFSET_LIE_SIN       0.258819045f /* sin(π/12) */
#endif
#ifndef APP_IMU_OFFSET_NORM_MIN_MPS2
#define APP_IMU_OFFSET_NORM_MIN_MPS2  8.0f
#endif
#ifndef APP_IMU_OFFSET_NORM_MAX_MPS2
#define APP_IMU_OFFSET_NORM_MAX_MPS2  11.0f
#endif
#ifndef APP_IMU_OFFSET_UPRIGHT_COS
#define APP_IMU_OFFSET_UPRIGHT_COS   0.965925826f /* cos(π/12) */
#endif
#ifndef APP_IMU_OFFSET_LIE_HOLD_MS
#define APP_IMU_OFFSET_LIE_HOLD_MS   3000u
#endif
#ifndef APP_IMU_OFFSET_UPRIGHT_SKIP_MS
#define APP_IMU_OFFSET_UPRIGHT_SKIP_MS 200u
#endif
#ifndef APP_IMU_OFFSET_PULSE_NM
#define APP_IMU_OFFSET_PULSE_NM      0.02f
#endif
#ifndef APP_IMU_OFFSET_PULSE_MS
#define APP_IMU_OFFSET_PULSE_MS      100u
#endif
#ifndef APP_IMU_OFFSET_PULSE_GAP_MS
#define APP_IMU_OFFSET_PULSE_GAP_MS  500u
#endif
#ifndef APP_IMU_OFFSET_SETTLE_MS
#define APP_IMU_OFFSET_SETTLE_MS     2000u
#endif
#ifndef APP_IMU_OFFSET_SAMPLE_COUNT
#define APP_IMU_OFFSET_SAMPLE_COUNT  1000u
#endif
#ifndef APP_IMU_OFFSET_SAMPLE_SPAN_MS
#define APP_IMU_OFFSET_SAMPLE_SPAN_MS 20000u
#endif
#define APP_IMU_OFFSET_SAMPLE_PERIOD_MS \
    (APP_IMU_OFFSET_SAMPLE_SPAN_MS / APP_IMU_OFFSET_SAMPLE_COUNT)
/** Last 128 KB sector of Bank2 — kept out of the firmware FLASH region. */
#ifndef APP_IMU_OFFSET_FLASH_ADDR
#define APP_IMU_OFFSET_FLASH_ADDR    0x081E0000UL
#endif
#ifndef APP_IMU_OFFSET_FLASH_SECTOR
#define APP_IMU_OFFSET_FLASH_SECTOR  7u
#endif
/** Reject stored/live offsets outside a real rest-bias range (avoids failsafe). */
#ifndef APP_IMU_OFFSET_ACCEL_MAX_MPS2
#define APP_IMU_OFFSET_ACCEL_MAX_MPS2  0.80f
#endif
#ifndef APP_IMU_OFFSET_GYRO_MAX_RADS
#define APP_IMU_OFFSET_GYRO_MAX_RADS   0.15f
#endif
#ifndef APP_IMU_OFFSET_CAL_TIMEOUT_MS
#define APP_IMU_OFFSET_CAL_TIMEOUT_MS  90000u
#endif
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
/* TIM2/TIM4 use TIM_ENCODERMODE_TI1 (= x2: count both edges of CH1 only).
 * Full x4 would be TIM_ENCODERMODE_TI12 → CPR = PPR * 4.
 * MT6835 ABZ @ 16384 PPR → 16384 * 2 = 32768 counts/rev. */
#define WHEEL_ENCODER_CPR            (32768u)
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
#define APP_CTRL_STRATEGY_DEFAULT    0  /* CTRL_STRATEGY_FF_CASCADE */
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

/**
 * Plant (measured): m=1.335 kg, COM h=0.145 m, gear motor:wheel = 3:16 (τ_w/τ_m = 16/3).
 * Gravity stiffness at axle: m·g·h ≈ 1.90 Nm/rad.
 * Same cmd on both motors → motor FF for full gravity cancel:
 *   K_ff_full = m·g·h / (2·16/3) ≈ 0.178 Nm (shaft).
 * At 5°: ~0.0155 Nm/motor gravity; keep cmd_max ≈ 3× that for catch margin.
 */
#ifndef APP_CTRL_CMD_MAX_TORQUE_NM
#define APP_CTRL_CMD_MAX_TORQUE_NM     0.080f  /* run3: less violent while hunting gains */
#endif

/* Cascade vel → pitch_ref:
 *   e = v_ref - v
 *   e_f = α·e_f + (1-α)·e     (leaky I; lower α = stronger decay)
 *   pitch_cmd = Kp·e + Kema·e_f + Kd·v̇ - Kacc·v̇_ref
 *   pitch_trim = -pitch_cmd
 *
 * Checkpoint 2026-08-20 (live hold, user restore after neutral tune):
 *   kp=0.08  kd=0.008  ema_α=0.8  ema_kp=0.03
 *   accel_kp=0.04  slew=80  pitch_ref_max≈15°
 */
#ifndef APP_CTRL_CASCADE_VEL_KP
#define APP_CTRL_CASCADE_VEL_KP          0.08f
#endif
#ifndef APP_CTRL_CASCADE_VEL_KD
#define APP_CTRL_CASCADE_VEL_KD          0.008f /* damp on filtered v̇ */
#endif
#ifndef APP_CTRL_CASCADE_VEL_ERR_EMA_ALPHA
#define APP_CTRL_CASCADE_VEL_ERR_EMA_ALPHA  0.80f
#endif
#ifndef APP_CTRL_CASCADE_VEL_EMA_KP
#define APP_CTRL_CASCADE_VEL_EMA_KP         0.03f
#endif
/** Lean FF from commanded speed ramp: θ ≈ (gear·2π·r/g)·v̇_ref ≈ 0.0048·v̇_ref. */
#ifndef APP_CTRL_CASCADE_VEL_ACCEL_KP
#define APP_CTRL_CASCADE_VEL_ACCEL_KP       0.04f /* rad / (turn/s²) — not scaled (no encoder) */
#endif
#ifndef APP_CTRL_CASCADE_PITCH_REF_MAX_RAD
#define APP_CTRL_CASCADE_PITCH_REF_MAX_RAD  (0.26f) /* ~15 deg */
#endif
/** Slew-limit |d vel_ref / dt| (motor turn/s²). 0 = off. 80 ≈ 0.5 s to 2 m/s. */
#ifndef APP_CTRL_VEL_REF_SLEW_TURNS_S2
#define APP_CTRL_VEL_REF_SLEW_TURNS_S2  80.0f
#endif

/* FF + PD (+ wheel-speed torque damp).
 * Baseline: ODrive/.../BALANCE_BASELINE.md + logs/krate013_db0017.csv
 * (k_rate 0.013 + gated deadband 0.0017). */
#ifndef APP_CTRL_FF_GRAV_K
#define APP_CTRL_FF_GRAV_K               0.14f
#endif
#ifndef APP_CTRL_FF_FB_K_PITCH
#define APP_CTRL_FF_FB_K_PITCH           0.055f
#endif
#ifndef APP_CTRL_FF_FB_K_RATE
#define APP_CTRL_FF_FB_K_RATE            0.013f
#endif
/** u += Kv·(v_ref - v); negative Kv brakes in current sign convention. */
#ifndef APP_CTRL_FF_FB_K_VEL
#define APP_CTRL_FF_FB_K_VEL             (-0.0005f)
#endif
#ifndef APP_CTRL_FF_FB_K_VEL_MAX_NM
#define APP_CTRL_FF_FB_K_VEL_MAX_NM      0.010f
#endif
/** Output LPF: higher = softer (cuts vib chatter). */
#ifndef APP_CTRL_FF_OUTPUT_ALPHA
#define APP_CTRL_FF_OUTPUT_ALPHA         0.50f
#endif
/**
 * hypothesis_lab build: experimental friction / debug (see docs/HYPOTHESIS_LAB.md).
 * Set 0 before merging back to production tune branches.
 */
#ifndef APP_HYPOTHESIS_LAB
#define APP_HYPOTHESIS_LAB               1
#endif

/**
 * Coulomb: u += sign(u)*D when near upright & slow (tunable over telemetry).
 * friction_mode=1 replaces this with static/kinetic two-level comp (v11).
 */
#ifndef APP_CTRL_TORQUE_DEADBAND_NM
#define APP_CTRL_TORQUE_DEADBAND_NM      0.004f
#endif
#ifndef APP_CTRL_TORQUE_DEADBAND_PITCH_MAX_RAD
#if APP_HYPOTHESIS_LAB
#define APP_CTRL_TORQUE_DEADBAND_PITCH_MAX_RAD  0.05f /* gate friction near upright */
#else
#define APP_CTRL_TORQUE_DEADBAND_PITCH_MAX_RAD  0.0f /* 0=user live; use 0.05 to gate */
#endif
#endif
#ifndef APP_CTRL_TORQUE_DEADBAND_RATE_MAX_RADS
#define APP_CTRL_TORQUE_DEADBAND_RATE_MAX_RADS  0.30f
#endif

/** friction_mode: 0=legacy deadband, 1=static (|ω|≤ε) + kinetic (|ω|>ε). */
#ifndef APP_CTRL_FRICTION_MODE
#define APP_CTRL_FRICTION_MODE           (APP_HYPOTHESIS_LAB ? 1 : 0)
#endif
#ifndef APP_CTRL_FRICTION_STATIC_NM
#define APP_CTRL_FRICTION_STATIC_NM      0.0045f
#endif
#ifndef APP_CTRL_FRICTION_KINETIC_NM
#define APP_CTRL_FRICTION_KINETIC_NM     0.003f
#endif
#ifndef APP_CTRL_FRICTION_VEL_EPS_TURNS_S
#define APP_CTRL_FRICTION_VEL_EPS_TURNS_S  0.05f
#endif

/**
 * Motor-shaft accel P (near upright): alpha_ref = (u - c*sign(ω))/J,
 * dτ = Kα*(α_ref - α_meas), gated. J/c from free-wheel USB ident (avg L/R).
 * SHELVED 2026-08: keep alpha_kp=0 (see ODrive/.../BALANCE_BASELINE.md § Shelved).
 * Code kept for a possible later revisit; prefer ODrive bidirectional anticogging.
 */
#ifndef APP_CTRL_MOTOR_J_KG_M2
#define APP_CTRL_MOTOR_J_KG_M2               1.12e-5f
#endif
#ifndef APP_CTRL_MOTOR_FRICTION_C_NM
#define APP_CTRL_MOTOR_FRICTION_C_NM         0.0052f
#endif
#ifndef APP_CTRL_ALPHA_KP
#define APP_CTRL_ALPHA_KP                    0.0f
#endif
#ifndef APP_CTRL_ALPHA_MAX_NM
#define APP_CTRL_ALPHA_MAX_NM                0.002f
#endif
#ifndef APP_CTRL_ALPHA_PITCH_MAX_RAD
#define APP_CTRL_ALPHA_PITCH_MAX_RAD         0.05f
#endif
#ifndef APP_CTRL_ALPHA_RATE_MAX_RADS
#define APP_CTRL_ALPHA_RATE_MAX_RADS         0.30f
#endif
/** Gate off when |motor ω| (robot-frame turn/s) exceeds this; 0 = no vel gate. */
#ifndef APP_CTRL_ALPHA_VEL_MAX_TURNS_S
#define APP_CTRL_ALPHA_VEL_MAX_TURNS_S       8.0f
#endif
/** EMA on α_meas: higher = smoother / more lag (0..1). */
#ifndef APP_CTRL_ALPHA_LPF
#define APP_CTRL_ALPHA_LPF                   0.80f
#endif

/**
 * Outer loop modes (param outer_mode, mutually exclusive):
 *   0 = velocity: v_ref = vel_ref_turns_s (manual / teleop)
 *   1 = position: v_ref = clamp( +(Kp·x_err + Kema·e_f) - Kd·v , ±v_max )
 *                 then cascade_vel → pitch_ref (needs cascade_vel_kp > 0;
 *                 pitch_trim = -PID so +v_err leans the correcting way)
 * e_f = α·e_f + (1-α)·x_err. Direct pitch trim from x is retired (pos_pitch_* unused).
 * SET pos_reset=1 (or enter mode 1) to zero x / e_f at current pose.
 */
#ifndef APP_CTRL_OUTER_MODE_DEFAULT
#define APP_CTRL_OUTER_MODE_DEFAULT          0u  /* APP_CTRL_OUTER_MODE_VEL */
#endif
#ifndef APP_WHEEL_RADIUS_M
#define APP_WHEEL_RADIUS_M                   0.04f
#endif
/** Motor:wheel gear (same as plant note: τ_w/τ_m = 16/3). */
#ifndef APP_WHEEL_GEAR_MOTOR
#define APP_WHEEL_GEAR_MOTOR                 3u
#endif
#ifndef APP_WHEEL_GEAR_WHEEL
#define APP_WHEEL_GEAR_WHEEL                 16u
#endif

/**
 * Antipatinage (wheel lift): see docs/ANTIPATINAGE.md.
 * APP_CTRL_ANTIPATINAGE_ENABLE=0 disables detection (pass-through).
 */
#ifndef APP_CTRL_ANTIPATINAGE_ENABLE
#define APP_CTRL_ANTIPATINAGE_ENABLE         0
#endif
/** 0 = disable SYNC_L/R (single-wheel lift) only; BOTH_AIR stays active. */
#ifndef APP_ANTIPAT_SYNC_ENABLE
#define APP_ANTIPAT_SYNC_ENABLE              0
#endif
#ifndef APP_ANTIPAT_TRACK_WIDTH_M
#define APP_ANTIPAT_TRACK_WIDTH_M            0.16f  /* 16 cm empattement */
#endif
#ifndef APP_ANTIPAT_TAU_MIN_NM
#define APP_ANTIPAT_TAU_MIN_NM               0.001f
#endif
#ifndef APP_ANTIPAT_ETA_ON
#define APP_ANTIPAT_ETA_ON                   8000.0f /* rad/s^2 per Nm; tune bench */
#endif
#ifndef APP_ANTIPAT_ETA_OFF
#define APP_ANTIPAT_ETA_OFF                  1000.0f
#endif
/** BOTH_AIR exit fallback when |α| drops (loaded contact); 0 = η-only exit. */
#ifndef APP_ANTIPAT_ALPHA_CONTACT_MAX_RADS2
#define APP_ANTIPAT_ALPHA_CONTACT_MAX_RADS2  500.0f
#endif
/** In BOTH_AIR: small τ_both + spinning ω ⇒ still airborne (block false recontact). */
#ifndef APP_ANTIPAT_TAU_BOTH_STEADY_AIR_NM
#define APP_ANTIPAT_TAU_BOTH_STEADY_AIR_NM   0.004f
#endif
#ifndef APP_ANTIPAT_OMEGA_AIR_MIN_TURNS_S
#define APP_ANTIPAT_OMEGA_AIR_MIN_TURNS_S    0.25f
#endif
#ifndef APP_ANTIPAT_K_DOM
#define APP_ANTIPAT_K_DOM                    1.4f
#endif
#ifndef APP_ANTIPAT_EPS_ABS_RADS
#define APP_ANTIPAT_EPS_ABS_RADS             0.08f  /* |psi_dot - psi_kin| */
#endif
#ifndef APP_ANTIPAT_K_REL
#define APP_ANTIPAT_K_REL                    0.35f
#endif
#ifndef APP_ANTIPAT_K_OFF
#define APP_ANTIPAT_K_OFF                    0.65f
#endif
#ifndef APP_ANTIPAT_T_ON_MS
#define APP_ANTIPAT_T_ON_MS                  60u
#endif
#ifndef APP_ANTIPAT_T_OFF_MS
#define APP_ANTIPAT_T_OFF_MS                 100u
#endif
#ifndef APP_ANTIPAT_T_ON_BOTH_MS
#define APP_ANTIPAT_T_ON_BOTH_MS             15u
#endif
#ifndef APP_ANTIPAT_T_OFF_BOTH_MS
#define APP_ANTIPAT_T_OFF_BOTH_MS            100u
#endif
#ifndef APP_ANTIPAT_T_MA_MS
#define APP_ANTIPAT_T_MA_MS                  100u
#endif
#ifndef APP_ANTIPAT_T_RECOVER_MS
#define APP_ANTIPAT_T_RECOVER_MS             150u
#endif
#ifndef APP_ANTIPAT_U_MIN_NM
#define APP_ANTIPAT_U_MIN_NM                 0.006f
#endif
#ifndef APP_ANTIPAT_PITCH_RATE_MIN_RADS
#define APP_ANTIPAT_PITCH_RATE_MIN_RADS      0.08f
#endif
#ifndef APP_ANTIPAT_K_SYNC
#define APP_ANTIPAT_K_SYNC                   0.0025f /* Nm/(turn/s) */
#endif
#ifndef APP_ANTIPAT_TAU_SYNC_MAX_NM
#define APP_ANTIPAT_TAU_SYNC_MAX_NM          0.015f
#endif
#ifndef APP_ANTIPAT_K_BOTH_V
#define APP_ANTIPAT_K_BOTH_V                 0.003f
#endif
#ifndef APP_ANTIPAT_TAU_BOTH_MAX_NM
#define APP_ANTIPAT_TAU_BOTH_MAX_NM          0.012f
#endif
#ifndef APP_CTRL_POS_KP
#define APP_CTRL_POS_KP                      0.0f   /* (turn/s) / m */
#endif
#ifndef APP_CTRL_POS_KD
#define APP_CTRL_POS_KD                      0.0f   /* (turn/s) / (turn/s) */
#endif
#ifndef APP_CTRL_POS_X_REF_M
#define APP_CTRL_POS_X_REF_M                 0.0f
#endif
#ifndef APP_CTRL_POS_V_MAX_TURNS_S
#define APP_CTRL_POS_V_MAX_TURNS_S           0.5f
#endif
/** EMA on x_err: higher = longer memory (0 = e_f tracks x_err instantly). */
#ifndef APP_CTRL_POS_ERR_EMA_ALPHA
#define APP_CTRL_POS_ERR_EMA_ALPHA           0.99f
#endif
/** Position-mode gain on EMA error into v_ref ((turn/s) / m of e_f). 0 = off. */
#ifndef APP_CTRL_POS_EMA_KP
#define APP_CTRL_POS_EMA_KP                  0.0f
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
/**
 * Complementary filter: pitch = α·(pitch+ωΔt) + (1-α)·pitch_accel.
 * 0.999 @ 1 kHz ≈ 0.16 Hz accel. Gyro/accel rest bias: lie-down cal in task_bias.
 */
#ifndef APP_IMU_COMPLEMENTARY_ALPHA
#define APP_IMU_COMPLEMENTARY_ALPHA  0.999f
#endif
/** Reject accel tilt when |‖a‖ − g| exceeds this (m/s²); use gyro-only update. */
#ifndef APP_IMU_ACCEL_NORM_TOL_MPS2
#define APP_IMU_ACCEL_NORM_TOL_MPS2  3.0f
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
#define APP_IMU_PITCH_ACCEL_UP_AXIS       0  /* also lying-cal |a_up| test */
#endif
#ifndef APP_IMU_PITCH_GYRO_AXIS
#define APP_IMU_PITCH_GYRO_AXIS           2
#endif
#ifndef APP_IMU_PITCH_GYRO_SIGN
#define APP_IMU_PITCH_GYRO_SIGN           1.0f
#endif
/** Yaw rate about vertical (up) axis — yaw rate loop feedback. */
#ifndef APP_IMU_YAW_GYRO_AXIS
#define APP_IMU_YAW_GYRO_AXIS             0  /* same as PITCH_ACCEL_UP_AXIS */
#endif
#ifndef APP_IMU_YAW_GYRO_SIGN
#define APP_IMU_YAW_GYRO_SIGN             -1.0f
#endif

/**
 * Yaw rate hold (ff_cascade): track gyro yaw rate (not heading angle).
 *   u_yaw = clamp( Kp·(ψ̇_ref − ψ̇) − Kd·dψ̇/dt , ±τ_max )
 *   τ_L = u − u_yaw,  τ_R = u + u_yaw
 * Telemetry names kept for compat: heading_ref_rad = ψ̇_ref (rad/s).
 * Gains: Kp Nm/(rad/s), Kd Nm/(rad/s²). Defaults: Kp=0.03, Kd=0.005, τ_max=0.01.
 * SET heading_reset=1 → ψ̇_ref ← 0 (integrated ψ display also zeroed).
 * heading_inc: ψ̇_ref = value (signed, rad/s, clamp ±YAW_RATE_REF_MAX).
 * heading_dec: ψ̇_ref = −value (old UI). heading_ref_rad is the same store.
 */
#ifndef APP_CTRL_YAW_RATE_REF_MAX_RADS
#define APP_CTRL_YAW_RATE_REF_MAX_RADS     2.0f
#endif
#ifndef APP_CTRL_HEADING_KP
#define APP_CTRL_HEADING_KP                  0.000f  /* Nm / (rad/s) */
#endif
#ifndef APP_CTRL_HEADING_KD
#define APP_CTRL_HEADING_KD                  0.0000f /* Nm / (rad/s²) on dψ̇/dt */
#endif
#ifndef APP_CTRL_HEADING_REF_RAD
#define APP_CTRL_HEADING_REF_RAD             0.0f   /* ψ̇_ref rad/s */
#endif
#ifndef APP_CTRL_HEADING_TORQUE_MAX_NM
#define APP_CTRL_HEADING_TORQUE_MAX_NM       0.01f
#endif
/** EMA on ψ̇ before yaw loop: y = α·y + (1−α)·ψ̇_raw ; 0 = off, higher = smoother. */
#ifndef APP_CTRL_YAW_RATE_LPF_ALPHA
#define APP_CTRL_YAW_RATE_LPF_ALPHA          0.99f
#endif

#endif /* APP_CONFIG_H */
