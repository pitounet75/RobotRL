#pragma once

/**
 * One-axis hardware check: MKS FS2804 (12N14P, 7 pp) + MT6835 ABZ.
 * PWM pins match ESP32FOC. Encoder is quadrature, not SPI.
 */

#ifndef MOTOR_AXIS
#define MOTOR_AXIS 0 /* 0 = left PWM, 1 = right PWM */
#endif

/*
 * Board silk (connector pin / GPIO):
 *   motor 0: A 30/IO18  B 31/IO19  Z 36/IO22
 *   motor 1: A 29/IO5   B 37/IO23  Z 33/IO21
 * ENC_SWAP_AB=1 exchanges A/B into the PCNT (count sign) without rewiring.
 */
#ifndef ENC_SWAP_AB
#define ENC_SWAP_AB 0
#endif

#if MOTOR_AXIS == 0
#ifndef ENC_PIN_A
#define ENC_PIN_A 18
#endif
#ifndef ENC_PIN_B
#define ENC_PIN_B 19
#endif
#ifndef ENC_PIN_Z
#define ENC_PIN_Z 22
#endif
#else
#ifndef ENC_PIN_A
#define ENC_PIN_A 5
#endif
#ifndef ENC_PIN_B
#define ENC_PIN_B 23
#endif
#ifndef ENC_PIN_Z
#define ENC_PIN_Z 21
#endif
#endif

/** MT6835 Z is idle-low, pulse-high. FALLING if the pulse is inverted. */
#ifndef ENC_Z_EDGE
#define ENC_Z_EDGE RISING
#endif

#if ENC_SWAP_AB
#define ENC_PCNT_PIN_A ENC_PIN_B
#define ENC_PCNT_PIN_B ENC_PIN_A
#else
#define ENC_PCNT_PIN_A ENC_PIN_A
#define ENC_PCNT_PIN_B ENC_PIN_B
#endif
/**
 * Must match the MT6835 ABZ_PPR register (ESP32MT6835Setup default = 16384).
 * PCNT full-quad → CPR = 4 * PPR.
 */
#ifndef ENC_PPR
#define ENC_PPR 16384u
#endif

#ifndef ENC_PCNT_UNIT
#if MOTOR_AXIS == 0
#define ENC_PCNT_UNIT 0
#else
#define ENC_PCNT_UNIT 1
#endif
#endif

/** Glitch filter in APB cycles (0 = off, max 1023). 250 ≈ 3 µs @ 80 MHz. */
#ifndef ENC_PCNT_FILTER
#define ENC_PCNT_FILTER 250
#endif

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

/** Shared BL9342 EN (12 V gate-drive). Low at reset — GPIO12 is a strap. */
#ifndef FOC_PIN_MEN
#define FOC_PIN_MEN 12
#endif

/** Open-loop Z search, mechanical revolutions per second. */
#ifndef Z_SEARCH_RPS
#define Z_SEARCH_RPS 1.0f
#endif
#ifndef Z_SEARCH_TURNS
#define Z_SEARCH_TURNS 2.0f
#endif

#ifndef FOC_POLE_PAIRS
#define FOC_POLE_PAIRS 7
#endif

#ifndef FOC_VBUS
#define FOC_VBUS 8.4f
#endif

#ifndef FOC_VOLTAGE_LIMIT
#define FOC_VOLTAGE_LIMIT 2.0f
#endif
#ifndef FOC_VOLTAGE_ALIGN
#define FOC_VOLTAGE_ALIGN 2.0f
#endif
#ifndef FOC_PWM_HZ
#define FOC_PWM_HZ 20000u
#endif

#ifndef FOC_VEL_P
#define FOC_VEL_P 0.08f
#endif
#ifndef FOC_VEL_I
#define FOC_VEL_I 0.25f
#endif
#ifndef FOC_VEL_D
#define FOC_VEL_D 0.0f
#endif
#ifndef FOC_VEL_LPF
#define FOC_VEL_LPF 0.02f
#endif
#ifndef FOC_VEL_RAMP
#define FOC_VEL_RAMP 5.0f
#endif
#ifndef FOC_VEL_LIMIT
#define FOC_VEL_LIMIT 80.0f
#endif

/** FOC task rate. Floor 8 kHz; climb toward 16 kHz while dt_max < 0.7 T. */
#ifndef FOC_LOOP_HZ
#define FOC_LOOP_HZ 8000u
#endif
#ifndef FOC_LOOP_HZ_MIN
#define FOC_LOOP_HZ_MIN 4000u
#endif
#ifndef FOC_LOOP_HZ_MAX
#define FOC_LOOP_HZ_MAX 16000u
#endif

#ifndef FOC_I_LIM
#define FOC_I_LIM 0.6f
#endif
#ifndef FOC_CURR_P
#define FOC_CURR_P 0.15f
#endif
#ifndef FOC_CURR_I
#define FOC_CURR_I 8.0f
#endif
#ifndef FOC_CURR_LPF
#define FOC_CURR_LPF 0.003f
#endif
#ifndef FOC_ANGLE_P
#define FOC_ANGLE_P 8.0f
#endif

/** Inline shunts: R010 × 50. ADC1 only (Wi‑Fi-safe). Mid-rail ~1.65 V. */
#ifndef CS_SHUNT_OHM
#define CS_SHUNT_OHM 0.01f
#endif
#ifndef CS_AMP_GAIN
#define CS_AMP_GAIN 50.0f
#endif
#if MOTOR_AXIS == 0
#ifndef CS_PIN_A
#define CS_PIN_A 39
#endif
#ifndef CS_PIN_B
#define CS_PIN_B 36
#endif
#else
#ifndef CS_PIN_A
#define CS_PIN_A 35
#endif
#ifndef CS_PIN_B
#define CS_PIN_B 34
#endif
#endif

#ifndef ACOG_BINS_MAX
#define ACOG_BINS_MAX 3600u
#endif
#ifndef ACOG_BINS_DEFAULT
#define ACOG_BINS_DEFAULT 3600u
#endif
#ifndef ACOG_SETTLE_POS
#define ACOG_SETTLE_POS 0.02f
#endif
#ifndef ACOG_SETTLE_VEL
#define ACOG_SETTLE_VEL 0.3f
#endif
#ifndef ACOG_SETTLE_S
#define ACOG_SETTLE_S 0.015f
#endif

#if MOTOR_AXIS == 0
#define HWCHK_AXIS_CHAR 'L'
#define HWCHK_PIN_UH FOC_PIN_L_UH
#define HWCHK_PIN_VH FOC_PIN_L_VH
#define HWCHK_PIN_WH FOC_PIN_L_WH
#define HWCHK_PIN_OTHER_UH FOC_PIN_R_UH
#define HWCHK_PIN_OTHER_VH FOC_PIN_R_VH
#define HWCHK_PIN_OTHER_WH FOC_PIN_R_WH
#else
#define HWCHK_AXIS_CHAR 'R'
#define HWCHK_PIN_UH FOC_PIN_R_UH
#define HWCHK_PIN_VH FOC_PIN_R_VH
#define HWCHK_PIN_WH FOC_PIN_R_WH
#define HWCHK_PIN_OTHER_UH FOC_PIN_L_UH
#define HWCHK_PIN_OTHER_VH FOC_PIN_L_VH
#define HWCHK_PIN_OTHER_WH FOC_PIN_L_WH
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif
#if MOTOR_AXIS == 0
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "hwchk-left"
#endif
#ifndef OTA_AP_SSID
#define OTA_AP_SSID "ESP32FOC-L"
#endif
#else
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "hwchk-right"
#endif
#ifndef OTA_AP_SSID
#define OTA_AP_SSID "ESP32FOC-R"
#endif
#endif
