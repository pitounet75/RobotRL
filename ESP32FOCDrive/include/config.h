#pragma once

#include <stdint.h>

/**
 * MKS ESP32 FOC V2.0, two FS2804 (12N14P, 7 pole pairs) + MT6835 in ABZ.
 * Voltage FOC only: no current sense anywhere in this firmware.
 */

/* Bit 0 = left (motor 0), bit 1 = right (motor 1). */
#ifndef FOC_AXIS_MASK
#define FOC_AXIS_MASK 0b01
#endif
#define AXIS_PRESENT(i) (((FOC_AXIS_MASK) >> (i)) & 1u)
#define AXIS_COUNT 2

/** Shared BL9342 gate-drive enable. GPIO12 is a strap pin: hold it low first. */
#ifndef FOC_PIN_MEN
#define FOC_PIN_MEN 12
#endif

/* {UH, VH, WH} per axis. */
static const int kAxisPwm[AXIS_COUNT][3] = {{32, 33, 25}, {26, 27, 14}};
/* {A, B, Z} per axis. Right A is GPIO5, a strap pin: verified on the bench. */
static const int kAxisEnc[AXIS_COUNT][3] = {{18, 19, 22}, {5, 23, 21}};
static const char kAxisName[AXIS_COUNT] = {'L', 'R'};
/* Robot frame: a positive command drives the robot forward. */
static const int8_t kAxisCmdSign[AXIS_COUNT] = {-1, +1};
/* PCNT unit index equals the axis index. */

/** MT6835 Z is idle-low, pulse-high. */
#ifndef ENC_Z_EDGE
#define ENC_Z_EDGE RISING
#endif
/** Must match the MT6835 ABZ_PPR register. PCNT full-quad -> CPR = 4 * PPR. */
#ifndef ENC_PPR
#define ENC_PPR 16384u
#endif
/** Glitch filter in APB cycles (max 1023). 250 ~ 3 us @ 80 MHz. */
#ifndef ENC_PCNT_FILTER
#define ENC_PCNT_FILTER 250
#endif
/**
 * Velocity window. 1 ms, not 2: a 400 Hz balance loop reading a 500 Hz
 * estimate gets a 0-2 ms variable age, which costs phase margin. At 65536 CPR
 * a 1 ms window still resolves 2*PI/65536/1ms = 0.096 rad/s.
 */
#ifndef ENC_VEL_MIN_DT
#define ENC_VEL_MIN_DT 0.001f
#endif

#ifndef FOC_POLE_PAIRS
#define FOC_POLE_PAIRS 7
#endif
#ifndef FOC_VBUS
#define FOC_VBUS 12.6f
#endif
/** 3.0 V over the 5.2 ohm phase is 0.58 A at stall. Raise at runtime. */
#ifndef FOC_VOLTAGE_LIMIT
#define FOC_VOLTAGE_LIMIT 3.0f
#endif
#ifndef FOC_VOLTAGE_ALIGN
#define FOC_VOLTAGE_ALIGN 2.0f
#endif
#ifndef FOC_PWM_HZ
#define FOC_PWM_HZ 20000u
#endif
/**
 * SinePWM is centered, so usable Uq tops out near Vbus/2 (~6.3 V here).
 * SpaceVectorPWM reaches Vbus/sqrt(3) if the robot ever needs more.
 */
#ifndef FOC_MODULATION
#define FOC_MODULATION FOCModulationType::SinePWM
#endif

/* Velocity PID output is in VOLTS (no current loop). */
#ifndef FOC_VEL_P
#define FOC_VEL_P 0.2f
#endif
#ifndef FOC_VEL_I
#define FOC_VEL_I 2.0f
#endif
#ifndef FOC_VEL_D
#define FOC_VEL_D 0.0f
#endif
#ifndef FOC_VEL_LPF
#define FOC_VEL_LPF 0.005f
#endif
#ifndef FOC_VEL_RAMP
#define FOC_VEL_RAMP 50.0f
#endif
#ifndef FOC_VEL_LIMIT
#define FOC_VEL_LIMIT 80.0f
#endif

#ifndef FOC_LOOP_HZ
#define FOC_LOOP_HZ 4000u
#endif
#ifndef FOC_LOOP_HZ_MIN
#define FOC_LOOP_HZ_MIN 4000u
#endif
#ifndef FOC_LOOP_HZ_MAX
#define FOC_LOOP_HZ_MAX 16000u
#endif

/** Open-loop Z search. */
#ifndef Z_SEARCH_RPS
#define Z_SEARCH_RPS 1.0f
#endif
#ifndef Z_SEARCH_TURNS
#define Z_SEARCH_TURNS 2.0f
#endif

/** Display only: I_est = (Uq - BEMF) / R. Never fed back into any loop. */
#ifndef FOC_PHASE_R
#define FOC_PHASE_R 5.2f
#endif
#ifndef FOC_KV
#define FOC_KV 220.0f
#endif

#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif
#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "esp32focdrive"
#endif
#ifndef OTA_AP_SSID
#define OTA_AP_SSID "ESP32FOCDrive"
#endif
