#pragma once

/**
 * Target ABZ pulses-per-revolution programmed into the MT6835.
 * Datasheet range: 1..16384. Register value stored is (PPR - 1).
 *
 * After changing this, flash + run this tool, power-cycle the MT6835,
 * then set STM32 WHEEL_ENCODER_CPR = MT6835_ABZ_PPR * 2
 * (RobotRL TIM2/TIM4 use TIM_ENCODERMODE_TI1 = x2; TI12 would be x4).
 */
#ifndef MT6835_ABZ_PPR
#define MT6835_ABZ_PPR 16384u
#endif

/** 1 = full EEPROM map dump on boot (default, read-only). 0 = short probe only. */
#ifndef MT6835_MODE_DUMP
#define MT6835_MODE_DUMP 1
#endif

/** 1 = allow volatile ABZ write / restore (see env write in platformio.ini). */
#ifndef MT6835_ALLOW_WRITE
#define MT6835_ALLOW_WRITE 0
#endif

/** 1 = burn volatile map to EEPROM after write. Never enable without a verified dump. */
#ifndef MT6835_PROGRAM_EEPROM
#define MT6835_PROGRAM_EEPROM 0
#endif

/** 1 = restore EEPROM map from include/mt6835_restore_map.h onto CS0. */
#ifndef MT6835_RESTORE
#define MT6835_RESTORE 0
#endif

/** Wait after EEPROM program before allowing power-down (datasheet: >= 6 s). */
#ifndef MT6835_EEPROM_WAIT_MS
#define MT6835_EEPROM_WAIT_MS 7000u
#endif

/* --- ESP32 VSPI pins (change to match your wiring) --- */
#ifndef MT6835_PIN_SCK
#define MT6835_PIN_SCK 18
#endif
#ifndef MT6835_PIN_MISO
#define MT6835_PIN_MISO 19
#endif
#ifndef MT6835_PIN_MOSI
#define MT6835_PIN_MOSI 23
#endif

/** Chip-select for encoder 0 (left / TIM2 side). Active low. */
#ifndef MT6835_PIN_CS0
#define MT6835_PIN_CS0 5
#endif

/**
 * Chip-select for encoder 1 (second chip on same SPI bus).
 * -1 = unused (program one encoder at a time; rewire CS0 to the other chip).
 */
#ifndef MT6835_PIN_CS1
#define MT6835_PIN_CS1 (-1)
#endif

#ifndef MT6835_SPI_HZ
#define MT6835_SPI_HZ 400000u
#endif
