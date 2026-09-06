#pragma once

#include <Arduino.h>
#include <stdint.h>

/** Read-only MT6835 SPI. Same 24-clock Mode 3 framing as ESP32MT6835Setup.
 *  This project never WRITEs or PROGs EEPROM — use ESP32MT6835Setup for that. */
class Mt6835 {
 public:
  explicit Mt6835(int8_t cs_pin);

  /** Drive CSN high. Call before SPI clocks exist. */
  void begin();

  uint8_t readReg(uint16_t addr);
  void lastRx(uint8_t *b0, uint8_t *b1, uint8_t *b2) const;

  uint8_t readUserId();
  bool busLooksAlive();
  uint32_t readAngleRaw();

  int8_t csPin() const { return cs_; }

 private:
  uint8_t transfer24(uint8_t cmd, uint16_t addr, uint8_t data_out);

  int8_t cs_;
  uint8_t rx0_;
  uint8_t rx1_;
  uint8_t rx2_;
};
