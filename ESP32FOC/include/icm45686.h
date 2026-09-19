#pragma once

#include <Arduino.h>
#include <stdint.h>

/** ICM-45686 on the shared VSPI bus (software CS). WHO_AM_I = 0xE9. */
class Icm45686 {
 public:
  explicit Icm45686(int8_t cs_pin);

  void beginCs();
  /** 0 = ok, -1 = SPI, -2 = WHO_AM_I mismatch. */
  int init();
  bool read(float accel_mps2[3], float gyro_rads[3], float *temp_c);

  uint8_t lastWhoAmI() const { return who_; }
  uint8_t spiMode() const { return spi_mode_; }

 private:
  bool xfer(uint8_t mode, uint8_t *tx, uint8_t *rx, size_t n);
  bool readRegs(uint8_t reg, uint8_t *buf, uint16_t len);
  bool writeReg(uint8_t reg, uint8_t val);

  int8_t cs_;
  uint8_t spi_mode_;
  uint8_t who_;
  float accel_scale_;
  float gyro_scale_;
  float temp_scale_;
};
