#include "icm45686.h"

#include <SPI.h>
#include <string.h>

#include "config.h"

namespace {

constexpr uint8_t kRegAccelX = 0x00;
constexpr uint8_t kRegPwrMgmt0 = 0x10;
constexpr uint8_t kRegAccelCfg0 = 0x1B;
constexpr uint8_t kRegGyroCfg0 = 0x1C;
constexpr uint8_t kRegWhoAmI = 0x72;
constexpr uint8_t kWhoAmI = 0xE9;
constexpr uint8_t kReadBit = 0x80;
constexpr uint8_t kOdr800 = 0x6;
constexpr uint8_t kAccel16g = 0x1;
constexpr uint8_t kGyro2000 = 0x1;

}  // namespace

Icm45686::Icm45686(int8_t cs_pin)
    : cs_(cs_pin),
      spi_mode_(SPI_MODE3),
      who_(0),
      accel_scale_(9.80665f / 2048.0f),
      gyro_scale_((3.14159265f / 180.0f) / 16.0f),
      temp_scale_(1.0f / 132.48f) {}

void Icm45686::beginCs() {
  if (cs_ < 0) {
    return;
  }
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
}

bool Icm45686::xfer(uint8_t mode, uint8_t *tx, uint8_t *rx, size_t n) {
  SPI.beginTransaction(SPISettings(ICM_SPI_HZ, MSBFIRST, mode));
  digitalWrite(cs_, LOW);
  delayMicroseconds(1);
  SPI.transferBytes(tx, rx, n);
  digitalWrite(cs_, HIGH);
  SPI.endTransaction();
  delayMicroseconds(1);
  return true;
}

bool Icm45686::readRegs(uint8_t reg, uint8_t *buf, uint16_t len) {
  uint8_t tx[32];
  uint8_t rx[32];
  const uint16_t total = (uint16_t)(1u + len);
  if (total > sizeof(tx)) {
    return false;
  }
  memset(tx, 0, total);
  tx[0] = (uint8_t)(reg | kReadBit);
  if (!xfer(spi_mode_, tx, rx, total)) {
    return false;
  }
  memcpy(buf, rx + 1, len);
  return true;
}

bool Icm45686::writeReg(uint8_t reg, uint8_t val) {
  uint8_t tx[4] = {(uint8_t)(reg & (uint8_t)~kReadBit), val, 0, 0};
  uint8_t rx[4] = {0, 0, 0, 0};
  return xfer(spi_mode_, tx, rx, 2);
}

int Icm45686::init() {
  beginCs();
  delay(50);

  uint8_t who = 0;
  spi_mode_ = SPI_MODE3;
  if (!readRegs(kRegWhoAmI, &who, 1) || who != kWhoAmI) {
    spi_mode_ = SPI_MODE0;
    if (!readRegs(kRegWhoAmI, &who, 1)) {
      who_ = who;
      return -1;
    }
  }
  who_ = who;
  if (who != kWhoAmI) {
    return -2;
  }

  if (!writeReg(kRegPwrMgmt0, 0x0F)) {
    return -1;
  }
  delay(2);
  if (!writeReg(kRegAccelCfg0, (uint8_t)((kOdr800 & 0x0F) | ((kAccel16g & 0x07) << 4)))) {
    return -1;
  }
  if (!writeReg(kRegGyroCfg0, (uint8_t)((kOdr800 & 0x0F) | ((kGyro2000 & 0x0F) << 4)))) {
    return -1;
  }
  return 0;
}

bool Icm45686::read(float accel_mps2[3], float gyro_rads[3], float *temp_c) {
  uint8_t raw[14];
  if (!readRegs(kRegAccelX, raw, 14)) {
    return false;
  }
  auto le16 = [](const uint8_t *p) -> int16_t {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
  };
  accel_mps2[0] = (float)le16(raw + 0) * accel_scale_;
  accel_mps2[1] = (float)le16(raw + 2) * accel_scale_;
  accel_mps2[2] = (float)le16(raw + 4) * accel_scale_;
  gyro_rads[0] = (float)le16(raw + 6) * gyro_scale_;
  gyro_rads[1] = (float)le16(raw + 8) * gyro_scale_;
  gyro_rads[2] = (float)le16(raw + 10) * gyro_scale_;
  if (temp_c) {
    *temp_c = 25.0f + (float)le16(raw + 12) * temp_scale_;
  }
  return true;
}
