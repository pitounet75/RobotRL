#include "mt6835.h"

#include <SPI.h>

#include "config.h"

namespace {

constexpr uint8_t kOpRead = 0b0011;

}  // namespace

Mt6835::Mt6835(int8_t cs_pin)
    : cs_(cs_pin), rx0_(0), rx1_(0), rx2_(0) {}

void Mt6835::begin() {
  if (cs_ < 0) {
    return;
  }
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
}

void Mt6835::lastRx(uint8_t *b0, uint8_t *b1, uint8_t *b2) const {
  if (b0) {
    *b0 = rx0_;
  }
  if (b1) {
    *b1 = rx1_;
  }
  if (b2) {
    *b2 = rx2_;
  }
}

uint8_t Mt6835::transfer24(uint8_t cmd, uint16_t addr, uint8_t data_out) {
  uint8_t tx[4] = {0, 0, 0, 0};
  uint8_t rx[4] = {0, 0, 0, 0};
  tx[0] = (uint8_t)(((cmd & 0x0Fu) << 4) | ((addr >> 8) & 0x0Fu));
  tx[1] = (uint8_t)(addr & 0xFFu);
  tx[2] = data_out;

  SPI.beginTransaction(SPISettings(MT6835_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_, LOW);
  delayMicroseconds(2);

  SPI.transferBytes(tx, rx, 3);

  delayMicroseconds(1);
  digitalWrite(cs_, HIGH);
  SPI.endTransaction();
  delayMicroseconds(2);

  rx0_ = rx[0];
  rx1_ = rx[1];
  rx2_ = rx[2];
  return rx[2];
}

uint8_t Mt6835::readReg(uint16_t addr) {
  return transfer24(kOpRead, addr, 0x00);
}

uint8_t Mt6835::readUserId() {
  return readReg(0x001);
}

bool Mt6835::busLooksAlive() {
  const uint8_t angle = readReg(0x003);
  const uint8_t mid = readReg(0x004);
  const uint8_t lo = readReg(0x005);
  if (angle == 0x00u && mid == 0x00u && lo == 0x00u) {
    return false;
  }
  if (angle == 0xFFu && mid == 0xFFu && lo == 0xFFu) {
    return false;
  }
  return true;
}

uint32_t Mt6835::readAngleRaw() {
  const uint8_t hi = readReg(0x003);
  const uint8_t mid = readReg(0x004);
  const uint8_t lo_stat = readReg(0x005);
  return ((uint32_t)hi << 13) | ((uint32_t)mid << 5) | ((uint32_t)lo_stat >> 3);
}
