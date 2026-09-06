#include "mt6835.h"

#include <SPI.h>
#include <string.h>

#include "config.h"

namespace {

constexpr uint8_t kOpRead = 0b0011;
constexpr uint8_t kOpWrite = 0b0110;
constexpr uint8_t kOpBurst = 0b1010;
constexpr uint8_t kOpProg = 0b1100;
constexpr uint8_t kAck = 0x55;

constexpr uint16_t kRegUserId = 0x001;
constexpr uint16_t kRegAngleHi = 0x003;
constexpr uint16_t kRegAbzResHi = 0x007;
constexpr uint16_t kRegAbzResLo = 0x008;
constexpr uint16_t kRegZeroPosHi = 0x009;

/*
 * Datasheet 7.6:
 *  - Mode 3 (CPOL=1, CPHA=1), SCK idle HIGH
 *  - Frame is 24 SCK edges while CSN is low: C3..C0 | A11..A0 | D7..D0
 *  - CSN falling starts, CSN rising ends (gaps / extra clocks while CSN=0 count)
 *  - TL >= 100 ns after CSN↓ before first SCK↓
 *  - TH >= 0.5·TSCK after last SCK↑ before CSN↑
 *  - MISO Hi-Z for the first 16 clocks; data only on clocks 17..24
 *
 * ESP32 Arduino: SPI.begin() starts the bus in MODE0 (SCK idle LOW).
 * Three SPI.transfer(byte) calls are three HW transactions — SCK can glitch
 * back to idle between bytes while CSN is still low, which the MT6835 counts
 * as extra bits. Writes then land on the wrong address. Program EEPROM then
 * burns the *entire* (now corrupted) map, including MagnTek-only bytes.
 *
 * Fix: one transferBytes() of 3 bytes = one 24-clock burst; CSN around it.
 */

}  // namespace

Mt6835::Mt6835(int8_t cs_pin, Role role)
    : cs_(cs_pin), role_(role), rx0_(0), rx1_(0), rx2_(0) {}

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
  /* 4-byte buffers: ESP32 HAL loads FIFO as uint32_t; only 24 bits are clocked. */
  uint8_t tx[4] = {0, 0, 0, 0};
  uint8_t rx[4] = {0, 0, 0, 0};
  tx[0] = (uint8_t)(((cmd & 0x0Fu) << 4) | ((addr >> 8) & 0x0Fu));
  tx[1] = (uint8_t)(addr & 0xFFu);
  tx[2] = data_out;

  SPI.beginTransaction(SPISettings(MT6835_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_, LOW);
  delayMicroseconds(2); /* TL >> 100 ns */

  /* One HW transaction: mosi_dbitlen = miso_dbitlen = 23 → 24 clocks both ways. */
  SPI.transferBytes(tx, rx, 3);

  delayMicroseconds(1); /* TH */
  digitalWrite(cs_, HIGH);
  SPI.endTransaction();
  delayMicroseconds(2);

  rx0_ = rx[0];
  rx1_ = rx[1];
  rx2_ = rx[2];
  return rx[2]; /* datasheet: DO7..DO0 on clocks 17..24 */
}

bool Mt6835::writesAllowed(const char *op) const {
  if (role_ == kWritable) {
    return true;
  }
  Serial.printf("REFUSED %s on CS=GPIO%d (read-only donor)\n", op, (int)cs_);
  return false;
}

uint8_t Mt6835::readReg(uint16_t addr) {
  return transfer24(kOpRead, addr, 0x00);
}

bool Mt6835::writeReg(uint16_t addr, uint8_t value) {
  if (!writesAllowed("WRITE")) {
    return false;
  }
  (void)transfer24(kOpWrite, addr, value);
  return true;
}

void Mt6835::readAbzRaw(uint8_t *hi, uint8_t *lo) {
  if (hi) {
    *hi = readReg(kRegAbzResHi);
  }
  if (lo) {
    *lo = readReg(kRegAbzResLo);
  }
}

uint8_t Mt6835::readUserId() {
  return readReg(kRegUserId);
}

uint16_t Mt6835::readAbzPpr() {
  uint8_t hi = 0;
  uint8_t lo = 0;
  readAbzRaw(&hi, &lo);
  const uint16_t res_code = (uint16_t)((hi << 6) | (lo >> 2));
  return (uint16_t)(res_code + 1u);
}

bool Mt6835::busLooksAlive() {
  const uint8_t id = readUserId();
  const uint8_t a0 = rx0_;
  const uint8_t a1 = rx1_;
  const uint8_t a2 = rx2_;
  const uint8_t angle = readReg(kRegAngleHi);
  const uint8_t hi = readReg(kRegAbzResHi);
  const uint8_t lo = readReg(kRegAbzResLo);

  (void)id;
  (void)a0;
  (void)a1;
  (void)a2;

  /* All-zero or all-one on ABZ+angle is a dead/floating bus. */
  if (hi == 0x00u && lo == 0x00u && angle == 0x00u) {
    return false;
  }
  if (hi == 0xFFu && lo == 0xFFu && angle == 0xFFu) {
    return false;
  }
  return true;
}

void Mt6835::eepromAddrList(uint16_t *out) {
  size_t i = 0;
  out[i++] = 0x001;
  for (uint16_t addr = 0x007; addr <= 0x0D2; ++addr) {
    out[i++] = addr;
  }
}

bool Mt6835::readEepromMap(uint8_t *vals) {
  uint16_t addrs[kEepromMapLen];
  eepromAddrList(addrs);
  for (size_t i = 0; i < kEepromMapLen; ++i) {
    vals[i] = readReg(addrs[i]);
  }
  return true;
}

bool Mt6835::writeEepromMap(const uint8_t *vals, Print *log) {
  if (!writesAllowed("WRITE_MAP")) {
    return false;
  }
  uint16_t addrs[kEepromMapLen];
  eepromAddrList(addrs);
  for (size_t i = 0; i < kEepromMapLen; ++i) {
    (void)writeReg(addrs[i], vals[i]);
    delayMicroseconds(200);
    const uint8_t got = readReg(addrs[i]);
    if (got != vals[i]) {
      if (log) {
        log->printf("MISMATCH addr=0x%03X wrote=0x%02X read=0x%02X (index %u)\n",
                    addrs[i], vals[i], got, (unsigned)i);
      }
      return false;
    }
  }
  return true;
}

bool Mt6835::setAbzPpr(uint16_t ppr) {
  if (!writesAllowed("SET_ABZ")) {
    return false;
  }
  if (ppr < 1u || ppr > 16384u) {
    return false;
  }
  const uint16_t code = (uint16_t)(ppr - 1u);

  const uint8_t hi = (uint8_t)((code >> 6) & 0xFFu);
  uint8_t lo = readReg(kRegAbzResLo);
  const uint8_t ab_swap = (uint8_t)(lo & 0x01u);
  lo = (uint8_t)(((code & 0x3Fu) << 2) | ab_swap);

  (void)writeReg(kRegAbzResHi, hi);
  delay(1);
  (void)writeReg(kRegAbzResLo, lo);
  delay(1);
  return readAbzPpr() == ppr;
}

bool Mt6835::programEeprom() {
  if (!writesAllowed("PROG")) {
    return false;
  }
  delay(2);
  const uint8_t ack = transfer24(kOpProg, 0x000, 0x00);
  return ack == kAck;
}

uint32_t Mt6835::readAngleRaw() {
  const uint8_t hi = readReg(0x003);
  const uint8_t mid = readReg(0x004);
  const uint8_t lo_stat = readReg(0x005);
  return ((uint32_t)hi << 13) | ((uint32_t)mid << 5) | ((uint32_t)lo_stat >> 3);
}

/* CRC-8 poly x^8+x^2+x+1, MagnTek / ODrive table. */
static uint8_t mt6835_crc8(const uint8_t *data, size_t len) {
  static const uint8_t kCrcTable[256] = {
      0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
      0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
      0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
      0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
      0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
      0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
      0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
      0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
      0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
      0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
      0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
      0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
      0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
      0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
      0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
      0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
  };
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    crc = kCrcTable[crc];
  }
  return crc;
}

Mt6835::AngleFrame Mt6835::readAngleBurst() {
  AngleFrame f = {};
  /* 8-byte buffers: ESP32 HAL FIFO is uint32_t-backed; clock exactly 48 bits. */
  uint8_t tx[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t rx[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  const uint16_t cmd = (uint16_t)(((uint16_t)kOpBurst << 12) | 0x003u);
  tx[0] = (uint8_t)(cmd >> 8);
  tx[1] = (uint8_t)(cmd & 0xFFu);
  tx[2] = 0xFFu;
  tx[3] = 0xFFu;
  tx[4] = 0xFFu;
  tx[5] = 0xFFu;

  SPI.beginTransaction(SPISettings(MT6835_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs_, LOW);
  delayMicroseconds(2);
  SPI.transferBytes(tx, rx, 6);
  delayMicroseconds(1);
  digitalWrite(cs_, HIGH);
  SPI.endTransaction();
  delayMicroseconds(2);

  rx0_ = rx[0];
  rx1_ = rx[1];
  rx2_ = rx[2];

  /* MISO Hi-Z on clocks 1-16; regs 0x003..0x006 on RX[2..5]. */
  f.hi = rx[2];
  f.mid = rx[3];
  f.lo_stat = rx[4];
  f.crc_got = rx[5];
  const uint8_t payload[3] = {f.hi, f.mid, f.lo_stat};
  f.crc_calc = mt6835_crc8(payload, 3);
  f.crc_ok = (f.crc_calc == f.crc_got);
  f.raw21 = ((uint32_t)f.hi << 13) | ((uint32_t)f.mid << 5) | ((uint32_t)f.lo_stat >> 3);
  f.raw21 &= 0x1FFFFFu;
  return f;
}

size_t Mt6835::dumpEepromMap(Print &out, const char *chip_label) const {
  Mt6835 *self = const_cast<Mt6835 *>(this);
  size_t count = 0;

  out.printf("# MT6835_EEPROM_DUMP label=%s cs_gpio=%d\n", chip_label, (int)cs_);
  out.printf("# Format: reg,0xADDR,0xVV  (save this block for restore/compare)\n");
  out.printf("# Read-only angle snapshot: 0x%06lX  ABZ_PPR=%u  busAlive=%s\n",
             (unsigned long)self->readAngleRaw(), (unsigned)self->readAbzPpr(),
             self->busLooksAlive() ? "yes" : "NO");

  auto dump_addr = [&](uint16_t addr) {
    const uint8_t v = self->readReg(addr);
    out.printf("reg,0x%03X,0x%02X\n", addr, v);
    ++count;
  };

  uint16_t addrs[kEepromMapLen];
  eepromAddrList(addrs);
  for (size_t i = 0; i < kEepromMapLen; ++i) {
    dump_addr(addrs[i]);
  }

  out.printf("# END count=%u\n", (unsigned)count);
  return count;
}
