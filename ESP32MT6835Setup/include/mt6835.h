#pragma once

#include <Arduino.h>
#include <stdint.h>

/** SPI Mode 3, one 24-clock frame per access (cmd[4] | addr[12] | data[8]). */
class Mt6835 {
 public:
  enum Role : uint8_t { kReadOnly = 0, kWritable = 1 };

  explicit Mt6835(int8_t cs_pin, Role role = kWritable);

  /** Drive CSN high. Call before SPI clocks exist. */
  void begin();

  uint8_t readReg(uint16_t addr);
  bool writeReg(uint16_t addr, uint8_t value);

  /** Last 24-bit MISO bytes (cmd, addr, data phases). */
  void lastRx(uint8_t *b0, uint8_t *b1, uint8_t *b2) const;

  uint8_t readUserId();
  void readAbzRaw(uint8_t *hi, uint8_t *lo);

  bool busLooksAlive();
  uint16_t readAbzPpr();
  bool setAbzPpr(uint16_t ppr);
  bool programEeprom();

  /** EEPROM-backed bytes: 0x001 + 0x007..0x0D2 (USER_ID, config, MagnTek-only, NLC). */
  static constexpr size_t kEepromMapLen = 1 + (0x0D2 - 0x007 + 1);
  static void eepromAddrList(uint16_t *out /*[kEepromMapLen]*/);

  bool readEepromMap(uint8_t *vals /*[kEepromMapLen]*/);
  /**
   * Volatile write of the full EEPROM-backed map, then readback.
   * On mismatch prints the first failing address to `log` and returns false.
   * Does not PROG.
   */
  bool writeEepromMap(const uint8_t *vals, Print *log);

  /** 21-bit angle from read-only regs 0x003..0x005 (3 separate CSN frames, no CRC). */
  uint32_t readAngleRaw();

  /**
   * Datasheet §7.6.9 burst: one CSN-low, C=1010 addr 0x003, 6 SPI bytes.
   * CRC-8 (x^8+x^2+x+1) over 0x003..0x005 vs 0x006 — same as ODrive.
   * Separate single-byte reads re-latch on each CSN↓ and make 0x006 meaningless.
   */
  struct AngleFrame {
    uint32_t raw21;
    uint8_t hi;
    uint8_t mid;
    uint8_t lo_stat;
    uint8_t crc_got;
    uint8_t crc_calc;
    bool crc_ok;
  };
  AngleFrame readAngleBurst();

  /**
   * Print all EEPROM-backed registers to a serial stream (parseable for restore).
   * Returns number of register bytes dumped.
   */
  size_t dumpEepromMap(Print &out, const char *chip_label) const;

  int8_t csPin() const { return cs_; }
  Role role() const { return role_; }

 private:
  uint8_t transfer24(uint8_t cmd, uint16_t addr, uint8_t data_out);
  bool writesAllowed(const char *op) const;

  int8_t cs_;
  Role role_;
  uint8_t rx0_;
  uint8_t rx1_;
  uint8_t rx2_;
};
