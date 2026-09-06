/**
 * ESP32 MT6835 SPI — spec-compliant 24-clock frames.
 *
 * Default (MT6835_MODE_DUMP=1): read-only full EEPROM map dump to serial.
 * MagnTek has no DEVICE_ID. 0x001 = USER_ID (free EEPROM byte, often 0x00).
 */

#include <Arduino.h>
#include <SPI.h>

#include "config.h"
#include "mt6835.h"

#if MT6835_RESTORE
#if __has_include("mt6835_restore_map.h")
#include "mt6835_restore_map.h"
#else
#error "Generate include/mt6835_restore_map.h: python dump_to_restore_map.py <dump.txt>"
#endif
#endif

static void initEsp32SpiMaster() {
  /* CSN must be high BEFORE SPI.begin(): Arduino-ESP32 starts the bus in
   * MODE0 (SCK idle LOW). A Mode0→Mode3 switch is a rising SCK edge. If CSN
   * is low, the MT6835 counts that edge and the next 24-bit frame is shifted. */
  if (MT6835_PIN_CS0 >= 0) {
    pinMode(MT6835_PIN_CS0, OUTPUT);
    digitalWrite(MT6835_PIN_CS0, HIGH);
  }
  if (MT6835_PIN_CS1 >= 0) {
    pinMode(MT6835_PIN_CS1, OUTPUT);
    digitalWrite(MT6835_PIN_CS1, HIGH);
  }

  /* MISO is Hi-Z on the slave for 16 clocks — weak pull-up avoids 0x00 reads. */
  pinMode(MT6835_PIN_MISO, INPUT_PULLUP);

  SPI.begin(MT6835_PIN_SCK, MT6835_PIN_MISO, MT6835_PIN_MOSI, -1);
  SPI.setHwCs(false);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setFrequency(MT6835_SPI_HZ);
}

static void dumpFrame(const char *name, uint16_t addr, Mt6835 &dev) {
  const uint8_t data = dev.readReg(addr);
  uint8_t r0 = 0;
  uint8_t r1 = 0;
  uint8_t r2 = 0;
  dev.lastRx(&r0, &r1, &r2);
  Serial.printf("  %-14s addr=0x%03X  MISO %02X %02X %02X  data=%02X\n", name, addr, r0, r1,
                r2, data);
}

static void probeEncoder(const char *name, int8_t cs) {
  Serial.printf("\n=== %s (CS=GPIO%d) ===\n", name, (int)cs);
  if (cs < 0) {
    Serial.println("skipped");
    return;
  }

  Mt6835 dev(cs, Mt6835::kReadOnly);
  dev.begin();

  Serial.println("Spec: MISO Hi-Z on clocks 1-16, data on 17-24 (3rd byte).");
  Serial.println("If data is in byte0/byte1, the 24-clock frame is broken.");

  dumpFrame("USER_ID", 0x001, dev);
  dumpFrame("ANGLE[20:13]", 0x003, dev);
  dumpFrame("ANGLE[12:5]", 0x004, dev);
  dumpFrame("ANGLE+STAT", 0x005, dev);
  dumpFrame("CRC", 0x006, dev);
  dumpFrame("ABZ_HI", 0x007, dev);
  dumpFrame("ABZ_LO", 0x008, dev);

  uint8_t hi = 0;
  uint8_t lo = 0;
  dev.readAbzRaw(&hi, &lo);
  Serial.printf("USER_ID=0x%02X  angle=0x%06lX  ABZ_PPR=%u  ABZ_OFF=%u  busAlive=%s\n",
                (unsigned)dev.readUserId(), (unsigned long)dev.readAngleRaw(),
                (unsigned)dev.readAbzPpr(), (unsigned)((lo >> 1) & 1u),
                dev.busLooksAlive() ? "yes" : "NO");
}

static void dumpChip(int8_t cs, const char *label) {
  if (cs < 0) {
    return;
  }
  Mt6835 dev(cs, Mt6835::kReadOnly);
  dev.begin();
  if (!dev.busLooksAlive()) {
    Serial.printf("\n# SKIP %s (CS=GPIO%d): bus not alive\n", label, (int)cs);
    return;
  }
  Serial.println();
  (void)dev.dumpEepromMap(Serial, label);
}

#if MT6835_ALLOW_WRITE || MT6835_RESTORE
static bool burnIfRequested(Mt6835 &dev, const char *label) {
#if MT6835_PROGRAM_EEPROM
  Serial.printf("PROG entire map on %s — keep power 7 s, no extra SPI...\n", label);
  if (!dev.programEeprom()) {
    Serial.println("ERROR: PROG ACK != 0x55 — map is volatile only");
    return false;
  }
  delay(MT6835_EEPROM_WAIT_MS);
  Serial.println("EEPROM wait done. Power-cycle now. Do not send more SPI.");
  return true;
#else
  Serial.printf("PROGRAM_EEPROM=0 — %s is volatile (lost on power-off).\n", label);
  Serial.println("Re-test on the motor, then flash restore_burn only if the map is good.");
  (void)dev;
  (void)label;
  return true;
#endif
}
#endif

#if MT6835_ALLOW_WRITE
static bool programOne(Mt6835 &dev) {
  if (!dev.busLooksAlive()) {
    Serial.println("ERROR: refuse write — bus not alive");
    return false;
  }
  const uint16_t before = dev.readAbzPpr();
  Serial.printf("ABZ PPR before: %u\n", (unsigned)before);
  if (before != MT6835_ABZ_PPR) {
    if (!dev.setAbzPpr(MT6835_ABZ_PPR)) {
      Serial.println("ERROR: volatile write failed");
      return false;
    }
    Serial.printf("volatile PPR=%u OK\n", (unsigned)dev.readAbzPpr());
  } else {
    Serial.println("Volatile already at target.");
  }
  return burnIfRequested(dev, "CS0");
}
#endif

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("ESP32MT6835Setup (24-clock Mode3 frames)");
  Serial.printf("MODE_DUMP=%d RESTORE=%d ALLOW_WRITE=%d PROGRAM_EEPROM=%d  SPI=%u Hz\n",
                (int)MT6835_MODE_DUMP, (int)MT6835_RESTORE,
                (int)MT6835_ALLOW_WRITE, (int)MT6835_PROGRAM_EEPROM,
                (unsigned)MT6835_SPI_HZ);
  Serial.printf("SCK=%d MISO=%d MOSI=%d CS0=%d CS1=%d\n", MT6835_PIN_SCK, MT6835_PIN_MISO,
                MT6835_PIN_MOSI, MT6835_PIN_CS0, MT6835_PIN_CS1);

  initEsp32SpiMaster();

  Serial.printf("MISO idle (CSN high, INPUT_PULLUP)=%d  (1=ok float/high, 0=stuck low)\n",
                digitalRead(MT6835_PIN_MISO));

  probeEncoder("encoder0", MT6835_PIN_CS0);
  if (MT6835_PIN_CS1 >= 0) {
    probeEncoder("encoder1", MT6835_PIN_CS1);
  }

#if MT6835_MODE_DUMP
  Serial.println("\n--- EEPROM map dump (read-only) ---");
  dumpChip(MT6835_PIN_CS0, "encoder0");
  dumpChip(MT6835_PIN_CS1, "encoder1");
  Serial.println("\nDone. Save serial log to a .txt file for compare/restore.");
#endif

#if MT6835_RESTORE
  Serial.println("\n--- RESTORE map.h -> CS0 ---");
  if (MT6835_RESTORE_MAP_COUNT != Mt6835::kEepromMapLen) {
    Serial.printf("ERROR: map count %u != %u\n", (unsigned)MT6835_RESTORE_MAP_COUNT,
                  (unsigned)Mt6835::kEepromMapLen);
  } else if (MT6835_PIN_CS0 < 0) {
    Serial.println("ERROR: CS0 not set");
  } else {
    Mt6835 target(MT6835_PIN_CS0, Mt6835::kWritable);
    target.begin();
    Serial.println("Dump TARGET before restore:");
    (void)target.dumpEepromMap(Serial, "target-before");
    Serial.printf("Writing %u EEPROM bytes to CS0...\n", (unsigned)Mt6835::kEepromMapLen);
    if (!target.writeEepromMap(kRestoreVals, &Serial)) {
      Serial.println("ERROR: verify failed — no PROG");
    } else {
      Serial.println("Volatile restore verified.");
      Serial.println("Dump TARGET after restore:");
      (void)target.dumpEepromMap(Serial, "target-after");
      (void)burnIfRequested(target, "CS0 target");
    }
  }
#endif

#if MT6835_ALLOW_WRITE
  Serial.println("\n--- WRITE mode ---");
  Mt6835 enc0(MT6835_PIN_CS0, Mt6835::kWritable);
  enc0.begin();
  (void)programOne(enc0);
#endif
}

void loop() {
#if (MT6835_ALLOW_WRITE || MT6835_RESTORE) && MT6835_PROGRAM_EEPROM
  /* Datasheet: no SPI for >=6 s after PROG. Do not poll. */
  delay(1000);
#else
  delay(5000);
#endif
}
