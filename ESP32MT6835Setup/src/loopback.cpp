/**
 * ESP32 SPI master loopback — no MT6835.
 *
 * Hardware: jumper MOSI (GPIO23) to MISO (GPIO19). Disconnect the encoder.
 * Expected: every TX byte comes back as RX.
 */

#include <Arduino.h>
#include <SPI.h>
#include <string.h>

#ifndef PIN_SCK
#define PIN_SCK 18
#endif
#ifndef PIN_MISO
#define PIN_MISO 19
#endif
#ifndef PIN_MOSI
#define PIN_MOSI 23
#endif
#ifndef PIN_CS
#define PIN_CS 5
#endif
#ifndef SPI_HZ
#define SPI_HZ 400000u
#endif

static const uint8_t kPatterns[] = {0x00, 0xFF, 0x55, 0xAA, 0xA5, 0x5A, 0x01, 0xFE,
                                    0x30, 0xC0, 0x12, 0x34};

static uint8_t transferOne(uint8_t tx) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
  const uint8_t rx = SPI.transfer(tx);
  SPI.endTransaction();
  return rx;
}

static bool runBurst(uint8_t mode) {
  uint8_t tx[sizeof(kPatterns)];
  uint8_t rx[sizeof(kPatterns)];
  memcpy(tx, kPatterns, sizeof(kPatterns));
  memset(rx, 0, sizeof(rx));

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, mode));
  SPI.transferBytes(tx, rx, sizeof(tx));
  SPI.endTransaction();

  bool ok = true;
  for (unsigned i = 0; i < sizeof(tx); i++) {
    const bool match = (rx[i] == tx[i]);
    Serial.printf("  [%u] TX=%02X RX=%02X %s\n", i, tx[i], rx[i], match ? "OK" : "FAIL");
    if (!match) {
      ok = false;
    }
  }
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("ESP32 SPI LOOPBACK");
  Serial.println("Jumper GPIO23 (MOSI) <-> GPIO19 (MISO). No MT6835 on the bus.");
  Serial.printf("SCK=%d MISO=%d MOSI=%d CS=%d (held HIGH)  %u Hz\n", PIN_SCK, PIN_MISO,
                PIN_MOSI, PIN_CS, (unsigned)SPI_HZ);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  SPI.setHwCs(false);
  SPI.setDataMode(SPI_MODE3);
  SPI.setFrequency(SPI_HZ);

  Serial.printf("MISO level before test (should follow MOSI jumper, often 1): %d\n",
                digitalRead(PIN_MISO));
}

void loop() {
  Serial.println();
  Serial.println("--- Mode 3, single-byte transfer() ---");
  bool ok_byte = true;
  for (unsigned i = 0; i < sizeof(kPatterns); i++) {
    const uint8_t tx = kPatterns[i];
    const uint8_t rx = transferOne(tx);
    const bool match = (rx == tx);
    Serial.printf("  TX=%02X RX=%02X %s\n", tx, rx, match ? "OK" : "FAIL");
    if (!match) {
      ok_byte = false;
    }
  }

  Serial.println("--- Mode 3, transferBytes() burst ---");
  const bool ok_m3 = runBurst(SPI_MODE3);

  Serial.println("--- Mode 0, transferBytes() burst ---");
  const bool ok_m0 = runBurst(SPI_MODE0);

  if (ok_byte && ok_m3) {
    Serial.println("VERDICT: ESP32 SPI master + MISO path OK (loopback Mode 3).");
  } else {
    Serial.println("VERDICT: ESP32 SPI FAIL — not an MT6835 problem.");
    Serial.println("  Check jumper 23<->19, no encoder, USB cable, board 3V3.");
  }
  if (!ok_m0 && ok_m3) {
    Serial.println("(Mode 0 fail with Mode 3 OK is still a pass for MT6835.)");
  }

  delay(2000);
}
