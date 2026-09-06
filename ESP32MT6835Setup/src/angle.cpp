/**
 * Read-only MT6835 angle monitor. One sample every 500 ms on Serial 115200.
 * Same 24-clock Mode 3 frames as the dump tool — no EEPROM writes.
 *
 *   pio run -e angle -t upload
 *   pio device monitor -e angle
 */

#include <Arduino.h>
#include <SPI.h>

#include "config.h"
#include "mt6835.h"

#ifndef MT6835_ANGLE_PERIOD_MS
#define MT6835_ANGLE_PERIOD_MS 500u
#endif

static Mt6835 enc(MT6835_PIN_CS0, Mt6835::kReadOnly);

static void initEsp32SpiMaster() {
  if (MT6835_PIN_CS0 >= 0) {
    pinMode(MT6835_PIN_CS0, OUTPUT);
    digitalWrite(MT6835_PIN_CS0, HIGH);
  }
  pinMode(MT6835_PIN_MISO, INPUT_PULLUP);

  SPI.begin(MT6835_PIN_SCK, MT6835_PIN_MISO, MT6835_PIN_MOSI, -1);
  SPI.setHwCs(false);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setFrequency(MT6835_SPI_HZ);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("ESP32MT6835 angle (read-only, 500 ms)");
  Serial.printf("SCK=%d MISO=%d MOSI=%d CS=%d  SPI=%u Hz\n", MT6835_PIN_SCK,
                MT6835_PIN_MISO, MT6835_PIN_MOSI, MT6835_PIN_CS0,
                (unsigned)MT6835_SPI_HZ);
  Serial.println("burst 6B @ 0x003  CRC-8 same as ODrive (poly 0x07)");

  initEsp32SpiMaster();
  enc.begin();
}

void loop() {
  const uint32_t t0 = millis();

  const Mt6835::AngleFrame f = enc.readAngleBurst();
  const uint8_t status = (uint8_t)(f.lo_stat & 0x07u);
  const float turn = (float)f.raw21 / 2097152.0f;
  const float deg = turn * 360.0f;

  Serial.printf("raw=%7lu  turn=%.5f  deg=%7.2f  regs=%02X %02X %02X  crc %s got=%02X calc=%02X  st=%u\n",
                (unsigned long)f.raw21, (double)turn, (double)deg, f.hi, f.mid, f.lo_stat,
                f.crc_ok ? "OK  " : "FAIL", f.crc_got, f.crc_calc, (unsigned)status);

  const uint32_t elapsed = millis() - t0;
  if (elapsed < MT6835_ANGLE_PERIOD_MS) {
    delay(MT6835_ANGLE_PERIOD_MS - elapsed);
  }
}
