/**
 * MT6835 stress monitor that mimics the ODrive_S encoder path, to tell apart
 * "the MT6835/magnet is marginal" from "the ODrive board is the problem".
 *
 * Matches the ODrive (fw commit c6b6fdf):
 *   - SPI mode 3, MSB first, 6-byte burst read of regs 0x003..0x006 (cmd 0xA003)
 *   - same CRC-8 (poly 0x07) on the 3 payload bytes
 *   - SPI clock ~2.625 MHz  (ODrive SPI3 = APB1 42 MHz / 16). Override with
 *     -DMT6835_SPI_HZ=5250000 to test the old /8 rate.
 *   - polled at ~8 kHz (ODrive current loop), with the SAME low-pass on the
 *     CRC-failure rate (tau ~1 s) so the printed spi_err is directly comparable
 *     to odrv0.axis0.encoder.spi_error_rate.
 *
 *   pio run -e odrive_match -t upload
 *   pio device monitor -e odrive_match
 *
 * Test: power the MT6835 from a COLD ESP32 (let it sit an hour), watch the
 * first ~10 s of output.
 *   - spi_err stays ~0             -> MT6835 + magnet + wiring are fine.
 *                                     The intermittent spike is the ODrive board.
 *   - spi_err ramps to ~1 after a
 *     few seconds, like the ODrive -> the MT6835 (or its supply/magnet) is the
 *                                     marginal part. Stop blaming the ODrive.
 */

#include <Arduino.h>
#include <SPI.h>

#include "config.h"
#include "mt6835.h"

#ifndef MT6835_POLL_HZ
#define MT6835_POLL_HZ 8000u   /* ODrive current loop rate */
#endif
#ifndef MT6835_PRINT_MS
#define MT6835_PRINT_MS 200u
#endif
#ifndef MT6835_RATE_TAU_S
#define MT6835_RATE_TAU_S 1.0f /* matches ODrive: (0.999875)^n filter ~ 1 s tau @ 8 kHz */
#endif

static Mt6835 enc(MT6835_PIN_CS0, Mt6835::kReadOnly);

static void initSpi() {
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
  delay(300);
  Serial.println();
  Serial.println("=== MT6835 odrive_match stress monitor ===");
  Serial.printf("SCK=%d MISO=%d MOSI=%d CS=%d\n", MT6835_PIN_SCK, MT6835_PIN_MISO,
                MT6835_PIN_MOSI, MT6835_PIN_CS0);
  Serial.printf("SPI req=%u Hz  poll=%u Hz  burst 6B @0x003  CRC-8 poly 0x07 (== ODrive)\n",
                (unsigned)MT6835_SPI_HZ, (unsigned)MT6835_POLL_HZ);
  Serial.println("cols: t_s  spi_err  fail/win  raw21  turn  status  lastCRC  minRaw..maxRaw");
  initSpi();
  enc.begin();
}

void loop() {
  static float rate = 0.0f;                    // == ODrive spi_error_rate
  static uint32_t win_reads = 0, win_fails = 0;
  static uint32_t tot_reads = 0, tot_fails = 0;
  static uint32_t min_raw = 0xFFFFFFFF, max_raw = 0;
  static uint32_t last_print = 0;
  static uint32_t last_read_us = 0;
  static Mt6835::AngleFrame last_f = {};

  const uint32_t poll_us = 1000000u / MT6835_POLL_HZ;
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_read_us) < poll_us) {
    return;
  }
  const float dt = (last_read_us == 0) ? (1.0f / MT6835_POLL_HZ)
                                       : (float)(now_us - last_read_us) * 1e-6f;
  last_read_us = now_us;

  const Mt6835::AngleFrame f = enc.readAngleBurst();
  last_f = f;
  const int fail = f.crc_ok ? 0 : 1;

  // ODrive-style low-pass: rate += dt/tau * (fail - rate)
  rate += (dt / MT6835_RATE_TAU_S) * ((float)fail - rate);
  if (rate < 0.0f) rate = 0.0f;
  if (rate > 1.0f) rate = 1.0f;

  win_reads++;
  tot_reads++;
  if (fail) {
    win_fails++;
    tot_fails++;
  } else {
    if (f.raw21 < min_raw) min_raw = f.raw21;
    if (f.raw21 > max_raw) max_raw = f.raw21;
  }

  const uint32_t now_ms = millis();
  if (now_ms - last_print >= MT6835_PRINT_MS) {
    const uint8_t status = (uint8_t)(last_f.lo_stat & 0x07u);
    const float turn = (float)last_f.raw21 / 2097152.0f;
    Serial.printf("%7.2f  spi=%.3f  %4lu/%-5lu  raw=%7lu  turn=%.5f  st=%u  crc=%s  [%lu..%lu]%s\n",
                  now_ms * 1e-3, (double)rate,
                  (unsigned long)win_fails, (unsigned long)win_reads,
                  (unsigned long)last_f.raw21, (double)turn, (unsigned)status,
                  last_f.crc_ok ? "OK" : "FAIL",
                  (unsigned long)(min_raw == 0xFFFFFFFF ? 0 : min_raw),
                  (unsigned long)max_raw,
                  rate > 0.05f ? "   <<< SPI degrading" : "");
    last_print = now_ms;
    win_reads = 0;
    win_fails = 0;
    min_raw = 0xFFFFFFFF;
    max_raw = 0;
  }
}
