/**
 * MT6835 on-chip User Auto-Calibration driver.
 *
 * The MT6835 DSP measures its own angle error while the shaft turns at a
 * constant speed and stores compensation coefficients to EEPROM. Target here:
 * flatten a localised ~90 deg electrical angle error near one mechanical spot
 * that locks this motor in closed loop.
 *
 * Datasheet: MagnTek MT6835 Rev 1.3, chapter 9.2 "User Auto-Calibration".
 *   - Speed window : AUTOCAL_FREQ[2:0] = reg 0x00E bits [6:4].
 *                    Factory default 0x3 = 400..800 rpm mechanical. Table below.
 *   - Start        : pull pin 4 (CAL_EN) to VDD. Rev 1.3 documents no SPI
 *                    trigger; the sibling MT6826S datasheet documents
 *                    "write 0x5E to reg 0x155" as equivalent - tried here as
 *                    the no-solder path, NOT guaranteed on the MT6835.
 *   - Status       : reg 0x113 bits [7:6] : 00 none / 01 running / 10 failed /
 *                    11 success.  (also PWM pin 10 duty: 50% run / 25% fail /
 *                    >99% ok)
 *   - Duration     : keep turning > 64 rounds at constant speed (~5..10 s at
 *                    400..800 rpm).
 *   - Save         : automatic. On success wait > 6 s, then power-cycle. No
 *                    separate PROG. No other SPI ops after success.
 *
 * Rig: MT6835 SPI on the ESP32 (config.h pins), motor phases on the ODrive,
 * ODrive spins open-loop (encoder.config.mode = 0, AXIS_STATE_LOCKIN_SPIN) so
 * it never drives this SPI bus.
 *
 * Build envs (platformio.ini):
 *   pio run -e autocal      -t upload   # dump map + decode regs. No writes.
 *   pio run -e autocal_run  -t upload   # try to start, poll status, clean up.
 *   pio device monitor -e autocal
 *
 * SEQUENCE
 *   1. ODrive: encoder.config.mode = 0; save_configuration(); reboot.
 *   2. Wire MT6835 SPI to the ESP32. If pin 4 (CAL_EN) is reachable, wire it to
 *      MT6835_PIN_CAL_EN and set that macro; otherwise rely on the 0x155 write.
 *   3. flash `autocal`, save the serial log. Check 0x00D == 0x07 and the
 *      decoded AUTOCAL_FREQ window.
 *   4. ODrive: AXIS_STATE_LOCKIN_SPIN at a mech RPM inside the window
 *      (11 pole pairs: rpm_mech = lockin.vel[rad/s elec] * 60 / (2*pi*11);
 *      ~600 rpm -> lockin.vel ~= 690).
 *   5. flash `autocal_run`. Watch status: 01 running -> 11 success.
 *      - status stuck 00 after the trigger  -> neither trigger worked, get at
 *        pin 4 physically.
 *      - status 10 failed -> speed outside the window / not constant / < 64
 *        rounds / weak field.
 *   6. On success: power stays up 6 s (handled here), then power-cycle the
 *      MT6835. Make sure CAL_EN ends up LOW for normal use.
 *   7. ODrive: encoder.config.mode = 260; reboot; redo offset cal once.
 */

#include <Arduino.h>
#include <SPI.h>

#include "config.h"
#include "mt6835.h"

/* --- MT6835 Rev 1.3 register map (confirmed against datasheet) --- */
#define MT6835_REG_0x00D 0x00D        /* [7:4] MagnTek | [3] ROT_DIR | [2:0] HYST  -> expect 0x07 */
#define MT6835_REG_AUTOCAL_FREQ 0x00E /* [7] GPIO_DS | [6:4] AUTOCAL_FREQ | [3:0] MagnTek */
#define MT6835_AUTOCAL_FREQ_SHIFT 4
#define MT6835_AUTOCAL_FREQ_MASK 0x07
#define MT6835_REG_CAL_STATUS 0x113   /* [7:6] cal status */
#define MT6835_CAL_STATUS_SHIFT 6
#define MT6835_CAL_STATUS_MASK 0x03

/* Undocumented-for-MT6835 SPI trigger, per the MT6826S datasheet. Volatile. */
#define MT6835_REG_CAL_TRIGGER 0x155
#define MT6835_CAL_TRIGGER_START 0x5E
#define MT6835_CAL_TRIGGER_STOP 0x00

/* ------------------------------------------------------------------ *
 *  Behaviour flags (platformio.ini build_flags)
 * ------------------------------------------------------------------ */

/* 0..7 = write this AUTOCAL_FREQ code first. <0 = keep the chip's current value
 * (factory default 3 = 400..800 rpm). */
#ifndef MT6835_AUTOCAL_FREQ_CODE
#define MT6835_AUTOCAL_FREQ_CODE (-1)
#endif
/* 1 = run the calibration (trigger + poll). 0 = dump/decode only. */
#ifndef MT6835_AUTOCAL_RUN
#define MT6835_AUTOCAL_RUN 0
#endif
/* 1 = attempt the SPI trigger 0x155 <- 0x5E. */
#ifndef MT6835_AUTOCAL_USE_REG_TRIGGER
#define MT6835_AUTOCAL_USE_REG_TRIGGER 1
#endif
/* ESP32 GPIO wired to MT6835 pin 4 (CAL_EN). <0 = not wired. Held HIGH to run. */
#ifndef MT6835_PIN_CAL_EN
#define MT6835_PIN_CAL_EN (-1)
#endif
/* Give up polling after this long. > 64 rounds is a few seconds in-window. */
#ifndef MT6835_AUTOCAL_TIMEOUT_MS
#define MT6835_AUTOCAL_TIMEOUT_MS 60000u
#endif
#ifndef MT6835_AUTOCAL_POLL_MS
#define MT6835_AUTOCAL_POLL_MS 500u
#endif

static const char *kFreqWindow[8] = {
    "0x0: 3200-6400 rpm", "0x1: 1600-3200 rpm", "0x2: 800-1600 rpm",
    "0x3: 400-800 rpm",   "0x4: 200-400 rpm",   "0x5: 100-200 rpm",
    "0x6: 50-100 rpm",    "0x7: 25-50 rpm",
};
static const char *kStatusName[4] = {"none", "running", "FAILED", "success"};

static Mt6835 enc(MT6835_PIN_CS0,
                 MT6835_AUTOCAL_RUN ? Mt6835::kWritable : Mt6835::kReadOnly);

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

static uint8_t calStatus() {
  const uint8_t r = enc.readReg(MT6835_REG_CAL_STATUS);
  return (uint8_t)((r >> MT6835_CAL_STATUS_SHIFT) & MT6835_CAL_STATUS_MASK);
}

static uint32_t angleDeg100() {
  const Mt6835::AngleFrame f = enc.readAngleBurst();
  return (uint32_t)((float)f.raw21 / 2097152.0f * 36000.0f);
}

static void decodeRegs(const char *tag) {
  const uint8_t r0d = enc.readReg(MT6835_REG_0x00D);
  const uint8_t r0e = enc.readReg(MT6835_REG_AUTOCAL_FREQ);
  const uint8_t rst = enc.readReg(MT6835_REG_CAL_STATUS);
  const uint8_t freq = (uint8_t)((r0e >> MT6835_AUTOCAL_FREQ_SHIFT) & MT6835_AUTOCAL_FREQ_MASK);
  const uint8_t st = (uint8_t)((rst >> MT6835_CAL_STATUS_SHIFT) & MT6835_CAL_STATUS_MASK);
  Serial.printf("--- regs %s ---\n", tag);
  Serial.printf("  0x00D = 0x%02X   %s\n", r0d,
                r0d == 0x07 ? "ok" : "EXPECTED 0x07 - power-cycle the MT6835 to restore from EEPROM");
  Serial.printf("  0x00E = 0x%02X   AUTOCAL_FREQ = %u  (%s)\n", r0e, freq, kFreqWindow[freq]);
  Serial.printf("  0x113 = 0x%02X   CAL_STATUS = %u (%s)\n", rst, st, kStatusName[st]);
}

static bool setFreqCode(uint8_t code) {
  code &= MT6835_AUTOCAL_FREQ_MASK;
  uint8_t r = enc.readReg(MT6835_REG_AUTOCAL_FREQ);
  r = (uint8_t)(r & ~(MT6835_AUTOCAL_FREQ_MASK << MT6835_AUTOCAL_FREQ_SHIFT));
  r = (uint8_t)(r | (code << MT6835_AUTOCAL_FREQ_SHIFT));
  Serial.printf("set AUTOCAL_FREQ = %u -> 0x00E = 0x%02X  (%s)\n", code, r, kFreqWindow[code]);
  if (!enc.writeReg(MT6835_REG_AUTOCAL_FREQ, r)) return false;
  delay(2);
  const uint8_t rb = enc.readReg(MT6835_REG_AUTOCAL_FREQ);
  const uint8_t got = (uint8_t)((rb >> MT6835_AUTOCAL_FREQ_SHIFT) & MT6835_AUTOCAL_FREQ_MASK);
  if (got != code) {
    Serial.printf("  READBACK MISMATCH 0x00E=0x%02X freq=%u\n", rb, got);
    return false;
  }
  return true;
}

static void calEnPin(bool high) {
#if (MT6835_PIN_CAL_EN >= 0)
  digitalWrite(MT6835_PIN_CAL_EN, high ? HIGH : LOW);
  Serial.printf("CAL_EN pin (GPIO%d) = %s\n", MT6835_PIN_CAL_EN, high ? "HIGH" : "LOW");
#else
  (void)high;
#endif
}

static void triggerStart() {
  calEnPin(true);
#if MT6835_AUTOCAL_USE_REG_TRIGGER
  Serial.printf("SPI trigger: 0x%03X <- 0x%02X\n", MT6835_REG_CAL_TRIGGER, MT6835_CAL_TRIGGER_START);
  enc.writeReg(MT6835_REG_CAL_TRIGGER, MT6835_CAL_TRIGGER_START);
  delay(2);
#endif
}

static void triggerStop() {
#if MT6835_AUTOCAL_USE_REG_TRIGGER
  Serial.printf("SPI trigger: 0x%03X <- 0x%02X\n", MT6835_REG_CAL_TRIGGER, MT6835_CAL_TRIGGER_STOP);
  enc.writeReg(MT6835_REG_CAL_TRIGGER, MT6835_CAL_TRIGGER_STOP);
#endif
  calEnPin(false);
}

static uint8_t pollCal() {
  const uint32_t t0 = millis();
  uint32_t last_deg = angleDeg100();
  uint8_t st = calStatus();
  Serial.printf("poll 0x113[7:6], timeout %lus...\n",
                (unsigned long)(MT6835_AUTOCAL_TIMEOUT_MS / 1000u));
  while (millis() - t0 < MT6835_AUTOCAL_TIMEOUT_MS) {
    st = calStatus();
    const uint32_t deg = angleDeg100();
    const int32_t d = (int32_t)deg - (int32_t)last_deg;
    last_deg = deg;
    Serial.printf("  t=%5lus  status=%u (%s)  angle=%6.2f  d=%+d\n",
                  (unsigned long)((millis() - t0) / 1000u), st, kStatusName[st],
                  deg / 100.0, (int)d);
    if (st == 2u || st == 3u) break;
    delay(MT6835_AUTOCAL_POLL_MS);
  }
  return st;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("=== MT6835 User Auto-Calibration ===");
  Serial.printf("SCK=%d MISO=%d MOSI=%d CS=%d  SPI=%u Hz\n", MT6835_PIN_SCK,
                MT6835_PIN_MISO, MT6835_PIN_MOSI, MT6835_PIN_CS0, (unsigned)MT6835_SPI_HZ);
  Serial.printf("flags: FREQ_CODE=%d RUN=%d REG_TRIGGER=%d CAL_EN_PIN=%d\n",
                MT6835_AUTOCAL_FREQ_CODE, MT6835_AUTOCAL_RUN,
                MT6835_AUTOCAL_USE_REG_TRIGGER, MT6835_PIN_CAL_EN);

#if (MT6835_PIN_CAL_EN >= 0)
  pinMode(MT6835_PIN_CAL_EN, OUTPUT);
  digitalWrite(MT6835_PIN_CAL_EN, LOW);
#endif

  initEsp32SpiMaster();
  enc.begin();
  delay(5);

  if (!enc.busLooksAlive()) {
    Serial.println("BUS DEAD (angle+ABZ all 0x00 or 0xFF). Check wiring/power. STOP.");
    return;
  }

  Serial.println();
  Serial.println(">>> BACKUP: full EEPROM-backed map (save this whole block) <<<");
  enc.dumpEepromMap(Serial, "autocal-pre");
  Serial.println();
  decodeRegs("start");

#if MT6835_AUTOCAL_RUN
  Serial.println();
  Serial.println("### RUN ### shaft MUST already be spinning at constant speed in the window.");

#if (MT6835_AUTOCAL_FREQ_CODE >= 0)
  if (!setFreqCode((uint8_t)MT6835_AUTOCAL_FREQ_CODE)) {
    Serial.println("freq-code set FAILED, aborting.");
    return;
  }
#else
  Serial.println("FREQ_CODE < 0: keeping the chip's current AUTOCAL_FREQ.");
#endif

  delay(50);
  triggerStart();
  const uint8_t st = pollCal();
  Serial.printf("finished: status=%u (%s)\n", st, kStatusName[st]);

  if (st != 3u) {
    triggerStop();
    Serial.println("not 'success' -> nothing saved.");
    if (st == 0u)
      Serial.println("status never left 'none': the trigger didn't take. Get at pin 4 (CAL_EN) physically.");
    else
      Serial.println("status 'FAILED': speed outside window / not constant / < 64 rounds / weak field.");
    decodeRegs("end");
    return;
  }

  Serial.println("SUCCESS. Coefficients auto-saved. Holding power 8 s (EEPROM settle), no SPI...");
  triggerStop();
  delay(8000);
  Serial.println("Done. POWER-CYCLE the MT6835 now. Ensure CAL_EN is LOW for normal use.");
  Serial.println("Then ODrive: encoder.config.mode = 260; reboot; redo offset cal once.");
#else
  Serial.println();
  Serial.println("DUMP/DECODE only (RUN flag off). Nothing was written.");
#endif

  Serial.println();
  Serial.println("=== idle angle print follows ===");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 1000u) return;
  last = millis();
  const Mt6835::AngleFrame f = enc.readAngleBurst();
  Serial.printf("raw=%7lu  deg=%7.2f  crc=%s  cal_status=%u\n", (unsigned long)f.raw21,
                (double)((float)f.raw21 / 2097152.0f * 360.0f), f.crc_ok ? "OK" : "FAIL",
                (unsigned)calStatus());
}
