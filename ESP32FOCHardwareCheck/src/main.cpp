/**
 * One-axis FS2804 hardware check: electrical offset, velocity, voltage-torque.
 * MT6835 via ABZ + ESP32 PCNT (full-quad), not SPI / GPIO edge ISRs.
 */

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <driver/rtc_io.h>
#include <driver/timer.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>
#include <string.h>

#include "anticog.h"
#include "config.h"
#include "dma_adc.h"
#include "dma_current_sense.h"
#include "pcnt_encoder.h"

namespace {

constexpr uint32_t kCalMagic = 0x48434B32u; /* HCK2 — ABZ, do not reuse HCK1 SPI */
constexpr const char *kNvsNs = "hwchk";
constexpr uint32_t kMonPeriodMs = 1000;
constexpr uint32_t kEncPeriodMs = 1000;

enum class RunMode : uint8_t { Enc, Idle, Openloop, Velocity, Torque, Accal };

struct CalRecord {
  uint32_t magic;
  float zero_electric_angle;
  int8_t sensor_direction;
  uint8_t pole_pairs;
  char axis;
  uint8_t reserved;
};

PcntEncoder encoder(ENC_PCNT_PIN_A, ENC_PCNT_PIN_B, ENC_PIN_Z, ENC_PPR,
                    (pcnt_unit_t)ENC_PCNT_UNIT);
BLDCMotor motor(FOC_POLE_PAIRS);
BLDCDriver3PWM driver(HWCHK_PIN_UH, HWCHK_PIN_VH, HWCHK_PIN_WH, FOC_PIN_MEN);
DmaInlineCurrentSense current_sense(CS_SHUNT_OHM, CS_AMP_GAIN, CS_PIN_A, CS_PIN_B);
Anticog anticog;
Preferences prefs;

volatile RunMode mode = RunMode::Enc;
volatile bool ac_finished = false;
bool motor_ready = false;
bool calibrated = false;
bool current_ok = false;
bool monitor_on = true;
float voltage_limit = FOC_VOLTAGE_LIMIT;
float align_voltage = FOC_VOLTAGE_ALIGN;
float current_limit = FOC_I_LIM;
uint32_t foc_hz = FOC_LOOP_HZ;
volatile uint32_t dt_us = 0;
volatile uint32_t dt_max_us = 0;
volatile uint32_t foc_late = 0;
TaskHandle_t foc_task_handle = nullptr;

char line[96];
uint8_t line_len = 0;

void setFocHz(uint32_t hz);

void holdPinLow(int8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

const char *dirName(Direction d) {
  if (d == Direction::CW) {
    return "CW";
  }
  if (d == Direction::CCW) {
    return "CCW";
  }
  return "UNKNOWN";
}

const char *modeName(RunMode m) {
  switch (m) {
    case RunMode::Enc:
      return "ENC";
    case RunMode::Openloop:
      return "OL";
    case RunMode::Velocity:
      return "VEL";
    case RunMode::Torque:
      return "TQ";
    case RunMode::Accal:
      return "ACCAL";
    default:
      return "IDLE";
  }
}

void applyLimits() {
  driver.voltage_limit = voltage_limit;
  motor.voltage_limit = voltage_limit;
  motor.PID_current_q.limit = voltage_limit;
  motor.PID_current_d.limit = voltage_limit;
  motor.current_limit = current_limit;
  motor.PID_velocity.limit = current_limit;
  motor.P_angle.limit = FOC_VEL_LIMIT;
}

void applyCurrentPids() {
  motor.PID_current_q.P = FOC_CURR_P;
  motor.PID_current_q.I = FOC_CURR_I;
  motor.PID_current_q.D = 0.0f;
  motor.PID_current_d.P = FOC_CURR_P;
  motor.PID_current_d.I = FOC_CURR_I;
  motor.PID_current_d.D = 0.0f;
  motor.LPF_current_q.Tf = FOC_CURR_LPF;
  motor.LPF_current_d.Tf = FOC_CURR_LPF;
  motor.P_angle.P = FOC_ANGLE_P;
}

void goEnc() {
  mode = RunMode::Enc;
  if (anticog.calibrating()) {
    anticog.abort();
  }
  if (motor_ready) {
    motor.target = 0.0f;
    motor.controller = MotionControlType::torque;
    motor.disable();
  }
  digitalWrite(FOC_PIN_MEN, LOW);
}

void goIdle() { goEnc(); }

void printOta() {
  if (WiFi.getMode() == WIFI_MODE_STA && WiFi.status() == WL_CONNECTED) {
    Serial.printf("ota: STA %s  ip=%s  host=%s\n", WIFI_SSID, WiFi.localIP().toString().c_str(),
                  OTA_HOSTNAME);
    return;
  }
  Serial.printf("ota: AP %s  ip=%s  host=%s  (join this SSID, upload 192.168.4.1)\n",
                OTA_AP_SSID, WiFi.softAPIP().toString().c_str(), OTA_HOSTNAME);
}

void setupOta() {
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);
  bool sta_ok = false;
  if (WIFI_SSID[0] != '\0') {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    const uint32_t t0 = millis();
    while (millis() - t0 < 8000u) {
      if (WiFi.status() == WL_CONNECTED) {
        sta_ok = true;
        break;
      }
      delay(100);
    }
  }
  if (!sta_ok) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(OTA_AP_SSID);
  }
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([]() {
    goEnc();
    Serial.println("ota: start — motor off");
    Serial.flush();
  });
  ArduinoOTA.onError([](ota_error_t err) { Serial.printf("ota: err %u\n", (unsigned)err); });
  ArduinoOTA.begin();
  printOta();
}

void printEncLine() {
  encoder.update();
  Serial.printf("enc axis=%c cnt=%ld idx=%d zn=%lu A=%d B=%d Z=%d ang=%.4f\n", HWCHK_AXIS_CHAR,
                (long)encoder.count(), (int)encoder.indexFound(),
                (unsigned long)encoder.zEdges(), (int)digitalRead(ENC_PIN_A),
                (int)digitalRead(ENC_PIN_B), (int)digitalRead(ENC_PIN_Z),
                (double)encoder.countAngle());
}

void ensureMotorReady() {
  if (motor_ready) {
    return;
  }
  driver.voltage_power_supply = FOC_VBUS;
  driver.pwm_frequency = FOC_PWM_HZ;
  applyLimits();
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&encoder);
  current_ok = current_sense.init() != 0;
  if (current_ok) {
    current_sense.linkDriver(&driver);
    motor.linkCurrentSense(&current_sense);
  }
  motor.voltage_sensor_align = align_voltage;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.target = 0.0f;
  motor.velocity_limit = FOC_VEL_LIMIT;
  motor.PID_velocity.P = FOC_VEL_P;
  motor.PID_velocity.I = FOC_VEL_I;
  motor.PID_velocity.D = FOC_VEL_D;
  motor.PID_velocity.output_ramp = FOC_VEL_RAMP;
  motor.LPF_velocity.Tf = FOC_VEL_LPF;
  applyCurrentPids();
  motor.init();
  motor.disable();
  digitalWrite(FOC_PIN_MEN, LOW);
  motor_ready = true;
}

int64_t encoderCount() { return encoder.count(); }

bool loadCal(CalRecord *out) {
  if (!prefs.begin(kNvsNs, true)) {
    return false;
  }
  const size_t n = prefs.getBytes("cal", out, sizeof(*out));
  prefs.end();
  if (n != sizeof(*out)) {
    return false;
  }
  if (out->magic != kCalMagic || out->axis != HWCHK_AXIS_CHAR) {
    return false;
  }
  if (out->pole_pairs == 0 || out->sensor_direction == 0) {
    return false;
  }
  if (!_isset(out->zero_electric_angle)) {
    return false;
  }
  return true;
}

bool saveCal() {
  if (anticog.calibrating()) {
    Serial.println("save: accal running — wait or enc");
    return false;
  }
  if (!calibrated || motor.sensor_direction == Direction::UNKNOWN ||
      !_isset(motor.zero_electric_angle)) {
    Serial.println("save: not calibrated");
    return false;
  }
  CalRecord rec{};
  rec.magic = kCalMagic;
  rec.zero_electric_angle = motor.zero_electric_angle;
  rec.sensor_direction = static_cast<int8_t>(motor.sensor_direction);
  rec.pole_pairs = static_cast<uint8_t>(motor.pole_pairs);
  rec.axis = HWCHK_AXIS_CHAR;
  rec.reserved = 0;
  if (!prefs.begin(kNvsNs, false)) {
    Serial.println("save: nvs fail");
    return false;
  }
  const size_t n = prefs.putBytes("cal", &rec, sizeof(rec));
  prefs.end();
  if (n != sizeof(rec)) {
    Serial.println("save: write fail");
    return false;
  }
  Serial.printf("save: axis=%c zero=%.4f dir=%s pp=%u  (reboot: zsearch then this zero)\n",
                rec.axis, (double)rec.zero_electric_angle, dirName(motor.sensor_direction),
                (unsigned)rec.pole_pairs);
  return true;
}

void forgetCal() {
  goIdle();
  calibrated = false;
  motor.sensor_direction = Direction::UNKNOWN;
  motor.zero_electric_angle = NOT_SET;
  if (prefs.begin(kNvsNs, false)) {
    prefs.remove("cal");
    prefs.end();
  }
  Serial.println("forget: nvs cleared, run cal");
}

void printStatus() {
  Serial.printf(
      "axis=%c mode=%s cal=%d idx=%d zn=%lu jmp=%lu cnt=%ld ang=%.4f vel=%.3f "
      "tgt=%.3f Uq=%.3f Iq=%.3f Isp=%.3f zero=%.4f dir=%s lim=%.2f ilim=%.2f "
      "hz=%u dt=%u dtmax=%u late=%u cs=%d ac=%d/%u\n",
      HWCHK_AXIS_CHAR, modeName(mode), (int)calibrated, (int)encoder.indexFound(),
      (unsigned long)encoder.zEdges(), (unsigned long)encoder.jumps(), (long)encoderCount(),
      (double)encoder.getAngle(), (double)motor.shaft_velocity, (double)motor.target,
      (double)motor.voltage.q, (double)motor.current.q, (double)motor.current_sp,
      (double)motor.zero_electric_angle, dirName(motor.sensor_direction),
      (double)voltage_limit, (double)current_limit, (unsigned)foc_hz, (unsigned)dt_us,
      (unsigned)dt_max_us, (unsigned)foc_late, (int)current_ok, (int)anticog.enabled(),
      (unsigned)anticog.bins());
}

void printHelp() {
  Serial.println("ESP32FOCHardwareCheck  FS2804  7 pp  current+DMA  MT6835 ABZ");
  Serial.println("  wheel UP.  enc → ol 3 → cal + save → vel 3 / tq 0.2");
  Serial.println("  enc                  motor off, cnt/zn @ 1 Hz");
  Serial.println("  ol <rad/s>           open-loop voltage, no index");
  Serial.println("  cal                  Z search + electrical align");
  Serial.println("  zsearch save forget  index only / NVS electrical");
  Serial.println("  vel <rad/s>          tq <A>   idle");
  Serial.println("  limit <V>  ilim <A>  hz <Hz>  alignv <V>  mon 0|1");
  Serial.println("  accal [bins]         ac on|off|save|forget   dt  csflip  csoff  jlog  wifioff");
  Serial.println("  status  ota  download");
}

bool applySavedElectrical() {
  CalRecord rec{};
  if (!loadCal(&rec)) {
    return false;
  }
  motor.pole_pairs = rec.pole_pairs;
  motor.sensor_direction = static_cast<Direction>(rec.sensor_direction);
  motor.zero_electric_angle = rec.zero_electric_angle;
  Serial.printf("nvs: zero=%.4f dir=%s pp=%u\n", (double)rec.zero_electric_angle,
                dirName(motor.sensor_direction), (unsigned)rec.pole_pairs);
  return true;
}

bool runZSearch(bool park) {
  ensureMotorReady();
  if (park) {
    goIdle();
  }
  encoder.clearIndex();
  applyLimits();
  motor.voltage_sensor_align = align_voltage;
  motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::voltage;
  motor.target = Z_SEARCH_RPS * _2PI;
  motor.enable();
  const uint32_t timeout_ms =
      (uint32_t)(Z_SEARCH_TURNS / Z_SEARCH_RPS * 1000.0f) + 200u;
  Serial.printf("zsearch: %.2f rps  timeout=%.1f turn\n", (double)Z_SEARCH_RPS,
                (double)Z_SEARCH_TURNS);
  const uint32_t t0 = millis();
  while (!encoder.indexFound() && (millis() - t0) < timeout_ms) {
    motor.loopFOC();
    motor.move();
  }
  const bool ok = encoder.indexFound();
  motor.target = 0.0f;
  motor.disable();
  if (park) {
    goEnc();
  }
  Serial.printf("zsearch: %s  cnt=%ld  Z=GPIO%d\n", ok ? "ok" : "FAIL (no index)",
                (long)encoder.count(), ENC_PIN_Z);
  return ok;
}

constexpr float kAlignOlRadS = 3.0f;
constexpr uint32_t kAlignOlMs = 500;
constexpr float kAlignMinRad = 0.25f;

bool runOpenloopDirectionAndZero() {
  applyLimits();
  motor.voltage_sensor_align = align_voltage;
  motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::voltage;
  motor.target = kAlignOlRadS;
  motor.enable();
  digitalWrite(FOC_PIN_MEN, HIGH);

  encoder.update();
  const int64_t cnt0 = encoder.count();
  Serial.printf("align: ol %.1f rad/s  Ulim=%.2f  MEN=%d  cnt=%ld\n", (double)kAlignOlRadS,
                (double)voltage_limit, (int)digitalRead(FOC_PIN_MEN), (long)cnt0);

  const uint32_t t0 = millis();
  while ((millis() - t0) < kAlignOlMs) {
    motor.loopFOC();
    motor.move();
  }

  encoder.update();
  const int64_t cnt1 = encoder.count();
  const int64_t dcnt = cnt1 - cnt0;
  const float moved = (float)dcnt * (_2PI / encoder.cpr());
  motor.target = 0.0f;
  motor.move();
  Serial.printf("align: ol moved=%.4f rad  dcnt=%ld\n", (double)moved, (long)dcnt);

  if (fabsf(moved) < kAlignMinRad) {
    Serial.println("align: FAIL — ol did not move the shaft");
    motor.disable();
    return false;
  }

  motor.sensor_direction = (dcnt > 0) ? Direction::CW : Direction::CCW;
  motor.setPhaseVoltage(align_voltage, 0.0f, _3PI_2);
  delay(700);
  encoder.update();
  motor.zero_electric_angle = 0.0f;
  motor.zero_electric_angle = motor.electricalAngle();
  motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
  motor.disable();
  Serial.printf("align: dir=%s  zero=%.4f\n", dirName(motor.sensor_direction),
                (double)motor.zero_electric_angle);
  return _isset(motor.zero_electric_angle);
}

int runInitFoc() {
  applyLimits();
  motor.voltage_sensor_align = align_voltage;
  if (motor.sensor_direction == Direction::UNKNOWN || !_isset(motor.zero_electric_angle)) {
    if (!runOpenloopDirectionAndZero()) {
      return 0;
    }
  }
  motor.enable();
  digitalWrite(FOC_PIN_MEN, HIGH);
  const int ok = motor.initFOC();
  Serial.printf("align: initFOC=%d  MEN=%d  zero=%.4f  dir=%s\n", ok,
                (int)digitalRead(FOC_PIN_MEN), (double)motor.zero_electric_angle,
                dirName(motor.sensor_direction));
  motor.disable();
  motor.target = 0.0f;
  return ok;
}

void finishAfterIndex() {
  if (!applySavedElectrical()) {
    Serial.println("zsearch: no nvs electrical — run cal");
    calibrated = false;
    goEnc();
    return;
  }
  const int ok = runInitFoc();
  goEnc();
  calibrated = (ok != 0) && motor.sensor_direction != Direction::UNKNOWN &&
               _isset(motor.zero_electric_angle);
  Serial.printf("zsearch: electrical %s  initFOC=%d\n", calibrated ? "ok" : "FAIL", ok);
}

void runCal() {
  ensureMotorReady();
  Serial.println("cal: Z search then ol 3 for dir/zero — motor will spin");
  if (!runZSearch(false)) {
    calibrated = false;
    goEnc();
    Serial.println("cal: FAIL (Z)");
    return;
  }
  motor.sensor_direction = Direction::UNKNOWN;
  motor.zero_electric_angle = NOT_SET;
  const int ok = runInitFoc();
  goEnc();
  calibrated = (ok != 0) && encoder.indexFound() &&
               motor.sensor_direction != Direction::UNKNOWN &&
               _isset(motor.zero_electric_angle);
  Serial.printf("cal: %s  zero=%.4f (%.1f deg) dir=%s  initFOC=%d\n",
                calibrated ? "ok" : "FAIL", (double)motor.zero_electric_angle,
                (double)(motor.zero_electric_angle * (180.0f / _PI)),
                dirName(motor.sensor_direction), ok);
  if (calibrated) {
    Serial.println("cal: type save — next boot only needs zsearch");
  }
}

void runZSearchCommand() {
  if (!runZSearch(true)) {
    calibrated = false;
    return;
  }
  finishAfterIndex();
}

bool requireCal() {
  if (!calibrated || !encoder.indexFound()) {
    Serial.println("need cal (or zsearch after save)");
    return false;
  }
  return true;
}

bool requireCurrent() {
  if (!current_ok) {
    Serial.println("need current — ADC DMA init failed");
    return false;
  }
  return true;
}

/**
 * Boot offsets are taken with M_EN low / PWM 0. FOC at Iq=0 uses centered
 * SinePWM (Ua=Ub=Uc=Ulim/2). The mid-rail then differs, Park(offset) is a
 * 7/rev phantom Iq, and the current PIDs torque L then R.
 * Recalibrate at that PWM with foc_task still in Enc so it does not write.
 */
bool calCurrentOffsets() {
  if (!requireCurrent()) {
    return false;
  }
  mode = RunMode::Enc;
  applyLimits();
  motor.enable();
  digitalWrite(FOC_PIN_MEN, HIGH);
  const float mid = 0.5f * driver.voltage_limit;
  driver.setPwm(mid, mid, mid);
  delay(80);
  current_sense.calibrateOffsets();

  float ia = 0.0f;
  float ib = 0.0f;
  float iq = 0.0f;
  float id = 0.0f;
  encoder.update();
  const float el = motor.electricalAngle();
  constexpr int kN = 32;
  for (int i = 0; i < kN; ++i) {
    const PhaseCurrent_s c = current_sense.getPhaseCurrents();
    ia += c.a;
    ib += c.b;
    const DQCurrent_s dq = current_sense.getDQCurrents(current_sense.getABCurrents(c), el);
    iq += dq.q;
    id += dq.d;
    delay(2);
  }
  ia /= (float)kN;
  ib /= (float)kN;
  iq /= (float)kN;
  id /= (float)kN;
  Serial.printf("cs: midrail U=%.2f  offA=%.3f V offB=%.3f V  Ia=%.3f Ib=%.3f Id=%.3f Iq=%.3f A\n",
                (double)mid, (double)current_sense.offsetA(), (double)current_sense.offsetB(),
                (double)ia, (double)ib, (double)id, (double)iq);
  if (fabsf(ia) > 0.08f || fabsf(ib) > 0.08f || fabsf(iq) > 0.08f) {
    Serial.println("cs: residual high after cal — try csflip if tq 0 still walks");
  }
  driver.setPwm(0.0f, 0.0f, 0.0f);
  return true;
}

void enterOpenloop(float rad_s) {
  ensureMotorReady();
  if (rad_s > FOC_VEL_LIMIT) {
    rad_s = FOC_VEL_LIMIT;
  } else if (rad_s < -FOC_VEL_LIMIT) {
    rad_s = -FOC_VEL_LIMIT;
  }
  applyLimits();
  motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::voltage;
  motor.target = rad_s;
  motor.enable();
  mode = RunMode::Openloop;
  Serial.printf("ol: tgt=%.2f rad/s  cap=%.0f  Ulim=%.2f  (open-loop)\n", (double)rad_s,
                (double)FOC_VEL_LIMIT, (double)voltage_limit);
}

void enterVelocity(float rad_s) {
  if (!requireCal()) {
    return;
  }
  ensureMotorReady();
  if (rad_s > FOC_VEL_LIMIT) {
    rad_s = FOC_VEL_LIMIT;
  } else if (rad_s < -FOC_VEL_LIMIT) {
    rad_s = -FOC_VEL_LIMIT;
  }
  if (!requireCurrent()) {
    return;
  }
  applyLimits();
  applyCurrentPids();
  if (!calCurrentOffsets()) {
    return;
  }
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.target = rad_s;
  motor.enable();
  mode = RunMode::Velocity;
  Serial.printf("vel: %.3f rad/s  (Iq loop)\n", (double)rad_s);
}

void enterTorque(float iq) {
  if (!requireCal()) {
    return;
  }
  ensureMotorReady();
  if (!requireCurrent()) {
    return;
  }
  if (iq > current_limit) {
    iq = current_limit;
  } else if (iq < -current_limit) {
    iq = -current_limit;
  }
  applyLimits();
  applyCurrentPids();
  if (!calCurrentOffsets()) {
    return;
  }
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.target = iq;
  motor.enable();
  mode = RunMode::Torque;
  Serial.printf("tq: Iq=%.3f A  (current torque, not V / not Nm)\n", (double)iq);
}

void enterAccal(uint16_t bins) {
  if (!requireCal() || !requireCurrent()) {
    return;
  }
  if (anticog.calibrating()) {
    Serial.println("accal: already running — enc to abort");
    return;
  }
  ensureMotorReady();
  applyLimits();
  applyCurrentPids();
  if (!calCurrentOffsets()) {
    return;
  }
  encoder.update();
  ac_finished = false;
  anticog.start(bins, encoder.getAngle());
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.target = anticog.holdAngle();
  motor.enable();
  mode = RunMode::Accal;
  Serial.printf("accal: %u bins  2 passes  wheel FREE  (enc aborts, never wait index)\n",
                (unsigned)anticog.bins());
}

void printAc() {
  Serial.printf("ac: valid=%d on=%d calib=%d bins=%u phase=%u idx=%u hold=%.4f\n",
                (int)anticog.valid(), (int)anticog.enabled(), (int)anticog.calibrating(),
                (unsigned)anticog.bins(), (unsigned)anticog.phase(),
                (unsigned)anticog.index(), (double)anticog.holdAngle());
}

char *skipSpaces(char *s) {
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  return s;
}

void handleLine(char *raw) {
  char *s = skipSpaces(raw);
  if (*s == '\0') {
    return;
  }
  char *arg = s;
  while (*arg && *arg != ' ' && *arg != '\t') {
    ++arg;
  }
  if (*arg) {
    *arg++ = '\0';
    arg = skipSpaces(arg);
  }

  if (strcmp(s, "help") == 0 || strcmp(s, "?") == 0) {
    printHelp();
    return;
  }
  if (strcmp(s, "status") == 0) {
    printStatus();
    return;
  }
  if (strcmp(s, "enc") == 0) {
    goEnc();
    Serial.println("enc: motor off — spin the shaft, cnt @ 1 Hz");
    return;
  }
  if (strcmp(s, "cal") == 0) {
    runCal();
    return;
  }
  if (strcmp(s, "zsearch") == 0) {
    runZSearchCommand();
    return;
  }
  if (strcmp(s, "save") == 0) {
    saveCal();
    return;
  }
  if (strcmp(s, "forget") == 0) {
    forgetCal();
    return;
  }
  if (strcmp(s, "idle") == 0 || strcmp(s, "stop") == 0) {
    goEnc();
    Serial.println("idle → enc");
    return;
  }
  if (strcmp(s, "ol") == 0 || strcmp(s, "openloop") == 0) {
    if (*arg == '\0') {
      Serial.println("usage: ol <rad/s>");
      return;
    }
    enterOpenloop(strtof(arg, nullptr));
    return;
  }
  if (strcmp(s, "vel") == 0) {
    if (*arg == '\0') {
      Serial.println("usage: vel <rad/s>");
      return;
    }
    enterVelocity(strtof(arg, nullptr));
    return;
  }
  if (strcmp(s, "tq") == 0 || strcmp(s, "torque") == 0) {
    if (*arg == '\0') {
      Serial.println("usage: tq <A>");
      return;
    }
    enterTorque(strtof(arg, nullptr));
    return;
  }
  if (strcmp(s, "accal") == 0) {
    uint16_t bins = ACOG_BINS_DEFAULT;
    if (*arg != '\0') {
      const long v = strtol(arg, nullptr, 10);
      if (v > 0) {
        bins = (uint16_t)v;
      }
    }
    enterAccal(bins);
    return;
  }
  if (strcmp(s, "ac") == 0) {
    if (*arg == '\0' || strcmp(arg, "status") == 0) {
      printAc();
      return;
    }
    if (strcmp(arg, "on") == 0) {
      if (!anticog.valid()) {
        Serial.println("ac: no map — accal then ac save");
        return;
      }
      if (anticog.calibrating()) {
        Serial.println("ac: wait for accal (calib, not index)");
        return;
      }
      anticog.setEnabled(true);
      Serial.println("ac: on");
      return;
    }
    if (strcmp(arg, "off") == 0) {
      anticog.setEnabled(false);
      Serial.println("ac: off");
      return;
    }
    if (strcmp(arg, "save") == 0) {
      if (anticog.calibrating()) {
        Serial.println("ac: save forbidden during accal");
        return;
      }
      Serial.printf("ac: save %s\n", anticog.save() ? "ok" : "FAIL");
      return;
    }
    if (strcmp(arg, "forget") == 0) {
      anticog.forget();
      Serial.println("ac: nvs cleared");
      return;
    }
    Serial.println("usage: ac on|off|save|forget");
    return;
  }
  if (strcmp(s, "hz") == 0) {
    if (*arg == '\0') {
      Serial.printf("hz: %u  dt=%u  dtmax=%u  T=%.0f us\n", (unsigned)foc_hz, (unsigned)dt_us,
                    (unsigned)dt_max_us, 1e6 / (double)foc_hz);
      return;
    }
    setFocHz((uint32_t)strtoul(arg, nullptr, 10));
    Serial.printf("hz: %u\n", (unsigned)foc_hz);
    return;
  }
  if (strcmp(s, "ilim") == 0) {
    if (*arg == '\0') {
      Serial.println("usage: ilim <A>");
      return;
    }
    current_limit = strtof(arg, nullptr);
    if (current_limit < 0.05f) {
      current_limit = 0.05f;
    }
    if (current_limit > 2.0f) {
      current_limit = 2.0f;
    }
    applyLimits();
    Serial.printf("ilim: %.2f A\n", (double)current_limit);
    return;
  }
  if (strcmp(s, "csflip") == 0) {
    current_sense.flipGains();
    Serial.printf("csflip: gainA=%.2f gainB=%.2f\n", (double)current_sense.gainA(),
                  (double)current_sense.gainB());
    return;
  }
  if (strcmp(s, "csoff") == 0) {
    ensureMotorReady();
    if (calCurrentOffsets()) {
      goEnc();
    }
    return;
  }
  if (strcmp(s, "dt") == 0) {
    dt_max_us = 0;
    foc_late = 0;
    Serial.println("dt: max/late cleared");
    return;
  }
  if (strcmp(s, "jlog") == 0) {
    Serial.printf(
        "jlog: jumps_total=%lu  overflow_events=%lu  (flagged when the residual after removing "
        "N clean kPcntLim wraps implies a velocity no real shaft can reach)\n",
        (unsigned long)encoder.jumps(), (unsigned long)encoder.overflowEvents());
    const uint8_t cap = encoder.jumpLogCap();
    for (uint8_t i = 0; i < cap; ++i) {
      const PcntEncoder::JumpEvent ev = encoder.jumpLogAt(i);
      if (ev.seq == 0) {
        continue;
      }
      Serial.printf("  seq=%lu  delta=%ld  elapsed_us=%lu  wrap_explained=%ld\n",
                    (unsigned long)ev.seq, (long)ev.delta, (unsigned long)ev.elapsed_us,
                    (long)ev.overflow_delta);
    }
    return;
  }
  if (strcmp(s, "limit") == 0) {
    if (*arg == '\0') {
      Serial.println("usage: limit <V>");
      return;
    }
    voltage_limit = strtof(arg, nullptr);
    if (voltage_limit < 0.2f) {
      voltage_limit = 0.2f;
    }
    if (voltage_limit > FOC_VBUS) {
      voltage_limit = FOC_VBUS;
    }
    applyLimits();
    Serial.printf("limit: %.2f V\n", (double)voltage_limit);
    return;
  }
  if (strcmp(s, "alignv") == 0) {
    if (*arg == '\0') {
      Serial.println("usage: alignv <V>");
      return;
    }
    align_voltage = strtof(arg, nullptr);
    if (align_voltage < 0.2f) {
      align_voltage = 0.2f;
    }
    if (align_voltage > voltage_limit) {
      align_voltage = voltage_limit;
    }
    motor.voltage_sensor_align = align_voltage;
    Serial.printf("alignv: %.2f V\n", (double)align_voltage);
    return;
  }
  if (strcmp(s, "mon") == 0) {
    monitor_on = (*arg != '0');
    Serial.printf("mon: %d\n", (int)monitor_on);
    return;
  }
  if (strcmp(s, "ota") == 0) {
    printOta();
    return;
  }
  if (strcmp(s, "wifioff") == 0) {
    WiFi.mode(WIFI_OFF);
    Serial.println("wifioff: WiFi off (reboot to restore OTA)");
    return;
  }
  if (strcmp(s, "download") == 0 || strcmp(s, "dl") == 0) {
    goEnc();
    Serial.println("download: IO0 hold low + restart → ROM");
    Serial.flush();
    delay(50);
    /* ESP32 (not S2/S3): no RTC_CNTL_FORCE_DOWNLOAD_BOOT. Keep GPIO0 low
     * across esp_restart() via the RTC pad hold so ROM samples download. */
    rtc_gpio_init(GPIO_NUM_0);
    rtc_gpio_set_direction(GPIO_NUM_0, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_0, 0);
    rtc_gpio_hold_en(GPIO_NUM_0);
    esp_restart();
    return;
  }
  Serial.println("unknown — help");
}

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (line_len == 0) {
        continue;
      }
      line[line_len] = '\0';
      line_len = 0;
      Serial.write('\n');
      handleLine(line);
      continue;
    }
    if (c == 0x08 || c == 0x7f) {
      if (line_len > 0) {
        line_len -= 1;
        Serial.print("\b \b");
      }
      continue;
    }
    if (line_len + 1u < sizeof(line)) {
      line[line_len++] = c;
      Serial.write(c);
    }
  }
}

void setupEncoder() { encoder.init(); }

bool IRAM_ATTR focTimerCb(void *) {
  BaseType_t hpw = pdFALSE;
  if (foc_task_handle != nullptr) {
    vTaskNotifyGiveFromISR(foc_task_handle, &hpw);
  }
  return hpw == pdTRUE;
}

void setFocHz(uint32_t hz) {
  if (hz < FOC_LOOP_HZ_MIN) {
    hz = FOC_LOOP_HZ_MIN;
  }
  if (hz > FOC_LOOP_HZ_MAX) {
    hz = FOC_LOOP_HZ_MAX;
  }
  foc_hz = hz;
  timer_pause(TIMER_GROUP_1, TIMER_0);
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 1000000ull / (uint64_t)foc_hz);
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
  dt_max_us = 0;
  timer_start(TIMER_GROUP_1, TIMER_0);
}

void startFocTimer() {
  timer_config_t cfg{};
  cfg.alarm_en = TIMER_ALARM_EN;
  cfg.counter_en = TIMER_PAUSE;
  cfg.intr_type = TIMER_INTR_LEVEL;
  cfg.counter_dir = TIMER_COUNT_UP;
  cfg.auto_reload = TIMER_AUTORELOAD_EN;
  cfg.divider = 80;
  timer_init(TIMER_GROUP_1, TIMER_0, &cfg);
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 1000000ull / (uint64_t)foc_hz);
  timer_enable_intr(TIMER_GROUP_1, TIMER_0);
  timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, focTimerCb, nullptr, ESP_INTR_FLAG_IRAM);
  timer_start(TIMER_GROUP_1, TIMER_0);
}

void focTask(void *) {
  startFocTimer();
  for (;;) {
    const uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (n > 1u) {
      foc_late += n - 1u;
    }
    const int64_t t0 = esp_timer_get_time();
    const RunMode m = mode;
    if (m == RunMode::Openloop || m == RunMode::Velocity || m == RunMode::Torque ||
        m == RunMode::Accal) {
      motor.loopFOC();
      if (m == RunMode::Accal && anticog.calibrating()) {
        const bool done =
            anticog.tick(encoder.getAngle(), motor.shaft_velocity, motor.current_sp, foc_hz);
        motor.target = anticog.holdAngle();
        if (done) {
          ac_finished = true;
        }
      }
      motor.move();
      if (m != RunMode::Accal && m != RunMode::Openloop && anticog.enabled()) {
        motor.current_sp += anticog.feedforward(encoder.getAngle());
      }
    }
    const uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
    dt_us = dt;
    if (dt > dt_max_us) {
      dt_max_us = dt;
    }
  }
}

void startFocTask() {
  /* Core 0: timer ISR + FOC. loop() on core 1 never yields except during a
   * blocking UART TX (tx buffer = 0). Same-core FOC only ran for that ~16 ms
   * print — 18° at ol 20, and mon 0 stopped the shaft entirely. */
  xTaskCreatePinnedToCore(focTask, "foc", 4096, nullptr, 20, &foc_task_handle, 0);
}

}  // namespace

void setup() {
  holdPinLow(FOC_PIN_MEN);
  Serial.setTxBufferSize(512);
  Serial.begin(115200);
  delay(200);

  holdPinLow(HWCHK_PIN_OTHER_UH);
  holdPinLow(HWCHK_PIN_OTHER_VH);
  holdPinLow(HWCHK_PIN_OTHER_WH);

  setupEncoder();

  Serial.printf("ESP32FOCHardwareCheck axis=%c  FS2804 pp=%d  Vbus=%.1f  Ulim=%.1f\n",
                HWCHK_AXIS_CHAR, FOC_POLE_PAIRS, (double)FOC_VBUS, (double)voltage_limit);
  Serial.printf("  ABZ A=GPIO%d B=GPIO%d Z=GPIO%d  swapAB=%d  PCNT%u  PPR=%u  CPR=%.0f  pcnt=%s\n",
                ENC_PIN_A, ENC_PIN_B, ENC_PIN_Z, ENC_SWAP_AB, (unsigned)ENC_PCNT_UNIT,
                (unsigned)ENC_PPR, (double)encoder.cpr(), encoder.ok() ? "ok" : "FAIL");
  Serial.printf("  PWM %d/%d/%d  M_EN=GPIO%d  other PWM %d/%d/%d LOW\n", HWCHK_PIN_UH,
                HWCHK_PIN_VH, HWCHK_PIN_WH, FOC_PIN_MEN, HWCHK_PIN_OTHER_UH,
                HWCHK_PIN_OTHER_VH, HWCHK_PIN_OTHER_WH);
  Serial.printf("  I sense GPIO%d/%d  R=%.3f x%.0f  loop=%u Hz  ilim=%.2f A\n", CS_PIN_A,
                CS_PIN_B, (double)CS_SHUNT_OHM, (double)CS_AMP_GAIN, (unsigned)foc_hz,
                (double)current_limit);

  goEnc();
  setupOta();
  startFocTask();
  if (current_sense.init()) {
    current_ok = true;
    Serial.printf("  ADC DMA ok  offA=%.3f V  offB=%.3f V  samples=%lu\n",
                  (double)current_sense.offsetA(), (double)current_sense.offsetB(),
                  (unsigned long)DmaAdc::samples());
  } else {
    current_ok = false;
    Serial.println("  ADC DMA FAIL — ol still works, vel/tq/accal need current");
  }
  if (anticog.load()) {
    Serial.printf("nvs: acog bins=%u  (ac on after zsearch)\n", (unsigned)anticog.bins());
  }
  Serial.println("mode ENC — motor off. Turn the shaft; cnt must move. Then cal.");
  printHelp();
}

void loop() {
  pollSerial();
  ArduinoOTA.handle();
  if (ac_finished) {
    ac_finished = false;
    goEnc();
    Serial.printf("accal: done bins=%u  type ac save then ac on\n", (unsigned)anticog.bins());
  }
  if (mode == RunMode::Enc || mode == RunMode::Idle) {
    encoder.update();
  }

  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (mode == RunMode::Accal) {
    if ((now - last_ms) >= kMonPeriodMs) {
      last_ms = now;
      printAc();
    }
  } else if (mode == RunMode::Enc) {
    if ((now - last_ms) >= kEncPeriodMs) {
      last_ms = now;
      printEncLine();
    }
  } else if (monitor_on && (now - last_ms) >= kMonPeriodMs) {
    last_ms = now;
    printStatus();
  }
  vTaskDelay(1);
}
