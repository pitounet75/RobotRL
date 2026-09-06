/**
 * Dual 2804 gimbal balancer — ESP32 FOC, 2× MT6835, ICM45686.
 */

#include <Arduino.h>
#include <SPI.h>
#include <SimpleFOC.h>
#include <math.h>
#include <stdlib.h>

#include "config.h"
#include "icm45686.h"
#include "mt6835_sensor.h"
#include "pitch_fusion.h"

static Mt6835Sensor enc_l(MT6835_PIN_CS_L);
static Mt6835Sensor enc_r(MT6835_PIN_CS_R);
static Icm45686 imu(ICM_PIN_CS);
static PitchFusionState fusion;
static bool imu_ok = false;

#if FOC_MODE != 0
static BLDCMotor motor_l(FOC_POLE_PAIRS);
static BLDCMotor motor_r(FOC_POLE_PAIRS);
#if FOC_PIN_L_EN >= 0
static BLDCDriver3PWM drv_l(FOC_PIN_L_UH, FOC_PIN_L_VH, FOC_PIN_L_WH, FOC_PIN_L_EN);
#else
static BLDCDriver3PWM drv_l(FOC_PIN_L_UH, FOC_PIN_L_VH, FOC_PIN_L_WH);
#endif
#if FOC_PIN_R_EN >= 0
static BLDCDriver3PWM drv_r(FOC_PIN_R_UH, FOC_PIN_R_VH, FOC_PIN_R_WH, FOC_PIN_R_EN);
#else
static BLDCDriver3PWM drv_r(FOC_PIN_R_UH, FOC_PIN_R_VH, FOC_PIN_R_WH);
#endif
static Commander command = Commander(Serial);
static void onMotorL(char *cmd) { command.motor(&motor_l, cmd); }
static void onMotorR(char *cmd) { command.motor(&motor_r, cmd); }
static void onTargetBoth(char *cmd) {
  const float t = (float)atof(cmd);
  motor_l.target = (float)FOC_CMD_SIGN_L * t;
  motor_r.target = (float)FOC_CMD_SIGN_R * t;
}

static void setupDriver(BLDCDriver3PWM &d) {
  d.voltage_power_supply = FOC_VBUS;
  d.voltage_limit = FOC_VOLTAGE_LIMIT;
  d.pwm_frequency = FOC_PWM_HZ;
  d.init();
}

static void setupMotor(BLDCMotor &m, BLDCDriver3PWM &d, Mt6835Sensor &s) {
  m.linkDriver(&d);
  m.linkSensor(&s);
  m.voltage_limit = FOC_VOLTAGE_LIMIT;
  m.voltage_sensor_align = FOC_VOLTAGE_ALIGN;
  m.foc_modulation = FOCModulationType::SinePWM;
  m.torque_controller = TorqueControlType::voltage;
#if FOC_MODE == 1
  m.controller = MotionControlType::velocity_openloop;
  m.target = FOC_OPENLOOP_VEL;
#else
  m.controller = MotionControlType::torque;
  m.target = 0.0f;
#endif
  m.init();
  m.initFOC();
}
#endif

static void holdCsHigh(int8_t pin) {
  if (pin < 0) {
    return;
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}

static void initSharedSpi() {
  holdCsHigh(MT6835_PIN_CS_L);
  holdCsHigh(MT6835_PIN_CS_R);
  holdCsHigh(ICM_PIN_CS);
  pinMode(PIN_SPI_MISO, INPUT_PULLUP);
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
  SPI.setHwCs(false);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setFrequency(MT6835_SPI_HZ);
}

static void probeEncoder(const char *name, Mt6835Sensor &s) {
  Serial.printf("  %s CS=GPIO%d  USER_ID=0x%02X  angle=0x%06lX  bus=%s\n", name,
                (int)s.chip().csPin(), (unsigned)s.chip().readUserId(),
                (unsigned long)s.chip().readAngleRaw(),
                s.chip().busLooksAlive() ? "yes" : "NO");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  initSharedSpi();
  enc_l.init();
  enc_r.init();
  imu.beginCs();
  const int imu_err = imu.init();
  imu_ok = (imu_err == 0);
  pitch_fusion_reset(&fusion);

  Serial.printf("ESP32FOC balancer MODE=%d  PP=%d  Vbus=%.1f  Ulim=%.1f\n", FOC_MODE,
                FOC_POLE_PAIRS, (double)FOC_VBUS, (double)FOC_VOLTAGE_LIMIT);
  probeEncoder("encL", enc_l);
  probeEncoder("encR", enc_r);
  Serial.printf("  IMU  CS=GPIO%d  WHO=0x%02X  mode=%u  %s\n", ICM_PIN_CS,
                (unsigned)imu.lastWhoAmI(), (unsigned)imu.spiMode(),
                imu_ok ? "ok" : (imu_err == -2 ? "WHO mismatch" : "SPI fail"));

#if FOC_MODE == 0
  Serial.println("sensors only. openloop / foc / balance to enable PWM.");
#else
  setupDriver(drv_l);
  setupDriver(drv_r);
  Serial.println("initFOC left...");
  setupMotor(motor_l, drv_l, enc_l);
  Serial.println("initFOC right...");
  setupMotor(motor_r, drv_r, enc_r);
  command.add('L', onMotorL, "left");
  command.add('R', onMotorR, "right");
  command.add('T', onTargetBoth, "both Uq or vel");
#if FOC_MODE == 3
  Serial.println("balance: PD on. failsafe |pitch|.");
#else
  Serial.println("Commander: T<val> both, L/R for one axis.");
#endif
#endif
}

#if FOC_MODE == 3
static void balanceStep() {
  static float integ = 0.0f;
  static uint32_t t_prev_us = 0;
  float acc[3] = {0, 0, 0};
  float gyr[3] = {0, 0, 0};
  PitchFusionOut po = {0, 0};
  if (!imu_ok || !imu.read(acc, gyr, nullptr) ||
      !pitch_fusion_update(&fusion, acc, gyr, (uint32_t)micros(), &po)) {
    motor_l.target = 0;
    motor_r.target = 0;
    return;
  }
  if (fabsf(po.pitch_rad) > BAL_FAILSAFE_RAD) {
    integ = 0.0f;
    motor_l.target = 0;
    motor_r.target = 0;
    return;
  }
  const float err = BAL_PITCH_REF_RAD - po.pitch_rad;
  const uint32_t t_us = (uint32_t)micros();
  float dt_s = (float)(t_us - t_prev_us) * 1e-6f;
  if (t_prev_us == 0u || dt_s <= 0.0f || dt_s > 0.05f) {
    dt_s = 0.001f;
  }
  t_prev_us = t_us;
  integ += err * dt_s;
  if (integ > BAL_I_LIMIT) {
    integ = BAL_I_LIMIT;
  } else if (integ < -BAL_I_LIMIT) {
    integ = -BAL_I_LIMIT;
  }

  /* Light PID → desired pitch accel, then invert I θ̈ − mgh sin(θ). */
  const float theta_ddot_des = BAL_K_PITCH * err + BAL_K_I * integ - BAL_K_RATE * po.pitch_rate_rads;
  float u = BAL_K_INERTIA * theta_ddot_des - BAL_K_FF * sinf(po.pitch_rad);
  if (u > FOC_VOLTAGE_LIMIT) {
    u = FOC_VOLTAGE_LIMIT;
  } else if (u < -FOC_VOLTAGE_LIMIT) {
    u = -FOC_VOLTAGE_LIMIT;
  }
  motor_l.target = (float)FOC_CMD_SIGN_L * u;
  motor_r.target = (float)FOC_CMD_SIGN_R * u;
}
#endif

void loop() {
#if FOC_MODE == 0
  enc_l.update();
  enc_r.update();
  float acc[3] = {0, 0, 0};
  float gyr[3] = {0, 0, 0};
  float temp = 0;
  PitchFusionOut po = {0, 0};
  if (imu_ok && imu.read(acc, gyr, &temp)) {
    (void)pitch_fusion_update(&fusion, acc, gyr, (uint32_t)micros(), &po);
  }
  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (now - last_ms >= 200u) {
    last_ms = now;
    Serial.printf("L=%.3f R=%.3f  pitch=%.3f rate=%.3f  T=%.1fC  imu=%d\n",
                  (double)enc_l.getAngle(), (double)enc_r.getAngle(), (double)po.pitch_rad,
                  (double)po.pitch_rate_rads, (double)temp, (int)imu_ok);
  }
#else
#if FOC_MODE == 3
  balanceStep();
#endif
  motor_l.loopFOC();
  motor_r.loopFOC();
  motor_l.move();
  motor_r.move();
  command.run();
#endif
}
