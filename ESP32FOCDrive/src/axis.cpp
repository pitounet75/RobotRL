#include "axis.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "board.h"
#include "cal_record.h"
#include "foc_task.h"

Axis::Axis(int i)
    : name(kAxisName[i]),
      idx(i),
      cmd_sign(kAxisCmdSign[i]),
      present(AXIS_PRESENT(i) != 0),
      encoder(kAxisEnc[i][0], kAxisEnc[i][1], kAxisEnc[i][2], ENC_PPR, pcnt_unit_t(i),
              6.0f * FOC_VEL_LIMIT, ENC_VEL_MIN_DT),
      driver(kAxisPwm[i][0], kAxisPwm[i][1], kAxisPwm[i][2]),
      motor(FOC_POLE_PAIRS),
      mode(Mode::Off),
      owner(Owner::Task),
      last_cmd_ms(0),
      cmd_timeout_ms(0),
      calibrated(false),
      voltage_limit(FOC_VOLTAGE_LIMIT),
      align_voltage(FOC_VOLTAGE_ALIGN),
      armed(false) {}

Axis axes[AXIS_COUNT] = {Axis(0), Axis(1)};

void axisApplyLimits(Axis &ax) {
  ax.driver.voltage_limit = ax.voltage_limit;
  ax.motor.voltage_limit = ax.voltage_limit;
  ax.motor.PID_velocity.limit = ax.voltage_limit; /* output is VOLTS */
  ax.motor.velocity_limit = FOC_VEL_LIMIT;
  /* align_voltage is set once by initAll() and can also be changed on its
   * own by the `alignv` command; re-clamp it here on every limit change so
   * a `limit` dropped below the current align voltage cannot leave the
   * calibration pass driving more volts than just requested. */
  if (ax.align_voltage > ax.voltage_limit) {
    ax.align_voltage = ax.voltage_limit;
  }
  ax.motor.voltage_sensor_align = ax.align_voltage;
}

void axisInitAll() {
  for (int i = 0; i < AXIS_COUNT; ++i) {
    Axis &ax = axes[i];
    if (!ax.present) {
      continue;
    }
    ax.encoder.init();

    ax.driver.voltage_power_supply = FOC_VBUS;
    ax.driver.pwm_frequency = FOC_PWM_HZ;
    ax.driver.init();

    ax.motor.linkDriver(&ax.driver);
    ax.motor.linkSensor(&ax.encoder);
    ax.motor.foc_modulation = FOC_MODULATION;
    ax.motor.torque_controller = TorqueControlType::voltage;
    ax.motor.controller = MotionControlType::torque;
    ax.motor.target = 0.0f;
    ax.motor.velocity_limit = FOC_VEL_LIMIT;
    ax.motor.PID_velocity.P = FOC_VEL_P;
    ax.motor.PID_velocity.I = FOC_VEL_I;
    ax.motor.PID_velocity.D = FOC_VEL_D;
    ax.motor.PID_velocity.output_ramp = FOC_VEL_RAMP;
    ax.motor.LPF_velocity.Tf = FOC_VEL_LPF;
    ax.motor.voltage_sensor_align = ax.align_voltage;
    axisApplyLimits(ax);
    ax.motor.init();
    ax.motor.disable();
  }
}

void axisArm(Axis &ax) {
  axisApplyLimits(ax);
  /* Guarded by ax.armed, which only axisArm()/axisDisarm() ever write —
   * NOT motor.enabled, which SimpleFOC also flips on its own (initFOC()
   * calls disable() internally on failure). Gating on motor.enabled would
   * let such a library-internal disable desync the M_EN refcount: a later
   * axisDisarm() would see motor.enabled already false and skip the
   * decrement, leaving M_EN — shared by both gate drivers — stuck high.
   * enable() always does setPwm(0,0,0) and resets the PIDs, so re-arming
   * an already-armed motor would also stall the shaft on every command.
   *
   * The whole test-and-set is under boardMotorPowerLock(): since the
   * task-7 failsafe, axisDisarm() is reachable from the core-0 FOC task as
   * well as core 1, so "decide to arm" and "increment the M_EN refcount"
   * must be one indivisible step or the refcount can lose an update to the
   * other core doing the same for a different axis. */
  boardMotorPowerLock();
  if (!ax.armed) {
    ax.motor.enable();
    boardMotorPowerRef(+1);
    ax.armed = true;
  }
  boardMotorPowerUnlock();
}

void axisDisarm(Axis &ax) {
  /* See axisArm() for why this is locked. */
  boardMotorPowerLock();
  if (ax.armed) {
    ax.motor.disable();
    boardMotorPowerRef(-1);
    ax.armed = false;
  }
  boardMotorPowerUnlock();
  /* shaft_velocity/voltage.q are only ever written by motor.move(), which
   * stops running the instant mode is Off -- so without this, a published
   * AxisState snapshot (drive_api.cpp) would keep echoing the last spin's
   * velocity/Uq forever after a park, next to a live, still-updating angle:
   * a consumer would see the wheel "still turning" with zero volts on the
   * phases. Covers both parking paths through this function: the failsafe
   * (foc_task.cpp) and driveStop() (drive_api.cpp). */
  ax.motor.shaft_velocity = 0.0f;
  ax.motor.voltage.q = 0.0f;
}

void axisSetMode(Axis &ax, Mode m) {
  ax.mode = m;
  (void)focSyncWithTask(); /* best-effort: mode is re-read every task pass. */
}

bool axisTakeOwnership(Axis &ax) {
  ax.owner = Owner::Cli;
  return focSyncWithTask();
}

void axisReleaseOwnership(Axis &ax) {
  ax.owner = Owner::Task;
  /* No safety issue in the false case: nobody keeps writing ax once
   * released, so a missed observation only delays the task picking the
   * axis back up, it does not create a race. */
  (void)focSyncWithTask();
}

/**
 * Calibration sequence, ported from ESP32FOCHardwareCheck/src/main.cpp
 * (runZSearch:383, slewElectric:419, rampElectricDown:433,
 * rampElectricUp:442, runOpenloopDirectionAndZero:450, runInitFoc:503).
 * Every current-sense step (current_sense.driverAlign(), .skip_align) is
 * dropped: this firmware has no current sense at all, voltage torque only.
 *
 * All of it runs while the caller owns ax (Owner::Cli): it drives the motor
 * directly via loopFOC()/move()/setPhaseVoltage(), never through the core-0
 * FOC task.
 */
namespace {

constexpr const char *kNvsNs = "drive";

const char *calKey(const Axis &ax) { return ax.idx == 0 ? "cal0" : "cal1"; }

const char *dirName(Direction d) {
  if (d == Direction::CW) {
    return "CW";
  }
  if (d == Direction::CCW) {
    return "CCW";
  }
  return "UNKNOWN";
}

/**
 * Polls the encoder in small steps instead of sleeping in one shot.
 * axisTakeOwnership()'s contract (see axis.h) requires the 16-bit PCNT
 * count to be re-read well inside its ~8192-count wrap window for as long
 * as the CLI, not the FOC task, owns the axis. A bare delay() during the
 * ramp/settle phases below would be exactly the passive wait that contract
 * forbids, so every wait here re-reads the encoder about once a
 * millisecond instead.
 */
void axisOwnedDelay(Axis &ax, uint32_t ms) {
  for (uint32_t i = 0; i < ms; ++i) {
    ax.encoder.update();
    delay(1);
  }
}

constexpr float kAlignOlRadS = 3.0f;
constexpr uint32_t kAlignOlMs = 500;
constexpr float kAlignMinRad = 0.25f;

void slewElectric(Axis &ax, float u, float el_from, float el_to, uint32_t ms) {
  float d = el_to - el_from;
  d = _normalizeAngle(d + _PI) - _PI;
  const int n = (int)(ms / 5u);
  if (n <= 0) {
    ax.motor.setPhaseVoltage(u, 0.0f, el_to);
    return;
  }
  for (int i = 1; i <= n; ++i) {
    ax.motor.setPhaseVoltage(u, 0.0f, _normalizeAngle(el_from + d * ((float)i / (float)n)));
    axisOwnedDelay(ax, 5);
  }
}

void rampElectricDown(Axis &ax, float u, float el, uint32_t ms) {
  const int n = (int)(ms / 5u);
  for (int i = 1; i <= n; ++i) {
    ax.motor.setPhaseVoltage(u * (1.0f - (float)i / (float)n), 0.0f, el);
    axisOwnedDelay(ax, 5);
  }
  ax.motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
}

void rampElectricUp(Axis &ax, float u, float el, uint32_t ms) {
  const int n = (int)(ms / 5u);
  for (int i = 1; i <= n; ++i) {
    ax.motor.setPhaseVoltage(u * ((float)i / (float)n), 0.0f, el);
    axisOwnedDelay(ax, 5);
  }
}

/**
 * Open-loop spin to detect the sensor direction, then a gently ramped
 * electrical-zero capture: rotor already at rest, so the transition to the
 * angle that reads zero_electric_angle is stepped through many small
 * voltage/angle increments (ramp up, slew, settle, ramp down) rather than
 * SimpleFOC's native alignSensor(), which jumps voltage in single steps and
 * kicks the shaft on this hardware.
 */
bool axisOpenloopDirectionAndZero(Axis &ax) {
  axisApplyLimits(ax);
  ax.motor.controller = MotionControlType::velocity_openloop;
  ax.motor.torque_controller = TorqueControlType::voltage;
  ax.motor.target = kAlignOlRadS;
  axisArm(ax);

  ax.encoder.update();
  const int64_t cnt0 = ax.encoder.count();
  Serial.printf("align %c: ol %.1f rad/s  Ulim=%.2f  cnt=%lld\n", ax.name, (double)kAlignOlRadS,
                (double)ax.voltage_limit, (long long)cnt0);

  const uint32_t t0 = millis();
  while ((millis() - t0) < kAlignOlMs) {
    ax.motor.loopFOC();
    ax.motor.move();
  }

  ax.encoder.update();
  const int64_t cnt1 = ax.encoder.count();
  const int64_t dcnt = cnt1 - cnt0;
  const float moved = (float)dcnt * (_2PI / ax.encoder.cpr());
  ax.motor.target = 0.0f;
  ax.motor.move();
  Serial.printf("align %c: ol moved=%.4f rad  dcnt=%lld\n", ax.name, (double)moved,
                (long long)dcnt);

  /* An open-loop spin at 3 rad/s for 500 ms should move the shaft ~1.5 rad.
   * A large mismatch means ENC_PPR differs from the MT6835 ABZ register or
   * pole_pairs is wrong. Warn only: slipping in open loop is possible. */
  const float expected = kAlignOlRadS * (kAlignOlMs / 1000.0f);
  const float ratio = fabsf(moved) / expected;
  Serial.printf("cal %c: moved=%.3f rad expected=%.3f ratio=%.2f%s\n", ax.name, (double)moved,
                (double)expected, (double)ratio,
                (ratio < 0.7f || ratio > 1.3f) ? "  WARNING check ENC_PPR / pole_pairs" : "");

  if (fabsf(moved) < kAlignMinRad) {
    Serial.printf("align %c: FAIL - ol did not move the shaft\n", ax.name);
    return false;
  }

  ax.motor.sensor_direction = (dcnt > 0) ? Direction::CW : Direction::CCW;
  const float el_from = _normalizeAngle(ax.motor.shaft_angle * (float)ax.motor.pole_pairs);
  rampElectricUp(ax, ax.align_voltage, el_from, 200);
  slewElectric(ax, ax.align_voltage, el_from, _3PI_2, 400);
  axisOwnedDelay(ax, 400);
  ax.encoder.update();
  ax.motor.zero_electric_angle = 0.0f;
  ax.motor.zero_electric_angle = ax.motor.electricalAngle();
  rampElectricDown(ax, ax.align_voltage, _3PI_2, 250);
  Serial.printf("align %c: dir=%s  zero=%.4f\n", ax.name, dirName(ax.motor.sensor_direction),
                (double)ax.motor.zero_electric_angle);
  return _isset(ax.motor.zero_electric_angle);
}

/**
 * Runs open-loop direction/zero detection only if the axis does not
 * already have a direction/zero (e.g. just loaded from NVS), then
 * initFOC(). With current sense gone there is nothing left to do in the
 * "already known" branch beyond that — the original's current-sense
 * repolarization pass is dropped entirely.
 */
int axisRunInitFoc(Axis &ax) {
  axisApplyLimits(ax);
  if (ax.motor.sensor_direction == Direction::UNKNOWN || !_isset(ax.motor.zero_electric_angle)) {
    if (!axisOpenloopDirectionAndZero(ax)) {
      return 0;
    }
  }
  axisArm(ax);
  const int ok = ax.motor.initFOC();
  Serial.printf("align %c: initFOC=%d  zero=%.4f  dir=%s\n", ax.name, ok,
                (double)ax.motor.zero_electric_angle, dirName(ax.motor.sensor_direction));
  ax.motor.target = 0.0f;
  return ok;
}

}  // namespace

bool axisZSearch(Axis &ax, bool park) {
  if (!ax.present) {
    /* Defense in depth: cli.cpp already filters on axes[i].present before
     * calling in, but this is also reachable straight from axisCal() below,
     * and an absent axis still shares board.h's M_EN reference count with
     * the real one -- arming it would power the real gate driver too. */
    Serial.printf("zsearch %c: FAIL (axis not present in this build)\n", ax.name);
    return false;
  }
  if (park) {
    if (!axisTakeOwnership(ax)) {
      /* Best-effort handback: ax.owner was already written Cli inside the
       * failed axisTakeOwnership(); do not leave it stuck there. Nothing
       * is armed yet, so there is nothing else to undo. */
      axisReleaseOwnership(ax);
      Serial.printf(
          "zsearch %c: FAIL (ownership handshake timed out, FOC task not responding)\n", ax.name);
      return false;
    }
  }
  ax.encoder.clearIndex();
  ax.motor.controller = MotionControlType::velocity_openloop;
  ax.motor.torque_controller = TorqueControlType::voltage;
  ax.motor.target = Z_SEARCH_RPS * _2PI;
  axisArm(ax);

  const uint32_t timeout_ms = (uint32_t)(Z_SEARCH_TURNS / Z_SEARCH_RPS * 1000.0f) + 200u;
  Serial.printf("zsearch %c: %.2f rps  timeout=%.1f turn\n", ax.name, (double)Z_SEARCH_RPS,
                (double)Z_SEARCH_TURNS);
  const uint32_t t0 = millis();
  while (!ax.encoder.indexFound() && (millis() - t0) < timeout_ms) {
    ax.motor.loopFOC();
    ax.motor.move();
  }
  /* velocity_openloop's move() always drives phases at up to voltage_limit
   * regardless of target: target=0 alone (with no further move()/loopFOC()
   * call) would leave the last full-voltage setpoint applied to a now-fixed
   * electrical angle for as long as the axis stays owned — through this
   * function's printf, through axisSetMode()'s up-to-50ms sync, even through
   * axisLoadCal()'s flash read. Cut the phases explicitly instead. */
  ax.motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
  const bool ok = ax.encoder.indexFound();
  ax.motor.target = 0.0f;
  Serial.printf("zsearch %c: %s  cnt=%lld\n", ax.name, ok ? "ok" : "FAIL (no index)",
                (long long)ax.encoder.count());

  if (!park) {
    /* axisCal() already owns ax and continues straight into direction/zero
     * detection: stay armed, stay owned, let the caller finish the job
     * (including restoring motor.controller before it eventually idles). */
    return ok;
  }

  ax.calibrated = false;
  if (ok) {
    if (axisLoadCal(ax)) {
      const int fok = axisRunInitFoc(ax);
      ax.calibrated = (fok != 0) && ax.motor.sensor_direction != Direction::UNKNOWN &&
                       _isset(ax.motor.zero_electric_angle);
      Serial.printf("zsearch %c: electrical %s  initFOC=%d\n", ax.name,
                    ax.calibrated ? "ok" : "FAIL", fok);
    } else {
      Serial.printf("zsearch %c: ok, no nvs electrical zero - run cal\n", ax.name);
    }
  }
  /* Standalone zsearch is done either way: undo the velocity_openloop mode
   * axisRunInitFoc()/axisOpenloopDirectionAndZero() may have left behind so
   * a later mode change that only touches ax.mode (not ax.motor.controller)
   * cannot leave the axis spinning open-loop at full voltage under torque
   * control's name. */
  ax.motor.controller = MotionControlType::torque;
  axisSetMode(ax, Mode::Off);
  axisDisarm(ax);
  axisReleaseOwnership(ax);
  return ok;
}

bool axisCal(Axis &ax) {
  if (!ax.present) {
    /* Defense in depth: cli.cpp already filters on axes[i].present before
     * calling in. An absent axis still shares board.h's M_EN reference count
     * with the real one, so arming it here would power the real gate driver
     * too. */
    Serial.printf("cal %c: FAIL (axis not present in this build)\n", ax.name);
    return false;
  }
  if (!axisTakeOwnership(ax)) {
    axisReleaseOwnership(ax); /* best-effort handback, see axisZSearch(). */
    Serial.printf("cal %c: FAIL (ownership handshake timed out, FOC task not responding)\n",
                  ax.name);
    return false;
  }
  Serial.printf("cal %c: Z search then ol %.1f for dir/zero - motor will spin\n", ax.name,
                (double)kAlignOlRadS);

  if (!axisZSearch(ax, false)) {
    ax.calibrated = false;
    ax.motor.controller = MotionControlType::torque; /* undo the Z-search's velocity_openloop */
    axisSetMode(ax, Mode::Off);
    axisDisarm(ax);
    axisReleaseOwnership(ax);
    Serial.printf("cal %c: FAIL (Z)\n", ax.name);
    return false;
  }

  ax.motor.sensor_direction = Direction::UNKNOWN;
  ax.motor.zero_electric_angle = NOT_SET;
  const int ok = axisRunInitFoc(ax);
  ax.motor.controller = MotionControlType::torque; /* undo open-loop dir/zero detection's mode */
  axisSetMode(ax, Mode::Off);
  axisDisarm(ax);
  ax.calibrated = (ok != 0) && ax.encoder.indexFound() &&
                  ax.motor.sensor_direction != Direction::UNKNOWN &&
                  _isset(ax.motor.zero_electric_angle);
  Serial.printf("cal %c: %s  zero=%.4f (%.1f deg) dir=%s  initFOC=%d\n", ax.name,
                ax.calibrated ? "ok" : "FAIL", (double)ax.motor.zero_electric_angle,
                (double)(ax.motor.zero_electric_angle * (180.0f / _PI)),
                dirName(ax.motor.sensor_direction), ok);
  if (ax.calibrated) {
    Serial.printf("cal %c: type save - next boot only needs zsearch\n", ax.name);
  }
  axisReleaseOwnership(ax);
  return ax.calibrated;
}

bool axisSaveCal(Axis &ax) {
  if (!ax.calibrated || ax.motor.sensor_direction == Direction::UNKNOWN ||
      !_isset(ax.motor.zero_electric_angle)) {
    Serial.printf("save %c: not calibrated\n", ax.name);
    return false;
  }
  CalRecord rec{};
  rec.magic = kCalMagic;
  rec.zero_electric_angle = ax.motor.zero_electric_angle;
  rec.sensor_direction = static_cast<int8_t>(ax.motor.sensor_direction);
  rec.pole_pairs = static_cast<uint8_t>(ax.motor.pole_pairs);
  rec.enc_ppr = (uint16_t)ENC_PPR;
  rec.axis = ax.name;

  Preferences prefs;
  if (!prefs.begin(kNvsNs, false)) {
    Serial.printf("save %c: nvs fail\n", ax.name);
    return false;
  }
  const size_t n = prefs.putBytes(calKey(ax), &rec, sizeof(rec));
  prefs.end();
  if (n != sizeof(rec)) {
    Serial.printf("save %c: write fail\n", ax.name);
    return false;
  }
  Serial.printf("save %c: zero=%.4f dir=%s pp=%u ppr=%u  (reboot: zsearch then this zero)\n",
                ax.name, (double)rec.zero_electric_angle, dirName(ax.motor.sensor_direction),
                (unsigned)rec.pole_pairs, (unsigned)rec.enc_ppr);
  return true;
}

void axisForgetCal(Axis &ax) {
  if (!ax.present) {
    /* Defense in depth: cli.cpp already filters on axes[i].present before
     * calling in. An absent axis still shares board.h's M_EN reference count
     * with the real one, so arming it here would power the real gate driver
     * too. */
    Serial.printf("forget %c: FAIL (axis not present in this build)\n", ax.name);
    return;
  }
  if (!axisTakeOwnership(ax)) {
    axisReleaseOwnership(ax); /* best-effort handback, see axisZSearch(). */
    Serial.printf("forget %c: FAIL (ownership handshake timed out, FOC task not responding)\n",
                  ax.name);
    return;
  }
  axisSetMode(ax, Mode::Off);
  axisDisarm(ax);
  ax.calibrated = false;
  ax.motor.sensor_direction = Direction::UNKNOWN;
  ax.motor.zero_electric_angle = NOT_SET;
  axisReleaseOwnership(ax);

  Preferences prefs;
  if (prefs.begin(kNvsNs, false)) {
    prefs.remove(calKey(ax));
    prefs.end();
  }
  Serial.printf("forget %c: nvs cleared, run cal\n", ax.name);
}

bool axisLoadCal(Axis &ax) {
  Preferences prefs;
  if (!prefs.begin(kNvsNs, true)) {
    return false;
  }
  CalRecord rec{};
  const size_t n = prefs.getBytes(calKey(ax), &rec, sizeof(rec));
  prefs.end();
  if (n != sizeof(rec) || !calRecordValid(rec, ax.name, (uint16_t)ENC_PPR)) {
    return false;
  }
  /* calRecordValid()'s signature is fixed by the record format and only
   * knows about axis/ENC_PPR; pole_pairs needs the exact same treatment
   * (reject before applying anything) but has nowhere else to live. A
   * firmware rebuilt for a different motor — different FOC_POLE_PAIRS —
   * must not silently keep applying an old electrical zero taken at the
   * wrong pole count, the same way a changed ENC_PPR already invalidates
   * the record above. */
  if (rec.pole_pairs != (uint8_t)FOC_POLE_PAIRS) {
    return false;
  }
  ax.motor.pole_pairs = rec.pole_pairs;
  ax.motor.sensor_direction = static_cast<Direction>(rec.sensor_direction);
  ax.motor.zero_electric_angle = rec.zero_electric_angle;
  Serial.printf("nvs %c: zero=%.4f dir=%s pp=%u\n", ax.name, (double)rec.zero_electric_angle,
                dirName(ax.motor.sensor_direction), (unsigned)rec.pole_pairs);
  return true;
}

bool axisRequireCal(Axis &ax) {
  if (!ax.calibrated || !ax.encoder.indexFound()) {
    Serial.printf("need cal %c (or zsearch after save)\n", ax.name);
    return false;
  }
  return true;
}

bool axisSetVelocity(Axis &ax, float rad_s) {
  if (!axisRequireCal(ax)) {
    return false;
  }
  rad_s = _constrain(rad_s, -FOC_VEL_LIMIT, FOC_VEL_LIMIT); /* SimpleFOC macro */
  axisApplyLimits(ax);
  /* Test-and-write under the same power lock axisArm()/axisDisarm() use:
   * ax.armed is volatile, but a bare `if (ax.armed && ...)` here would still
   * race the core-0 failsafe, which can disarm ax between the test and the
   * ax.motor.target store below and make this path report "applied" for a
   * setpoint nothing will actually drive. Locking excludes axisDisarm() for
   * the duration; nothing slow (no printf, no focSyncWithTask()) happens
   * inside. */
  boardMotorPowerLock();
  if (ax.armed && ax.mode == Mode::Velocity) {
    ax.motor.target = rad_s; /* live setpoint: do NOT re-arm */
    boardMotorPowerUnlock();
    return true;
  }
  boardMotorPowerUnlock();
  ax.motor.controller = MotionControlType::velocity;
  ax.motor.torque_controller = TorqueControlType::voltage;
  ax.motor.target = rad_s;
  if (ax.armed) {
    /* Coming from another already-armed mode (ol/tq): axisArm() below is a
     * no-op (it is guarded by ax.armed, see its own comment), so the
     * PID_velocity.reset() a fresh enable() would give us never happens.
     * Without this, the velocity loop would start from whatever integral
     * and output a previous session left behind, instead of zero. */
    ax.motor.PID_velocity.reset();
  }
  /* No priming read here (no encoder.update()/getVelocity() call): the
   * encoder belongs to the core-0 FOC task for as long as ax.owner stays
   * Owner::Task, which it does here -- this function never takes ownership.
   * A core-1 call into update()/getVelocity() would race the task on state
   * that count()'s internal lock does not cover, and once the axis is
   * already armed in another mode (the branch above), move() is calling
   * getVelocity() every tick on core 0 already: two unsynchronized callers
   * of the same non-atomic 64-bit counter. It is also unnecessary: our
   * getVelocity() derives velocity from the 64-bit count and a timestamp,
   * so a stale previous sample just yields a very long elapsed interval,
   * i.e. a velocity near zero for one cycle -- never a spike -- and the
   * PID's output_ramp bounds the resulting step regardless. */
  axisSetMode(ax, Mode::Velocity);
  axisArm(ax);
  return true;
}

bool axisSetTorque(Axis &ax, float volts) {
  if (!axisRequireCal(ax)) {
    return false;
  }
  volts = _constrain(volts, -ax.voltage_limit, ax.voltage_limit);
  axisApplyLimits(ax);
  /* See axisSetVelocity() for why this test-and-write is locked. */
  boardMotorPowerLock();
  if (ax.armed && ax.mode == Mode::Torque) {
    ax.motor.target = volts; /* live setpoint: do NOT re-arm */
    boardMotorPowerUnlock();
    return true;
  }
  boardMotorPowerUnlock();
  ax.motor.controller = MotionControlType::torque;
  ax.motor.torque_controller = TorqueControlType::voltage;
  ax.motor.target = volts;
  /* No PID_velocity.reset() here: voltage torque control never reads
   * PID_velocity, so a stale integral left over from a previous Velocity
   * session cannot leak into it -- axisSetVelocity() resets it on its own
   * way back in, which is the only path that consumes it.
   *
   * No priming read here either (no encoder.update()/getVelocity() call):
   * the encoder belongs to the core-0 FOC task for as long as ax.owner
   * stays Owner::Task, which it does here -- this function never takes
   * ownership. A core-1 call into update()/getVelocity() would race the
   * task on state that count()'s internal lock does not cover, and once the
   * axis is already armed in another mode, move() is calling getVelocity()
   * every tick on core 0 already: two unsynchronized callers of the same
   * non-atomic 64-bit counter. It is also unnecessary: our getVelocity()
   * derives velocity from the 64-bit count and a timestamp, so a stale
   * previous sample just yields a very long elapsed interval, i.e. a
   * velocity near zero for one cycle -- never a spike. */
  axisSetMode(ax, Mode::Torque);
  axisArm(ax);
  return true;
}

float axisCurrentEstimate(const Axis &ax) {
  /* Display only. Setting motor.phase_resistance in SimpleFOC would also
   * change the sign convention of the velocity PID gains, so this is kept
   * entirely separate from anything SimpleFOC touches. */
  const float bemf = ax.motor.shaft_velocity / (FOC_KV * _SQRT3) / _RPM_TO_RADS;
  return (ax.motor.voltage.q - bemf) / FOC_PHASE_R;
}
