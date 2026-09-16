#include "axis.h"

#include "board.h"
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
  /* enable() always does setPwm(0,0,0) and resets the PIDs: re-arming an
   * already-armed motor would stall the shaft on every command. */
  if (!ax.motor.enabled) {
    ax.motor.enable();
    boardMotorPowerRef(+1);
  }
  ax.armed = true;
}

void axisDisarm(Axis &ax) {
  if (ax.motor.enabled) {
    ax.motor.disable();
    boardMotorPowerRef(-1);
  }
  ax.armed = false;
}

void axisSetMode(Axis &ax, Mode m) {
  ax.mode = m;
  focSyncWithTask();
}
