# Balance robot — ODrive + STM32 bring-up sequence

Two separate XDrive boards (axis0 each): **left** (FDCAN1), **right** (FDCAN2).  
USB one drive at a time for configuration.

## Step 1 — Align left and right ODrive (same config, direction only differs)

```powershell
cd H:\Projects\RobotRL\ODrive\OdriveTool\Commands

python align_odrive_bringup.py --serial <LEFT_SERIAL>  --direction -1 --step align --save
python align_odrive_bringup.py --serial <RIGHT_SERIAL> --direction  1 --step align --save
```

Sets (both sides):

- `torque_constant = 8.27 / 750` (~0.011 Nm/A, **motor shaft**)
- Gear: **15 / 80** (motor / wheel pulley) → wheel torque = motor torque × **80/15 ≈ 5.33**
- `current_lim = 10 A` (same)
- `startup_closed_loop_control = True`
- Velocity mode baseline: `vel_gain=0.1`, `vel_integrator_gain=0.32`, `pos_gain=20`
- Anticogging **disabled** until step 3

Dump and diff after:

```powershell
python dumpconfig_paths.py > Configs\odrive_left.txt
python dumpconfig_paths.py > Configs\odrive_right.txt
```

Check: same `torque_constant`, `current_lim`, gains; only `motor.config.direction` differs.

---

## Step 2 — Tune velocity mode (USB, wheels off ground)

Use `velocity_mode.txt` or odrivetool interactively on **each** drive:

1. Closed loop, `CONTROL_MODE_VELOCITY_CONTROL`, `INPUT_MODE_PASSTHROUGH`
2. Start `input_vel = 0`
3. Small steps: 0.05, 0.1 turn/s — no oscillation / overshoot
4. Adjust `vel_gain`, `vel_integrator_gain` (copy final values to the other drive)
5. `save_configuration()` on both

Reference starting point (from 283 left): `vel_gain=0.1`, `vel_integrator_gain=0.32`.

---

## Step 3 — Anticogging calibration (each drive)

Follow `calibrate_anticoggging.txt` or:

```powershell
python ..\..\run_anticogging_calibration.py --serial <SERIAL> --axis 0
```

Requirements:

- `INPUT_MODE_PASSTHROUGH` during cal
- Widen `calib_pos_threshold` / `calib_vel_threshold` if index stalls
- After success: `anticogging_enabled=True`, `pre_calibrated=True`, `save_configuration()`

---

## Step 4 — Limit both ODrives (~0.1 Nm **at wheel**)

ODrive `input_torque` and `torque_lim` are **motor-shaft Nm**. With gear **15/80**:

| At wheel | At motor (ODrive) |
|----------|-------------------|
| **0.10 Nm** (~251 g on Ø80 mm) | **0.01875 Nm** (= 0.10 × 15/80) |
| Iq ≈ **1.70 A** | Kt ≈ 0.011 Nm/A |

After anticogging, on **each** drive:

```powershell
python align_odrive_bringup.py --serial <SERIAL> --direction <±1> --step limit-torque --save
```

Default: `--wheel-torque-lim 0.1` → sets `motor.config.torque_lim ≈ 0.01875` Nm and `current_lim ≈ 1.7 A`.

Verify in odrivetool: ramp `input_torque` in torque mode; `Iq` should plateau before violent acceleration.

---

## Step 5 — STM32 firmware: torque limit

In `Core/Inc/app_config.h`:

```c
/* Motor-shaft Nm sent to ODrive (0.1 Nm at wheel with 15/80 gear). */
#define APP_CTRL_CMD_MAX_TORQUE_NM     0.01875f
```

Rebuild and flash. Watch Live Expressions:

- `g_ctrl_cmd_torque_nm`
- `g_motor_tx_odrive_torque_left_nm` / `_right_nm`

---

## Step 6 — STM32: lower gains and retry

Start conservative (example):

```c
#define APP_CTRL_FF_GRAV_K           0.05f   /* sin(theta) FF, tune up slowly */
#define APP_CTRL_FF_FB_K_PITCH       0.05f
#define APP_CTRL_FF_FB_K_RATE        0.02f
#define APP_CTRL_FF_OUTPUT_ALPHA     0.85f
```

- Strategy: `g_ctrl_strategy = 3` (ff_cascade)
- E-stop angle: `APP_CTRL_PITCH_FAILSAFE_RAD` (~45°)
- Increase gains / max torque only after stable balance on bench

---

## Step 7 — Back to STM32 torque mode (runtime)

Firmware already runs `odrive_torque_mode_startup()` (torque + passthrough).  
ODrive flash keeps `torque_lim≈0.019` Nm (motor) and anticogging map from step 3.

---

## Serial numbers (from dumps)

| Wheel | Serial |
|-------|--------|
| Left  | 55517627953204 |
| Right | 55354418671668 |
