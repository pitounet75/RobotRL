# hypothesis_lab firmware

Experimental control features on branch `hypothesis_lab`. Production tune stays on `tune_param_16384_PPR`.

Build flag: `APP_HYPOTHESIS_LAB=1` in `app_config.h` (default on this branch).

## Snapshot v11 — two-level wheel friction

Replaces the legacy single deadband (`torque_deadband_nm`) when `friction_mode=1`:

| Param | Default | Role |
|---|---|---|
| `friction_mode` | 1 | 0 = legacy deadband, 1 = static + kinetic |
| `friction_static_nm` | 0.0045 | \|ω\| ≤ ε: add `sign(u)·static` |
| `friction_kinetic_nm` | 0.003 | \|ω\| > ε: add `sign(ω)·kinetic` |
| `friction_vel_eps_turns_s` | 0.05 | Wheel speed boundary (turn/s) |

Gates (unchanged): `torque_deadband_pitch_max_rad`, `torque_deadband_rate_max_rads`. On this branch defaults use pitch gate **0.05 rad** (~3°).

Debug (Live Expressions):

- `g_ctrl_friction_comp_nm` — last friction torque added to `u_raw` (Nm)
- `g_ctrl_friction_regime` — 0 off, 1 static, 2 kinetic

## Flash + RPC

```bash
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.7 apply TelemetryServer/save_params_hypothesis_friction.txt
```

Toggle back to legacy without reflash:

```bash
python TelemetryServer/scripts/tune_params.py --esp32-host 192.168.1.7 set friction_mode 0
```

## Compare A/B

1. `friction_mode 0` + `torque_deadband_nm 0.004` (legacy)
2. `friction_mode 1` + static 0.0045 / kinetic 0.003 (hypothesis)

Same cascade gains as `save_params_user_restore.txt`; only friction model changes.
