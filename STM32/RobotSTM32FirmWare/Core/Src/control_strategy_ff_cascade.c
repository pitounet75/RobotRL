/**
 * @file control_strategy_ff_cascade.c
 * @brief Velocity cascade + gravity feedforward (torque-mode plant).
 *
 *   pitch_ref_eff = pitch_ref + PID_vel(v_ref - v)
 *   u_ff          = -K_ff * sin(pitch)
 *   u_fb          = Kθ*(pitch_ref_eff - pitch) - Kω*pitch_rate
 *   u             = low_pass(u_ff + u_fb)
 */

#include "control_strategy.h"

#include "app_config.h"
#include "app_ctrl_params.h"
#include "pid_controller.h"

#include <math.h>

static pid_controller_t s_pid_vel;
static float s_u_prev;

static float clampf(float x, float lo, float hi)
{
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}

static float grav_ff_sin(float pitch_rad)
{
	const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
	const float pitch = clampf(pitch_rad, -p->pitch_failsafe_rad, p->pitch_failsafe_rad);
	return sinf(pitch);
}

void control_strategy_ff_cascade_reset(void)
{
	const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
	pid_init(&s_pid_vel,
	         p->cascade_vel_kp,
	         p->cascade_vel_ki,
	         p->cascade_vel_kd,
	         -p->cascade_pitch_ref_max_rad,
	         p->cascade_pitch_ref_max_rad);
	s_u_prev = 0.0f;
}

void control_strategy_ff_cascade_update(const control_strategy_input_t *in,
                                        control_strategy_output_t *out)
{
	if (in == NULL || out == NULL) {
		return;
	}

	const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();

	const float pitch_trim = pid_update(&s_pid_vel,
	                                    in->vel_ref_turns_s,
	                                    in->vel_wheel_turns_s,
	                                    0.0f,
	                                    in->dt_s);

	const float pitch_ref_eff = clampf(in->pitch_ref_rad + pitch_trim,
	                                   -p->cascade_pitch_ref_max_rad,
	                                   p->cascade_pitch_ref_max_rad);

	const float theta_err = pitch_ref_eff - in->pitch_rad;

	const float u_ff = -p->ff_grav_k * grav_ff_sin(in->pitch_rad);
	const float u_fb = p->ff_fb_k_pitch * theta_err
	                 - p->ff_fb_k_rate * in->pitch_rate_rads;

	const float u_raw = u_ff + u_fb;
	const float alpha = p->ff_output_alpha;
	const float u = alpha * s_u_prev + (1.0f - alpha) * u_raw;
	s_u_prev = u;

	const float cmd = clampf(u, -p->cmd_max_torque_nm, p->cmd_max_torque_nm);

	out->u_vel = pitch_trim;
	out->u_ff = u_ff;
	out->u_fb = u_fb;
	out->u_balance = u_ff + u_fb;
	out->cmd = cmd;
	out->ok = true;
	out->estop = false;
	out->torque_left_nm = cmd;
	out->torque_right_nm = cmd;
}
