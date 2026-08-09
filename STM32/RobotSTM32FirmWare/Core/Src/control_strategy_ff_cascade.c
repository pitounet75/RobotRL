/**
 * @file control_strategy_ff_cascade.c
 * @brief Gravity FF + pitch PD + optional direct wheel-speed torque damp
 *        + gated motor-accel P (near upright).
 *
 *   pitch_ref_eff = pitch_ref + PID_vel(v_ref - v)   (cascade gains usually 0)
 *   u_ff          = -K_ff * sin(pitch)
 *   u_fb          = Kθ*(pitch_ref_eff - pitch) - Kω*pitch_rate - Kv*(v - v_ref)
 *   u             = low_pass(u_ff + u_fb [+ Coulomb])
 *   α_ref         = (u - c*sign(ω)) / J
 *   Δτ_L/R        = Kα*(α_ref - α_meas)   (gated; ODrive ω)
 *   τ_L/R         = clamp(u + Δτ)
 */

#include "control_strategy.h"

#include "app_config.h"
#include "app_ctrl_params.h"
#include "pid_controller.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float k_two_pi = (float)(2.0 * M_PI);
/** Deadzone on motor speed for Coulomb sign (turn/s). */
static const float k_omega_sign_eps_turns = 0.05f;

static pid_controller_t s_pid_vel;
static float s_u_prev;
static float s_pos_offset_turns;
static bool s_pos_offset_valid;

typedef struct {
    float vel_prev_turns;
    float alpha_rads2;
    uint32_t last_update_ms;
    bool have_prev;
} alpha_est_t;

static alpha_est_t s_alpha_l;
static alpha_est_t s_alpha_r;

/* Debug taps (Live Expressions). */
volatile float g_ctrl_pos_x_m;
volatile float g_ctrl_pos_v_ref_turns_s;
volatile float g_ctrl_pos_pitch_trim_rad;

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

static float friction_sign(float omega_turns_s, float u_nm)
{
	if (omega_turns_s > k_omega_sign_eps_turns) {
		return 1.0f;
	}
	if (omega_turns_s < -k_omega_sign_eps_turns) {
		return -1.0f;
	}
	/* Near zero: oppose commanded torque (breakaway). */
	if (u_nm > 1.0e-6f) {
		return 1.0f;
	}
	if (u_nm < -1.0e-6f) {
		return -1.0f;
	}
	return 0.0f;
}

static void alpha_est_reset(alpha_est_t *e)
{
	e->vel_prev_turns = 0.0f;
	e->alpha_rads2 = 0.0f;
	e->last_update_ms = 0u;
	e->have_prev = false;
}

static float alpha_est_update(alpha_est_t *e, bool valid, float vel_turns_s,
                              uint32_t update_ms, float lpf)
{
	if (!valid) {
		return e->alpha_rads2;
	}

	if (!e->have_prev || update_ms != e->last_update_ms) {
		if (e->have_prev) {
			uint32_t dt_ms = update_ms - e->last_update_ms;
			/* Wrap-safe enough for ms ticks; reject huge gaps. */
			if (dt_ms > 0u && dt_ms < 200u) {
				const float dt_s = (float)dt_ms * 1.0e-3f;
				const float a_turns = (vel_turns_s - e->vel_prev_turns) / dt_s;
				const float a_rad = a_turns * k_two_pi;
				const float a_clamped = clampf(lpf, 0.0f, 0.999f);
				e->alpha_rads2 = a_clamped * e->alpha_rads2
				               + (1.0f - a_clamped) * a_rad;
			}
		}
		e->vel_prev_turns = vel_turns_s;
		e->last_update_ms = update_ms;
		e->have_prev = true;
	}
	return e->alpha_rads2;
}

static float alpha_torque_correction(float u_nm, float omega_turns_s, float alpha_meas_rads2,
                                     const app_ctrl_params_snapshot_t *p,
                                     float pitch_rad, float pitch_rate_rads)
{
	if (p->alpha_kp <= 0.0f || p->motor_J <= 0.0f) {
		return 0.0f;
	}
	if (fabsf(pitch_rad) > p->alpha_pitch_max_rad ||
	    fabsf(pitch_rate_rads) > p->alpha_rate_max_rads) {
		return 0.0f;
	}
	if (p->alpha_vel_max_turns_s > 0.0f &&
	    fabsf(omega_turns_s) > p->alpha_vel_max_turns_s) {
		return 0.0f;
	}

	const float sgn = friction_sign(omega_turns_s, u_nm);
	const float alpha_ref = (u_nm - p->motor_friction_c * sgn) / p->motor_J;
	float dtau = p->alpha_kp * (alpha_ref - alpha_meas_rads2);
	return clampf(dtau, -p->alpha_max_nm, p->alpha_max_nm);
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
	alpha_est_reset(&s_alpha_l);
	alpha_est_reset(&s_alpha_r);
	s_pos_offset_valid = false;
	s_pos_offset_turns = 0.0f;
	g_ctrl_pos_x_m = 0.0f;
	g_ctrl_pos_v_ref_turns_s = 0.0f;
	g_ctrl_pos_pitch_trim_rad = 0.0f;
}

void control_strategy_ff_cascade_update(const control_strategy_input_t *in,
                                        control_strategy_output_t *out)
{
	if (in == NULL || out == NULL) {
		return;
	}

	const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();

	/* Keep outer-loop gains in sync with live params (init only snapshots once). */
	s_pid_vel.kp = p->cascade_vel_kp;
	s_pid_vel.ki = p->cascade_vel_ki;
	s_pid_vel.kd = p->cascade_vel_kd;
	s_pid_vel.out_min = -p->cascade_pitch_ref_max_rad;
	s_pid_vel.out_max = p->cascade_pitch_ref_max_rad;
	s_pid_vel.i_min = s_pid_vel.out_min;
	s_pid_vel.i_max = s_pid_vel.out_max;

	/* --- Station-keeping: x → v_ref + pitch trim --- */
	float vel_ref = in->vel_ref_turns_s;
	float pitch_pos_trim = 0.0f;
	float x_m = 0.0f;

	if (app_ctrl_params_consume_pos_reset() || !s_pos_offset_valid) {
		if (in->pos_wheel_valid) {
			s_pos_offset_turns = in->pos_wheel_turns;
			s_pos_offset_valid = true;
		}
	}

	const bool pos_active = (p->pos_kp > 0.0f || p->pos_pitch_kp > 0.0f) &&
	                        in->pos_wheel_valid && s_pos_offset_valid &&
	                        p->wheel_radius_m > 0.0f;
	if (pos_active) {
		const float turns = in->pos_wheel_turns - s_pos_offset_turns;
		x_m = turns * k_two_pi * p->wheel_radius_m;
		const float x_err = p->pos_x_ref_m - x_m;
		vel_ref = p->pos_kp * x_err - p->pos_kd * in->vel_wheel_turns_s;
		vel_ref = clampf(vel_ref, -p->pos_v_max_turns_s, p->pos_v_max_turns_s);
		/* Manual vel_ref from telemetry adds on top when position loop is on. */
		vel_ref += in->vel_ref_turns_s;
		vel_ref = clampf(vel_ref, -p->pos_v_max_turns_s, p->pos_v_max_turns_s);

		pitch_pos_trim = p->pos_pitch_kp * x_err;
		pitch_pos_trim = clampf(pitch_pos_trim, -p->pos_pitch_max_rad, p->pos_pitch_max_rad);
	}
	g_ctrl_pos_x_m = x_m;
	g_ctrl_pos_v_ref_turns_s = vel_ref;
	g_ctrl_pos_pitch_trim_rad = pitch_pos_trim;

	const float pitch_trim = pid_update(&s_pid_vel,
	                                    vel_ref,
	                                    in->vel_wheel_turns_s,
	                                    0.0f,
	                                    in->dt_s);

	const float pitch_ref_eff = clampf(in->pitch_ref_rad + pitch_trim + pitch_pos_trim,
	                                   -p->cascade_pitch_ref_max_rad,
	                                   p->cascade_pitch_ref_max_rad);

	const float theta_err = pitch_ref_eff - in->pitch_rad;

	float u_vel = APP_CTRL_FF_FB_K_VEL * (vel_ref - in->vel_wheel_turns_s);
	u_vel = clampf(u_vel, -APP_CTRL_FF_FB_K_VEL_MAX_NM, APP_CTRL_FF_FB_K_VEL_MAX_NM);

	const float u_ff = -p->ff_grav_k * grav_ff_sin(in->pitch_rad);
	const float u_fb = p->ff_fb_k_pitch * theta_err
	                 - p->ff_fb_k_rate * in->pitch_rate_rads
	                 + u_vel;

	float u_raw = u_ff + u_fb;
	if (p->torque_deadband_nm > 0.0f &&
	    fabsf(u_raw) > 1.0e-5f &&
	    fabsf(in->pitch_rad) <= p->torque_deadband_pitch_max_rad &&
	    fabsf(in->pitch_rate_rads) <= p->torque_deadband_rate_max_rads) {
		u_raw += copysignf(p->torque_deadband_nm, u_raw);
	}
	const float alpha_out = p->ff_output_alpha;
	const float u = alpha_out * s_u_prev + (1.0f - alpha_out) * u_raw;
	s_u_prev = u;

	const float alpha_l = alpha_est_update(&s_alpha_l,
	                                       in->vel_motor_l_valid,
	                                       in->vel_motor_l_turns_s,
	                                       in->vel_motor_l_update_ms,
	                                       p->alpha_lpf);
	const float alpha_r = alpha_est_update(&s_alpha_r,
	                                       in->vel_motor_r_valid,
	                                       in->vel_motor_r_turns_s,
	                                       in->vel_motor_r_update_ms,
	                                       p->alpha_lpf);

	float dtau_l = 0.0f;
	float dtau_r = 0.0f;
	if (in->vel_motor_l_valid) {
		dtau_l = alpha_torque_correction(u, in->vel_motor_l_turns_s, alpha_l,
		                                 p, in->pitch_rad, in->pitch_rate_rads);
	}
	if (in->vel_motor_r_valid) {
		dtau_r = alpha_torque_correction(u, in->vel_motor_r_turns_s, alpha_r,
		                                 p, in->pitch_rad, in->pitch_rate_rads);
	}

	const float cmd_l = clampf(u + dtau_l, -p->cmd_max_torque_nm, p->cmd_max_torque_nm);
	const float cmd_r = clampf(u + dtau_r, -p->cmd_max_torque_nm, p->cmd_max_torque_nm);
	const float cmd = 0.5f * (cmd_l + cmd_r);

	out->u_vel = u_vel;
	out->u_ff = u_ff;
	out->u_fb = u_fb;
	out->u_balance = u_ff + u_fb;
	out->cmd = cmd;
	out->ok = true;
	out->estop = false;
	out->torque_left_nm = cmd_l;
	out->torque_right_nm = cmd_r;
}
