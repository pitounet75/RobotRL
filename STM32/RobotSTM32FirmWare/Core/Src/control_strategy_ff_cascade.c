/**
 * @file control_strategy_ff_cascade.c
 * @brief Gravity FF + pitch PD + optional direct wheel-speed torque damp
 *        + gated motor-accel P (near upright).
 *
 *   Outer modes (outer_mode):
 *     0 vel:  v_ref = vel_ref (manual)
 *     1 pos:  v_ref = clamp(+(Kp·x_err + Kema·e_f) - Kd·v, ±v_max)
 *   Cascade (leaky I):
 *     e = v_ref - v,  e_f = α·e_f + (1-α)·e
 *     pitch_trim = -(Kp·e + Kema·e_f + Kd·v̇)
 *   u_ff          = -K_ff * sin(pitch)
 *   u_fb          = Kθ*(pitch_ref_eff - pitch) - Kω*pitch_rate - Kv*(v - v_ref)
 *   u             = low_pass(u_ff + u_fb [+ friction comp])
 *   α_ref         = (u - c*sign(ω)) / J
 *   Δτ_L/R        = Kα*(α_ref - α_meas)   (gated; ODrive ω)
 *   u_yaw         = clamp(Kp·wrap(ψ_ref−ψ) − Kd·ψ̇, ±τ_yaw)   (ψ from ∫yaw_rate)
 *   τ_L/R         = clamp(u ± u_yaw + Δτ)
 */

#include "control_strategy.h"

#include "app_config.h"
#include "app_ctrl_params.h"
#include "wheel_contact.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const float k_two_pi = (float)(2.0 * M_PI);
/** Deadzone on motor speed for Coulomb sign (turn/s). */
static const float k_omega_sign_eps_turns = 0.05f;

static float s_u_prev;
static float s_pos_offset_turns;
static bool s_pos_offset_valid;
static float s_x_err_ema;
static float s_vel_prev_turns;
static float s_vel_dot_turns_s2;
static bool s_vel_dot_valid;
static float s_heading_rad;
static bool s_heading_valid;
static float s_vel_ref_slew; /* slew-limited command to cascade */
static float s_vel_err_ema;  /* leaky integrator on (v_ref - v) */
static float s_vel_ref_prev; /* for v̇_ref accel FF */
static bool s_vel_ref_prev_valid;

/** LPF on d(v)/dt for cascade D (higher = smoother / more lag). */
static const float k_vel_dot_lpf = 0.85f;

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
volatile float g_ctrl_pos_err_ema_m;
volatile float g_ctrl_heading_rad;
volatile float g_ctrl_heading_torque_nm;
volatile float g_ctrl_friction_comp_nm;
volatile uint8_t g_ctrl_friction_regime; /* 0=off, 1=static, 2=kinetic */

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

static float wrap_pi(float a)
{
	while (a > (float)M_PI) {
		a -= (float)(2.0 * M_PI);
	}
	while (a < -(float)M_PI) {
		a += (float)(2.0 * M_PI);
	}
	return a;
}

static bool friction_gates_ok(const app_ctrl_params_snapshot_t *p, float pitch_rad, float pitch_rate_rads)
{
	if (p->torque_deadband_rate_max_rads > 0.0f &&
	    fabsf(pitch_rate_rads) > p->torque_deadband_rate_max_rads) {
		return false;
	}
	if (p->torque_deadband_pitch_max_rad > 0.0f &&
	    fabsf(pitch_rad) > p->torque_deadband_pitch_max_rad) {
		return false;
	}
	if (p->torque_deadband_pitch_max_rad <= 0.0f && fabsf(pitch_rad) > 0.0f) {
		return false;
	}
	return true;
}

static float friction_comp_nm(float omega_turns_s, float u_nm, const app_ctrl_params_snapshot_t *p,
                              uint8_t *regime_out)
{
	*regime_out = 0;
	if (p->friction_mode >= 0.5f) {
		const float eps = p->friction_vel_eps_turns_s;
		if (fabsf(omega_turns_s) <= eps) {
			if (fabsf(u_nm) < 1.0e-5f) {
				return 0.0f;
			}
			*regime_out = 1;
			return copysignf(p->friction_static_nm, u_nm);
		}
		if (p->friction_kinetic_nm > 0.0f) {
			*regime_out = 2;
			return copysignf(p->friction_kinetic_nm, omega_turns_s);
		}
		return 0.0f;
	}
	if (p->torque_deadband_nm > 0.0f && fabsf(u_nm) > 1.0e-5f) {
		*regime_out = 1;
		return copysignf(p->torque_deadband_nm, u_nm);
	}
	return 0.0f;
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
	s_u_prev = 0.0f;
	alpha_est_reset(&s_alpha_l);
	alpha_est_reset(&s_alpha_r);
	s_pos_offset_valid = false;
	s_pos_offset_turns = 0.0f;
	s_x_err_ema = 0.0f;
	s_vel_prev_turns = 0.0f;
	s_vel_dot_turns_s2 = 0.0f;
	s_vel_dot_valid = false;
	s_heading_rad = 0.0f;
	s_heading_valid = false;
	s_vel_ref_slew = 0.0f;
	s_vel_err_ema = 0.0f;
	s_vel_ref_prev = 0.0f;
	s_vel_ref_prev_valid = false;
	g_ctrl_pos_x_m = 0.0f;
	g_ctrl_pos_v_ref_turns_s = 0.0f;
	g_ctrl_pos_pitch_trim_rad = 0.0f;
	g_ctrl_pos_err_ema_m = 0.0f;
	g_ctrl_heading_rad = 0.0f;
	g_ctrl_heading_torque_nm = 0.0f;
	g_ctrl_friction_comp_nm = 0.0f;
	g_ctrl_friction_regime = 0;
	wheel_contact_reset();
}

void control_strategy_ff_cascade_update(const control_strategy_input_t *in,
                                        control_strategy_output_t *out)
{
	if (in == NULL || out == NULL) {
		return;
	}

	const app_ctrl_params_snapshot_t *p = app_ctrl_params_snapshot();
	const wheel_contact_output_t *wcp = wheel_contact_last_output();

	if (wcp->reanchor_pos && in->pos_wheel_valid) {
		s_pos_offset_turns = wcp->pos_offset_turns_new;
		s_pos_offset_valid = true;
		s_x_err_ema = 0.0f;
	}
	if (wcp->reset_vel_integrators) {
		s_vel_err_ema = 0.0f;
		s_vel_dot_valid = false;
		s_vel_prev_turns = wcp->vel_wheel_touch_turns_s;
	}
	if (wcp->reset_u_lpf) {
		s_u_prev = wcp->u_lpf_seed_nm;
	}

	const bool integrator_trust = wcp->integrator_trust;
	const float recover_ramp = wcp->recover_ramp;

	/* --- Outer loop: velocity mode XOR position mode (x → bounded v_ref) --- */
	float vel_ref = 0.0f;
	float x_m = 0.0f;
	float x_err_ema = s_x_err_ema;
	const bool pos_mode = (p->outer_mode >= 0.5f);

	if (app_ctrl_params_consume_pos_reset() || (pos_mode && !s_pos_offset_valid)) {
		if (in->pos_wheel_valid) {
			s_pos_offset_turns = in->pos_wheel_turns;
			s_pos_offset_valid = true;
			s_x_err_ema = 0.0f;
			x_err_ema = 0.0f;
		}
	}

	if (pos_mode) {
		if (integrator_trust && in->pos_wheel_valid && s_pos_offset_valid && p->wheel_radius_m > 0.0f) {
			const float turns = in->pos_wheel_turns - s_pos_offset_turns;
			x_m = turns * k_two_pi * p->wheel_radius_m;
			const float x_err = p->pos_x_ref_m - x_m;

			const float a = clampf(p->pos_err_ema_alpha, 0.0f, 0.9999f);
			x_err_ema = a * s_x_err_ema + (1.0f - a) * x_err;
			s_x_err_ema = x_err_ema;

			vel_ref = (p->pos_kp * x_err + p->pos_ema_kp * x_err_ema)
			        - p->pos_kd * in->vel_wheel_turns_s;
			vel_ref = clampf(vel_ref, -p->pos_v_max_turns_s, p->pos_v_max_turns_s);
		} else if (!integrator_trust) {
			x_m = wcp->x_m_frozen_m;
			x_err_ema = s_x_err_ema;
		}
		/* else: hold v_ref=0 until odometry is valid */
	} else {
		vel_ref = in->vel_ref_turns_s;
	}

	/* Slew-limit teleop/pos v_ref so slider steps don't bang the lean loop. */
	float vel_ref_dot = 0.0f;
	if (integrator_trust) {
		const float slew = p->vel_ref_slew_turns_s2;
		if (slew > 0.0f && in->dt_s > 1.0e-6f) {
			const float max_step = slew * in->dt_s;
			float dv = vel_ref - s_vel_ref_slew;
			if (dv > max_step) {
				dv = max_step;
			} else if (dv < -max_step) {
				dv = -max_step;
			}
			s_vel_ref_slew += dv;
			vel_ref_dot = dv / in->dt_s;
		} else {
			if (s_vel_ref_prev_valid && in->dt_s > 1.0e-6f) {
				vel_ref_dot = (vel_ref - s_vel_ref_prev) / in->dt_s;
			}
			s_vel_ref_slew = vel_ref;
		}
		vel_ref = s_vel_ref_slew;
		s_vel_ref_prev = vel_ref;
		s_vel_ref_prev_valid = true;
	} else {
		vel_ref = s_vel_ref_slew * recover_ramp;
	}

	g_ctrl_pos_x_m = x_m;
	g_ctrl_pos_v_ref_turns_s = vel_ref;
	g_ctrl_pos_err_ema_m = x_err_ema;

	/* Filtered wheel accel for cascade D (pid uses measurement_dot, not d(error)/dt). */
	float vel_dot = 0.0f;
	if (integrator_trust && in->dt_s > 1.0e-6f) {
		const float raw_dot = (in->vel_wheel_turns_s - s_vel_prev_turns) / in->dt_s;
		if (s_vel_dot_valid) {
			s_vel_dot_turns_s2 = k_vel_dot_lpf * s_vel_dot_turns_s2
			                   + (1.0f - k_vel_dot_lpf) * raw_dot;
		} else {
			s_vel_dot_turns_s2 = raw_dot;
			s_vel_dot_valid = true;
		}
		vel_dot = s_vel_dot_turns_s2;
		s_vel_prev_turns = in->vel_wheel_turns_s;
	}

	const float vel_err = vel_ref - in->vel_wheel_turns_s;
	if (integrator_trust) {
		const float a = clampf(p->cascade_vel_err_ema_alpha, 0.0f, 0.9999f);
		s_vel_err_ema = a * s_vel_err_ema + (1.0f - a) * vel_err;
	}
	float pitch_cmd = p->cascade_vel_kp * vel_err
	                + p->cascade_vel_ema_kp * s_vel_err_ema
	                + p->cascade_vel_kd * vel_dot
	                - p->cascade_vel_accel_kp * vel_ref_dot;
	pitch_cmd = clampf(pitch_cmd, -p->cascade_pitch_ref_max_rad, p->cascade_pitch_ref_max_rad);
	float pitch_trim = -pitch_cmd;
	if (!integrator_trust) {
		pitch_trim = wcp->pitch_trim_frozen_rad * (1.0f - recover_ramp) + pitch_trim * recover_ramp;
	}
	g_ctrl_pos_pitch_trim_rad = pitch_trim;

	const float pitch_ref_eff = clampf(in->pitch_ref_rad + pitch_trim,
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
	if (friction_gates_ok(p, in->pitch_rad, in->pitch_rate_rads)) {
		uint8_t regime = 0;
		const float comp = friction_comp_nm(in->vel_wheel_turns_s, u_raw, p, &regime);
		u_raw += comp;
		g_ctrl_friction_comp_nm = comp;
		g_ctrl_friction_regime = regime;
	} else {
		g_ctrl_friction_comp_nm = 0.0f;
		g_ctrl_friction_regime = 0;
	}
	const float alpha_out = p->ff_output_alpha;
	float u = alpha_out * s_u_prev + (1.0f - alpha_out) * u_raw;
	if (integrator_trust) {
		s_u_prev = u;
	}

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
	const bool both_air_like = wcp->both_active || (wcp->mode == WC_MODE_BOTH_AIR);
	if (!both_air_like && in->vel_motor_l_valid) {
		dtau_l = alpha_torque_correction(u, in->vel_motor_l_turns_s, alpha_l,
		                                 p, in->pitch_rad, in->pitch_rate_rads);
	}
	if (!both_air_like && in->vel_motor_r_valid) {
		dtau_r = alpha_torque_correction(u, in->vel_motor_r_turns_s, alpha_r,
		                                 p, in->pitch_rad, in->pitch_rate_rads);
	}

	/* --- Heading hold: differential torque from integrated yaw --- */
	float u_yaw = 0.0f;
	if (app_ctrl_params_consume_heading_reset() || !s_heading_valid) {
		s_heading_rad = 0.0f;
		s_heading_valid = true;
	}
	if (in->dt_s > 1.0e-6f) {
		s_heading_rad += in->yaw_rate_rads * in->dt_s;
		s_heading_rad = wrap_pi(s_heading_rad);
	}
	const bool heading_on = (p->heading_kp != 0.0f || p->heading_kd > 0.0f) &&
	                        p->heading_torque_max_nm > 0.0f;
	if (heading_on && wcp->mode != WC_MODE_BOTH_AIR) {
		const float herr = wrap_pi(p->heading_ref_rad - s_heading_rad);
		u_yaw = p->heading_kp * herr - p->heading_kd * in->yaw_rate_rads;
		u_yaw = clampf(u_yaw, -p->heading_torque_max_nm, p->heading_torque_max_nm);
	}
	g_ctrl_heading_rad = s_heading_rad;
	g_ctrl_heading_torque_nm = u_yaw;

	const float tau_l_pre = u + dtau_l - u_yaw;
	const float tau_r_pre = u + dtau_r + u_yaw;

	wheel_contact_input_t wci = {
		.dt_s = in->dt_s,
		.u = u,
		.pitch_rad = in->pitch_rad,
		.pitch_rate_rads = in->pitch_rate_rads,
		.pitch_ref_rad = in->pitch_ref_rad,
		.pitch_trim_rad = pitch_trim,
		.vel_motor_l_turns_s = in->vel_motor_l_turns_s,
		.vel_motor_r_turns_s = in->vel_motor_r_turns_s,
		.vel_motor_l_valid = in->vel_motor_l_valid,
		.vel_motor_r_valid = in->vel_motor_r_valid,
		.alpha_l_rads2 = alpha_l,
		.alpha_r_rads2 = alpha_r,
		.yaw_rate_rads = in->yaw_rate_rads,
		.tau_l_pre_nm = tau_l_pre,
		.tau_r_pre_nm = tau_r_pre,
		.vel_wheel_turns_s = in->vel_wheel_turns_s,
		.pos_wheel_turns = in->pos_wheel_turns,
		.x_m = x_m,
		.pos_wheel_valid = in->pos_wheel_valid,
	};
	wheel_contact_output_t wco;
	wheel_contact_update(&wci, &wco);

	const float u_eff = wco.u_cmd_nm;
	float cmd_l;
	float cmd_r;

	if (wco.both_active) {
		cmd_l = wco.tau_both_l_nm;
		cmd_r = wco.tau_both_r_nm;
	} else if (wco.mode == WC_MODE_SYNC_L) {
		cmd_l = u_eff + dtau_l + wco.tau_sync_l_nm;
		cmd_r = u_eff + dtau_r + u_yaw;
	} else if (wco.mode == WC_MODE_SYNC_R) {
		cmd_l = u_eff + dtau_l - u_yaw;
		cmd_r = u_eff + dtau_r + wco.tau_sync_r_nm;
	} else {
		cmd_l = u_eff + dtau_l - u_yaw;
		cmd_r = u_eff + dtau_r + u_yaw;
	}

	cmd_l = clampf(cmd_l, -p->cmd_max_torque_nm, p->cmd_max_torque_nm);
	cmd_r = clampf(cmd_r, -p->cmd_max_torque_nm, p->cmd_max_torque_nm);
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
