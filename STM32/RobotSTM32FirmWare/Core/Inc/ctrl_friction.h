/**
 * @file ctrl_friction.h
 * @brief Coulomb / deadband compensation added to balance u_raw.
 */
#ifndef CTRL_FRICTION_H
#define CTRL_FRICTION_H

#include <stdint.h>

void ctrl_friction_reset(void);
float ctrl_friction_comp(float omega_turns_s, float u_raw_nm, float pitch_rad, float pitch_rate_rads,
                         uint8_t *regime_out);

#endif /* CTRL_FRICTION_H */
