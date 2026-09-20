/**
 * @file vel_fit.h
 * @brief Sliding second-order least-squares fit: velocity + acceleration.
 *
 * Feeds on position samples at a fixed rate and fits pos(i) = a + b*i + c*i^2
 * over the last VEL_FIT_N samples, then reports velocity and acceleration at
 * the window CENTRE. That costs (N-1)/2 samples of lag but is far quieter than
 * differentiating: on the wheel ABZ (32768 counts/turn, dt = 2 ms) the
 * quantisation noise on acceleration drops from 5.4 turn/s^2 (double
 * difference) to 0.15 turn/s^2, against a free-wheel signature near
 * 14 turn/s^2.
 *
 * Position, not velocity, is the input: fitting the raw integrated count keeps
 * the two outputs consistent (accel is the derivative of the reported vel) and
 * avoids stacking a filter on an already differentiated signal.
 */
#ifndef VEL_FIT_H
#define VEL_FIT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Window length in samples. Odd, so the centre is a sample. */
#ifndef VEL_FIT_N
#define VEL_FIT_N 11
#endif

typedef struct {
    float pos[VEL_FIT_N];
    uint8_t head;  /**< Next write slot. */
    uint8_t count; /**< Samples held, saturating at VEL_FIT_N. */
} vel_fit_t;

void vel_fit_reset(vel_fit_t *f);

/**
 * Push one position sample taken dt_s after the previous one.
 *
 * @param pos_turns   Integrated position (turn). Must not wrap.
 * @param dt_s        Sample period; assumed constant over the window.
 * @param vel_turns_s Velocity at the window centre. Untouched unless true.
 * @param acc_turns_s2 Acceleration at the window centre. Untouched unless true.
 * @return false until the window is full, or if dt_s is unusable: outputs are
 *         then left alone and the caller must treat them as stale.
 */
bool vel_fit_push(vel_fit_t *f, float pos_turns, float dt_s,
                  float *vel_turns_s, float *acc_turns_s2);

/** Lag of the reported values, in samples: the fit is centred. */
#define VEL_FIT_LAG_SAMPLES ((VEL_FIT_N - 1) / 2)

#ifdef __cplusplus
}
#endif

#endif /* VEL_FIT_H */
