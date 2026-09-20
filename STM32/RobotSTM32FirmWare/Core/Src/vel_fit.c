/**
 * @file vel_fit.c
 * @brief Sliding second-order least-squares fit over evenly spaced positions.
 *
 * Fits pos(i) = a + b*i + c*i^2 on i = -M..M (M = (N-1)/2, newest at i = +M)
 * and reports the fit at i = 0, the window centre. With even spacing the normal
 * equations collapse to two sums, because the odd moments vanish:
 *
 *   b = SUM(i*y) / SUM(i^2)
 *   c = [ SUM(i^2*y) - (SUM(i^2)/N) * SUM(y) ] / [ SUM(i^4) - SUM(i^2)^2/N ]
 *
 *   vel = b / dt        acc = 2*c / dt^2
 *
 * No matrix, no division by a runtime-computed determinant: the moments depend
 * only on N. For N = 11: SUM(i^2) = 110, SUM(i^4) = 1958, denominator = 858.
 */

#include "vel_fit.h"

#include <string.h>

#if (VEL_FIT_N < 5) || ((VEL_FIT_N % 2) == 0)
#error "VEL_FIT_N must be odd and >= 5"
#endif

#define VEL_FIT_M (VEL_FIT_N / 2)

void vel_fit_reset(vel_fit_t *f)
{
    if (f == 0) {
        return;
    }
    memset(f, 0, sizeof(*f));
}

bool vel_fit_push(vel_fit_t *f, float pos_turns, float dt_s,
                  float *vel_turns_s, float *acc_turns_s2)
{
    if (f == 0 || dt_s <= 0.0f) {
        return false;
    }

    f->pos[f->head] = pos_turns;
    f->head = (uint8_t)((f->head + 1u) % (uint8_t)VEL_FIT_N);
    if (f->count < (uint8_t)VEL_FIT_N) {
        f->count++;
    }
    if (f->count < (uint8_t)VEL_FIT_N) {
        return false;
    }

    /* head now points at the OLDEST sample (i = -M). */
    float s_y = 0.0f;
    float s_iy = 0.0f;
    float s_i2y = 0.0f;
    float s_i2 = 0.0f;
    float s_i4 = 0.0f;

    for (int k = 0; k < VEL_FIT_N; k++) {
        const int idx = (int)((f->head + (uint8_t)k) % (uint8_t)VEL_FIT_N);
        const float y = f->pos[idx];
        const float i = (float)(k - VEL_FIT_M);
        const float i2 = i * i;

        s_y += y;
        s_iy += i * y;
        s_i2y += i2 * y;
        s_i2 += i2;
        s_i4 += i2 * i2;
    }

    const float den_c = s_i4 - (s_i2 * s_i2) / (float)VEL_FIT_N;
    if (den_c <= 0.0f) {
        return false;
    }

    const float b = s_iy / s_i2;
    const float c = (s_i2y - (s_i2 / (float)VEL_FIT_N) * s_y) / den_c;

    if (vel_turns_s != 0) {
        *vel_turns_s = b / dt_s;
    }
    if (acc_turns_s2 != 0) {
        *acc_turns_s2 = 2.0f * c / (dt_s * dt_s);
    }
    return true;
}
