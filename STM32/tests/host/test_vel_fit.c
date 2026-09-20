/**
 * @file test_vel_fit.c
 * @brief Sliding second-order fit: exactness, lag, and quantisation noise.
 */

#include "test_harness.h"

#include "vel_fit.h"

#include <math.h>

#define DT 0.002f

/** 32768 counts/turn ABZ: one count is this many turns. */
#define COUNT_TURNS (1.0f / 32768.0f)

static float quantise(float pos_turns)
{
    return roundf(pos_turns / COUNT_TURNS) * COUNT_TURNS;
}

void test_vel_fit_invalid_until_window_full(void)
{
    vel_fit_t f;
    vel_fit_reset(&f);

    float vel = -1.0f;
    float acc = -1.0f;
    for (int i = 0; i < VEL_FIT_N - 1; i++) {
        TEST_ASSERT(!vel_fit_push(&f, 0.1f * (float)i, DT, &vel, &acc));
    }
    /* Outputs must be left alone while invalid. */
    TEST_ASSERT_NEAR(-1.0f, vel, 1e-6f);
    TEST_ASSERT_NEAR(-1.0f, acc, 1e-6f);

    TEST_ASSERT(vel_fit_push(&f, 0.1f * (float)(VEL_FIT_N - 1), DT, &vel, &acc));
}

void test_vel_fit_constant_velocity(void)
{
    vel_fit_t f;
    vel_fit_reset(&f);

    const float v = 3.0f; /* turn/s */
    float vel = 0.0f;
    float acc = 0.0f;
    bool ok = false;
    for (int i = 0; i < VEL_FIT_N; i++) {
        ok = vel_fit_push(&f, v * DT * (float)i, DT, &vel, &acc);
    }

    TEST_ASSERT(ok);
    TEST_ASSERT_NEAR(v, vel, 1e-3f);
    TEST_ASSERT_NEAR(0.0f, acc, 1e-2f);
}

void test_vel_fit_constant_acceleration(void)
{
    vel_fit_t f;
    vel_fit_reset(&f);

    /* Free wheel shedding speed: pos = v0*t + a*t^2/2. */
    const float v0 = 5.0f;
    const float a = -14.0f; /* turn/s^2, wheel-side free-wheel signature */
    float vel = 0.0f;
    float acc = 0.0f;
    for (int i = 0; i < VEL_FIT_N; i++) {
        const float t = DT * (float)i;
        (void)vel_fit_push(&f, v0 * t + 0.5f * a * t * t, DT, &vel, &acc);
    }

    /* A quadratic is fitted exactly: both outputs hold at the window centre. */
    const float t_centre = DT * (float)VEL_FIT_LAG_SAMPLES;
    TEST_ASSERT_NEAR(v0 + a * t_centre, vel, 1e-2f);
    TEST_ASSERT_NEAR(a, acc, 1e-1f);
}

void test_vel_fit_reports_centre_not_newest(void)
{
    vel_fit_t f;
    vel_fit_reset(&f);

    const float v0 = 0.0f;
    const float a = 100.0f;
    float vel = 0.0f;
    float acc = 0.0f;
    for (int i = 0; i < VEL_FIT_N; i++) {
        const float t = DT * (float)i;
        (void)vel_fit_push(&f, v0 * t + 0.5f * a * t * t, DT, &vel, &acc);
    }

    /* The newest sample sits at 10 * DT; the centre at 5 * DT. Reporting the
     * newest would give 2.0 turn/s here, so this pins the documented lag. */
    TEST_ASSERT_NEAR(a * DT * (float)VEL_FIT_LAG_SAMPLES, vel, 1e-2f);
}

void test_vel_fit_quantisation_noise_bounded(void)
{
    vel_fit_t f;
    vel_fit_reset(&f);

    /* Constant speed, position quantised to whole encoder counts: the only
     * error source is quantisation, as on the real ABZ. */
    const float v = 1.0f;
    float worst_vel_err = 0.0f;
    float worst_acc_err = 0.0f;
    float vel = 0.0f;
    float acc = 0.0f;

    for (int i = 0; i < 2000; i++) {
        const float pos = quantise(v * DT * (float)i);
        if (vel_fit_push(&f, pos, DT, &vel, &acc)) {
            const float vel_err = fabsf(vel - v);
            const float acc_err = fabsf(acc);
            if (vel_err > worst_vel_err) {
                worst_vel_err = vel_err;
            }
            if (acc_err > worst_acc_err) {
                worst_acc_err = acc_err;
            }
        }
    }

    /* Budget: acceleration noise must stay far under the 14 turn/s^2
     * free-wheel signature the antipatinage has to detect. Double-differencing
     * the same signal would sit near 5.4 turn/s^2. */
    TEST_ASSERT(worst_vel_err < 0.01f);
    TEST_ASSERT(worst_acc_err < 1.5f);
}
