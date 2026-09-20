/**
 * @file ctrl_math.h
 * @brief Tiny shared helpers for ctrl_* loops.
 */
#ifndef CTRL_MATH_H
#define CTRL_MATH_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline float ctrl_clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static inline float ctrl_wrap_pi(float a)
{
    while (a > (float)M_PI) {
        a -= (float)(2.0 * M_PI);
    }
    while (a < -(float)M_PI) {
        a += (float)(2.0 * M_PI);
    }
    return a;
}

static inline float ctrl_two_pi(void)
{
    return (float)(2.0 * M_PI);
}

#endif /* CTRL_MATH_H */
