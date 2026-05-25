/**
 * @file test_harness.h
 * @brief Minimal host test runner (no external deps).
 */
#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int g_tests_run;
static int g_tests_failed;

#define TEST_ASSERT(cond)                                                          \
    do {                                                                           \
        if (!(cond)) {                                                             \
            fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
            g_tests_failed++;                                                      \
        }                                                                          \
    } while (0)

#define TEST_ASSERT_NEAR(expected, actual, eps)                                    \
    do {                                                                           \
        const float _e = (expected);                                               \
        const float _a = (actual);                                                 \
        if (fabsf(_e - _a) > (eps)) {                                              \
            fprintf(stderr, "  FAIL %s:%d: expected %.6f got %.6f (eps %.6f)\n",   \
                    __FILE__, __LINE__, (double)_e, (double)_a, (double)(eps));     \
            g_tests_failed++;                                                      \
        }                                                                          \
    } while (0)

#define TEST_RUN(name)                                                             \
    do {                                                                           \
        const char *_name = (name);                                                \
        int _fail_before = g_tests_failed;                                         \
        printf("TEST %s\n", _name);                                                 \
        name(void);                                                                \
        g_tests_run++;                                                             \
        if (g_tests_failed == _fail_before) {                                      \
            printf("  PASS\n");                                                    \
        }                                                                          \
    } while (0)

static inline int test_harness_summary(void)
{
    printf("SUMMARY %d/%d PASS\n", g_tests_run - g_tests_failed, g_tests_run);
    return (g_tests_failed == 0) ? 0 : 1;
}

#endif /* TEST_HARNESS_H */
