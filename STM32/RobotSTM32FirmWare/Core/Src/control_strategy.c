/**
 * @file control_strategy.c
 */

#include "control_strategy.h"

#include "app_config.h"

#include <stddef.h>

typedef struct {
    const char *name;
    void (*reset)(void);
    void (*update)(const control_strategy_input_t *in, control_strategy_output_t *out);
} control_strategy_ops_t;

volatile control_strategy_id_t g_ctrl_strategy = (control_strategy_id_t)APP_CTRL_STRATEGY_DEFAULT;

void control_strategy_dual_pid_reset(void);
void control_strategy_dual_pid_update(const control_strategy_input_t *in, control_strategy_output_t *out);

void control_strategy_linear_reset(void);
void control_strategy_linear_update(const control_strategy_input_t *in, control_strategy_output_t *out);

void control_strategy_cascade_reset(void);
void control_strategy_cascade_update(const control_strategy_input_t *in, control_strategy_output_t *out);

static const control_strategy_ops_t s_ops[CTRL_STRATEGY_COUNT] = {
    [CTRL_STRATEGY_DUAL_PID] = {
        .name = "dual_pid",
        .reset = control_strategy_dual_pid_reset,
        .update = control_strategy_dual_pid_update,
    },
    [CTRL_STRATEGY_LINEAR] = {
        .name = "linear",
        .reset = control_strategy_linear_reset,
        .update = control_strategy_linear_update,
    },
    [CTRL_STRATEGY_CASCADE] = {
        .name = "cascade",
        .reset = control_strategy_cascade_reset,
        .update = control_strategy_cascade_update,
    },
};

static control_strategy_id_t s_active = (control_strategy_id_t)APP_CTRL_STRATEGY_DEFAULT;

static bool strategy_id_valid(control_strategy_id_t id)
{
    return (unsigned)id < (unsigned)CTRL_STRATEGY_COUNT;
}

void control_strategy_init(void)
{
    for (unsigned i = 0u; i < (unsigned)CTRL_STRATEGY_COUNT; i++) {
        if (s_ops[i].reset != NULL) {
            s_ops[i].reset();
        }
    }
    s_active = (control_strategy_id_t)APP_CTRL_STRATEGY_DEFAULT;
    g_ctrl_strategy = s_active;
}

control_strategy_id_t control_strategy_get(void)
{
    return s_active;
}

const char *control_strategy_name(control_strategy_id_t id)
{
    if (!strategy_id_valid(id)) {
        return "?";
    }
    return s_ops[id].name;
}

bool control_strategy_set(control_strategy_id_t id)
{
    if (!strategy_id_valid(id)) {
        return false;
    }
    if (id == s_active) {
        g_ctrl_strategy = id;
        return true;
    }
    if (s_ops[s_active].reset != NULL) {
        s_ops[s_active].reset();
    }
    s_active = id;
    g_ctrl_strategy = id;
    if (s_ops[s_active].reset != NULL) {
        s_ops[s_active].reset();
    }
    return true;
}

void control_strategy_update(const control_strategy_input_t *in, control_strategy_output_t *out)
{
    if (in == NULL || out == NULL) {
        return;
    }

    /* Live switch: g_ctrl_strategy written from debugger or Jetson later. */
    if (g_ctrl_strategy != s_active && strategy_id_valid(g_ctrl_strategy)) {
        (void)control_strategy_set(g_ctrl_strategy);
    }

    if (s_ops[s_active].update != NULL) {
        s_ops[s_active].update(in, out);
    }
}
