/**
 * @file app_motor_command.c
 */

#include "app_motor_command.h"

#include <string.h>

#if defined(__ARM_ARCH)
#define APP_MOTOR_CMD_DMB() __asm volatile("dmb" ::: "memory")
#elif defined(__GNUC__)
#define APP_MOTOR_CMD_DMB() __asm volatile("" ::: "memory")
#else
#define APP_MOTOR_CMD_DMB() ((void)0)
#endif

static volatile uint32_t s_cmd_seq;
static app_motor_command_t s_cmd;

void app_motor_command_publish(const app_motor_command_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    uint32_t seq = s_cmd_seq + 1u;
    s_cmd_seq = seq;
    APP_MOTOR_CMD_DMB();
    s_cmd = *cmd;
    APP_MOTOR_CMD_DMB();
    s_cmd_seq = seq + 1u;
}

bool app_motor_command_read(app_motor_command_t *out)
{
    if (out == NULL) {
        return false;
    }

    for (uint32_t attempt = 0u; attempt < 4u; attempt++) {
        uint32_t seq1 = s_cmd_seq;
        if ((seq1 & 1u) != 0u) {
            continue;
        }
        APP_MOTOR_CMD_DMB();
        *out = s_cmd;
        APP_MOTOR_CMD_DMB();
        if (s_cmd_seq == seq1) {
            return out->valid;
        }
    }

    return false;
}
