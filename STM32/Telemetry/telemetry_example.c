/**
 * @file telemetry_example.c
 * @brief Sample STM32 integration (copy/adapt into your firmware).
 *
 * Enable by defining TELEMETRY_EXAMPLE_ENABLE in your project and linking this file.
 * Requires HAL UART (example uses huart4).
 */

#include "telemetry_example.h"

#include <string.h>

#if defined(TELEMETRY_EXAMPLE_ENABLE)

#include "stm32h7xx_hal.h"

extern UART_HandleTypeDef huart4;

/* Replace with your real robot state (shown as static placeholders). */
static struct {
    float enc_l;
    float enc_r;
    float omega;
    float angle;
    float x;
    float y;
    float cmd_l;
    float cmd_r;
    float angle_ref;
} g_robot;

static int example_uart_write(void *ctx, const uint8_t *data, uint16_t len)
{
    (void)ctx;
    /* Use DMA TX in production; ensure buffer stays valid until DMA completes. */
    return (HAL_UART_Transmit_DMA(&huart4, (uint8_t *)data, len) == HAL_OK) ? 0 : -1;
}

void telemetry_example_init(telemetry_t *tel)
{
    telemetry_init(tel, example_uart_write, NULL);
    telemetry_set_error(tel, ""); /* no error */
}

void telemetry_example_poll_rx(telemetry_t *tel)
{
    uint8_t byte;
    while (HAL_UART_Receive(&huart4, &byte, 1, 0) == HAL_OK) {
        telemetry_rx_feed(tel, &byte, 1);
    }
}

void telemetry_example_update_frame(telemetry_t *tel)
{
    telemetry_frame_t frame = {
        .encoder_speed_l = g_robot.enc_l,
        .encoder_speed_r = g_robot.enc_r,
        .omega = g_robot.omega,
        .angle = g_robot.angle,
        .distance_x = g_robot.x,
        .distance_y = g_robot.y,
        .setpoint_l = g_robot.cmd_l,
        .setpoint_r = g_robot.cmd_r,
        .angle_setpoint = g_robot.angle_ref,
    };
    telemetry_set_frame(tel, &frame);
}

/* Optional: registered application message (type >= 0x0100). */
#if 0

static int example_imu_encode(void *user, uint8_t *payload, uint16_t cap, uint16_t *out_len)
{
    (void)user;
    if (cap < 12u) {
        return -1;
    }
    memset(payload, 0, 12u);
    *out_len = 12u;
    return 0;
}

static const telemetry_field_def_t imu_response_fields[] = {
    {"accel_x", TELEMETRY_TYPE_FLOAT},
    {"accel_y", TELEMETRY_TYPE_FLOAT},
    {"accel_z", TELEMETRY_TYPE_FLOAT},
};

static void example_register_app_messages(telemetry_t *tel)
{
    (void)telemetry_register(tel, &(telemetry_message_def_t){
                                       .key = "imu",
                                       .message_type = 0x0100u,
                                       .request_fields = NULL,
                                       .request_field_count = 0,
                                       .response_fields = imu_response_fields,
                                       .response_field_count = 3,
                                       .encode = example_imu_encode,
                                   });
}

#endif

/*
 * --- Paste into your project -----------------------------------------------
 *
 * telemetry_t g_tel;
 *
 * void main(void) {
 *     HAL_Init();
 *     MX_USART4_UART_Init();
 *     telemetry_example_init(&g_tel);
 *     for (;;) {
 *         telemetry_example_poll_rx(&g_tel);
 *         balance_loop_step();
 *     }
 * }
 *
 * void balance_loop_step(void) {
 *     read_sensors_update_g_robot();
 *     run_controller_update_g_robot();
 *     telemetry_example_update_frame(&g_tel);
 * }
 *
 * void USART4_IRQHandler(void) {
 *     HAL_UART_IRQHandler(&huart4);
 *     uint8_t b;
 *     if (HAL_UART_Receive(&huart4, &b, 1, 0) == HAL_OK)
 *         telemetry_rx_feed(&g_tel, &b, 1);
 * }
 *
 * // 1 ms timer ISR (HAL TIM or SysTick wrapper)
 * void telemetry_1ms_isr(void) {
 *     telemetry_tick_1ms(&g_tel);
 * }
 * ---------------------------------------------------------------------------
 */

#endif /* TELEMETRY_EXAMPLE_ENABLE */
