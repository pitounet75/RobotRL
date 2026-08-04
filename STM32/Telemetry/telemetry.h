/**
 * @file telemetry.h
 * @brief UART binary telemetry: registration, framing, built-in dictionary RPCs.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include "telemetry_config.h"
#include "telemetry_frame.h"
#include "telemetry_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TELEMETRY_MAX_MESSAGES
#define TELEMETRY_MAX_MESSAGES 32u
#endif

#ifndef TELEMETRY_MAX_KEY_LEN
#define TELEMETRY_MAX_KEY_LEN 48u
#endif

#ifndef TELEMETRY_MAX_FIELDS
#define TELEMETRY_MAX_FIELDS 32u
#endif

#ifndef TELEMETRY_MAX_FIELD_NAME_LEN
#define TELEMETRY_MAX_FIELD_NAME_LEN 32u
#endif

#ifndef TELEMETRY_MAX_DESC_PAYLOAD
#define TELEMETRY_MAX_DESC_PAYLOAD 384u
#endif

#ifndef TELEMETRY_MAX_FRAME_SIZE
#define TELEMETRY_MAX_FRAME_SIZE 512u
#endif

#ifndef TELEMETRY_MAX_DICT_PAYLOAD
#define TELEMETRY_MAX_DICT_PAYLOAD 384u
#endif

#ifndef TELEMETRY_MAX_ERROR_LEN
#define TELEMETRY_MAX_ERROR_LEN 128u
#endif

/** One payload field in a registered message type. */
typedef struct {
    const char *name;
    const char *type_name; /**< TELEMETRY_TYPE_* string, e.g. TELEMETRY_TYPE_FLOAT */
} telemetry_field_def_t;

typedef struct telemetry telemetry_t;

/**
 * Transport: send raw bytes on UART.
 * Use DMA TX (e.g. HAL_UART_Transmit_DMA); return 0 when transfer started.
 */
typedef int (*telemetry_write_fn)(void *user_data, const uint8_t *data, uint16_t len);

/**
 * Optional ascending encoder for periodic / event telemetry.
 * Fills payload only (no header). Return 0 on success, negative on error.
 */
typedef int (*telemetry_encode_fn)(void *user_data, uint8_t *payload, uint16_t capacity, uint16_t *out_len);

/**
 * Optional custom descending handler (host -> device).
 * Return 0 on success (library may still send a reply if you use telemetry_send_frame).
 */
typedef int (*telemetry_descend_fn)(telemetry_t *tel, uint16_t sequence_id, const uint8_t *payload,
                                    uint16_t payload_len, void *user_data);

typedef struct {
    const char *key;
    uint16_t message_type;
    /** Schema for host -> device (descending request) payload. May be empty. */
    const telemetry_field_def_t *request_fields;
    uint16_t request_field_count;
    /** Schema for device -> host (ascending response / telemetry) payload. May be empty. */
    const telemetry_field_def_t *response_fields;
    uint16_t response_field_count;
    void *user_data;
    /** Encodes response_fields into a payload (ascending). */
    telemetry_encode_fn encode;
    /** Handles an incoming request (descending). */
    telemetry_descend_fn on_descend;
    /**
     * Expected descending payload length when not derivable from request_fields.
     * 0 = empty. TELEMETRY_DESCEND_PAYLOAD_CSTRING = NUL-terminated string body.
     */
    uint16_t descend_payload_len;
} telemetry_message_def_t;

/** Use for descend_payload_len when payload is a NUL-terminated string. */
#define TELEMETRY_DESCEND_PAYLOAD_CSTRING 0xFFFFu

struct telemetry {
    telemetry_write_fn write;
    void *write_ctx;

    telemetry_message_def_t messages[TELEMETRY_MAX_MESSAGES];
    uint16_t message_count;

    uint16_t next_sequence_id;

    telemetry_frame_t telemetry_frame;
    uint32_t frame_number_source;

    telemetry_config_t odrive_config;

    float command_encoder_speed_l;
    float command_encoder_speed_r;

    char error_message[TELEMETRY_MAX_ERROR_LEN];

    /* GetTelemetryFrame streaming: 0 = off, else period in ms */
    uint32_t stream_interval_ms;
    uint32_t stream_elapsed_ms;

    /* RX parser */
    uint8_t rx_buf[TELEMETRY_MAX_FRAME_SIZE];
    uint16_t rx_len;
};

/**
 * Initialize telemetry instance. Does not touch UART hardware.
 */
void telemetry_init(telemetry_t *tel, telemetry_write_fn write, void *write_ctx);

/**
 * Register a message type (key must be unique). Returns 0 or negative.
 */
int telemetry_register(telemetry_t *tel, const telemetry_message_def_t *def);

/**
 * Feed received UART bytes (call from IRQ or main loop).
 */
void telemetry_rx_feed(telemetry_t *tel, const uint8_t *data, uint16_t len);

/**
 * Send a framed message (ascending or response). Uses @p sequence_id as-is.
 */
int telemetry_send_frame(telemetry_t *tel, uint16_t message_type, uint16_t sequence_id,
                         telemetry_error_code_t error_code, const uint8_t *payload, uint16_t payload_len);

/**
 * Send ascending telemetry for a registered type (auto sequence id).
 * The sequence id advances only after the transport accepts the frame.
 */
int telemetry_send(telemetry_t *tel, uint16_t message_type);

/** Update data returned by GetTelemetryFrame (call from control loop). */
void telemetry_set_frame(telemetry_t *tel, const telemetry_frame_t *frame);

/** Read back the current telemetry frame snapshot. */
void telemetry_get_frame(const telemetry_t *tel, telemetry_frame_t *frame);

/** Set error text returned by GetError (type 0). Empty string means no error. */
void telemetry_set_error(telemetry_t *tel, const char *message);

/**
 * Call from a 1 ms timer (HAL tick or TIM ISR). Sends TelemetryFrame when streaming is enabled.
 */
void telemetry_tick_1ms(telemetry_t *tel);

/** Current streaming period in ms (0 = disabled). */
uint32_t telemetry_get_stream_interval_ms(const telemetry_t *tel);

void telemetry_get_command_encoder_speeds(const telemetry_t *tel, float *left, float *right);
void telemetry_set_odrive_config(telemetry_t *tel, const telemetry_config_t *cfg);
void telemetry_get_odrive_config(const telemetry_t *tel, telemetry_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
