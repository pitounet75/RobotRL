/**
 * @file telemetry_protocol.h
 * @brief Binary telemetry frame layout (UART), protocol version 1.
 *
 *   [magic:u16][length:u16][version:u8][sequence:u16][message_type:u16][error_code:u8]
 *   [payload:0..N][crc8:u8]
 *
 * @p length = bytes from version through crc8 (inclusive).
 * @p Host requests: set error_code to TELEMETRY_ERR_NONE (STM32 ignores it on RX).
 * @p STM32 responses: error_code signals result; see telemetry_error_code_t.
 */

#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEMETRY_MAGIC 0x544Du /* 'TM' */
#define TELEMETRY_PROTOCOL_VERSION 1u

#define TELEMETRY_PREFIX_SIZE 4u
#define TELEMETRY_BODY_HEADER_SIZE 6u /* version + seq + type + error */
#define TELEMETRY_PAYLOAD_OFFSET (TELEMETRY_PREFIX_SIZE + TELEMETRY_BODY_HEADER_SIZE)

#define TELEMETRY_OFF_VERSION 4u
#define TELEMETRY_OFF_SEQUENCE 5u
#define TELEMETRY_OFF_MESSAGE_TYPE 7u
#define TELEMETRY_OFF_ERROR_CODE 9u

#define TELEMETRY_MIN_BODY_LEN 7u
#define TELEMETRY_MIN_FRAME_SIZE (TELEMETRY_PREFIX_SIZE + TELEMETRY_MIN_BODY_LEN)

#define TELEMETRY_BODY_LEN(payload_len) ((uint16_t)(TELEMETRY_BODY_HEADER_SIZE + (payload_len) + 1u))
#define TELEMETRY_FRAME_LEN(payload_len) ((uint16_t)(TELEMETRY_PREFIX_SIZE + TELEMETRY_BODY_LEN(payload_len)))
#define TELEMETRY_PAYLOAD_LEN(body_len) ((uint16_t)((body_len) - TELEMETRY_MIN_BODY_LEN))

/** CRC-8 poly 0x07. ~40 bytes ROM, ~10-20 cycles/byte on Cortex-M (vs CRC16 ~2x cost). */
uint8_t telemetry_crc8(const uint8_t *data, uint16_t len);

typedef enum {
    TELEMETRY_ERR_INVALID_MESSAGE = 0,
    TELEMETRY_ERR_NONE = 1,
    TELEMETRY_ERR_PROTOCOL_VERSION = 2,
    TELEMETRY_ERR_INVALID_PAYLOAD = 3,
    TELEMETRY_ERR_UNKNOWN_MESSAGE_TYPE = 4,
    TELEMETRY_ERR_UNKNOWN_MESSAGE_KEY = 5,
    TELEMETRY_ERR_TX_FAILED = 6,
} telemetry_error_code_t;

typedef enum {
    TELEM_MSG_GET_ERROR = 0,
    TELEM_MSG_GET_DICTIONARY = 1,
    TELEM_MSG_GET_MESSAGE_DESCRIPTION = 2,
    TELEM_MSG_GET_TELEMETRY_FRAME = 3,
    TELEM_MSG_GET_CONFIG = 4,
    TELEM_MSG_SET_ENCODER_SPEEDS = 5,
} telemetry_builtin_msg_t;

#define TELEMETRY_DICTIONARY_CATALOG \
    "GlobalError : 0, Dictionary : 1, MessageDescription : 2, TelemetryFrame : 3, GetConfig : 4, SetEncoderSpeeds : 5, BalanceFrame : 256"

#define TELEMETRY_KEY_GLOBAL_ERROR "GlobalError"
#define TELEMETRY_KEY_DICTIONARY "Dictionary"
#define TELEMETRY_KEY_MESSAGE_DESCRIPTION "MessageDescription"
#define TELEMETRY_KEY_TELEMETRY_FRAME "TelemetryFrame"
#define TELEMETRY_KEY_GET_CONFIG "GetConfig"
#define TELEMETRY_KEY_SET_ENCODER_SPEEDS "SetEncoderSpeeds"

#define TELEMETRY_USER_MSG_TYPE_MIN 0x0100u

#define TELEMETRY_TYPE_STRING "String"
#define TELEMETRY_TYPE_INT8 "Int8"
#define TELEMETRY_TYPE_INT16 "Int16"
#define TELEMETRY_TYPE_INT32 "Int32"
#define TELEMETRY_TYPE_UINT32 "UInt32"
#define TELEMETRY_TYPE_FLOAT "float"
#define TELEMETRY_TYPE_DOUBLE "double"

#define TELEMETRY_SCHEMA_SECTION_REQUEST "request"
#define TELEMETRY_SCHEMA_SECTION_RESPONSE "response"
#define TELEMETRY_FIELD_DICT_SEPARATOR ", "

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_PROTOCOL_H */
