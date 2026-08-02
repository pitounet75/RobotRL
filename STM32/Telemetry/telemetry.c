/**
 * @file telemetry.c
 */

#include "telemetry.h"

#include <string.h>

static const telemetry_field_def_t global_error_response_fields[] = {
    {"error", TELEMETRY_TYPE_STRING},
};

static const telemetry_field_def_t dictionary_response_fields[] = {
    {"catalog", TELEMETRY_TYPE_STRING},
};

static const telemetry_field_def_t message_description_request_fields[] = {
    {"message_key", TELEMETRY_TYPE_STRING},
};

static const telemetry_field_def_t message_description_response_fields[] = {
    {"schemas", TELEMETRY_TYPE_STRING},
};

static const telemetry_field_def_t telemetry_frame_request_fields[] = {
    {"streaming_interval_ms", TELEMETRY_TYPE_UINT32},
};

static const telemetry_field_def_t telemetry_frame_response_fields[] = {
    {"frame_number", TELEMETRY_TYPE_UINT32},
    {"encoder_speed_l", TELEMETRY_TYPE_FLOAT},
    {"encoder_speed_r", TELEMETRY_TYPE_FLOAT},
    {"omega", TELEMETRY_TYPE_FLOAT},
    {"angle", TELEMETRY_TYPE_FLOAT},
    {"distance_x", TELEMETRY_TYPE_FLOAT},
    {"distance_y", TELEMETRY_TYPE_FLOAT},
    {"setpoint_l", TELEMETRY_TYPE_FLOAT},
    {"setpoint_r", TELEMETRY_TYPE_FLOAT},
    {"angle_setpoint", TELEMETRY_TYPE_FLOAT},
};

static const telemetry_field_def_t get_config_response_fields[] = {
    {"config_version", TELEMETRY_TYPE_UINT32},
};

static const telemetry_field_def_t set_encoder_speeds_request_fields[] = {
    {"encoder_speed_l", TELEMETRY_TYPE_FLOAT},
    {"encoder_speed_r", TELEMETRY_TYPE_FLOAT},
};

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

uint8_t telemetry_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; bit++) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((crc << 1) ^ 0x07u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static int telemetry_write_all(telemetry_t *tel, const uint8_t *data, uint16_t len)
{
    if (tel == NULL || tel->write == NULL) {
        return -1;
    }
    return tel->write(tel->write_ctx, data, len);
}

int telemetry_send_frame(telemetry_t *tel, uint16_t message_type, uint16_t sequence_id,
                         telemetry_error_code_t error_code, const uint8_t *payload, uint16_t payload_len)
{
    if (tel == NULL) {
        return -1;
    }
    const uint16_t body_len = TELEMETRY_BODY_LEN(payload_len);
    const uint16_t frame_len = TELEMETRY_FRAME_LEN(payload_len);
    if (frame_len > TELEMETRY_MAX_FRAME_SIZE) {
        return -2;
    }

    uint8_t frame[TELEMETRY_MAX_FRAME_SIZE];
    write_u16_le(&frame[0], TELEMETRY_MAGIC);
    write_u16_le(&frame[2], body_len);
    frame[TELEMETRY_OFF_VERSION] = TELEMETRY_PROTOCOL_VERSION;
    write_u16_le(&frame[TELEMETRY_OFF_SEQUENCE], sequence_id);
    write_u16_le(&frame[TELEMETRY_OFF_MESSAGE_TYPE], message_type);
    frame[TELEMETRY_OFF_ERROR_CODE] = (uint8_t)error_code;
    if (payload_len > 0u && payload != NULL) {
        memcpy(&frame[TELEMETRY_PAYLOAD_OFFSET], payload, payload_len);
    }
    frame[TELEMETRY_PAYLOAD_OFFSET + payload_len] =
        telemetry_crc8(frame, (uint16_t)(TELEMETRY_PAYLOAD_OFFSET + payload_len));

    if (telemetry_write_all(tel, frame, frame_len) != 0) {
        return -3;
    }
    return 0;
}

static void telemetry_set_protocol_error(telemetry_t *tel, const char *message)
{
    telemetry_set_error(tel, message);
}

static void telemetry_reply_error(telemetry_t *tel, uint16_t message_type, uint16_t sequence_id,
                                  telemetry_error_code_t code, const char *message)
{
    telemetry_set_protocol_error(tel, message);
    (void)telemetry_send_frame(tel, message_type, sequence_id, code, NULL, 0);
}

void telemetry_init(telemetry_t *tel, telemetry_write_fn write, void *write_ctx)
{
    if (tel == NULL) {
        return;
    }
    memset(tel, 0, sizeof(*tel));
    tel->write = write;
    tel->write_ctx = write_ctx;
    tel->next_sequence_id = 1u;
    tel->odrive_config.config_version = 0u;
}

void telemetry_set_frame(telemetry_t *tel, const telemetry_frame_t *frame)
{
    if (tel == NULL || frame == NULL) {
        return;
    }
    tel->frame_number_source++;
    tel->telemetry_frame = *frame;
    tel->telemetry_frame.frame_number = tel->frame_number_source;
}

void telemetry_get_frame(const telemetry_t *tel, telemetry_frame_t *frame)
{
    if (tel == NULL || frame == NULL) {
        return;
    }
    *frame = tel->telemetry_frame;
}

uint32_t telemetry_get_stream_interval_ms(const telemetry_t *tel)
{
    if (tel == NULL) {
        return 0u;
    }
    return tel->stream_interval_ms;
}

void telemetry_set_error(telemetry_t *tel, const char *message)
{
    if (tel == NULL) {
        return;
    }
    if (message == NULL) {
        tel->error_message[0] = '\0';
        return;
    }
    strncpy(tel->error_message, message, TELEMETRY_MAX_ERROR_LEN - 1u);
    tel->error_message[TELEMETRY_MAX_ERROR_LEN - 1u] = '\0';
}

void telemetry_get_command_encoder_speeds(const telemetry_t *tel, float *left, float *right)
{
    if (tel == NULL) {
        return;
    }
    if (left != NULL) {
        *left = tel->command_encoder_speed_l;
    }
    if (right != NULL) {
        *right = tel->command_encoder_speed_r;
    }
}

void telemetry_set_odrive_config(telemetry_t *tel, const telemetry_config_t *cfg)
{
    if (tel == NULL || cfg == NULL) {
        return;
    }
    tel->odrive_config = *cfg;
}

void telemetry_get_odrive_config(const telemetry_t *tel, telemetry_config_t *cfg)
{
    if (tel == NULL || cfg == NULL) {
        return;
    }
    *cfg = tel->odrive_config;
}

static int telemetry_append_cstring(uint8_t *out, uint16_t cap, uint16_t *pos, const char *str)
{
    const uint16_t len = (uint16_t)(strlen(str) + 1u);
    if ((uint32_t)(*pos) + len > cap) {
        return -1;
    }
    memcpy(&out[*pos], str, len);
    *pos = (uint16_t)(*pos + len);
    return 0;
}

static int telemetry_append_bytes(uint8_t *out, uint16_t cap, uint16_t *pos, const char *text, uint16_t len)
{
    if ((uint32_t)(*pos) + len > cap) {
        return -1;
    }
    memcpy(&out[*pos], text, len);
    *pos = (uint16_t)(*pos + len);
    return 0;
}

static int telemetry_build_field_dictionary(const telemetry_field_def_t *fields, uint16_t count, uint8_t *out,
                                            uint16_t cap, uint16_t *out_len)
{
    uint16_t pos = 0;
    for (uint16_t i = 0; i < count; i++) {
        const char *name = fields[i].name;
        const char *type_name = fields[i].type_name;
        const uint16_t name_len = (uint16_t)strlen(name);
        const uint16_t type_len = (uint16_t)strlen(type_name);
        if (i > 0u) {
            if (telemetry_append_bytes(out, cap, &pos, TELEMETRY_FIELD_DICT_SEPARATOR,
                                       (uint16_t)strlen(TELEMETRY_FIELD_DICT_SEPARATOR)) != 0) {
                return -1;
            }
        }
        if (telemetry_append_bytes(out, cap, &pos, name, name_len) != 0) {
            return -1;
        }
        if (telemetry_append_bytes(out, cap, &pos, " : ", 3u) != 0) {
            return -1;
        }
        if (telemetry_append_bytes(out, cap, &pos, type_name, type_len) != 0) {
            return -1;
        }
    }
    if ((uint32_t)pos + 1u > cap) {
        return -1;
    }
    out[pos++] = '\0';
    *out_len = pos;
    return 0;
}

static int telemetry_build_message_description_payload(const telemetry_message_def_t *msg, uint8_t *out,
                                                       uint16_t cap, uint16_t *out_len)
{
    uint16_t pos = 0;
    uint16_t section_len = 0;
    uint8_t section_buf[TELEMETRY_MAX_DESC_PAYLOAD / 2u];

    if (telemetry_append_cstring(out, cap, &pos, TELEMETRY_SCHEMA_SECTION_REQUEST) != 0) {
        return -1;
    }
    if (msg->request_field_count == 0u) {
        if (telemetry_append_cstring(out, cap, &pos, "") != 0) {
            return -1;
        }
    } else if (telemetry_build_field_dictionary(msg->request_fields, msg->request_field_count, section_buf,
                                                (uint16_t)sizeof(section_buf), &section_len) != 0) {
        return -1;
    } else if (telemetry_append_bytes(out, cap, &pos, (const char *)section_buf, section_len) != 0) {
        return -1;
    }

    if (telemetry_append_cstring(out, cap, &pos, TELEMETRY_SCHEMA_SECTION_RESPONSE) != 0) {
        return -1;
    }
    if (msg->response_field_count == 0u) {
        if (telemetry_append_cstring(out, cap, &pos, "") != 0) {
            return -1;
        }
    } else if (telemetry_build_field_dictionary(msg->response_fields, msg->response_field_count, section_buf,
                                                  (uint16_t)sizeof(section_buf), &section_len) != 0) {
        return -1;
    } else if (telemetry_append_bytes(out, cap, &pos, (const char *)section_buf, section_len) != 0) {
        return -1;
    }

    *out_len = pos;
    return 0;
}

static int telemetry_validate_field_array(const telemetry_field_def_t *fields, uint16_t count)
{
    if (count == 0u) {
        return 0;
    }
    if (fields == NULL) {
        return -1;
    }
    for (uint16_t i = 0; i < count; i++) {
        if (fields[i].name == NULL || fields[i].type_name == NULL) {
            return -2;
        }
        if (strlen(fields[i].name) >= TELEMETRY_MAX_FIELD_NAME_LEN) {
            return -3;
        }
    }
    return 0;
}

static int telemetry_validate_message_def(const telemetry_message_def_t *def)
{
    if (telemetry_validate_field_array(def->request_fields, def->request_field_count) != 0) {
        return -1;
    }
    if (telemetry_validate_field_array(def->response_fields, def->response_field_count) != 0) {
        return -2;
    }
    if (def->request_field_count == 0u && def->response_field_count == 0u) {
        return -3;
    }
    return 0;
}

int telemetry_register(telemetry_t *tel, const telemetry_message_def_t *def)
{
    if (tel == NULL || def == NULL || def->key == NULL) {
        return -1;
    }
    if (def->message_type < TELEMETRY_USER_MSG_TYPE_MIN) {
        return -2;
    }
    if (tel->message_count >= TELEMETRY_MAX_MESSAGES) {
        return -3;
    }
    if (strlen(def->key) >= TELEMETRY_MAX_KEY_LEN) {
        return -4;
    }
    if (def->request_field_count > TELEMETRY_MAX_FIELDS || def->response_field_count > TELEMETRY_MAX_FIELDS) {
        return -7;
    }
    if (telemetry_validate_message_def(def) != 0) {
        return -8;
    }

    for (uint16_t i = 0; i < tel->message_count; i++) {
        if (tel->messages[i].message_type == def->message_type) {
            return -5;
        }
        if (strcmp(tel->messages[i].key, def->key) == 0) {
            return -6;
        }
    }

    tel->messages[tel->message_count++] = *def;
    return 0;
}

static const telemetry_message_def_t *telemetry_find_by_type(const telemetry_t *tel, uint16_t type)
{
    for (uint16_t i = 0; i < tel->message_count; i++) {
        if (tel->messages[i].message_type == type) {
            return &tel->messages[i];
        }
    }
    return NULL;
}

static const telemetry_message_def_t *telemetry_find_by_key(const telemetry_t *tel, const char *key)
{
    for (uint16_t i = 0; i < tel->message_count; i++) {
        if (strcmp(tel->messages[i].key, key) == 0) {
            return &tel->messages[i];
        }
    }
    return NULL;
}

static const telemetry_message_def_t *telemetry_builtin_def(const char *key)
{
    static telemetry_message_def_t def;

    if (strcmp(key, TELEMETRY_KEY_GLOBAL_ERROR) == 0) {
        def = (telemetry_message_def_t){
            .key = TELEMETRY_KEY_GLOBAL_ERROR,
            .message_type = TELEM_MSG_GET_ERROR,
            .response_fields = global_error_response_fields,
            .response_field_count = 1,
        };
        return &def;
    }
    if (strcmp(key, TELEMETRY_KEY_DICTIONARY) == 0) {
        def = (telemetry_message_def_t){
            .key = TELEMETRY_KEY_DICTIONARY,
            .message_type = TELEM_MSG_GET_DICTIONARY,
            .response_fields = dictionary_response_fields,
            .response_field_count = 1,
        };
        return &def;
    }
    if (strcmp(key, TELEMETRY_KEY_MESSAGE_DESCRIPTION) == 0) {
        def = (telemetry_message_def_t){
            .key = TELEMETRY_KEY_MESSAGE_DESCRIPTION,
            .message_type = TELEM_MSG_GET_MESSAGE_DESCRIPTION,
            .request_fields = message_description_request_fields,
            .request_field_count = 1,
            .response_fields = message_description_response_fields,
            .response_field_count = 1,
        };
        return &def;
    }
    if (strcmp(key, TELEMETRY_KEY_TELEMETRY_FRAME) == 0) {
        def = (telemetry_message_def_t){
            .key = TELEMETRY_KEY_TELEMETRY_FRAME,
            .message_type = TELEM_MSG_GET_TELEMETRY_FRAME,
            .request_fields = telemetry_frame_request_fields,
            .request_field_count = 1,
            .response_fields = telemetry_frame_response_fields,
            .response_field_count =
                (uint16_t)(sizeof(telemetry_frame_response_fields) / sizeof(telemetry_frame_response_fields[0])),
        };
        return &def;
    }
    if (strcmp(key, TELEMETRY_KEY_GET_CONFIG) == 0) {
        def = (telemetry_message_def_t){
            .key = TELEMETRY_KEY_GET_CONFIG,
            .message_type = TELEM_MSG_GET_CONFIG,
            .response_fields = get_config_response_fields,
            .response_field_count = 1,
        };
        return &def;
    }
    if (strcmp(key, TELEMETRY_KEY_SET_ENCODER_SPEEDS) == 0) {
        def = (telemetry_message_def_t){
            .key = TELEMETRY_KEY_SET_ENCODER_SPEEDS,
            .message_type = TELEM_MSG_SET_ENCODER_SPEEDS,
            .request_fields = set_encoder_speeds_request_fields,
            .request_field_count = 2,
        };
        return &def;
    }
    return NULL;
}

static void telemetry_handle_get_error(telemetry_t *tel, uint16_t sequence_id)
{
    const uint16_t len = (uint16_t)(strlen(tel->error_message) + 1u);
    (void)telemetry_send_frame(tel, TELEM_MSG_GET_ERROR, sequence_id, TELEMETRY_ERR_NONE,
                               (const uint8_t *)tel->error_message, len);
}

static void telemetry_handle_get_dictionary(telemetry_t *tel, uint16_t sequence_id)
{
    const uint16_t len = (uint16_t)(strlen(TELEMETRY_DICTIONARY_CATALOG) + 1u);
    (void)telemetry_send_frame(tel, TELEM_MSG_GET_DICTIONARY, sequence_id, TELEMETRY_ERR_NONE,
                             (const uint8_t *)TELEMETRY_DICTIONARY_CATALOG, len);
}

static void telemetry_send_telemetry_frame(telemetry_t *tel, uint16_t sequence_id)
{
    uint8_t payload[TELEMETRY_FRAME_PAYLOAD_LEN];
    telemetry_frame_encode(&tel->telemetry_frame, payload);
    (void)telemetry_send_frame(tel, TELEM_MSG_GET_TELEMETRY_FRAME, sequence_id, TELEMETRY_ERR_NONE, payload,
                               TELEMETRY_FRAME_PAYLOAD_LEN);
}

static void telemetry_handle_get_telemetry_frame(telemetry_t *tel, uint16_t sequence_id, const uint8_t *payload,
                                                 uint16_t payload_len)
{
    if (payload_len != TELEMETRY_FRAME_REQUEST_PAYLOAD_LEN) {
        telemetry_reply_error(tel, TELEM_MSG_GET_TELEMETRY_FRAME, sequence_id, TELEMETRY_ERR_INVALID_PAYLOAD,
                              "TelemetryFrame: expected 4-byte request");
        return;
    }

    const uint32_t interval_ms = read_u32_le(payload);
    tel->stream_interval_ms = interval_ms;
    tel->stream_elapsed_ms = 0u;

    telemetry_send_telemetry_frame(tel, sequence_id);

    if (interval_ms == 0u) {
        tel->stream_interval_ms = 0u;
    }
}

static void telemetry_handle_get_config(telemetry_t *tel, uint16_t sequence_id, uint16_t payload_len)
{
    if (payload_len != 0u) {
        telemetry_reply_error(tel, TELEM_MSG_GET_CONFIG, sequence_id, TELEMETRY_ERR_INVALID_PAYLOAD,
                              "GetConfig: empty request expected");
        return;
    }
    (void)telemetry_send_frame(tel, TELEM_MSG_GET_CONFIG, sequence_id, TELEMETRY_ERR_NONE,
                               (const uint8_t *)&tel->odrive_config, TELEMETRY_CONFIG_PAYLOAD_LEN);
}

static void telemetry_handle_set_encoder_speeds(telemetry_t *tel, uint16_t sequence_id, const uint8_t *payload,
                                                uint16_t payload_len)
{
    if (payload_len != 8u) {
        telemetry_reply_error(tel, TELEM_MSG_SET_ENCODER_SPEEDS, sequence_id, TELEMETRY_ERR_INVALID_PAYLOAD,
                              "SetEncoderSpeeds: expected 8-byte request");
        return;
    }
    memcpy(&tel->command_encoder_speed_l, &payload[0], sizeof(float));
    memcpy(&tel->command_encoder_speed_r, &payload[4], sizeof(float));
    (void)telemetry_send_frame(tel, TELEM_MSG_SET_ENCODER_SPEEDS, sequence_id, TELEMETRY_ERR_NONE, NULL, 0);
}

void telemetry_tick_1ms(telemetry_t *tel)
{
    if (tel == NULL || tel->stream_interval_ms == 0u) {
        return;
    }

    tel->stream_elapsed_ms++;
    if (tel->stream_elapsed_ms < tel->stream_interval_ms) {
        return;
    }

    tel->stream_elapsed_ms = 0u;
    const uint16_t seq = tel->next_sequence_id++;
    telemetry_send_telemetry_frame(tel, seq);
}

static void telemetry_handle_get_message_description(telemetry_t *tel, uint16_t sequence_id,
                                                   const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len == 0u || payload[payload_len - 1u] != 0u) {
        telemetry_reply_error(tel, TELEM_MSG_GET_MESSAGE_DESCRIPTION, sequence_id, TELEMETRY_ERR_INVALID_PAYLOAD,
                              "MessageDescription: key must be NUL-terminated");
        return;
    }

    const char *key = (const char *)payload;
    const telemetry_message_def_t *msg = telemetry_builtin_def(key);
    if (msg == NULL) {
        msg = telemetry_find_by_key(tel, key);
    }
    if (msg == NULL) {
        telemetry_reply_error(tel, TELEM_MSG_GET_MESSAGE_DESCRIPTION, sequence_id, TELEMETRY_ERR_UNKNOWN_MESSAGE_KEY,
                              "MessageDescription: unknown key");
        return;
    }

    uint8_t desc_payload[TELEMETRY_MAX_DESC_PAYLOAD];
    uint16_t desc_len = 0;
    if (telemetry_build_message_description_payload(msg, desc_payload, TELEMETRY_MAX_DESC_PAYLOAD, &desc_len) != 0) {
        telemetry_reply_error(tel, TELEM_MSG_GET_MESSAGE_DESCRIPTION, sequence_id, TELEMETRY_ERR_INVALID_PAYLOAD,
                              "MessageDescription: schema too large");
        return;
    }
    (void)telemetry_send_frame(tel, TELEM_MSG_GET_MESSAGE_DESCRIPTION, sequence_id, TELEMETRY_ERR_NONE, desc_payload,
                               desc_len);
}

static void telemetry_dispatch_frame(telemetry_t *tel, const uint8_t *frame, uint16_t frame_len)
{
    if (frame_len < TELEMETRY_MIN_FRAME_SIZE) {
        return;
    }

    const uint16_t magic = read_u16_le(&frame[0]);
    const uint16_t body_len = read_u16_le(&frame[2]);
    const uint8_t version = frame[TELEMETRY_OFF_VERSION];
    const uint16_t sequence_id = read_u16_le(&frame[TELEMETRY_OFF_SEQUENCE]);
    const uint16_t message_type = read_u16_le(&frame[TELEMETRY_OFF_MESSAGE_TYPE]);

    if (magic != TELEMETRY_MAGIC || body_len < TELEMETRY_MIN_BODY_LEN) {
        return;
    }
    if ((uint32_t)TELEMETRY_PREFIX_SIZE + body_len != frame_len) {
        return;
    }

    const uint16_t payload_len = TELEMETRY_PAYLOAD_LEN(body_len);
    const uint8_t *payload = &frame[TELEMETRY_PAYLOAD_OFFSET];
    const uint8_t rx_crc = frame[frame_len - 1u];
    const uint8_t calc_crc = telemetry_crc8(frame, (uint16_t)(frame_len - 1u));

    if (rx_crc != calc_crc) {
        telemetry_reply_error(tel, message_type, sequence_id, TELEMETRY_ERR_INVALID_MESSAGE, "checksum mismatch");
        return;
    }

    if (version != TELEMETRY_PROTOCOL_VERSION) {
        telemetry_reply_error(tel, message_type, sequence_id, TELEMETRY_ERR_PROTOCOL_VERSION,
                              "unsupported protocol version");
        return;
    }

    /* error byte on requests is for host use only — ignored on RX */

    switch (message_type) {
    case TELEM_MSG_GET_ERROR:
        if (payload_len != 0u) {
            telemetry_reply_error(tel, message_type, sequence_id, TELEMETRY_ERR_INVALID_PAYLOAD,
                                  "GlobalError: empty request expected");
            return;
        }
        telemetry_handle_get_error(tel, sequence_id);
        return;

    case TELEM_MSG_GET_DICTIONARY:
        if (payload_len != 0u) {
            telemetry_reply_error(tel, message_type, sequence_id, TELEMETRY_ERR_INVALID_PAYLOAD,
                                  "Dictionary: empty request expected");
            return;
        }
        telemetry_handle_get_dictionary(tel, sequence_id);
        return;

    case TELEM_MSG_GET_MESSAGE_DESCRIPTION:
        telemetry_handle_get_message_description(tel, sequence_id, payload, payload_len);
        return;

    case TELEM_MSG_GET_TELEMETRY_FRAME:
        telemetry_handle_get_telemetry_frame(tel, sequence_id, payload, payload_len);
        return;

    case TELEM_MSG_GET_CONFIG:
        telemetry_handle_get_config(tel, sequence_id, payload_len);
        return;

    case TELEM_MSG_SET_ENCODER_SPEEDS:
        telemetry_handle_set_encoder_speeds(tel, sequence_id, payload, payload_len);
        return;

    default: {
        const telemetry_message_def_t *msg = telemetry_find_by_type(tel, message_type);
        if (msg != NULL && msg->on_descend != NULL) {
            (void)msg->on_descend(tel, sequence_id, payload, payload_len, msg->user_data);
            return;
        }
        telemetry_reply_error(tel, message_type, sequence_id, TELEMETRY_ERR_UNKNOWN_MESSAGE_TYPE,
                              "unknown message type");
        return;
    }
    }
}

static void telemetry_reset_rx(telemetry_t *tel)
{
    tel->rx_len = 0;
}

static void telemetry_try_parse(telemetry_t *tel)
{
    while (tel->rx_len >= TELEMETRY_PREFIX_SIZE) {
        if (read_u16_le(&tel->rx_buf[0]) != TELEMETRY_MAGIC) {
            memmove(tel->rx_buf, &tel->rx_buf[1], tel->rx_len - 1u);
            tel->rx_len--;
            continue;
        }

        if (tel->rx_len < TELEMETRY_PREFIX_SIZE) {
            return;
        }

        const uint16_t body_len = read_u16_le(&tel->rx_buf[2]);
        if (body_len < TELEMETRY_MIN_BODY_LEN) {
            memmove(tel->rx_buf, &tel->rx_buf[1], tel->rx_len - 1u);
            tel->rx_len--;
            continue;
        }

        const uint16_t frame_len = (uint16_t)(TELEMETRY_PREFIX_SIZE + body_len);
        if (frame_len > TELEMETRY_MAX_FRAME_SIZE) {
            telemetry_reset_rx(tel);
            return;
        }
        if (tel->rx_len < frame_len) {
            return;
        }

        telemetry_dispatch_frame(tel, tel->rx_buf, frame_len);

        if (tel->rx_len > frame_len) {
            memmove(tel->rx_buf, &tel->rx_buf[frame_len], tel->rx_len - frame_len);
        }
        tel->rx_len = (uint16_t)(tel->rx_len - frame_len);
    }
}

void telemetry_rx_feed(telemetry_t *tel, const uint8_t *data, uint16_t len)
{
    if (tel == NULL || data == NULL || len == 0u) {
        return;
    }

    for (uint16_t i = 0; i < len; i++) {
        if (tel->rx_len >= TELEMETRY_MAX_FRAME_SIZE) {
            telemetry_reset_rx(tel);
        }
        tel->rx_buf[tel->rx_len++] = data[i];
        telemetry_try_parse(tel);
    }
}

int telemetry_send(telemetry_t *tel, uint16_t message_type)
{
    if (tel == NULL) {
        return -1;
    }

    const telemetry_message_def_t *msg = telemetry_find_by_type(tel, message_type);
    if (msg == NULL || msg->encode == NULL) {
        return -2;
    }

    uint8_t frame[TELEMETRY_MAX_FRAME_SIZE];
    const uint16_t max_payload =
        (uint16_t)(TELEMETRY_MAX_FRAME_SIZE - TELEMETRY_MIN_FRAME_SIZE);
    uint16_t payload_len = 0;
    if (msg->encode(msg->user_data, &frame[TELEMETRY_PAYLOAD_OFFSET], max_payload, &payload_len) != 0) {
        return -3;
    }

    const uint16_t seq = tel->next_sequence_id++;
    const uint16_t body_len = TELEMETRY_BODY_LEN(payload_len);
    const uint16_t frame_len = TELEMETRY_FRAME_LEN(payload_len);
    if (frame_len > TELEMETRY_MAX_FRAME_SIZE) {
        return -4;
    }

    write_u16_le(&frame[0], TELEMETRY_MAGIC);
    write_u16_le(&frame[2], body_len);
    frame[TELEMETRY_OFF_VERSION] = TELEMETRY_PROTOCOL_VERSION;
    write_u16_le(&frame[TELEMETRY_OFF_SEQUENCE], seq);
    write_u16_le(&frame[TELEMETRY_OFF_MESSAGE_TYPE], message_type);
    frame[TELEMETRY_OFF_ERROR_CODE] = (uint8_t)TELEMETRY_ERR_NONE;
    frame[TELEMETRY_PAYLOAD_OFFSET + payload_len] =
        telemetry_crc8(frame, (uint16_t)(TELEMETRY_PAYLOAD_OFFSET + payload_len));

    if (telemetry_write_all(tel, frame, frame_len) != 0) {
        return -5;
    }
    return 0;
}
