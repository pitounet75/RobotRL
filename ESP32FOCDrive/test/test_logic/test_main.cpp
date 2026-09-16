#include <Arduino.h>
#include <unity.h>

#include "cmd_parse.h"
#include "enc_math.h"

void setUp() {}
void tearDown() {}

void test_parse_bare_command() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("status", &p));
  TEST_ASSERT_EQUAL_STRING("status", p.cmd);
  TEST_ASSERT_EQUAL_UINT8(0b11, p.axis_mask);
  TEST_ASSERT_FALSE(p.has_value);
}

void test_parse_value_without_axis() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("  vel  3.5 ", &p));
  TEST_ASSERT_EQUAL_STRING("vel", p.cmd);
  TEST_ASSERT_EQUAL_UINT8(0b11, p.axis_mask);
  TEST_ASSERT_TRUE(p.has_value);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.5f, p.value);
}

void test_parse_axis_prefix() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("vel R -2", &p));
  TEST_ASSERT_EQUAL_STRING("vel", p.cmd);
  TEST_ASSERT_EQUAL_UINT8(0b10, p.axis_mask);
  TEST_ASSERT_TRUE(p.has_value);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.0f, p.value);
}

void test_parse_axis_only() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("cal l", &p));
  TEST_ASSERT_EQUAL_STRING("cal", p.cmd);
  TEST_ASSERT_EQUAL_UINT8(0b01, p.axis_mask);
  TEST_ASSERT_FALSE(p.has_value);
}

void test_parse_rejects_empty() {
  ParsedCmd p{};
  TEST_ASSERT_FALSE(parseCmd("   ", &p));
}

void test_parse_long_command_is_truncated_not_overflowed() {
  ParsedCmd p{};
  TEST_ASSERT_TRUE(parseCmd("abcdefghijklmnopqrstuvwxyz 1", &p));
  TEST_ASSERT_EQUAL_UINT32(11, strlen(p.cmd));
  TEST_ASSERT_TRUE(p.has_value);
}

void test_fold_small_forward_delta() {
  TEST_ASSERT_EQUAL_INT32(5, encFoldDelta(100, 105, 16384));
}

void test_fold_small_backward_delta() {
  TEST_ASSERT_EQUAL_INT32(-5, encFoldDelta(105, 100, 16384));
}

/* PCNT resets to 0 at h_lim instead of wrapping two's-complement: going
 * forward past +16384 reappears near 0, which is a -16384 raw jump. */
void test_fold_hlim_reset_reads_as_forward() {
  TEST_ASSERT_EQUAL_INT32(10, encFoldDelta(16380, -16374, 16384));
}

void test_fold_llim_reset_reads_as_backward() {
  TEST_ASSERT_EQUAL_INT32(-10, encFoldDelta(-16380, 16374, 16384));
}

void test_angle_from_count_positive() {
  int32_t rot = 0;
  float shaft = 0.0f;
  encAngleFromCount(65536 + 16384, 65536, &rot, &shaft);
  TEST_ASSERT_EQUAL_INT32(1, rot);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.5708f, shaft);
}

/* Negative counts must still yield shaft in [0, 2PI): SimpleFOC 2.3.3 treats a
 * negative getSensorAngle() as an error and skips the update entirely. */
void test_angle_from_count_negative_stays_in_range() {
  int32_t rot = 0;
  float shaft = 0.0f;
  encAngleFromCount(-16384, 65536, &rot, &shaft);
  TEST_ASSERT_EQUAL_INT32(-1, rot);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.7124f, shaft);
}

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_parse_bare_command);
  RUN_TEST(test_parse_value_without_axis);
  RUN_TEST(test_parse_axis_prefix);
  RUN_TEST(test_parse_axis_only);
  RUN_TEST(test_parse_rejects_empty);
  RUN_TEST(test_parse_long_command_is_truncated_not_overflowed);
  RUN_TEST(test_fold_small_forward_delta);
  RUN_TEST(test_fold_small_backward_delta);
  RUN_TEST(test_fold_hlim_reset_reads_as_forward);
  RUN_TEST(test_fold_llim_reset_reads_as_backward);
  RUN_TEST(test_angle_from_count_positive);
  RUN_TEST(test_angle_from_count_negative_stays_in_range);
  UNITY_END();
}

void loop() {}
