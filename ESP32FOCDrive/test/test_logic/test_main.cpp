#include <Arduino.h>
#include <unity.h>

#include "cmd_parse.h"

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

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_parse_bare_command);
  RUN_TEST(test_parse_value_without_axis);
  RUN_TEST(test_parse_axis_prefix);
  RUN_TEST(test_parse_axis_only);
  RUN_TEST(test_parse_rejects_empty);
  RUN_TEST(test_parse_long_command_is_truncated_not_overflowed);
  UNITY_END();
}

void loop() {}
