#include <Arduino.h>
#include <unity.h>

#include "self_test.h"

void setUp() {}
void tearDown() {}

static void unityReport(const char *name, bool ok, void *) {
  TEST_ASSERT_TRUE_MESSAGE(ok, name);
}

void test_self_test_suite() {
  const SelfTestResult r = selfTestRun(unityReport, nullptr);
  TEST_ASSERT_GREATER_THAN_INT(0, r.run);
  TEST_ASSERT_EQUAL_INT(0, r.failed);
}

void setup() {
  delay(2000);
  UNITY_BEGIN();
  RUN_TEST(test_self_test_suite);
  UNITY_END();
}

void loop() {}
