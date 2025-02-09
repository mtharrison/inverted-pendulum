#include <Arduino.h>
#include <unity.h>
#include "limit.h"

void setUp(void) {}

void tearDown(void) {}

void test_default_limit_not_triggered(void)
{
    LimitSwitch limit(13);
    TEST_ASSERT_FALSE(limit.triggered());
}

void setup()
{
  UNITY_BEGIN(); // IMPORTANT LINE!
  RUN_TEST(test_default_limit_not_triggered);
  UNITY_END(); // stop unit testing
}

void loop() {}