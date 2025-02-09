#include <Arduino.h>
#include <unity.h>
#include "limit.h"

#define DEBOUNCE_DELAY 10

// Mock Arduino pin functions
uint8_t mockPinState = HIGH;
void digitalWrite(uint8_t pin, uint8_t val) { mockPinState = val; }
int digitalRead(uint8_t pin) { return mockPinState; }
void pinMode(uint8_t pin, uint8_t mode) {} // No-op for testing

// Test variables
LimitSwitch* limitSwitch;
const uint8_t TEST_PIN = 13;

void setUp(void) {
    // Reset pin state before each test
    mockPinState = HIGH;
    limitSwitch = new LimitSwitch(TEST_PIN);
}

void tearDown(void) {
    delete limitSwitch;
}

void test_default_limit_not_triggered(void) {
    TEST_ASSERT_FALSE(limitSwitch->triggered());
}

void test_triggered_when_pin_low(void) {
    mockPinState = LOW;
    TEST_ASSERT_TRUE(limitSwitch->triggered());
}

void test_release_after_activation(void) {
    // Activate first
    mockPinState = LOW;
    delay(DEBOUNCE_DELAY + 1);
    TEST_ASSERT_TRUE(limitSwitch->triggered());
    
    // Release
    mockPinState = HIGH;
    delay(DEBOUNCE_DELAY + 1);
    TEST_ASSERT_FALSE(limitSwitch->triggered());
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_default_limit_not_triggered);
    RUN_TEST(test_triggered_when_pin_low);
    RUN_TEST(test_release_after_activation);
    UNITY_END();
}

void loop() {}