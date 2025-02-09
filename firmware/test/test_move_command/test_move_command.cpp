#include <unity.h>

// Test-only minimal implementation
struct PendulumState {
    int32_t target_position = 0;
    bool enabled = true;
};

// Mock FreeRTOS functions
extern "C" {
    typedef void* SemaphoreHandle_t;
    #define pdTRUE 1
    #define portMAX_DELAY 0
    
    SemaphoreHandle_t dataMutex;
    
    int xSemaphoreTake(SemaphoreHandle_t, unsigned int) {
        return pdTRUE; // Assume mutex always acquired
    }
    
    int xSemaphoreGive(SemaphoreHandle_t) {
        return pdTRUE; // Assume mutex always released
    }
}

PendulumState motorState;

void handle_move_command(int action) {
    if(xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        if(motorState.enabled) {
            motorState.target_position += action;
        }
        xSemaphoreGive(dataMutex);
    }
}

void setUp() {
    motorState = PendulumState();
}

void test_mutex_protected_move() {
    handle_move_command(300);
    TEST_ASSERT_EQUAL(300, motorState.target_position);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_mutex_protected_move);
    UNITY_END();
}

void loop() {}