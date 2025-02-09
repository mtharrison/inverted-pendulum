#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

struct PendulumState {
    int32_t target_position = 0;
    bool enabled = true;
};

// Declare globals without defining them
extern PendulumState motorState;
extern SemaphoreHandle_t dataMutex;

// Function declarations
void communicate(void* parameters);
void handle_move_command(int action);