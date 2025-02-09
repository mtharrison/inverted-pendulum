#include "main.h"

// Define globals exactly once
PendulumState motorState;
SemaphoreHandle_t dataMutex = xSemaphoreCreateMutex();