#pragma once
#include <cstring>

// Simple Serial mock
class MockSerial {
public:
    static char buffer[256];
    static size_t readBytesUntil(char, char* dest, size_t len) {
        strncpy(dest, buffer, len);
        return strlen(buffer);
    }
    static void reset() { memset(buffer, 0, 256); }
};
char MockSerial::buffer[256] = {0};

// Minimal FreeRTOS mock
SemaphoreHandle_t xSemaphoreCreateMutex() { return nullptr; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }

// Test version of Serial
#define Serial MockSerial