#include "Arduino.h"
#include "ArduinoJson.h"
#include <ESP32Encoder.h>
#include "freeRTOS/task.h"
#include "string.h"

#define PI 3.14159265358979323846
#define SERIAL_BUFFER_SIZE 100

#define REQ_TYPE_OBSERVE 0

int x = 0;
float speed = 0;
int theta = 0;
float angular_velocity = 0;
int terminal = 0;

// Task: Respond to requests over serial
void communicate(void* parameters) {
  for (;;) {
    if (Serial.available()) {
      JsonDocument request;
      char buffer[SERIAL_BUFFER_SIZE];
      Serial.readBytesUntil(']', buffer, SERIAL_BUFFER_SIZE);
      deserializeJson(request, buffer);

      int id = request[0];
      int reqType = request[1];

      JsonDocument response;
      response.add(id);
      if (reqType == REQ_TYPE_OBSERVE) {
        response.add(STATUS::OK);
        response.add(x);
        response.add(speed);
        response.add(theta);
        response.add(angular_velocity);
        response.add(terminal);
      }
      else {
        response.add(STATUS::FAIL);
        response.add(String("Unknown request type"));
      }

      serializeJson(response, Serial);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
// Task: Monitor the system and update state
void monitor(void* parameters) {
  ESP32Encoder encoder;
  encoder.attachFullQuad(13, 14);

  int64_t time_us = esp_timer_get_time();

  for (;;) {
    int64_t prevTime_us = time_us;
    int prev_theta = theta;

    int encoderRaw = encoder.getCount();
    time_us = esp_timer_get_time();
    int64_t dt = time_us - prevTime_us;
    theta = (encoderRaw / 2400) * 2 * PI;
    if (dt > 0) {
      angular_velocity = (theta - prev_theta) / dt;
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// Task: Act on the system based on actions
// kill the task if the terminal state is reached
void act(void* parameters) {
  for (;;) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
    communicate,    // Function that should be called
    "Communicate",  // Name of the task (for debugging)
    2056,           // Stack size (bytes)
    NULL,           // Parameter to pass
    3,              // Task priority
    NULL,            // Task handle
    1
  );

  xTaskCreatePinnedToCore(
    monitor,        // Function that should be called
    "Monitor",      // Name of the task (for debugging)
    5000,           // Stack size (bytes)
    NULL,           // Parameter to pass
    3,              // Task priority
    NULL, 1            // Task handle
  );

  xTaskCreatePinnedToCore(
    act,            // Function that should be called
    "act",          // Name of the task (for debugging)
    1000,           // Stack size (bytes)
    NULL,           // Parameter to pass
    3,              // Task priority
    NULL, 1            // Task handle
  );
}

void loop() {
  vTaskDelete(NULL);
}
