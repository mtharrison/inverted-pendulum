#include "Arduino.h"
#include "ArduinoJson.h"
#include <ESP32Encoder.h>
#include "freeRTOS/task.h"
#include "string.h"
#include "limit.h"

#define SERIAL_BUFFER_SIZE 100

#define REQ_TYPE_OBSERVE 0

int x = 0;
double speed = 0;
double theta = 0;
double angular_velocity = 0;
int terminal_limitL = 0;
int terminal_limitR = 0;

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
        response.add(terminal_limitL);
        response.add(terminal_limitR);
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

  LimitSwitch limitL = LimitSwitch(1, 12);
  LimitSwitch limitR = LimitSwitch(2, 10);

  int64_t time_us = esp_timer_get_time();
  float filtered_velocity = 0.0; // For low-pass filter
  const float alpha = 0.1; // Smoothing factor (adjust as needed)

  for (;;) {
    terminal_limitL = limitL.triggered();
    terminal_limitR = limitR.triggered();

    int64_t prevTime_us = time_us;
    float prev_theta = theta;

    int encoderRaw = encoder.getCount();
    time_us = esp_timer_get_time();
    int64_t dt = time_us - prevTime_us;

    theta = ((float)encoderRaw / 2400.0) * 2.0 * PI;
    
    // Convert dt to seconds and calculate velocity
    if (dt > 0) { // Avoid division by zero
      float angular_velocity = (theta - prev_theta) / ((float)dt / 1e6);
      // Apply low-pass filter
      filtered_velocity = alpha * angular_velocity + (1 - alpha) * filtered_velocity;
    }

    // Use 'filtered_velocity' for smooth output
    angular_velocity = filtered_velocity;

    vTaskDelay(pdMS_TO_TICKS(1)); // Sample every 10ms for better velocity averaging
  }
}

// // Task: Monitor the system and update state
// void monitor(void* parameters) {
//   ESP32Encoder encoder;
//   encoder.attachFullQuad(13, 14);

//   int64_t time_us = esp_timer_get_time();

//   for (;;) {
//     int64_t prevTime_us = time_us;
//     float prev_theta = theta;

//     int encoderRaw = encoder.getCount();
//     time_us = esp_timer_get_time();
//     int64_t dt = time_us - prevTime_us;

//     theta = ((float)encoderRaw / 2400.0) * 2.0 * PI;
    
//     // Convert dt to seconds and calculate velocity
//     if (dt > 0) { // Avoid division by zero
//       angular_velocity = (theta - prev_theta) / ((float)dt / 1e6);
//     }

//     vTaskDelay(pdMS_TO_TICKS(10)); // Sample every 10ms for better velocity averaging
//   }
// }

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
