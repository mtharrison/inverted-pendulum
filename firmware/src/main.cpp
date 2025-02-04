#include "Arduino.h"
#include "ArduinoJson.h"
#include <ESP32Encoder.h>
#include "freeRTOS/task.h"
#include "string.h"

int x = 0;
int dx_dt = 0;
int theta = 0;
int dtheta_dt = 0;
int terminal = 0;
int desired_x = 0;

// Task: Respond to requests over serial
void communicate(void* parameters) {
  for (;;) { 
    if (Serial.available()) {
      JsonDocument request;
      char buffer[100];
      Serial.readBytesUntil(']', buffer, 100);
      deserializeJson(request, buffer);
      
      int id = request[0];
      const char* reqType = request[1];

      JsonDocument response;
      response.add(id);
      if (strcmp(reqType, "observe") == 0) {
        response.add(1);           // STATUS 
        response.add(x);           // X
        response.add(dx_dt);       // dX
        response.add(theta);       // theta
        response.add(dtheta_dt);   // dtheta
        response.add(terminal);    // terminal
      }
      else {
        response.add(0);
        response.add(String("Unknown request type"));
      }

      serializeJson(response, Serial);
    }
    
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// Task: Monitor the system and update states (x, dx_dt, theta, dtheta_dt, terminal)
void monitor(void* parameters) {
  ESP32Encoder encoder;
  encoder.attachFullQuad(13, 14);

  for (;;) { 
    int count = encoder.getCount() % 2400;
    theta = count;
    vTaskDelay(3 / portTICK_PERIOD_MS);
  }
}

// Task: Act on the system based on actions
// kill the task if the terminal state is reached
void act(void* parameters) {
  for (;;) { 
    vTaskDelay(500 / portTICK_PERIOD_MS);
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
