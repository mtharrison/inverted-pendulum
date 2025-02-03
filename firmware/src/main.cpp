#include "Arduino.h"
#include "freeRTOS/task.h"

int x = 0;
int dx_dt = 0;
int theta = 0;
int dtheta_dt = 0;
int terminal = 0;
int desired_x = 0;

// Task: Communicate via serial
void communicate(void* parameters) {
  for (;;) { // infinite loop

  }
}

// Task: Monitor the system and update states (x, dx_dt, theta, dtheta_dt, terminal)
void monitor(void* parameters) {
  for (;;) { // infinite loop

  }
}

// Task: Act on the system based on actions
// kill the task if the terminal state is reached
void act(void* parameters) {
  for (;;) { // infinite loop

  }
}

void setup() {
  xTaskCreate(
    communicate,    // Function that should be called
    "Communicate",  // Name of the task (for debugging)
    1000,           // Stack size (bytes)
    NULL,           // Parameter to pass
    1,              // Task priority
    NULL            // Task handle
  );

  xTaskCreate(
    monitor,        // Function that should be called
    "Monitor",      // Name of the task (for debugging)
    1000,           // Stack size (bytes)
    NULL,           // Parameter to pass
    1,              // Task priority
    NULL            // Task handle
  );

  xTaskCreate(
    act,            // Function that should be called
    "act",          // Name of the task (for debugging)
    1000,           // Stack size (bytes)
    NULL,           // Parameter to pass
    1,              // Task priority
    NULL            // Task handle
  );
}

void loop() {}
