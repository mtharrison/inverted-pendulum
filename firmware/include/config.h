// config.h
#ifndef CONFIG_H
#define CONFIG_H

#define RESET_BIT	    BIT0
#define RESET_CLEAR_BIT	BIT1

// Pin Definitions
#define MOTOR_ENABLE_PIN    15
#define STEP_PIN            5
#define DIR_PIN             4
#define ENCODER_PIN_A       18
#define ENCODER_PIN_B       8
#define LIMIT_L_PIN         16
#define LIMIT_R_PIN         17

#define MICROSTEPS          1600
#define MAX_SPEED           (20000)
#define MAX_ACCEL           (50000)
#define SAFE_SPEED          4000

// Task Configuration
#define TASK_STACK_SIZE             4096
#define TASK_PRIORITY_COMMUNICATE   3
#define TASK_PRIORITY_MONITOR       4
#define TASK_PRIORITY_ACT           5

// Communication Configuration
#define SERIAL_BUFFER_SIZE  256

// Encoder Configuration
#define ENCODER_STEPS_PER_REV 2400
#define ENCODER_TO_RAD     (2 * PI / ENCODER_STEPS_PER_REV)

// Filter Configuration
#define VELOCITY_FILTER_ALPHA 0.2f

// Smoothing Configuration
#define SPEED_SMOOTHING_ENABLED true    // Can be set to false to disable smoothing
#define SPEED_SMOOTHING_RATE 0.2f       // Higher = faster response (0.0-1.0)
#define SPEED_UPDATE_PERIOD_MS 5        // How often speed is updated (ms)

// Network Configuration
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;

#endif // CONFIG_H
