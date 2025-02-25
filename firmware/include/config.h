// config.h
#ifndef CONFIG_H
#define CONFIG_H

// Pin Definitions
#define MOTOR_ENABLE_PIN     15
#define STEP_PIN            5
#define DIR_PIN             4
#define ENCODER_PIN_A       16
#define ENCODER_PIN_B       17
#define LIMIT_L_PIN         18
#define LIMIT_R_PIN         8

#define MICROSTEPS          1600
#define MAX_SPEED           (2000)
#define MAX_ACCEL          (10000)
#define SAFE_SPEED         1000

// Task Configuration
#define TASK_STACK_SIZE    4096
#define TASK_PRIORITY_COMMUNICATE 3
#define TASK_PRIORITY_MONITOR    4
#define TASK_PRIORITY_ACT        5

// Communication Configuration
#define SERIAL_BUFFER_SIZE  256

// Encoder Configuration
#define ENCODER_STEPS_PER_REV 2400
#define ENCODER_TO_RAD     (2 * PI / ENCODER_STEPS_PER_REV)

// Filter Configuration
#define VELOCITY_FILTER_ALPHA 0.2f

// Network Configuration
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;

#endif // CONFIG_H
