#include "Arduino.h"
#include <AccelStepper.h>
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "limit.h"

// Pin Definitions
#define MOTOR_ENABLE_PIN     18
#define STEP_PIN            16
#define DIR_PIN             7
#define ENCODER_PIN_A       13
#define ENCODER_PIN_B       14
#define LIMIT_L_PIN         12
#define LIMIT_R_PIN         10

// Motor Configuration
#define MICROSTEPS          1600
#define MAX_SPEED           (2000)
#define MAX_ACCEL          (10000)
#define SAFE_SPEED         3000

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
#define VELOCITY_FILTER_ALPHA 0.1f

// Communication Configuration
#define SERIAL_BUFFER_SIZE  256

void DEBUG(const char* message, ...) {
    va_list args;
    va_start(args, message);
    char buffer[SERIAL_BUFFER_SIZE];
    vsnprintf(buffer, SERIAL_BUFFER_SIZE, message, args);
    va_end(args);
    Serial.print("DEBUG: ");
    Serial.println(buffer);
}

struct PendulumState {
    int32_t current_position = 0;
    bool limitL = false;
    bool limitR = false;
    bool enabled = true;
    bool resetting = false;
    int extent = 0;
};

// Shared data protection
SemaphoreHandle_t dataMutex = xSemaphoreCreateMutex();

// Global state of the system
PendulumState motorState;

void monitor(void* parameters) {
    LimitSwitch limitL(LIMIT_L_PIN);
    LimitSwitch limitR(LIMIT_R_PIN);

    for (;;) {
        bool limitStateL = limitL.triggered();
        bool limitStateR = limitR.triggered();

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        motorState.limitL = limitStateL;
        motorState.limitR = limitStateR;
        xSemaphoreGive(dataMutex);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void act(void* parameters) {
    AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(MAX_ACCEL);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);

    for (;;) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        bool limitStateL = motorState.limitL;
        bool limitStateR = motorState.limitR;
        xSemaphoreGive(dataMutex);

        if (limitStateL || limitStateR) {
            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.enabled = false;
            xSemaphoreGive(dataMutex);

            // RESET PROCEDURE

            stepper.setSpeed(SAFE_SPEED);
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            stepper.moveTo(100000);
            while (!limitStateR) {
                stepper.run();
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                limitStateR = motorState.limitR;
                xSemaphoreGive(dataMutex);
                vTaskDelay(pdMS_TO_TICKS(0.1));
            }

            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            long rightPosition = stepper.currentPosition();
            stepper.setCurrentPosition(rightPosition);
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            stepper.moveTo(-100000);
            while (!limitStateL) {
                stepper.run();
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                limitStateL = motorState.limitL;
                xSemaphoreGive(dataMutex);
                vTaskDelay(pdMS_TO_TICKS(0.1));
            }

            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            long leftPosition = stepper.currentPosition();
            stepper.setCurrentPosition(leftPosition);


            long center = (rightPosition + leftPosition) / 2;
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            stepper.moveTo(center);
            stepper.runToPosition();
            stepper.setCurrentPosition(0);

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.enabled = true;
            motorState.resetting = false;
            motorState.extent = abs(center - leftPosition);
            DEBUG("Extent: %d", motorState.extent);
            xSemaphoreGive(dataMutex);
            stepper.setSpeed(0);
            continue;
        }

        // if (enabled) {
        //     stepper.setSpeed(speed);
        //     stepper.run();

        //     xSemaphoreTake(dataMutex, portMAX_DELAY);
        //     motorState.current_position = stepper.currentPosition();
        //     xSemaphoreGive(dataMutex);
        // }

        vTaskDelay(pdMS_TO_TICKS(0.1));
    }
}

void setup() {
    Serial.begin(115200);

    // xTaskCreatePinnedToCore(communicate, "Communicate", TASK_STACK_SIZE, NULL,
    //     TASK_PRIORITY_COMMUNICATE, NULL, 1);
    xTaskCreatePinnedToCore(monitor, "Monitor", TASK_STACK_SIZE, NULL,
        TASK_PRIORITY_MONITOR, NULL, 1);
    xTaskCreatePinnedToCore(act, "Act", TASK_STACK_SIZE, NULL,
        TASK_PRIORITY_ACT, NULL, 0);

    disableCore0WDT();
}

void loop() {
    vTaskDelete(NULL);
}
