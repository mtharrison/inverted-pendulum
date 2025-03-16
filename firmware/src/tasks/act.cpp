#include "config.h"
#include "Arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "USB.h"
#include "task.h"
#include "RLStepper.h"

enum ActionState {
    RESET_IDLE,
    RESET_TO_LEFT,
    RESET_TO_RIGHT,
    RESET_TO_CENTER,
    RESET_COMPLETE,
    RUN_MOTOR
};

static ActionState action_state = RESET_IDLE;

void handle_reset(PendulumState& motorState, StepperRMT* stepper) {
    stepper->setSpeed(-0.2);
    USBSerial.println("Running to left...");
    while (!motorState.limitL) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }   

    int left_extent = stepper->getPosition();
    USBSerial.println("Left extent: " + String(left_extent));
    stepper->setSpeed(0.2);
    USBSerial.println("Running to right...");
    while (!motorState.limitR) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    int right_extent = stepper->getPosition();
    USBSerial.println("Right extent: " + String(right_extent));
    int center = (left_extent + right_extent) / 2;

    stepper->setSpeed(-0.2);
    USBSerial.println("Running to center...");
    while (stepper->getPosition() > center) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    USBSerial.println("Stopped...");

    stepper->setSpeed(0.0f);
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    delay(500);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

    xEventGroupSetBits(resetEventGroup, RESET_CLEAR_BIT);
    motorState.resetting = false;
    motorState.enabled = true;
    motorState.speed = 0.0f;
    motorState.current_position = 0;
    motorState.extent = abs((right_extent - left_extent) / 2);
    USBSerial.println("Extent: " + String(motorState.extent));
    stepper->resetPosition();
}

void act(void* parameters) {
    StepperRMT stepper(
        STEP_PIN,
        DIR_PIN,
        RMT_CHANNEL_0,
        MAX_SPEED,
        MAX_ACCEL
    );

    stepper.begin();
    stepper.setSpeed(0.0f);

    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

    USBSerial.println("Act task started...");

    while (true) {

        if (motorState.limitL || motorState.limitR) {
            motorState.enabled = false;
        }

        if (motorState.resetting) {
            USBSerial.println("Resetting...");
            handle_reset(motorState, &stepper);
        }
        else {
            if (motorState.enabled) {
                //motorState.velocity = stepper.getSpeed();
                motorState.current_position = stepper.getPosition();
                stepper.setSpeed(motorState.speed);
            }
            else {
                stepper.setSpeed(0.0f);
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
