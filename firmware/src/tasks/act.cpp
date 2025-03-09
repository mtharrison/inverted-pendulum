#include <AccelStepper.h>
#include "config.h"
#include "task.h"

void act(void* parameters) {
    AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(MAX_ACCEL);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);

    for (;;) {
        bool enabled = motorState.enabled;
        float max_speed = motorState.max_speed;
        bool limitStateL = motorState.limitL;
        bool limitStateR = motorState.limitR;

        if (limitStateL || limitStateR) {
            stepper.stop();
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            motorState.enabled = false;
        }

        if (xEventGroupGetBits(resetEventGroup) & RESET_BIT) {
            motorState.enabled = false;
            stepper.setSpeed(SAFE_SPEED);
            digitalWrite(MOTOR_ENABLE_PIN, LOW);

            if (limitStateL) {
                stepper.move(100);
                stepper.runToPosition();
            }

            if (limitStateR) {
                stepper.move(-100);
                stepper.runToPosition();
            }
            
            while (!limitStateR) {
                stepper.runSpeed();
                limitStateR = motorState.limitR;
            }

            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            long rightPosition = stepper.currentPosition();
            stepper.setCurrentPosition(rightPosition);
            
            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            stepper.setSpeed(-SAFE_SPEED);
            while (!limitStateL) {
                stepper.runSpeed();
                limitStateL = motorState.limitL;
            }

            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            long leftPosition = stepper.currentPosition();
            stepper.setCurrentPosition(leftPosition);

            long center = (rightPosition + leftPosition) / 2;
            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            stepper.moveTo(center);
            stepper.runToPosition();
            stepper.setCurrentPosition(0);

            motorState.current_position = 0;
            motorState.speed = 0;
            motorState.enabled = true;
            motorState.resetting = false;
            motorState.extent = abs(center - leftPosition);
            motorState.max_speed = 0;
            xEventGroupClearBits(resetEventGroup, RESET_BIT);
            stepper.setSpeed(0);
            stepper.stop();
            continue;
        }

        if (enabled) {
            stepper.setMaxSpeed(max_speed);
            if (max_speed > 0) {
                stepper.moveTo(9999);
            }
            else {
                stepper.moveTo(-9999);
            }
            stepper.run();
            motorState.current_position = (int)stepper.currentPosition();
        }

        motorState.speed = stepper.speed();
        motorState.target_position = stepper.targetPosition();

        ets_delay_us(1);
    }
}