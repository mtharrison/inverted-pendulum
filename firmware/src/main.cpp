#include "Arduino.h"
#include <ArduinoJson.h>
#include <AccelStepper.h>
#include <ESP32Encoder.h>
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "limit.h"
#include "config.h"

#define RESET_BIT	    BIT0
#define RESET_CLEAR_BIT	BIT1

#include "USB.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef enum {
    CMD_RESET,
    CMD_MOVE,
    CMD_SENSE,
    CMD_UNKNOWN
} CommandType;

typedef struct {
    int id;
    CommandType command;
    float argument;
    int hasArgument;  // 1 if an argument was parsed, 0 otherwise
} ParsedMessage;

struct PendulumState {
    int current_position = 0;
    float max_speed = 0;
    float speed = 0;
    float theta = 0;
    float velocity = 0;
    float angular_velocity = 0;
    bool limitL = false;
    bool limitR = false;
    bool enabled = true;
    bool resetting = false;
    int extent = 0;
    int target_position;
};

// Global state of the system
PendulumState motorState;

ParsedMessage parseMessage(const char *message) {
    ParsedMessage parsed;
    parsed.id = 0;
    parsed.command = CMD_UNKNOWN;
    parsed.argument = 0.0f;
    parsed.hasArgument = 0;

    // We will tokenize the message, so make a copy to avoid altering the original:
    char buffer[128];
    strncpy(buffer, message, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // Tokenize on '|'
    char *token = strtok(buffer, "|");
    if (!token) {
        return parsed; // No tokens, not a valid message
    }
    // 1) Parse the ID:
    parsed.id = atoi(token);

    // 2) Parse the command:
    token = strtok(NULL, "|");
    if (!token) {
        return parsed; // No command found
    }
    if (strcmp(token, "reset") == 0) {
        parsed.command = CMD_RESET;
    } else if (strcmp(token, "move") == 0) {
        parsed.command = CMD_MOVE;
    } else if (strcmp(token, "sense") == 0) {
        parsed.command = CMD_SENSE;
    } else {
        parsed.command = CMD_UNKNOWN;
    }

    // 3) Optionally parse the argument (if present):
    token = strtok(NULL, "|");
    if (token) {
        parsed.argument = (float)atof(token);
        parsed.hasArgument = 1;
    }

    return parsed;
}

void DEBUG(const char* message, ...) {
    va_list args;
    va_start(args, message);
    char buffer[SERIAL_BUFFER_SIZE];
    vsnprintf(buffer, SERIAL_BUFFER_SIZE, message, args);
    va_end(args);
    USBSerial.print("DEBUG: ");
    USBSerial.println(buffer);
}

EventGroupHandle_t resetEventGroup = xEventGroupCreate();

void sendState(int id) {
    char buffer[200];
    int len = snprintf(buffer, sizeof(buffer),
      "id=%d|current_position=%d|velocity=%f|theta=%f|angular_velocity=%f|limitL=%d|limitR=%d|speed=%f|enabled=%d|resetting=%d|extent=%d\n",
        id, motorState.current_position, motorState.velocity, motorState.theta, motorState.angular_velocity,
        motorState.limitL, motorState.limitR, motorState.speed, motorState.enabled, motorState.resetting, motorState.extent);

    USBSerial.write(buffer, len);
    USBSerial.flush();
}

void communicate(void* parameters) {
    char buffer[SERIAL_BUFFER_SIZE];

    ESP32Encoder encoder;
    encoder.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B);
    LimitSwitch limitL(LIMIT_L_PIN);
    LimitSwitch limitR(LIMIT_R_PIN);

    float filtered_angular_velocity = 0;
    float filtered_velocity = 0;
    int32_t lastPosition = 0;
    int32_t lastTheta = 0;

    int64_t lastTime = esp_timer_get_time();

    for (;;) {
        if (USBSerial.available()) {
            size_t len = USBSerial.readBytesUntil('\n', buffer, SERIAL_BUFFER_SIZE - 1);
            buffer[len] = '\0';

            ParsedMessage pm = parseMessage(buffer);

            if (pm.command == CMD_SENSE) {
                sendState(pm.id);
            }
            else if (pm.command == CMD_MOVE) {
                float speed = pm.argument;
                if (motorState.enabled && !motorState.resetting) {
                    motorState.max_speed = speed * MAX_SPEED;
                }
                sendState(pm.id);
            }
            else if (pm.command == CMD_RESET) {
                motorState.resetting = true;
                xEventGroupSetBits(resetEventGroup, RESET_BIT);
                sendState(pm.id);
            }
            
        }
        else {
            bool limitStateL = limitL.triggered();
            bool limitStateR = limitR.triggered();

            motorState.limitL = limitStateL;
            motorState.limitR = limitStateR;

            // Update position and velocities
            int64_t now = esp_timer_get_time();
            float dt = (now - lastTime) / 1e6;
            lastTime = now;

            int32_t encoderCount = encoder.getCount();
            float newTheta = (encoderCount * ENCODER_TO_RAD) + PI;

            long currentPosition = motorState.current_position;


            if (dt > 0) {
                float angular_velocity = (newTheta - motorState.theta) / dt;
                filtered_angular_velocity = VELOCITY_FILTER_ALPHA * angular_velocity +
                    (1 - VELOCITY_FILTER_ALPHA) * filtered_angular_velocity;

                float velocity = (currentPosition - lastPosition) / dt;
                filtered_velocity = VELOCITY_FILTER_ALPHA * velocity +
                    (1 - VELOCITY_FILTER_ALPHA) * filtered_velocity;
            }

            lastTheta = newTheta;
            lastPosition = currentPosition;

            motorState.theta = newTheta;
            motorState.angular_velocity = filtered_angular_velocity;
            motorState.velocity = filtered_velocity;
        }
        // yield for micros

    }
}

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
            stepper.setMaxSpeed(MAX_SPEED);
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            motorState.enabled = false;
        }

        if (xEventGroupGetBits(resetEventGroup) & RESET_BIT) {
            motorState.enabled = false;
            stepper.setMaxSpeed(MAX_SPEED);
            stepper.setSpeed(SAFE_SPEED);
            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            while (!limitStateR) {
                stepper.runSpeed();
                limitStateR = motorState.limitR;
                // vTaskDelay(pdMS_TO_TICKS(0.01));
            }

            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            long rightPosition = stepper.currentPosition();
            stepper.setCurrentPosition(rightPosition);
            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            stepper.setSpeed(-SAFE_SPEED);
            while (!limitStateL) {
                stepper.runSpeed();
                limitStateL = motorState.limitL;
                // vTaskDelay(pdMS_TO_TICKS(0.01));
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

        ets_delay_us(5);
    }
}

void setup() {
    // USB.begin();
    USBSerial.begin(115200);

    disableCore0WDT();
    disableCore1WDT();

    xTaskCreatePinnedToCore(communicate, "Communicate", TASK_STACK_SIZE * 2, NULL,
        TASK_PRIORITY_COMMUNICATE, NULL, 1);
    xTaskCreatePinnedToCore(act, "Act", TASK_STACK_SIZE * 2, NULL,
        TASK_PRIORITY_ACT, NULL, 0);

    
}

void loop() {
    vTaskDelete(NULL);
}
