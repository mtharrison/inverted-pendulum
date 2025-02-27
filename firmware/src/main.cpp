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

// Shared data protection
SemaphoreHandle_t dataMutex = xSemaphoreCreateMutex();

// Global state of the system
PendulumState motorState;

void DEBUG(const char* message, ...) {
    va_list args;
    va_start(args, message);
    char buffer[SERIAL_BUFFER_SIZE];
    vsnprintf(buffer, SERIAL_BUFFER_SIZE, message, args);
    va_end(args);
    Serial.print("DEBUG: ");
    Serial.println(buffer);
}

EventGroupHandle_t resetEventGroup = xEventGroupCreate();

void communicate(void* parameters) {
    JsonDocument request;
    char buffer[SERIAL_BUFFER_SIZE];

    for (;;) {
        if (Serial.available()) {
            size_t len = Serial.readBytesUntil('\n', buffer, SERIAL_BUFFER_SIZE - 1);
            buffer[len] = '\0';

            DeserializationError error = deserializeJson(request, buffer);
            if (error) {
                DEBUG("Failed to parse JSON: %s", error.c_str());
                continue;
            }

            int id = request["id"];
            String command = String((const char*)request["command"]);
            JsonDocument response;
            response["id"] = id;
            response["status"] = "OK";

            if (command == "sense") {
                response["current_position"] = motorState.current_position;
                response["velocity"] = motorState.velocity;
                response["theta"] = motorState.theta;
                response["angular_velocity"] = motorState.angular_velocity;
                response["limitL"] = motorState.limitL;
                response["limitR"] = motorState.limitR;
                response["speed"] = motorState.speed;
                response["enabled"] = motorState.enabled;
                response["resetting"] = motorState.resetting;
                response["extent"] = motorState.extent;
            }
            else if (command == "move") {
                float speed = request["params"]["speed"].as<float>();

                if (motorState.enabled && !motorState.resetting) {
                    motorState.max_speed = speed * MAX_SPEED;
                }
                response["speed"] = motorState.speed;
                response["target_position"] = motorState.target_position;
            }
            else if (command == "reset") {
                motorState.resetting = true;
                xEventGroupSetBits(resetEventGroup, RESET_BIT);
            }
            else {
                response["status"] = "ERROR";
                response["message"] = "Invalid request type";
            }

            serializeJson(response, Serial);
            Serial.println();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void monitor(void* parameters) {
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

        vTaskDelay(pdMS_TO_TICKS(1));
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
    Serial.begin(115200);

    xTaskCreatePinnedToCore(communicate, "Communicate", TASK_STACK_SIZE, NULL,
        TASK_PRIORITY_COMMUNICATE, NULL, 1);
    xTaskCreatePinnedToCore(monitor, "Monitor", TASK_STACK_SIZE, NULL,
        TASK_PRIORITY_MONITOR, NULL, 1);
    xTaskCreatePinnedToCore(act, "Act", TASK_STACK_SIZE, NULL,
        TASK_PRIORITY_ACT, NULL, 0);

    disableCore0WDT();
}

void loop() {
    vTaskDelete(NULL);
}
