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
    int32_t current_position = 0;
    int32_t target_position = 0;
    double theta = 0;
    double velocity = 0;
    double angular_velocity = 0;
    bool limitL = false;
    bool limitR = false;
    bool enabled = true;
    bool resetting = false;
    long extent = 0;
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

void handle_move_command(int action) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
        if (motorState.enabled) {
            motorState.target_position += action;
        }
        xSemaphoreGive(dataMutex);
    }
}

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
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                response["current_position"] = motorState.current_position;
                response["velocity"] = motorState.velocity;
                response["theta"] = motorState.theta;
                response["angular_velocity"] = motorState.angular_velocity;
                response["limitL"] = motorState.limitL;
                response["limitR"] = motorState.limitR;
                response["target_position"] = motorState.target_position;
                response["enabled"] = motorState.enabled;
                response["resetting"] = motorState.resetting;
                response["extent"] = motorState.extent;
                xSemaphoreGive(dataMutex);
            }
            else if (command == "move") {
                long distance = request["params"]["distance"].as<long>();

                xSemaphoreTake(dataMutex, portMAX_DELAY);
                if (motorState.enabled) {
                    motorState.target_position += distance;
                }
                response["target_position"] = motorState.target_position; 
                xSemaphoreGive(dataMutex);
            }
            else if (command == "reset") {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                motorState.resetting = true;
                xSemaphoreGive(dataMutex);
                xEventGroupSetBits(resetEventGroup, RESET_BIT);
            }
            else {
                response["status"] = "ERROR";
                response["message"] = "Invalid request type";
            }

            serializeJson(response, Serial);
            Serial.println();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
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

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        motorState.limitL = limitStateL;
        motorState.limitR = limitStateR;
        xSemaphoreGive(dataMutex);

        // Update position and velocities
        int64_t now = esp_timer_get_time();
        float dt = (now - lastTime) / 1e6;
        lastTime = now;

        int32_t encoderCount = encoder.getCount();
        float newTheta = encoderCount * ENCODER_TO_RAD;

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

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        motorState.theta = newTheta;
        motorState.angular_velocity = filtered_angular_velocity;
        motorState.velocity = filtered_velocity;
        xSemaphoreGive(dataMutex);

        vTaskDelay(pdMS_TO_TICKS(10));
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
        bool enabled = motorState.enabled;
        int32_t target = motorState.target_position;
        bool limitStateL = motorState.limitL;
        bool limitStateR = motorState.limitR;
        xSemaphoreGive(dataMutex);

        if (limitStateL || limitStateR) {
            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.enabled = false;
            xSemaphoreGive(dataMutex);
        }

        if (xEventGroupGetBits(resetEventGroup) & RESET_BIT) {
            stepper.setMaxSpeed(SAFE_SPEED);
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            stepper.moveTo(100000);
            while (!limitStateR) {
                stepper.run();
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                limitStateR = motorState.limitR;
                xSemaphoreGive(dataMutex);
                vTaskDelay(pdMS_TO_TICKS(1));
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
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            digitalWrite(MOTOR_ENABLE_PIN, LOW);
            long leftPosition = stepper.currentPosition();
            stepper.setCurrentPosition(leftPosition);

            stepper.setMaxSpeed(MAX_SPEED);
            long center = (rightPosition + leftPosition) / 2;
            digitalWrite(MOTOR_ENABLE_PIN, HIGH);
            stepper.moveTo(center);
            stepper.runToPosition();
            stepper.setCurrentPosition(0);

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.current_position = 0;
            motorState.target_position = 0;
            motorState.enabled = true;
            motorState.resetting = false;
            motorState.extent = abs(center - leftPosition);
            xSemaphoreGive(dataMutex);
            xEventGroupClearBits(resetEventGroup, RESET_BIT);
            continue;
        }

        if (enabled) {
            stepper.moveTo(target);
            stepper.run();

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.current_position = stepper.currentPosition();
            xSemaphoreGive(dataMutex);
        }

        // vTaskDelay(pdMS_TO_TICKS(0.1));
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
