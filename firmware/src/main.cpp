#include "Arduino.h"
#include <ArduinoJson.h>
#include <AccelStepper.h>
#include <ESP32Encoder.h>
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "limit.h"

// Configuration
#define SERIAL_BUFFER_SIZE 256
#define MICROSTEPS 1600
#define MOTOR_ENABLE 18
#define STEP_PIN 16
#define DIR_PIN 7
#define ENCODER_PIN_A 13
#define ENCODER_PIN_B 14
#define LIMIT_L_PIN 12
#define LIMIT_R_PIN 10

// Shared data protection
SemaphoreHandle_t dataMutex = xSemaphoreCreateMutex();

// Motor control structure
struct PendulumState {
    int32_t current_position = 0;
    int32_t target_position = 0;
    double theta = 0;
    double angular_velocity = 0;
    bool limitL = false;
    bool limitR = false;
    bool enabled = true;
};

enum class RequestType : uint8_t {
    Observe = 0, 
    Step = 1
};

PendulumState motorState;

void communicate(void* parameters) {
    JsonDocument request;
    char buffer[SERIAL_BUFFER_SIZE];

    for (;;) {
        if (Serial.available()) {
            size_t len = Serial.readBytesUntil('\n', buffer, SERIAL_BUFFER_SIZE-1);
            buffer[len] = '\0';


            DeserializationError error = deserializeJson(request, buffer);
            if (error) {
                Serial.print("{\"error\":\"");
                Serial.print(error.c_str());
                Serial.println("\"}");
                continue;
            }

            JsonDocument response;
            int id = request[0];
            auto reqType = static_cast<RequestType>(request["type"].as<uint8_t>());

            response["id"] = id;
            
            if (reqType == RequestType::Observe) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                response["status"] = "OK";
                response["position"] = motorState.current_position;
                response["theta"] = motorState.theta;
                response["angular_velocity"] = motorState.angular_velocity;
                response["limitL"] = motorState.limitL;
                response["limitR"] = motorState.limitR;
                response["target"] = motorState.target_position;
                response["enabled"] = motorState.enabled;
                xSemaphoreGive(dataMutex);
            }
            else if (reqType == RequestType::Step) {
                int step = request[2];
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                if (motorState.enabled) {
                    motorState.target_position += step;
                }
                response["status"] = motorState.enabled ? "OK" : "DISABLED";
                response["position"] = motorState.current_position;
                response["theta"] = motorState.theta;
                response["angular_velocity"] = motorState.angular_velocity;
                response["limitL"] = motorState.limitL;
                response["limitR"] = motorState.limitR;
                response["target"] = motorState.target_position;
                response["enabled"] = motorState.enabled;
                xSemaphoreGive(dataMutex);
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

    const float alpha = 0.1;
    float filtered_velocity = 0;
    int64_t lastTime = esp_timer_get_time();

    for (;;) {
        bool limitStateL = limitL.triggered();
        bool limitStateR = limitR.triggered();

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        motorState.limitL = limitStateL;
        motorState.limitR = limitStateR;
        
        if (limitStateL || limitStateR) {
            motorState.enabled = false;
            digitalWrite(MOTOR_ENABLE, LOW); // Active LOW enable
        }
        xSemaphoreGive(dataMutex);

        // Update position and velocity
        int64_t now = esp_timer_get_time();
        float dt = (now - lastTime) / 1e6;
        lastTime = now;

        int32_t encoderCount = encoder.getCount();
        float newTheta = (encoderCount / 2400.0) * 2 * PI;
        
        if (dt > 0) {
            float velocity = (newTheta - motorState.theta) / dt;
            filtered_velocity = alpha * velocity + (1 - alpha) * filtered_velocity;
        }

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        motorState.theta = newTheta;
        motorState.angular_velocity = filtered_velocity;
        xSemaphoreGive(dataMutex);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void act(void* parameters) {
    AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    stepper.setMaxSpeed(100000);
    stepper.setAcceleration(10000);
    pinMode(MOTOR_ENABLE, OUTPUT);

    for (;;) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        bool enabled = motorState.enabled;
        int32_t target = motorState.target_position;
        xSemaphoreGive(dataMutex);

        digitalWrite(MOTOR_ENABLE, enabled ? LOW : HIGH);
        
        if (enabled) {
            stepper.moveTo(target);
            stepper.run();
            
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.current_position = stepper.currentPosition();
            xSemaphoreGive(dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    Serial.begin(115200);
    xTaskCreatePinnedToCore(communicate, "Communicate", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(monitor, "Monitor", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(act, "Act", 4096, NULL, 3, NULL, 1);
}

void loop() { vTaskDelete(NULL); }