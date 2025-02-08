#include "Arduino.h"
#include <ArduinoJson.h>
#include <AccelStepper.h>
#include <ESP32Encoder.h>
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "limit.h"

const char* ssid = "SKYHI4C2-2.4ghz";
const char* password = "1gxFtvUHC6xi";

// OTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

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
    bool needs_reset = false;
};

enum class RequestType : int {
    Observe = 0, 
    Step = 1,
    Reset = 2
};

PendulumState motorState;

void send_debug_message(const char* message) {
    Serial.print("{\"status\":\"DEBUG\",\"message\":\"");
    Serial.print(message);
    Serial.println("\"}");
}

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
            auto reqType = static_cast<RequestType>(request[1].as<int>());

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
            else if (reqType == RequestType::Reset) {
                // bool does not need to be protected by mutex because it is atomic
                motorState.needs_reset = true;
                send_debug_message("Reset requested");
                // busy wait until the reset is complete
                // we don't want to process any other commands until the reset is complete
                while (motorState.needs_reset) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }

                response["status"] = motorState.enabled ? "OK" : "DISABLED";
                response["position"] = motorState.current_position;
                response["theta"] = motorState.theta;
                response["angular_velocity"] = motorState.angular_velocity;
                response["limitL"] = motorState.limitL;
                response["limitR"] = motorState.limitR;
                response["target"] = motorState.target_position;
                response["enabled"] = motorState.enabled;
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
        bool needs_reset = motorState.needs_reset;
        bool limitStateL = motorState.limitL;
        bool limitStateR = motorState.limitR;
        xSemaphoreGive(dataMutex);

        // if a limit switch was triggered, disable the motor
        if (limitStateL || limitStateR) {
            digitalWrite(MOTOR_ENABLE, LOW);
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.enabled = false;
            xSemaphoreGive(dataMutex);
        }

        if (needs_reset) {
            send_debug_message("Resetting pendulum");
            digitalWrite(MOTOR_ENABLE, HIGH);
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.enabled = true;
            xSemaphoreGive(dataMutex);

            // to reset we need to move the cart to both limits, where they will stop and then back to the center
            // setting the center as the 0 position
            // being aware that we may currently be at a limit, we need to move away from it first
            if (limitStateL) {
                send_debug_message(sprintf("Moving away from left limit"
                int32_t left_limit_position = stepper.currentPosition();
                stepper.moveTo(1000);
                while (limitStateR == false) {
                    stepper.run();
                    vTaskDelay(pdMS_TO_TICKS(1));
                    xSemaphoreTake(dataMutex, portMAX_DELAY);
                    limitStateR = motorState.limitR;
                    xSemaphoreGive(dataMutex);
                }

                // stop the motor
                digitalWrite(MOTOR_ENABLE, LOW);
                stepper.setCurrentPosition(stepper.currentPosition());

                // move to the center
                int32_t right_limit_position = stepper.currentPosition();
                int32_t center_position = (left_limit_position + right_limit_position) / 2;
                digitalWrite(MOTOR_ENABLE, HIGH);
                stepper.moveTo(center_position);

                while (stepper.currentPosition() != center_position) {
                    stepper.run();
                    vTaskDelay(pdMS_TO_TICKS(1));
                }

                // update the current position to the center
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                motorState.current_position = center_position;
                motorState.target_position = center_position;
                motorState.needs_reset = false;
                xSemaphoreGive(dataMutex);
                send_debug_message("Reset complete");
            }
        }

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
     WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Connection Failed! Rebooting...");
      delay(5000);
      ESP.restart();
    }

    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

    ArduinoOTA.begin();

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    xTaskCreatePinnedToCore(communicate, "Communicate", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(monitor, "Monitor", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(act, "Act", 4096, NULL, 3, NULL, 1);
}

void loop() { ArduinoOTA.handle();}