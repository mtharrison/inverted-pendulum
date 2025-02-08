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

#define MAX_SPEED 100000
#define MAX_ACCEL 10000 * 10
#define SAFE_SPEED 1000

// Shared data protection
SemaphoreHandle_t dataMutex = xSemaphoreCreateMutex();

// Motor control structure
// shared global state of the system
struct PendulumState {
    // current position of the cart
    int32_t current_position = 0;  
    // target position of the cart 
    int32_t target_position = 0;
    // current angle of the pendulum
    double theta = 0;
    // current angular velocity of the pendulum
    double angular_velocity = 0;
    // whether the left limit switch is triggered
    bool limitL = false;
    // whether the right limit switch is triggered
    bool limitR = false;
    // whether the motor is enabled
    bool enabled = true;
    // whether the system needs to be reset
    bool needs_reset = false;
};

// Request types that can be received over serial
enum class RequestType : int {
    Observe = 0, 
    Step = 1,
    Reset = 2
};

// Global state of the system
PendulumState motorState;

// Send a debug message over serial
void DEBUG(const char* message, ...) {
    va_list args;
    va_start(args, message);
    char buffer[256];
    vsnprintf(buffer, 256, message, args);
    va_end(args);
    Serial.println(buffer);
}

// This task is responsible for receiving commands from the serial port
// and sending back the current state of the system
void communicate(void* parameters) {
    JsonDocument request;
    char buffer[SERIAL_BUFFER_SIZE];

    for (;;) {
        if (Serial.available()) {
            size_t len = Serial.readBytesUntil('\n', buffer, SERIAL_BUFFER_SIZE-1);
            buffer[len] = '\0';

            DeserializationError error = deserializeJson(request, buffer);
            if (error) {
                DEBUG("Failed to parse JSON: %s", error.c_str());
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
                DEBUG("Target position set to: %d", motorState.target_position);
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
                // reset the system and respond with the new state once the reset is complete
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                motorState.needs_reset = true;
                xSemaphoreGive(dataMutex);

                DEBUG("Waiting for reset to complete");
                while (motorState.needs_reset) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }

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

// This task is responsible for monitoring the state
// and updating the global state of the system
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

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// This task is responsible for controlling the motor
void act(void* parameters) {
    AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(MAX_ACCEL);
    pinMode(MOTOR_ENABLE, OUTPUT);
    digitalWrite(MOTOR_ENABLE, HIGH);

    for (;;) {
        // read the current state of the system
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        bool enabled = motorState.enabled;
        int32_t target = motorState.target_position;
        bool needs_reset = motorState.needs_reset;
        bool limitStateL = motorState.limitL;
        bool limitStateR = motorState.limitR;
        xSemaphoreGive(dataMutex);

        // if a limit switch was triggered, disable the motor
        if (limitStateL || limitStateR) {
            DEBUG("Limit switch triggered, disabling motor");
            digitalWrite(MOTOR_ENABLE, LOW);
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.enabled = false;
            xSemaphoreGive(dataMutex);
        }

        if (needs_reset) {
            stepper.setMaxSpeed(SAFE_SPEED);
            DEBUG("Motor reset requested");
            
            // first we need to find both limit switches
            // move the motor to the right until the right limit switch is triggered
            // then move the motor to the left until the left limit switch is triggered
            // finally, move the motor to the center and set the current position to 0
            // clear the needs_reset flag

            // find the right limit switch

            DEBUG("Finding right limit switch");
            digitalWrite(MOTOR_ENABLE, HIGH);
            stepper.moveTo(-100000);
            while (!limitStateR) {
                stepper.run();
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                limitStateR = motorState.limitR;
                xSemaphoreGive(dataMutex);
            }

            digitalWrite(MOTOR_ENABLE, LOW);
            long rightPosition = stepper.currentPosition();
            stepper.setCurrentPosition(rightPosition);
            DEBUG("Found right limit switch at %d", rightPosition);

            // find the left limit switch

            DEBUG("Finding left limit switch");
            digitalWrite(MOTOR_ENABLE, HIGH);
            stepper.moveTo(100000);
            while (!limitStateL) {
                stepper.run();
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                limitStateL = motorState.limitL;
                xSemaphoreGive(dataMutex);
            }

            digitalWrite(MOTOR_ENABLE, LOW);
            long leftPosition = stepper.currentPosition();
            stepper.setCurrentPosition(leftPosition);
            DEBUG("Found left limit switch at %d", leftPosition);

            // move to the center
            stepper.setMaxSpeed(MAX_SPEED);
            long center = (rightPosition + leftPosition) / 2;
            DEBUG("Moving to center at %d", center);
            digitalWrite(MOTOR_ENABLE, HIGH);
            stepper.moveTo(center);
            stepper.runToPosition();
            stepper.setCurrentPosition(0);
            DEBUG("Moved to center");

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.current_position = 0;
            motorState.target_position = 0;
            motorState.enabled = true;
            motorState.needs_reset = false;
            xSemaphoreGive(dataMutex);

            continue;
        }

        if (enabled) {
            stepper.moveTo(target);
            stepper.run();
            
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            motorState.current_position = stepper.currentPosition();
            xSemaphoreGive(dataMutex);

            if (stepper.distanceToGo() != 0) {
                DEBUG("Moving to %d, current position: %d", target, motorState.current_position);
            }
        }

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