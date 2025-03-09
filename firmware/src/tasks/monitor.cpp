#include "Arduino.h"
#include <ESP32Encoder.h>
#include "config.h"
#include "limit.h"
#include "task.h"

void monitor(void* parameters) {
    ESP32Encoder encoder;
    encoder.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B);

    LimitSwitch limitL(LIMIT_L_PIN);
    LimitSwitch limitR(LIMIT_R_PIN);

    int64_t lastTime = esp_timer_get_time();
    float filtered_angular_velocity = 0;
    float filtered_velocity = 0;
    int32_t lastPosition = 0;
    int32_t lastTheta = 0;

    for (;;) {
        motorState.limitL = limitL.triggered();
        motorState.limitR = limitR.triggered();

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

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}