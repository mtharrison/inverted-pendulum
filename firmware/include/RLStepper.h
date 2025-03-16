#pragma once

#include <Arduino.h>
#include "driver/rmt.h"

/**
 * A simple RMT-based stepper driver for ESP32
 * that continuously generates step pulses, with
 * a background FreeRTOS task that updates speed
 * (including smooth acceleration).
 */
class StepperRMT {
public:
    /**
     * @param stepPin       GPIO pin for "STEP" signal
     * @param dirPin        GPIO pin for "DIR" signal
     * @param rmtChannel    Which RMT channel to use (0..7)
     * @param maxSpeed      Maximum speed in steps/sec
     * @param acceleration  Acceleration in steps/sec^2
     */
    StepperRMT(int stepPin, 
               int dirPin, 
               rmt_channel_t rmtChannel, 
               float maxSpeed, 
               float acceleration);

    /**
     * Initialize pins, configure RMT, and start the background task
     */
    void begin();

    /**
     * Set desired normalized speed in [-1, 1].
     * -1 => -maxSpeed (reverse)
     *  0 => stop
     * +1 => +maxSpeed (forward)
     */
    void setSpeed(float normalizedSpeed);

    /**
     * @return The current (instantaneous) speed in steps/sec
     *         (may still be ramping toward the target)
     */
    float getSpeed();

    /**
     * @return Approximate current position in steps.
     *         (Software counter, increments on pulses commanded to the motor)
     */
    long getPosition();

    /**
     * Update position by speed * dt
     */
    void updatePosition(float dt);

    void resetPosition();

private:
    // pin assignments
    int _stepPin;
    int _dirPin;

    // RMT channel
    rmt_channel_t _rmtChannel;

    // speed parameters
    float _maxSpeed;   // steps/sec
    float _accel;      // steps/sec^2

    // Current and target speeds (steps/sec)
    volatile float _currentSpeed;
    volatile float _targetSpeed;

    // Position in steps (software counted)
    volatile long _position;

    // FreeRTOS task handle
    TaskHandle_t _taskHandle;

    // For locking shared variables
    portMUX_TYPE _speedMutex = portMUX_INITIALIZER_UNLOCKED;

    /**
     * The background task that:
     *  1) Smoothly ramps speed toward _targetSpeed
     *  2) Updates the RMT item to match _currentSpeed
     *  3) Updates _position based on speed
     * Runs every ~5ms to keep motor speed smooth.
     */
    static void stepperTask(void* param);

    /**
     * Configures the RMT peripheral, sets an initial slow pulse.
     */
    void configureRMT();

    /**
     * Updates the RMT item for the current speed
     * Called from the stepperTask context (not ISR).
     */
    void updateRMT(float speedStepsPerSec);

    /**
     * Move speed smoothly by at most _accel * dt each iteration
     */
    void rampSpeed(float dt);

    
};
