#include "RLStepper.h"

StepperRMT::StepperRMT(int stepPin, 
                       int dirPin, 
                       rmt_channel_t rmtChannel, 
                       float maxSpeed, 
                       float acceleration)
    : _stepPin(stepPin),
      _dirPin(dirPin),
      _rmtChannel(rmtChannel),
      _maxSpeed(maxSpeed),
      _accel(acceleration),
      _currentSpeed(0.0f),
      _targetSpeed(0.0f),
      _position(0),
      _taskHandle(nullptr)
{
}

void StepperRMT::begin() {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    // Set initial DIR = LOW
    digitalWrite(_dirPin, LOW);

    // Configure RMT peripheral for generating step pulses
    configureRMT();

    // Create a dedicated FreeRTOS task to handle acceleration & RMT updates
    xTaskCreatePinnedToCore(
        StepperRMT::stepperTask,  // task function
        "StepperTask",            // name
        2048,                     // stack size
        this,                     // task parameter (this pointer)
        2,                        // priority
        &_taskHandle,
        0                         // pin to core 1 (optional, you can use 0 or tskNO_AFFINITY)
    );
}

/**
 * Called in user code to set speed in the range [-1, 1].
 * We'll just update _targetSpeed in a thread-safe manner.
 */
void StepperRMT::setSpeed(float normalizedSpeed) {
    // Clamp to [-1,1]
    if (normalizedSpeed > 1.0f) normalizedSpeed = 1.0f;
    if (normalizedSpeed < -1.0f) normalizedSpeed = -1.0f;

    float newTarget = normalizedSpeed * _maxSpeed;

    // USBSerial.printf("Setting speed to %f\n", newTarget);

    portENTER_CRITICAL(&_speedMutex);
    _targetSpeed = newTarget;
    portEXIT_CRITICAL(&_speedMutex);
}

float StepperRMT::getSpeed() {
    portENTER_CRITICAL(&_speedMutex);
    float s = _currentSpeed;
    portEXIT_CRITICAL(&_speedMutex);
    return s;
}

long StepperRMT::getPosition() {
    portENTER_CRITICAL(&_speedMutex);
    long p = _position;
    portEXIT_CRITICAL(&_speedMutex);
    return p;
}

//-----------------------------------------
// PRIVATE METHODS
//-----------------------------------------

void StepperRMT::configureRMT() {
    rmt_config_t config;
    config.rmt_mode                   = RMT_MODE_TX;
    config.channel                    = _rmtChannel;
    config.gpio_num                   = (gpio_num_t)_stepPin;
    config.mem_block_num              = 1;
    // We'll set clk_div=80 for 1 tick=1us if using the 80MHz APB clock
    config.clk_div                    = 80;
    config.tx_config.loop_en          = true;   // loop items continuously
    config.tx_config.carrier_en       = false;
    config.tx_config.idle_output_en   = true;
    config.tx_config.idle_level       = RMT_IDLE_LEVEL_LOW;

    // Not used in TX mode, but let's zero them out
    config.tx_config.carrier_level    = RMT_CARRIER_LEVEL_HIGH;
    config.tx_config.carrier_duty_percent = 50;
    config.tx_config.carrier_freq_hz  = 30000;
    config.flags                      = 0;

    rmt_config(&config);
    rmt_driver_install(_rmtChannel, 0, 0);

    // Write an initial item so the motor is effectively idle
    // We can choose a very long period => extremely slow stepping
    rmt_item32_t item[1];
    item[0].level0    = 0; 
    item[0].duration0 = 500000;  // 500 ms
    item[0].level1    = 0;
    item[0].duration1 = 500000;  // 500 ms
    rmt_write_items(_rmtChannel, item, 1, true);
}

/**
 * Updates the RMT item to produce steps at 'speedStepsPerSec'.
 * Called in the stepperTask, not in an ISR.
 */
void StepperRMT::updateRMT(float speedStepsPerSec) {
    float speedAbs = fabsf(speedStepsPerSec);

    // Set direction pin
    if (speedStepsPerSec >= 0.0f) {
        digitalWrite(_dirPin, HIGH);
    } else {
        digitalWrite(_dirPin, LOW);
    }

    // If speed is near zero, produce a long period -> effectively stop
    if (speedAbs < 1.0f) {
        rmt_item32_t item[1];
        item[0].level0    = 0;
        item[0].duration0 = 8480; // 500 ms
        item[0].level1    = 0;
        item[0].duration1 = 8480; // 500 ms
        rmt_write_items(_rmtChannel, item, 1, true);
        return;
    }

    // Period in microseconds for one full step (high + low)
    float period_us = 1000000.0f / speedAbs;

    // RMT durations are 15-bit max (~32767). For speeds < ~30 steps/s,
    // period might exceed 65535, so clamp it:
    if (period_us > 65535.0f) {
        period_us = 65535.0f;
    }

    uint32_t halfPeriod = (uint32_t)(period_us * 0.5f);
    if (halfPeriod < 1) halfPeriod = 1;

    rmt_item32_t item[1];
    item[0].level0    = 1;
    item[0].duration0 = halfPeriod;
    item[0].level1    = 0;
    item[0].duration1 = halfPeriod;

    rmt_write_items(_rmtChannel, item, 1, true);
}

/**
 * Accelerates/decelerates _currentSpeed toward _targetSpeed by up to _accel*dt
 */
void StepperRMT::rampSpeed(float dt) {
    float diff = _targetSpeed - _currentSpeed;
    float maxStep = _accel * dt; // how much speed can change in dt
    if (diff > maxStep) {
        _currentSpeed += maxStep;
    } else if (diff < -maxStep) {
        _currentSpeed -= maxStep;
    } else {
        _currentSpeed = _targetSpeed;
    }
}

/**
 * Update our software “step” counter based on how many steps
 * we commanded in this interval: steps = speed * dt
 */
void StepperRMT::updatePosition(float dt) {
    float stepsF = _currentSpeed * dt;
    _position += (long)stepsF; // integer cast
}

void StepperRMT::resetPosition() {
    _position = (long)0;
}

/**
 * This static function is the FreeRTOS task entry point. 
 * It just casts the void* back to StepperRMT* and runs an infinite loop 
 * with a 5ms delay.
 */
void StepperRMT::stepperTask(void* param) {
    StepperRMT* driver = static_cast<StepperRMT*>(param);

    const TickType_t delayTicks = pdMS_TO_TICKS(5);  // 5 ms
    const float dt = 0.005f; // 5 ms in seconds

    while (true) {
        // Lock while we modify speed and position
        portENTER_CRITICAL(&driver->_speedMutex);
        
        // 1) Ramp speed
        driver->rampSpeed(dt);

        // 2) Update position
        driver->updatePosition(dt);

        // 3) Update RMT with new speed
        float curSpeed = driver->_currentSpeed;
        portEXIT_CRITICAL(&driver->_speedMutex);

        // The actual RMT write
        driver->updateRMT(curSpeed);

        // Sleep 5 ms
        vTaskDelay(1);
    }

    // Theoretically unreachable
    vTaskDelete(nullptr);
}
