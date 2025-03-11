#include "config.h"
#include "Arduino.h"
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "USB.h"
#include "task.h"

// RMT channel for stepper control
#define RMT_CHANNEL RMT_CHANNEL_0
#define RMT_CLK_DIV 20  // 80MHz / 20 = 4MHz (0.25us resolution)
#define RMT_QUEUE_SIZE 1

// Reset state machine states
enum ResetState {
    RESET_IDLE,
    RESET_TO_LEFT,
    RESET_TO_RIGHT,
    RESET_TO_CENTER,
    RESET_COMPLETE
};

// State for keeping track of stepping
volatile bool is_running = false;
volatile int current_speed = 0;
volatile bool current_direction = true;  // true = forward, false = backward

// Reset tracking variables
static ResetState reset_state = RESET_IDLE;
static int left_position = 0;
static int right_position = 0;

// RMT item buffer for continuous stepping
rmt_item32_t rmt_step_pattern[1];

// RMT TX complete callback
static void IRAM_ATTR rmt_tx_callback(rmt_channel_t channel, void *arg) {
    // If still running, send the step pattern again (continuous operation)
    if (is_running && abs(current_speed) > 0) {
        // Update position based on direction (increment or decrement)
        // Since we reversed the direction pin logic, we need to reverse this too
        if (current_direction) {
            motorState.current_position++;  // Forward direction (positive speed)
        } else {
            motorState.current_position--;  // Backward direction (negative speed)
        }
        
        ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL, rmt_step_pattern, 1, false));
    }
}

// Calculate RMT period from speed (steps/second)
static uint32_t speed_to_rmt_period(int speed) {
    if (speed <= 0) return 0;
    
    // Convert speed to period in microseconds
    uint32_t period_us = 1000000 / speed;
    
    // Clamp to reasonable values - adjusted for higher resolution
    if (period_us < 12) period_us = 12;  // Max ~83,333 steps/sec with 0.25us resolution
    if (period_us > 10000) period_us = 10000;  // Min 100 steps/sec
    
    return period_us;
}

// Update the RMT step pattern based on current speed
static void update_step_pattern() {
    if (current_speed == 0) {
        is_running = false;
        return;
    }
    
    uint32_t period_us = speed_to_rmt_period(abs(current_speed));
    
    // With RMT_CLK_DIV=20, each tick is 0.25us, so multiply microseconds by 4
    uint32_t period_ticks = period_us * 4;
    
    // Create a pulse pattern for one step - ensure minimum of 8 ticks for each phase (2us)
    rmt_step_pattern[0].level0 = 1; // HIGH
    rmt_step_pattern[0].duration0 = max(period_ticks / 2, 8U); // 50% duty cycle, min 2us
    rmt_step_pattern[0].level1 = 0; // LOW
    rmt_step_pattern[0].duration1 = max(period_ticks / 2, 8U); // 50% duty cycle, min 2us
}

// Target speed for acceleration control
static int target_speed = 0;
static unsigned long last_accel_time = 0;

// Start or stop the stepping
static void control_stepper(int speed) {
    // Log input speed for debugging
    static int last_logged_speed = 0;
    if (speed != last_logged_speed) {
        //USBSerial.printf("control_stepper called with speed = %d\n", speed);
        last_logged_speed = speed;
    }

    bool direction = speed >= 0;
    speed = abs(speed);
    
    // If stopping, do it immediately
    if (speed == 0) {
        if (is_running) {
            is_running = false;
            //USBSerial.println("Stopping motor");
        }
        current_speed = 0;
        return;
    }
    
    // If direction changed or we're starting from stop
    if (direction != current_direction || current_speed == 0) {
        // Stop first if running
        if (is_running) {
            is_running = false;
            //USBSerial.println("Stopping before direction change");
            
            // Wait for motor to fully stop
            delay(50);  // 50ms full stop
        }
        
        // Update direction
        current_direction = direction;
        //USBSerial.printf("Setting direction pin to %s\n", direction ? "LOW (RIGHT)" : "HIGH (LEFT)");
        
        // Set direction pin - reversed to match actual hardware behavior
        digitalWrite(DIR_PIN, direction ? LOW : HIGH);
        current_speed = 0;
    }
    
    // Set new speed directly - no acceleration for now to simplify debugging
    current_speed = direction ? speed : -speed;
    //USBSerial.printf("Setting current_speed to %d\n", current_speed);
    
    // Update the step pattern
    update_step_pattern();
    
    // Start stepping if not already running
    if (!is_running && speed > 0) {
        //USBSerial.println("Starting motor");
        is_running = true;
        ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL, rmt_step_pattern, 1, false));
    }
}

// Process reset state machine
static void handle_reset(PendulumState &state) {
    // Debug the current state
    static ResetState last_reset_state = RESET_IDLE;
    if (reset_state != last_reset_state) {
        //USBSerial.print("Reset state changed to: ");
        switch (reset_state) {
            case RESET_IDLE: USBSerial.println("RESET_IDLE"); break;
            case RESET_TO_LEFT: USBSerial.println("RESET_TO_LEFT"); break;
            case RESET_TO_RIGHT: USBSerial.println("RESET_TO_RIGHT"); break;
            case RESET_TO_CENTER: USBSerial.println("RESET_TO_CENTER"); break;
            case RESET_COMPLETE: USBSerial.println("RESET_COMPLETE"); break;
        }
        last_reset_state = reset_state;
    }
    
    // Reset state machine
    switch (reset_state) {
        case RESET_IDLE:
            if (state.resetting) {
                // Zero out speed when entering reset mode
                state.speed = 0;
                
                // Start reset process - actual hardware behavior shows we're moving right first
                reset_state = RESET_TO_RIGHT;
                //USBSerial.println("Starting reset sequence - moving right");
                
                // Use the new control_stepper function
                control_stepper(SAFE_SPEED);
            }
            break;
            
        case RESET_TO_LEFT:
            if (state.limitL) {
                // Hit left limit
                control_stepper(0);  // Stop
                
                // Add a small delay to ensure we've fully stopped
                vTaskDelay(pdMS_TO_TICKS(50));
                
                left_position = state.current_position;
                
                // Calculate center position
                int center_position = (left_position + right_position) / 2;
                state.extent = right_position - left_position;
                
                // Log positions
                //USBSerial.printf("Left pos: %d, Right pos: %d, Center: %d\n", 
                    // left_position, right_position, center_position);
                
                // Move to center
                reset_state = RESET_TO_CENTER;
                state.target_position = center_position;
                
                // Give a short pause before changing direction
                vTaskDelay(pdMS_TO_TICKS(100));
                
                // Determine direction to center - should be right from the left limit
                //USBSerial.println("Starting move to center");
                
                // Use the new control_stepper function
                control_stepper(SAFE_SPEED);
                
                // Log the limit detection for debugging
                //USBSerial.println("Left limit detected, moving to center");
            }
            
            // Add safety check - if we're exceeding reasonable position limit
            if (!state.limitL && state.current_position < -50000) {
                // Emergency stop if we've gone too far
                control_stepper(0);
                //USBSerial.println("Emergency stop - exceeded left position limit");
                reset_state = RESET_IDLE;
                state.resetting = false;
            }
            break;
            
        case RESET_TO_RIGHT:
            if (state.limitR) {
                // Hit right limit
                control_stepper(0);  // Stop
                
                // Add a small delay to ensure we've fully stopped
                vTaskDelay(pdMS_TO_TICKS(50));
                
                // Store the position at right limit
                right_position = state.current_position;
                
                // Now move to left limit
                reset_state = RESET_TO_LEFT;
                //USBSerial.println("Moving to left limit");
                
                // Use the new control_stepper function
                control_stepper(-SAFE_SPEED);
                
                // Log the limit detection for debugging
                //USBSerial.println("Right limit detected, moving left");
            }
            
            // Add safety check - if we're exceeding reasonable position limit
            if (!state.limitR && state.current_position > 50000) {
                // Emergency stop if we've gone too far
                control_stepper(0);
                //USBSerial.println("Emergency stop - exceeded right position limit");
                reset_state = RESET_IDLE;
                state.resetting = false;
            }
            break;
            
        case RESET_TO_CENTER:
            {
                int center_position = (left_position + right_position) / 2;

                //USBSerial.print(center_position);
                //USBSerial.print(" | ");
                //USBSerial.print(state.current_position);
                //USBSerial.print(" | ");
                //USBSerial.print(current_direction);
                //USBSerial.print(" | ");
                //USBSerial.print(current_speed);
                //USBSerial.print("\n");
                
                // Log progress to center periodically
                static int center_debug_counter = 0;
                if (++center_debug_counter >= 40) { // Every ~200ms
                    center_debug_counter = 0;
                    //USBSerial.printf("Moving to center: Current=%d, Target=%d, Diff=%d\n", 
                        // state.current_position, center_position, 
                        // state.current_position - center_position);
                }
                
                // Check if we're close to center (within 10 steps)
                if (abs(state.current_position - center_position) < 10) {
                    // At center position
                    //USBSerial.println("Reached center position, reset complete");
                    
                    // Make sure we completely stop the motor
                    control_stepper(0);
                    
                    // Allow time for motor to stop
                    vTaskDelay(pdMS_TO_TICKS(50));
                    
                    reset_state = RESET_COMPLETE;
                    
                    // Signal completion
                    xEventGroupSetBits(resetEventGroup, RESET_CLEAR_BIT);
                    state.resetting = false;
                }
                else {
                    // Adjust speed/direction to reach center
                    if (state.current_position < center_position) {
                        // Need to move right
                        if (current_speed <= 0) { // If stopped or moving wrong direction
                            //USBSerial.println("Moving right to center");
                            control_stepper(SAFE_SPEED);
                        }
                    } else {
                        // Need to move left
                        if (current_speed >= 0) { // If stopped or moving wrong direction
                            //USBSerial.println("Moving left to center");
                            control_stepper(-SAFE_SPEED);
                        }
                    }
                }
            }
            break;
            
        case RESET_COMPLETE:
            reset_state = RESET_IDLE;
            break;
    }
}

void act(void* parameters) {
    // Configure pins
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    
    // Initially disable motor
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);  // Assuming HIGH is disable, adjust if needed
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    
    // Configure RMT for step generation
    rmt_config_t rmt_cfg = {};
    rmt_cfg.rmt_mode = RMT_MODE_TX;
    rmt_cfg.channel = RMT_CHANNEL;
    rmt_cfg.gpio_num = (gpio_num_t)STEP_PIN;
    rmt_cfg.mem_block_num = 1;
    rmt_cfg.clk_div = RMT_CLK_DIV;
    rmt_cfg.tx_config.loop_en = false;
    rmt_cfg.tx_config.carrier_en = false;
    rmt_cfg.tx_config.idle_output_en = true;
    rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    
    ESP_ERROR_CHECK(rmt_config(&rmt_cfg));
    ESP_ERROR_CHECK(rmt_driver_install(RMT_CHANNEL, RMT_QUEUE_SIZE, 0));
    rmt_register_tx_end_callback(rmt_tx_callback, NULL);
    
    int last_speed = 0;
    
    for (;;) {
        // Enable/disable motor
        digitalWrite(MOTOR_ENABLE_PIN, motorState.enabled ? LOW : HIGH);
        
        if (motorState.enabled) {
            // Check if we're in reset mode
            if (motorState.resetting || reset_state != RESET_IDLE) {
                // Handle reset state machine
                handle_reset(motorState);
            } else {
                // Normal operation - motorState.speed is already scaled in communicate.cpp
                // Just constrain it to MAX_SPEED
                int target_speed = (int)constrain(abs(motorState.speed), 0.0f, (float)MAX_SPEED);
                
                // Apply direction from motorState.speed
                if (motorState.speed < 0) {
                    target_speed = -target_speed;
                }
                
                // // Extended debug output
                // USBSerial.printf("Speed value: %f, Target: %d, Current: %d, Direction: %d, Running: %d\n", 
                //     motorState.speed, target_speed, current_speed, current_direction, is_running);
                
                // Check limit switches and prevent motion in that direction
                if ((motorState.limitL && target_speed < 0) || 
                    (motorState.limitR && target_speed > 0)) {
                    target_speed = 0;
                }
                
                // Compare with the last speed we set
                if (target_speed != last_speed) {
                    // USBSerial.printf("Speed changed from %d to %d\n", last_speed, target_speed);
                    last_speed = target_speed;
                    
                    // Use the new control_stepper implementation without acceleration
                    control_stepper(target_speed);
                }
            }
        } else {
            // Motor disabled - stop movement
            if (last_speed != 0) {
                control_stepper(0);
                last_speed = 0;
            }
            
            // Reset the reset state if motor is disabled
            if (reset_state != RESET_IDLE) {
                reset_state = RESET_IDLE;
            }
        }
        
        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
