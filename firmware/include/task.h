#ifndef TASK_H
#define TASK_H

#include "freertos/task.h"

void communicate(void* parameters);
void act(void* parameters);
void monitor(void* parameters);

struct PendulumState {
    int current_position = 0;
    int target_position;
    int extent = 0;
    
    float max_speed = 0;
    float speed = 0;
    float theta = 0;
    float velocity = 0;
    float angular_velocity = 0;

    bool limitL = false;
    bool limitR = false;
    bool enabled = true;
    bool resetting = false;
};

extern PendulumState motorState;

extern EventGroupHandle_t resetEventGroup;

#endif // TASK_H
