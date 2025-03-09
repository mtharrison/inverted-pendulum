#include "Arduino.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "USB.h"

#include "config.h"
#include "task.h"

EventGroupHandle_t resetEventGroup = xEventGroupCreate();
PendulumState motorState;

void setup() {
    USBSerial.begin(115200);
    disableCore0WDT();
    disableCore1WDT();

    // Create tasks
    xTaskCreatePinnedToCore(communicate, "Communicate", TASK_STACK_SIZE * 2, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(act, "Act", TASK_STACK_SIZE * 2, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(monitor, "Monitor", TASK_STACK_SIZE * 2, NULL, 5, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}
