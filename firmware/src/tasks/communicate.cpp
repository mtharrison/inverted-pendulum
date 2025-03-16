#include "Arduino.h"
#include "config.h"
#include "serde.h"
#include "task.h"
#include "USB.h"

void sendState(int id) {
    char buffer[200];
    int len = snprintf(buffer, sizeof(buffer),
      "id=%d|current_position=%d|velocity=%f|theta=%f|angular_velocity=%f|limitL=%d|limitR=%d|speed=%f|enabled=%d|resetting=%d|extent=%d\n",
        id, motorState.current_position, motorState.velocity, motorState.theta, motorState.angular_velocity,
        motorState.limitL, motorState.limitR, motorState.speed, motorState.enabled, motorState.resetting, motorState.extent);

    USBSerial.write(buffer, len);
    USBSerial.flush();
}

void communicate(void* parameters)
{
    static char rxBuffer[SERIAL_BUFFER_SIZE];
    static size_t rxPos = 0;

    for (;;)
    {
        while (USBSerial.available() > 0)
        {
            int c = USBSerial.read();
            if (c < 0) {
                break;
            }

            if (c == '\n')
            {
                rxBuffer[rxPos] = '\0';
                ParsedMessage pm = parseMessage(rxBuffer);
                if (pm.command == CMD_SENSE) 
                {
                    sendState(pm.id);
                }
                else if (pm.command == CMD_MOVE) 
                {
                    float speed = pm.argument;
                    if (motorState.enabled && !motorState.resetting) 
                    {
                        motorState.speed = speed;
                    }
                    sendState(pm.id);
                }
                else if (pm.command == CMD_RESET)
                {
                    motorState.resetting = true;
                    xEventGroupSetBits(resetEventGroup, RESET_BIT);
                    sendState(pm.id);
                }
                rxPos = 0;
            }
            else
            {
                if (rxPos < SERIAL_BUFFER_SIZE - 1)
                {
                    rxBuffer[rxPos++] = static_cast<char>(c);
                }
                else
                {
                    rxPos = 0;
                }
            }
        }

        taskYIELD();
    }
}
