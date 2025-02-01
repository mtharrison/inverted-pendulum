#include <Arduino.h>
#include "ArduinoJson.h"

uint8_t recv_index = 0x12;
uint8_t send_index = 0x34;

void setup() {
    Serial.begin(115200);

    
    
}

void loop() {
  while (Serial.available()) {
    char buffer[100];
    Serial.readBytesUntil('}', buffer, 100);
    StaticJsonDocument<100> input;
    deserializeJson(input, buffer);
    int key = input["key"];
    key *= 10;
    input["key"] = key;

    serializeJson(input, Serial);
  }
}

// x954