#include <Arduino.h>
#include "limit.h"

LimitSwitch::LimitSwitch(int id, int pin) : id(id), pin(pin) {
  pinMode(pin, INPUT_PULLUP);
}

bool LimitSwitch::triggered() {
  return digitalRead(pin) == LOW;
}
