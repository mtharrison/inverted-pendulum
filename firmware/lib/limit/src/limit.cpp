#include <Arduino.h>
#include "limit.h"

LimitSwitch::LimitSwitch(int pin) : pin(pin) {
  pinMode(pin, INPUT_PULLUP);
}

bool LimitSwitch::triggered() {
  return digitalRead(pin) == LOW;
}
