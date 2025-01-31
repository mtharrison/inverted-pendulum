#include <Arduino.h>
#include "limit.h"

bool limit1Triggered = false;
bool limit2Triggered = false;

void IRAM_ATTR interruptHandler1() {
  limit1Triggered = true;
}

void IRAM_ATTR interruptHandler2() {
  limit2Triggered = true;
}

LimitSwitch::LimitSwitch(int id, int pin) : id(id), pin(pin) {
  pinMode(pin, INPUT_PULLUP);
}

void LimitSwitch::init() {
  if (id == 1)
    attachInterrupt(pin, interruptHandler1, FALLING);

  if (id == 2)
    attachInterrupt(pin, interruptHandler2, FALLING);
}


bool LimitSwitch::triggered() {
  if (id == 1) {
    if (limit1Triggered) {
      limit1Triggered = false;
      return true;
    }
    return false;
  }

  if (id == 2) {
    if (limit2Triggered) {
      limit2Triggered = false;
      return true;
    }
    return false;
  }

  return false;
}
