#include <Arduino.h>
#include "limit.h"
#include <sys/time.h>

long time() {
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  return ms;
}

LimitSwitch::LimitSwitch(int pin, int debounceMs) : pin(pin), debounceMs(debounceMs) {
  pinMode(pin, INPUT_PULLUP);
  lastTriggerMs = time();
}

bool LimitSwitch::triggered() {
  if (digitalRead(pin) == LOW) {
    long now = time();
    if ((now - lastTriggerMs) > debounceMs) {
      lastTriggerMs = now;
      return true;
    }
  }

  return false;
}
