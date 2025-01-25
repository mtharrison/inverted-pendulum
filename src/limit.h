#include <Arduino.h>

class LimitSwitch {
public:
  LimitSwitch(int pin, int debounceMs);
  // call this every loop to check for changes
  bool triggered();

public:
  int pin;
  int debounceMs;
  long lastTriggerMs;
};
