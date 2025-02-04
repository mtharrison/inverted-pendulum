#include <Arduino.h>

class LimitSwitch {
public:
  LimitSwitch(int pin);
  bool triggered();
  void init();

public:
  int pin;
};
