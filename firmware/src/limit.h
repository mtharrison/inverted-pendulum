#include <Arduino.h>

class LimitSwitch {
public:
  LimitSwitch(int id, int pin);
  bool triggered();
  void init();

public:
  int id;
  int pin;
};
