#include <Arduino.h>

class RotaryEncoder {
public:
  RotaryEncoder(int inputDT, int inputCLK, int inputSW);
  // call this every loop to check for changes
  bool poll();
  // returns the current position
  int read_position();

  void reset();

public:
  int inputDT;
  int inputCLK;
  int inputSW;
  int position = 0;
  int lastStateCLK;
  int lastStateDT;
};
