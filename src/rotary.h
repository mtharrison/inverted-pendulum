#include <Arduino.h>

class RotaryEncoder {
public:
  RotaryEncoder(uint8_t inputDT, uint8_t inputCLK, uint8_t inputSW);
  // call this every loop to check for changes
  bool poll();
  // returns the current position
  int read_position();

  void reset();

public:
  uint inputDT;
  uint inputCLK;
  int inputSW;
  int position = 0;
  int lastStateCLK;
};
