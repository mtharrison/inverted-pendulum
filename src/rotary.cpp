#include "rotary.h"

RotaryEncoder::RotaryEncoder(uint8_t inputDT, uint8_t inputCLK, uint8_t inputSW) : inputDT(inputDT), inputCLK(inputCLK), inputSW(inputSW) {
  pinMode(inputDT, INPUT);
  pinMode(inputCLK, INPUT);
  pinMode(inputSW, INPUT_PULLUP);

  lastStateCLK = digitalRead(inputCLK);
}

bool RotaryEncoder::poll() {
  int previousPosition = position;
  int currentStateCLK = digitalRead(inputCLK);

  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
    if (digitalRead(inputDT) != currentStateCLK) {
      position++;
    }
    else {
      position--;
    }
  }

  if (digitalRead(inputSW) == LOW) {
    position = 0;
  }

  lastStateCLK = currentStateCLK;
  return position != previousPosition;
}

int RotaryEncoder::read_position() {
  return position;
}

void RotaryEncoder::reset() {
  position = 0;
}
