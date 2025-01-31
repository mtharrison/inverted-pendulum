#include "rotary.h"

RotaryEncoder::RotaryEncoder(int inputDT, int inputCLK, int inputSW) : inputDT(inputDT), inputCLK(inputCLK), inputSW(inputSW) {
  pinMode(inputDT, INPUT_PULLUP);
  pinMode(inputCLK, INPUT_PULLUP);

  if (inputSW != -1)
    pinMode(inputSW, INPUT_PULLUP);

  lastStateCLK = digitalRead(inputCLK);
}

bool RotaryEncoder::poll() {
  int previousPosition = position;
  int currentStateCLK = digitalRead(inputCLK);

  if (currentStateCLK != lastStateCLK ) {
    if (digitalRead(inputDT) != currentStateCLK) {
      position++;
    }
    else {
      position--;
    }
  }

  if (inputSW != -1) {
    if (digitalRead(inputSW) == LOW) {
      position = 0;
    }
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
