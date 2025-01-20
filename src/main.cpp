/*
 * Microstepping demo
 *
 * This requires that microstep control pins be connected in addition to STEP,DIR
 *
 * Copyright (C)2015 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>

int inputCLK = 33;
int inputDT = 27;
// int aState;
int lastStateCLK;
int counter = 0;
int lastCounter = 0;

void setup() {
  pinMode(inputCLK, INPUT);
  pinMode(inputDT, INPUT);
  Serial.begin(9600);
  lastStateCLK = digitalRead(inputCLK);
}

int read_position() {
  int currentStateCLK = digitalRead(inputCLK);
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
    if (digitalRead(inputDT) != currentStateCLK) {
      counter++;
    }
    else {
      counter--;
    }
  }
  lastStateCLK = currentStateCLK;
  return counter;
}

void loop() {
  if (counter != lastCounter) {
    lastCounter = counter;
    Serial.print("Position: ");
    Serial.println(counter);
  }

  read_position();
}

// void read_position() {
//   int currentStateCLK = digitalRead(inputCLK);
//   if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
//     // outputB != outputA state, encoder is rotating clockwise
//     if (digitalRead(inputDT) != currentStateCLK) {
//       counter++;
//     }
//     else {
//       counter--;
//     }
//     Serial.print("Position: ");
//     Serial.println(counter);
//   }
//   lastStateCLK = currentStateCLK;
// }

// void loop() {
//   read_position();
// }


// class RotaryEncoder {
// public:
//   RotaryEncoder(uint8_t inputDT, uint8_t inputCLK, uint8_t pinSw) : inputDT(inputDT), inputCLK(inputCLK), pinSw(pinSw) {
//     pinMode(inputDT, INPUT);
//     pinMode(inputCLK, INPUT);
//     pinMode(pinSw, INPUT_PULLUP);

//     lastStateCLK = digitalRead(inputCLK);
//   }

//   int8_t read_position() {
//     int currentStateCLK = digitalRead(inputCLK);

//     if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
//       // If the DT state is different than the CLK state then
//       // the encoder is rotating CCW so decrement
//       if (digitalRead(inputDT) != currentStateCLK) {
//         position--;
//       }
//       else {
//         position++;
//       }
//     }

//     lastStateCLK = currentStateCLK;
//     return position;
//   }

//   bool read_switch() {
//     return digitalRead(pinSw) == LOW;
//   }

// public:
//   uint8_t inputDT;
//   uint8_t inputCLK;
//   uint8_t pinSw;
//   int8_t position = 0;
//   uint8_t lastStateCLK;
// };

// RotaryEncoder knob;

// void setup()
// {
//   Serial.begin(9600);
//   knob = RotaryEncoder(27, 33, 13);
// }

// void loop()
// {
//   Serial.print("Position: ");
//   Serial.println(knob.read_position());

// }

//  // MultiStepper.pde
// // -*- mode: C++ -*-
// //
// // Shows how to multiple simultaneous steppers
// // Runs one stepper forwards and backwards, accelerating and decelerating
// // at the limits. Runs other steppers at the same time
// //
// // Copyright (C) 2009 Mike McCauley
// // $Id: MultiStepper.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

// #include <AccelStepper.h>

// // // Define some steppers and the pins the will use
// AccelStepper stepper1(AccelStepper::DRIVER, 23, 22, 0, 0);

// // void setup()
// // {
// //   stepper1.setMaxSpeed(1000.0);
// //   stepper1.setAcceleration(200.0);
// //   stepper1.moveTo(2000);
// // }

// // void loop()
// // {
// //   // Change direction at the limits
// //   if (stepper1.distanceToGo() == 0)
// //     stepper1.moveTo(-stepper1.currentPosition());
// //   stepper1.run();
// // }


// #define encoder1Dt 19
// #define encoder1Ck 21
// #define encoder1Sw 18
// #define limitR 5

// int counter = 0;
// int aState;
// int aLastState;
// int swState;
// int position = 0;
// bool emergencyStop = false;

// void inter() {
//   emergencyStop = true;
// }

// void setup() {
//   Serial.begin(9600);
//   pinMode(encoder1Dt, INPUT);
//   pinMode(encoder1Ck, INPUT);
//   pinMode(encoder1Ck, INPUT_PULLDOWN);
//   pinMode(limitR, INPUT_PULLUP);

//   aLastState = digitalRead(encoder1Dt);

//   attachInterrupt(limitR, inter, FALLING);

//   stepper1.setMaxSpeed(100000.0);
//   stepper1.setAcceleration(200000.0);
// }

// void loop() {

//   if (emergencyStop) {
//     stepper1.stop();
//     Serial.println("Emergency stop");
//     emergencyStop = false;
//   }

//   swState = digitalRead(encoder1Sw);

//   // if (digitalRead(limitR) == LOW) {
//   //   counter = 0;
//   // }

//   if (swState == 0) {
//     // counter = 0;
//   }
//   aState = digitalRead(encoder1Dt); // Reads the "current" state of the outputA
//   // If the previous and the current state of the outputA are different, that means a Pulse has occured
//   if (aState != aLastState) {
//     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//     if (digitalRead(encoder1Ck) != aState) {
//       counter++;
//     }
//     else {
//       counter--;
//     }
//     Serial.print("Position: ");
//     Serial.println(counter);
//   }
//   aLastState = aState; // Updates the previous state of the outputA with the current state

//   if (counter * 1000 != position) {
//     position = counter * 1000;
//     stepper1.moveTo(position);
//   }

//   stepper1.run();
// }
