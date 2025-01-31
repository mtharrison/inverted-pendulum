#include <Arduino.h>
#include "rotary.h"
#include "limit.h"
#include <AccelStepper.h>

#define MICROSTEPS 1600
#define MOTOR_ENABLE 18

#include <ESP32Encoder.h>

ESP32Encoder encoder;
LimitSwitch limitL = LimitSwitch(1, 12);
LimitSwitch limitR = LimitSwitch(2, 10);
AccelStepper stepper(AccelStepper::DRIVER, 16, 7);

void setup() {
  Serial.begin(9600);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(13, 14);

  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(10000);

  limitL.init();
  limitR.init();
  digitalWrite(MOTOR_ENABLE, HIGH);

  delay(5000);
}

// void emergency() {
//   digitalWrite(MOTOR_ENABLE, LOW);
//   int currPos = stepper.currentPosition();
//   stepper.setCurrentPosition(currPos);
//   encoder.reset();
//   digitalWrite(MOTOR_ENABLE, HIGH);
//   stepper.runToNewPosition(0);
//   Serial.println("Emergency stop, homed gantry");
// }

// void homeGantry() {
//   stepper.setMaxSpeed(2000);
//   Serial.println("Homing gantry");

//   Serial.println("Homing left -100");
//   stepper.moveTo(100 * MICROSTEPS);
//   while (!limitL.triggered() && !limitR.triggered()) {
//     stepper.run();
//   }

//   digitalWrite(MOTOR_ENABLE, LOW);
//   int leftLimit = stepper.currentPosition();
//   Serial.print("Limit triggered at ");
//   Serial.println(leftLimit);
//   stepper.setCurrentPosition(leftLimit);
//   Serial.print("Current position");
//   Serial.println(stepper.currentPosition());
//   encoder.reset();

//   // move away from limit
//   digitalWrite(MOTOR_ENABLE, HIGH);
//   stepper.runToNewPosition(0);

//   Serial.println("Homing right 100");
//   stepper.moveTo(-100 * MICROSTEPS);
//   while (!limitL.triggered() && !limitR.triggered()) {
//     stepper.run();
//   }

//   digitalWrite(MOTOR_ENABLE, LOW);
//   int rightLimit = stepper.currentPosition();
//   Serial.print("Limit triggered at");
//   Serial.println(rightLimit);
//   stepper.setCurrentPosition(rightLimit);
//   Serial.print("Current position");
//   encoder.reset();

//   digitalWrite(MOTOR_ENABLE, HIGH);
//   int middle = (leftLimit + rightLimit) / 2;
//   Serial.print("Moving to middle: ");
//   Serial.println(middle);

//   stepper.runToNewPosition((leftLimit + rightLimit) / 2);
//   stepper.setCurrentPosition(0);
//   encoder.reset();
//   Serial.println("Gantry homed");
//   stepper.setMaxSpeed(50000);
// }

// bool homed = false;

int pos = 150;

void loop() {

  // if(Serial.available())                                   // if there is data comming
  // {
  //   String message = Serial.readStringUntil('\n');         // read string until meet newline character
  //   char command = message.charAt(0);
  //   int num = atoi(message.substring(1).c_str());

  //   switch (command)
  //   {
  //   case 'P':
  //     position = num;
  //     break;
  //   case 'S':
  //     speed = num;
  //     break;

  //   default:
  //     break;
  //   }

  //   stepper.moveTo(position); 
  //   stepper.setSpeed(speed);

  //   Serial.println("Position     = " + String(position));
  //   Serial.println("Speed        = " + String(speed) + "\n\n");
  // }

  // stepper.runSpeedToPosition()

  // Serial.println("Encoder count = " + String((int32_t)encoder.getCount()));

  // m = -m;
  // stepper.moveTo(m);


  // // if (!homed)
  // //   homeGantry();
  // // homed = true;

  // if (encoder.poll()) {
  //   int pos = encoder.read_position();
  //   if (pos % 100 == 0) {
  //     Serial.print("Position: ");
  //     Serial.println(encoder.read_position());
  //   }

  //   // stepper.moveTo(-encoder.read_position() * MICROSTEPS);
  // }


  // if (limitL.triggered()) {
  //   Serial.println("Limit L triggered");
  // }

  // if (limitR.triggered()) {
  //   Serial.println("Limit R triggered");
  // }
  // // if (limitL.triggered() || limitR.triggered()) {
  //   emergency();
  // }

  // stepper.run();
  // pos = -pos;
  stepper.moveTo(pos);
  stepper.runToPosition();

  delay(1000);

  stepper.moveTo(-pos);
  stepper.runToPosition();

  delay(1000);
}
