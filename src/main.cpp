#include <Arduino.h>
#include "rotary.h"
#include "limit.h"
#include <AccelStepper.h>

#define MICROSTEPS 1600

#define MOTOR_ENABLE 47


RotaryEncoder encoder = RotaryEncoder(4, 15, 12);
LimitSwitch limitL = LimitSwitch(10, 1000);
LimitSwitch limitR = LimitSwitch(8, 1000);
AccelStepper stepper(AccelStepper::DRIVER, 14, 21);

void setup() {
  pinMode(MOTOR_ENABLE, OUTPUT);
  Serial.begin(9600);
  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(1000);

  digitalWrite(0, HIGH);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void emergency() {
  digitalWrite(MOTOR_ENABLE, LOW);
  int currPos = stepper.currentPosition();
  stepper.setCurrentPosition(currPos);
  encoder.reset();
  digitalWrite(MOTOR_ENABLE, HIGH);
  stepper.runToNewPosition(0);
  Serial.println("Emergency stop, homed gantry");
}

void homeGantry() {
  stepper.setMaxSpeed(2000);
  Serial.println("Homing gantry");

  Serial.println("Homing left -100");
  stepper.moveTo(100 * MICROSTEPS);
  while (!limitL.triggered() && !limitR.triggered()) {
    stepper.run();
  }

  digitalWrite(MOTOR_ENABLE, LOW);
  int leftLimit = stepper.currentPosition();
  Serial.print("Limit triggered at ");
  Serial.println(leftLimit);
  stepper.setCurrentPosition(leftLimit);
  Serial.print("Current position");
  Serial.println(stepper.currentPosition());
  encoder.reset();

  // move away from limit
  digitalWrite(MOTOR_ENABLE, HIGH);
  stepper.runToNewPosition(0);

  Serial.println("Homing right 100");
  stepper.moveTo(-100 * MICROSTEPS);
  while (!limitL.triggered() && !limitR.triggered()) {
    stepper.run();
  }

  digitalWrite(MOTOR_ENABLE, LOW);
  int rightLimit = stepper.currentPosition();
  Serial.print("Limit triggered at");
  Serial.println(rightLimit);
  stepper.setCurrentPosition(rightLimit);
  Serial.print("Current position");
  encoder.reset();

  digitalWrite(MOTOR_ENABLE, HIGH);
  int middle = (leftLimit + rightLimit) / 2;
  Serial.print("Moving to middle: ");
  Serial.println(middle);

  stepper.runToNewPosition((leftLimit + rightLimit) / 2);
  stepper.setCurrentPosition(0);
  encoder.reset();
  Serial.println("Gantry homed");
  stepper.setMaxSpeed(50000);
}

bool homed = false;

void loop() {

  if (!homed)
    homeGantry();
  homed = true;

  if (encoder.poll()) {
    Serial.print("Position: ");
    Serial.println(encoder.read_position());
    stepper.moveTo(-encoder.read_position() * MICROSTEPS);
  }

  if (limitL.triggered() || limitR.triggered()) {
    emergency();
  }

  stepper.run();
}
