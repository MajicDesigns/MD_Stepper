/*
 Example sketch for the MD_Stepper library.

 Control the stepper speed using an analog pot.

 Example program to drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins INA1..INB2.
 A potentiometer is connected to POT_PIN.

 The higher the potentiometer value, the faster the motor speed.
 */

#include <MD_Stepper.h>

// Define hardware pins
const uint8_t INA1 = 4;
const uint8_t INA2 = 5;
const uint8_t INB1 = 6;
const uint8_t INB2 = 7;
const uint8_t POT_PIN = A0;

MD_Stepper S(INA1, INA2, INB1, INB2);

void setup(void) 
{
  S.begin();
  S.setStepMode(MD_Stepper::HALF);
}

void loop(void) 
{
  uint16_t pot = analogRead(POT_PIN);
  uint16_t speed = map(pot, 0, 1023, 0, 2000);

  S.setSpeed(speed);
  if (!S.isBusy()) S.start();
  S.run();
}


