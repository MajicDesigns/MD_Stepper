/*
 Example sketch for the MD_Stepper library.

 Control 2 steppers speed using an analog pot.

 Example program to drives a unipolar or bipolar stepper motor.
 The stepper motors are attached to digital pins AINA1..AINB2, BINA1..BINB2.
 Potentiometers are connected to POT_PINA and POT_PINB.

 The higher the potentiometer value, the faster the motor speed.
 */

#include <MD_Stepper.h>

// Define hardware pins
const uint8_t AINA1 = 4;
const uint8_t AINA2 = 5;
const uint8_t AINB1 = 6;
const uint8_t AINB2 = 7;
const uint8_t POT_PINA = A0;

const uint8_t BINA1 = 8;
const uint8_t BINA2 = 9;
const uint8_t BINB1 = 10;
const uint8_t BINB2 = 11;
const uint8_t POT_PINB = A1;

MD_Stepper SA(AINA1, AINA2, AINB1, AINB2);
MD_Stepper SB(BINA1, BINA2, BINB1, BINB2);

const uint16_t MAX_SPEED = 1500;   // steps/sec

void setup(void) 
{
  SA.begin();
  SA.setStepMode(MD_Stepper::HALF);
  SB.begin();
  SB.setStepMode(MD_Stepper::HALF);
}

void loop(void) 
{
  uint16_t pot, speed;
  
  pot = analogRead(POT_PINA);
  speed = map(pot, 0, 1023, 0, MAX_SPEED);
  SA.setSpeed(speed);
  if (!SA.isBusy()) SA.start();

  pot = analogRead(POT_PINB);
  speed = map(pot, 0, 1023, 0, MAX_SPEED);
  SB.setSpeed(speed);
  if (!SB.isBusy()) SB.start();

#if !ENABLE_AUTORUN
  SA.run();
  SB.run();
#endif
}


