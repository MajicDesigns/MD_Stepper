/*
 Example sketch for the MD_Stepper library.

 Control the stepper position using an analog pot.

 Example program to drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins INA1..INB2.
 A potentiometer is connected to POT_PIN.

 The motor position follows the pot position.
 */

#include <MD_Stepper.h>

// Define hardware pins
const uint8_t INA1 = 4;
const uint8_t INA2 = 5;
const uint8_t INB1 = 6;
const uint8_t INB2 = 7;
const uint8_t POT_PIN = A0;

const uint16_t STEPS_PER_REV = 200;  // number of steps for one motor revolution

MD_Stepper S(INA1, INA2, INB1, INB2);

// the previous reading from the analog input
int previous = 0;

void setup(void) 
{
  S.begin();
  S.setSpeed(300);
  S.resetPosition();
}

void loop(void) 
{
  uint16_t pot = analogRead(POT_PIN);
  uint16_t placeCur = map(pot, 0, 1023, 0, STEPS_PER_REV);

  // if we have stopped last move, move a number steps equal to the change in position
  if (S.moveToGo() == 0)
  {
    int16_t moves = placeCur - S.getPosition();

    if (moves != 0)
    {
      S.move(placeCur - S.getPosition());
      S.start();
    }
  }
#if !ENABLE_AUTORUN
  S.run();
#endif
}