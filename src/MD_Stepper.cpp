#include <MD_Stepper.h>

/**
 * \file
 * \brief Code file for MD_Stepper library class.
 */

MD_Stepper::MD_Stepper(uint8_t inA, uint8_t inB, uint8_t inC, uint8_t inD, uint16_t speedMax)
{
  _in[0] = inA;
  _in[1] = inB;
  _in[2] = inC;
  _in[3] = inD;
  setMaxSpeed(speedMax);
};

void MD_Stepper::begin(uint8_t busyPin)
{
  setBusyPin(busyPin);

  // set step motor pins as OUTPUT
  for (uint8_t i = 0; i < MAX_PINS; i++)
    pinMode(_in[i], OUTPUT);

  // initialise default values
  _status = _options = 0;
  _moveCountSet = _stepCount = 0;
  setStepMode(FULL);
  setDirection(true);
  setMotorLockTime(20);
  enableMotorLock(false);
  stop();
}

void MD_Stepper::enableMotorLock(bool b)
{ 
  if (b) flagSet(_options, O_LOCKED); 
  else   flagClr(_options, O_LOCKED); 

  if (b && !(flagChk(_status, S_BUSY)))
    _timeMark = millis();
}

void MD_Stepper::setMaxSpeed(uint16_t s) 
{ 
  if (s > 0) _speedMax = s; 
  if (_speedSet > s) setSpeed(s); 
}

void MD_Stepper::setSpeed(uint16_t s) 
{ 
  if (_speedSet == s) return;
  if (s > _speedMax) s = _speedMax;

  _speedSet = s;
  calcStepTick(_speedSet);
}

void MD_Stepper::setStepMode(stepMode_t mode)
{
  if (mode == _stepMode)
    return;

  stepMode_t sm = getStepMode();  // remember current mode

  // now record the change and recalculate a full/half step delay
  _stepMode = mode;
  _seqCur = 0;
  calcStepTick(_speedSet);

  // and if we are changing number of steps/cycle also halve 
  // or double counts, speeds, etc.
  if (sm == HALF)         // currently doing HALF
  {
    _stepCount /= 2;
    _moveCountSet /= 2;
    setSpeed(_speedSet / 2);
    _speedMax /= 2;
  }
  else if (mode == HALF)  // changing to HALF 
  {
    _stepCount *= 2;
    _moveCountSet *= 2;
    _speedMax *= 2;
    setSpeed(_speedSet * 2);
  }
}

bool MD_Stepper::run(void)
// run the appropriate motion sequence
{
  bool b = false;

  if (_runState != IDLE) 
    b = runFSM();
  else // not running at all - handle the lock release
  { 
    if (!flagChk(_options, O_LOCKED) && flagChk(_status, S_LOCKED))
    {
      if (millis() - _timeMark >= _timeLockRelease)
      {
        for (uint8_t i = 0; i < MAX_PINS; i++)
          digitalWrite(_in[i], LOW);

        PRINTS("\nLock released");
        flagClr(_status, S_LOCKED);
      }
    }
  }

  if (b) _stepCount += ((flagChk(_status, S_FWD)) ? 1 : -1);

  if (_pinBusy != 0xff)
    digitalWrite(_pinBusy, flagChk(_status, S_BUSY) ? HIGH : LOW);

  return(b);
}

void MD_Stepper::move(int32_t dist)
{
  setDirection(dist > 0);
  _moveCountSet = abs(dist);

  if (dist != 0)
    flagSet(_status, S_RUNMOVE);
  else
    flagClr(_status, S_RUNMOVE);
}

inline void MD_Stepper::calcStepTick(uint16_t spd)
  // Calculate the step delay (microseconds) given the 
  // requested speed (steps/sec) and stepper mode.
{
  if (spd != 0)
    _stepTick = 100000L / spd;
  PRINT("\nus/pulse: ", _stepTick);
}

bool MD_Stepper::runFSM(void)
// return true if a step was executed
{
  bool b = false;

  switch (_runState)
  {
  case IDLE:                // not doing anything at the moment
    break;

  case INIT:                // initialize before the next run
    if ((!flagChk(_status, S_RUNMOVE) && _speedSet == 0) ||  // free run with no speed set, don't do anything
        (flagChk(_status, S_RUNMOVE) && _moveCountSet == 0)) // move run with no moves set, also don't do anything
    {
      flagClr(_status, S_RUNMOVE);
    }
    else
    {
      flagSet(_status, S_BUSY);
      flagClr(_status, S_LOCKED);
      calcStepTick(_speedSet);
      _runState = RUN;
      _timeMark = 0;    // force a move first up
    }
    // deliberately fall through

  case RUN:                 // running the motor
    if (micros() - _timeMark >= _stepTick)
    {
      // time for a step has expired, run the motor
      singleStep();
      if (flagChk(_status, S_RUNMOVE))
      {
        _moveCountSet--;
        if (_moveCountSet == 0) _runState = STOP;
      }
      b = true;
      _timeMark = micros();
    }
    break;

  case STOP:                // stopping the motor
    flagClr(_status, S_BUSY);
    flagClr(_status, S_RUNMOVE);
    flagSet(_status, S_LOCKED);
    _runState = IDLE;      // next stage is to wait until something start up again
    _timeMark = millis();  // for lock release later if option enabled
    break;
  }

  return(b);
}

void MD_Stepper::singleStep(void)
{
  // Step patterns are defined as bits in a nybble, one bit per output pin.
  // - For each nybble the LSB is the state if _in[0], next up _in[1], etc.
  // - The lower nybble in a byte is the even numbered step, the high nybble
  //   the odd numbered step.
  // - Each array of step data contains the number of steps in the first element,
  //   followed by the actual step patterns steps (eg, 8 steps will have 4 bytes
  //   following).
  // 
  static uint8_t stepFull[] = { 4, 0x63, 0x9c };             // 4 full steps: AB-BC-CD-DA pattern
  static uint8_t stepWave[] = { 4, 0x21, 0x84 };             // 4 wave steps: A-B-C-D pattern
  static uint8_t stepHalf[] = { 8, 0x31, 0x62, 0xc4, 0x98 }; // 8 half steps: A-AB-B-BC-C-CD-D-DA pattern
  static uint8_t* table[] = { stepFull, stepWave, stepHalf };

  uint8_t *steps = table[_stepMode];
  uint8_t stepPattern = steps[(_seqCur >> 1) + 1]; // divide by 2 and offet 1 (the first element)

  // retrieve the relevant step pattern
  if (_seqCur & 1) stepPattern >>= 4;      // odd step patterns are in the high nybble

  // now set the relevant outputs
  for (uint8_t i = 0; i < MAX_PINS; i++)
    digitalWrite(_in[i], (stepPattern & _BV(i)) ? HIGH : LOW);

  // adjust for next pass - note this exploits that the size of steps is
  // either 4 (0..3) or 8 (0..7) to keep the index in bounds.
  _seqCur = (_seqCur + (flagChk(_status, S_FWD) ? 1 : -1)) & (steps[0]-1);
}


