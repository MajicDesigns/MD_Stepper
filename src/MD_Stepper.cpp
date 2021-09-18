#include <MD_Stepper.h>

#if ENABLE_AUTORUN
#include <avr/io.h>
#include <avr/interrupt.h>
#endif

/**
 * \file
 * \brief Code file for MD_Stepper library class.
 */

MD_Stepper::MD_Stepper(uint8_t inA1, uint8_t inA2, uint8_t inB1, uint8_t inB2)
{
  _in[0] = inA1;
  _in[1] = inB1;   // note this is deliberately ...
  _in[2] = inA2;   // ... swapped with this one!
  _in[3] = inB2;
};

MD_Stepper::~MD_Stepper(void)
// Last one out the door turns out the lights
{
#if ENABLE_AUTORUN
  if (flagChk(_status, S_AUTORUN))
  {
    disableInstance();
    if (_instCount == 0)
    {
      stopTimer();
      detachISR();
    }
  }
#endif
}

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

#if ENABLE_AUTORUN
  // Set up AUTORUN global data and hardware
  if (!_bInitialised)
  {
    PRINTS("\nISR initialising");
    for (uint8_t i = 0; i < MAX_INSTANCE; i++)
      _cbInstance[i] = nullptr;

    _instCount = 0;    // we have no pins globally allocated yet

    setTimerMode();
    setFrequency(ISR_FREQUENCY);
    attachISR();

    _bInitialised = true; // don't do this in any other instance
  }

  if (enableInstance()) flagSet(_status, S_AUTORUN);
#endif
}

#if ENABLE_AUTORUN
bool MD_Stepper::enableInstance(void)
// Enable the autorun on this instance
{
  bool found = false;

  for (uint8_t i = 0; i < MAX_INSTANCE; i++)
  {
    if (_cbInstance[i] == nullptr)
    {
      found = true;
      _cbInstance[i] = this; // save ourselves in this slot
      _instCount++;          // one less pin to allocate
      break;
    }
  }

  return(found);
}

void MD_Stepper::disableInstance(void)
// disable the autorun on this instance
{
  for (uint8_t i = 0; i < MAX_INSTANCE; i++)
  {
    if (_cbInstance[i] == this)
    {
      _cbInstance[i] = nullptr;         // erase ourselves from the slot
      if (_instCount > 0) _instCount--; // one slot is now free
      break;
    }
  }
}
#endif

void MD_Stepper::enableMotorLock(bool b)
{ 
  if (b) flagSet(_options, O_LOCKED); 
  else   flagClr(_options, O_LOCKED); 

  if (b && !(flagChk(_status, S_BUSY)))
    _timeMark = millis();
}

void MD_Stepper::setSpeed(uint16_t s) 
{ 
  if (_speedSet == s) return;

  _speedSet = s;
  if (s == 0) 
    stop();
  else
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
  }
  else if (mode == HALF)  // changing to HALF 
  {
    _stepCount *= 2;
    _moveCountSet *= 2;
    setSpeed(_speedSet * 2);
  }
}

bool MD_Stepper::run(void)
// run the appropriate motion sequence
// called from loop() or within the ISR when ENABLE_AUTORUN
{
  bool b = false;

  if (_runState != IDLE)
  {
    b = runFSM();

    if (b) _stepCount += ((flagChk(_status, S_FWD)) ? 1 : -1);
  }
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
  // requested speed (steps/sec)
{
  if (spd != 0)
    _stepTick = 1000000L / spd;
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
    PRINTS("\n->INIT");
    if ((_speedSet == 0) ||                                  // no speed set doesn't do anything
        (flagChk(_status, S_RUNMOVE) && _moveCountSet == 0)) // move run with no moves set, also don't do anything
    {
      PRINT(" not starting: SpSet[", _speedSet);
      PRINT("] Mv[", flagChk(_status, S_RUNMOVE));
      PRINT("] MvSet[", _moveCountSet);
      PRINTS("]");
      if (_moveCountSet == 0) flagClr(_status, S_RUNMOVE);
      _runState = IDLE;
      PRINTS("\n->to IDLE");
    }
    else
    {
      PRINTS(" starting");
      flagSet(_status, S_BUSY);
      flagClr(_status, S_LOCKED);
      _runState = RUN;
      _timeMark = 0;    // force a move first up
    }
    break;

  case RUN:             // running the motor
    {
      uint32_t now = micros();

      if (now - _timeMark >= _stepTick)
      {
        // time for a step has expired, run the motor
        PRINT("\n->RUN ", now - _timeMark);
        PRINT("/", _stepTick);
        _timeMark = now;
        singleStep();
        if (flagChk(_status, S_RUNMOVE))
        {
          _moveCountSet--;
          if (_moveCountSet == 0) _runState = STOP;
        }
        b = true;
      }
    }
    break;

  case STOP:                // stopping the motor
    PRINTS("\n->STOP");
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
  //PRINT(" SS #", _seqCur);
  for (uint8_t i = 0; i < MAX_PINS; i++)
  {
    digitalWrite(_in[i], (stepPattern & _BV(i)) ? HIGH : LOW);
    //PRINT(" ", (stepPattern & _BV(i)) ? 1 : 0);
  }

  // adjust for next pass - note this exploits that the size of steps is
  // either 4 (0..3) or 8 (0..7) to keep the index in bounds.
  _seqCur = (_seqCur + (flagChk(_status, S_FWD) ? 1 : -1)) & (steps[0]-1);
}


#if ENABLE_AUTORUN
// AUTORUN enabling functions and data (ISR related)

// Global data
bool MD_Stepper::_bInitialised = false;
volatile uint8_t MD_Stepper::_instCount;
MD_Stepper* MD_Stepper::_cbInstance[MAX_INSTANCE];

// Interrupt service routine that for the timer
#if USE_TIMER == 1
ISR(TIMER1_OVF_vect)
#elif USE_TIMER == 2
ISR(TIMER2_OVF_vect)
#endif
{
  if (MD_Stepper::_instCount)      // only do this if there are pins to process
  {
    for (uint8_t i = 0; i < MD_Stepper::MAX_INSTANCE; i++)
      if (MD_Stepper::_cbInstance[i] != nullptr) MD_Stepper::_cbInstance[i]->run();
  }
}

inline void MD_Stepper::setTimerMode(void)
{
#if USE_TIMER == 1
  TCCR1B = _BV(WGM13);
#elif USE_TIMER == 2
  TCCR2B = _BV(WGM22);
#endif
}

void MD_Stepper::setFrequency(uint32_t freq)
// Set the timer to count closest to the required frequency.
{
  uint8_t scale = 0;

  // The counter runs backwards after TOP, interrupt is at BOTTOM -
  // so multiply divide cycles by 2
  uint32_t cycles = F_CPU / (freq * 2);

#if USE_TIMER == 1
  // Work out the prescaler for this number of cycles
  if (cycles < TIMER_RESOLUTION) scale = _BV(CS10);              // prescale /1 (full xtal)
  else if ((cycles >>= 3) < TIMER_RESOLUTION) scale = _BV(CS11);              // prescale /8
  else if ((cycles >>= 3) < TIMER_RESOLUTION) scale = _BV(CS11) | _BV(CS10);  // prescale /64
  else if ((cycles >>= 2) < TIMER_RESOLUTION) scale = _BV(CS12);              // prescale /256
  else if ((cycles >>= 2) < TIMER_RESOLUTION) scale = _BV(CS12) | _BV(CS10);  // prescale /1024
  else     // request was out of bounds, set as maximum
  {
    cycles = TIMER_RESOLUTION - 1;
    scale = _BV(CS12) | _BV(CS10);
  }

  // now set up the counts
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));  // clear prescaler value
  TCCR1B |= scale;
  OCR1A = cycles;     // OCR1A is TOP in phase correct pwm mode for Timer1
  TCNT1 = 0;

#elif USE_TIMER == 2
  // Work out the prescaler for this number of cycles
  if (cycles < TIMER_RESOLUTION) scale = _BV(CS20);              // prescale /1 (full xtal)
  else if ((cycles >>= 3) < TIMER_RESOLUTION) scale = _BV(CS21);              // prescale /8
  else if ((cycles >>= 2) < TIMER_RESOLUTION) scale = _BV(CS21) | _BV(CS20);  // prescale /32
  else if ((cycles >>= 1) < TIMER_RESOLUTION) scale = _BV(CS22);              // prescale /64
  else if ((cycles >>= 1) < TIMER_RESOLUTION) scale = _BV(CS22) | _BV(CS20);  // prescale /128
  else if ((cycles >>= 1) < TIMER_RESOLUTION) scale = _BV(CS22) | _BV(CS21);  // prescale /256 
  else if ((cycles >>= 2) < TIMER_RESOLUTION) scale = _BV(CS22) | _BV(CS21) | _BV(CS20); // prescale by /1024
  else     // request was out of bounds, set as maximum
  {
    cycles = TIMER_RESOLUTION - 1;
    scale = _BV(CS22) | _BV(CS21) | _BV(CS20);
  }

  // now set up the counts
  TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));  // clear prescaler value
  TCCR2B |= scale;
  OCR2A = cycles;     // OCR2A is TOP in phase correct pwm mode for Timer2
  TCNT2 = 0;

#else
#error USE_TIMER not defined correctly  
#endif
}

inline void MD_Stepper::attachISR(void)
// Start the timer and enable interrupt
{
  // set timer overflow interrupt enable bit
#if USE_TIMER == 1
  TIMSK1 = _BV(TOIE1);
#elif USE_TIMER == 2
  TIMSK2 = _BV(TOIE2);
#endif
  sei();                // interrupts globally enabled
}

inline void MD_Stepper::detachISR(void)
// Stop the timer interrupt 
{
  // clears timer overflow interrupt enable bit 
#if USE_TIMER == 1
  TIMSK1 &= ~_BV(TOIE1);
#elif USE_TIMER == 2
  TIMSK2 &= ~_BV(TOIE2);
#endif
}

inline void MD_Stepper::stopTimer(void)
// Stop the timer
{
  // clears all clock selects bits
#if USE_TIMER == 1
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
#elif USE_TIMER == 2
  TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
#endif
}


#endif

