#include <MD_Stepper.h>

#if ENABLE_AUTORUN
#include <avr/io.h>
#include <avr/interrupt.h>
#endif

/**
 * \file
 * \brief Code file for MD_Stepper library class.
 */

 // Step patterns data is defined as bits in a nybble, one bit per output pin.
 // - For each nybble the LSB is the state if _in[0], next up _in[1], etc.
 // - The lower nybble in a byte is the even numbered step, the high nybble
 //   the odd numbered step.
 // - Each array of step data containg 4 bytes. The shorter sequqnces are repeated
 //   to fill out to the longest (HALF wave) siuze of table.
 // 
const uint8_t WAVETABLE_SIZE = 4;
const PROGMEM uint8_t MD_Stepper::stepFull[WAVETABLE_SIZE] = { 0x63, 0x9c, 0x63, 0x9c }; // 4 full steps: AB-BC-CD-DA pattern, repeated
const PROGMEM uint8_t MD_Stepper::stepWave[WAVETABLE_SIZE] = { 0x21, 0x84, 0x21, 0x84 }; // 4 wave steps: A-B-C-D pattern, repeated
const PROGMEM uint8_t MD_Stepper::stepHalf[WAVETABLE_SIZE] = { 0x31, 0x62, 0xc4, 0x98 }; // 8 half steps: A-AB-B-BC-C-CD-D-DA pattern

MD_Stepper::MD_Stepper(uint8_t inA1, uint8_t inA2, uint8_t inB1, uint8_t inB2)
{
  _pin[0].id = inA1;
  _pin[1].id = inB1;   // note this is deliberately ...
  _pin[2].id = inA2;   // ... swapped with this one!
  _pin[3].id = inB2;
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

  // set step motor pins as OUTPUT and get the port/bit
  for (uint8_t i = 0; i < MAX_PINS; i++)
  {
    pinMode(_pin[i].id, OUTPUT);
    _pin[i].reg = portOutputRegister(digitalPinToPort(_pin[i].id)); // output pin register
    _pin[i].mask = digitalPinToBitMask(_pin[i].id);                 // mask for pin in that register
  }

  // initialise default values
  _status = _options = 0;
  _moveCountSet = _stepCount = 0;
  setStepMode(HALF);
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
  stepMode_t sm = getStepMode();  // remember current mode

  // now record the change and recalculate a full/half step delay
#if ENABLE_AUTORUN
  // disable interrupts as we are changing the table references 
  // that are used by the interrupt driven stepping.
  noInterrupts();
#endif
  _stepMode = mode;
  _stepCur = 0;
  switch (mode)
  {
  case WAVE: _stepTable = stepWave; break;
  case FULL: _stepTable = stepFull; break;
  case HALF: _stepTable = stepHalf; break;
  }
  calcStepTick(_speedSet);

  // and if we are changing number of steps/cycle also halve 
  // or double counts, speeds, etc.
  if (sm != mode)           // we are actually making a change
  {
    if (sm == HALF)         // currently doing HALF
    {
      _stepCount /= 2;
      _moveCountSet /= 2;
      setSpeed(_speedSet / 2);
    }
    else if (mode == HALF)       // changing to HALF 
    {
      _stepCount *= 2;
      _moveCountSet *= 2;
      setSpeed(_speedSet * 2);
    }
  }
#if ENABLE_AUTORUN
  interrupts();
#endif
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

void MD_Stepper::run(void)
// return true if a step was executed
{
  switch (_runState)
  {
  case STP_IDLE:                // not doing anything at the moment
    if (!flagChk(_options, O_LOCKED) && flagChk(_status, S_LOCKED))
    {
      if (millis() - _timeMark >= _timeLockRelease)
      {
        for (uint8_t i = 0; i < MAX_PINS; i++)
          digitalWrite(_pin[i].id, LOW);

        PRINTS("\nLock released");
        flagClr(_status, S_LOCKED);
      }
    }
    break;

  case STP_INIT:                // initialize before the next run
    PRINTS("\n->INIT");
    if ((_speedSet == 0) ||                                  // no speed set doesn't do anything
        (flagChk(_status, S_RUNMOVE) && _moveCountSet == 0)) // move run with no moves set, also don't do anything
    {
      PRINT(" not starting: SpSet[", _speedSet);
      PRINT("] Mv[", flagChk(_status, S_RUNMOVE));
      PRINT("] MvSet[", _moveCountSet);
      PRINTS("]");
      if (_moveCountSet == 0) flagClr(_status, S_RUNMOVE);
      _runState = STP_IDLE;
      PRINTS("\n->to IDLE");
    }
    else
    {
      PRINTS(" starting to RUN");
      flagSet(_status, S_BUSY);
      flagClr(_status, S_LOCKED);
      _runState = STP_RUN;
      _timeMark = 0;    // force a move first up
    }
    break;

  case STP_RUN:                 // running the motor
#if !ENABLE_AUTORUN
    runStepISR();
#endif
    break;

  case STP_STOP:                // stopping the motor
    PRINTS("\n->STOP!!");
    flagClr(_status, S_BUSY);
    flagClr(_status, S_RUNMOVE);
    flagSet(_status, S_LOCKED);
    _runState = STP_IDLE;      // next stage is to wait until something start up again
    _timeMark = millis();      // for lock release later if option enabled
    break;
  }

  // write the current busy status to output pin if it is defined
  if (_pinBusy != 0xff)
    digitalWrite(_pinBusy, flagChk(_status, S_BUSY) ? HIGH : LOW);
}

void MD_Stepper::runStepISR(void)
// ISR run to step the motors
{
  if (_runState != STP_RUN)
    return;

  uint32_t now = micros();

  if (now - _timeMark >= _stepTick)     // time for a step has expired, step the motor
  {
    // now step the motor
    uint8_t stepPattern = pgm_read_byte(_stepTable + (_stepCur >> 1));  // divide by 2

    // retrieve the relevant step pattern
    // note: odd step patterns are in the high nybble
    if (_stepCur & 1) stepPattern >>= 4;

    // now set the relevant outputs
    //PRINT("\n S", _seqCur);
    for (uint8_t i = 0; i < MAX_PINS; i++)
    {
      //digitalWrite(_pin[i].id, (stepPattern & _BV(i)) ? HIGH : LOW);
      //PRINT(" ", (stepPattern & _BV(i)) ? 1 : 0);
      if (stepPattern & _BV(i))
        *_pin[i].reg |= _pin[i].mask;   // HIGH
      else
        *_pin[i].reg &= ~_pin[i].mask;  // LOW
    }

    // adjust for next pass - note this exploits that the size of steps tables 
    // are all 8 steps long ([0..3,0..3] or [0..7] sequence elements) to keep the 
    // index in bounds.
    _stepCur = (_stepCur + (flagChk(_status, S_FWD) ? 1 : -1)) & 7;
    _timeMark = now;                    // save now for next time

    // increment the step count
    _stepCount += ((flagChk(_status, S_FWD)) ? 1 : -1);

    // if moving only a set number of steps, check this here
    if (flagChk(_status, S_RUNMOVE))
    {
      _moveCountSet--;
      if (_moveCountSet == 0)
      {
        PRINTS("\n-->MOVE END");
        _runState = STP_STOP;
      }
    }
  }
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
      if (MD_Stepper::_cbInstance[i] != nullptr) MD_Stepper::_cbInstance[i]->runStepISR();
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

