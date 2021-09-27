/*
Example sketch for the MD_Stepper library.

Sketch to control the torque tests documented in https://arduinoplusplus.wordpress.com blog.

Uses Serial monitor, 2 tact switches and stepper motor to conduct torque tests.
The motor is attached to digital pins INA1..INB2.
Switches are connected to RUN_PIN and ESTOP_PIN.

Dependencies:
MD_cmdProcessor is available at https://github.com/MajicDesigns/MD_cmdProcessor or the Arduino Library Manager
MD_UISwitch is available at https://github.com/MajicDesigns/MD_UISwitch or the Arduino Library Manager
*/

#include <MD_cmdProcessor.h>
#include <MD_UISwitch.h>
#include <MD_Stepper.h>

// Define hardware pins
const uint8_t INA1 = 4;
const uint8_t INA2 = 5;
const uint8_t INB1 = 6;
const uint8_t INB2 = 7;

const uint8_t RUN_PIN = 2;
const uint8_t ESTOP_PIN = 3;

// Geometry dependent
const bool DIR_DOWN = false;    // weight moving down motor rotation direction
const bool DIR_UP = !DIR_DOWN;  // just the opposite of DOWN

const uint16_t CRAWL_SPEED = 600;   // in steps/sec
const uint16_t DROP_SPEED = 1200;   // in steps/sec

typedef enum { IDLE, RUN_TEST, RUN_UL, RUN_LL, SPAN_LIMITS } runState_t;

// Stack for command queue
class cStack
{
public:
  inline void push(runState_t s)
  {
    if (_index < (STACK_SIZE - 1))
    {
      _index++;
      _data[_index] = s;
    }
  }

  inline runState_t pop(void)
  {
    runState_t s = IDLE;

    if (_index >= 0)
    {
      s = _data[_index];
      _index--;
    }

    return(s);
  }

  inline void clear(void) { _index = -1; }

private:
  static const uint8_t STACK_SIZE = 5;

  runState_t _data[STACK_SIZE];
  int8_t _index = -1;
};

// Global data
cStack stack;
MD_Stepper S(INA1, INA2, INB1, INB2);
MD_UISwitch_Digital swRun(RUN_PIN);
MD_UISwitch_Digital swEStop(ESTOP_PIN);

int32_t limitTop;              // upper home position
runState_t runningMode = IDLE;
static enum { INIT, RUN, MOVE_TO_LL, WAIT_START, RAISE_LOAD } stage = INIT;

#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))

// handler function prototypes
void handlerHelp(char* param);

// handler functions

void handlerS(char* param)
{
  uint16_t s = strtoul(param, nullptr, 0);

  S.setSpeed(s);
  Serial.print(F("\nSpeed: "));
  Serial.print(S.getSpeed());
}

void handlerL(char* param)
{
  Serial.print(F("\nLock: "));
  S.enableMotorLock(!S.getMotorLock());
  Serial.print(S.getMotorLock() ? F("on") : F("off"));
}

void handlerM(char* param)
{
  Serial.print(F("\nSet Mode: "));
  switch (toupper(param[0]))
  {
  case 'H': S.setStepMode(MD_Stepper::HALF);  Serial.print(F("HALF")); break;
  case 'F': S.setStepMode(MD_Stepper::FULL);  Serial.print(F("FULL")); break;
  default:  Serial.print(F("Unknown")); break;
  }
}

void handlerU(char* param) { Serial.print(F("\nUp @ speed (e-Stop)")); S.setDirection(DIR_UP); S.start(); }
void handlerD(char* param) { Serial.print(F("\nDown @ speed (e-Stop)")); S.setDirection(DIR_DOWN); S.start(); }
void handlerT(char* param) { handlerM(param); handlerS(&param[1]);  runningMode = RUN_TEST; }

void handlerP(char* param)
{
  Serial.print(F("\nCurrent Posn: "));
  Serial.print(S.getPosition());
}

void handlerHR(char* param)
{
  S.resetPosition();
  Serial.print(F("\nReset Home"));
  handlerP(nullptr);
}

void handlerH(char* param)
{
  Serial.print(F("\nMove Home"));
  S.moveHome();
}

void handlerLC(char* param) { Serial.print(F("\nLimit Calibration")); stack.push(RUN_UL); runningMode = RUN_LL; }
void handlerLU(char* param) { runningMode = RUN_UL; }
void handlerLL(char* param) { runningMode = RUN_LL; }
void handlerLS(char* param) { runningMode = SPAN_LIMITS; }

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "s",  handlerS,   "v",    "Speed set to v full step/sec", 1 },
  { "l",  handlerL,   "",     "toggle Lock motor", 1 },
  { "u",  handlerU,   "",     "Up @ speed", 1 },
  { "d",  handlerD,   "",     "Down @ speed", 1 },
  { "m",  handlerM,   "t",    "stepper Mode t=Full, Half", 1},
  { "t",  handlerT,   "m v",  "Test cycle in mode m ([F]ull,[H]alf) speed v", 1 },

  { "h",  handlerH,    "",    "move to Home", 2 },
  { "hr", handlerHR,   "",    "Home Reset", 2 },
  { "p",  handlerP,    "",    "Position report", 2 },

  { "lc", handlerLC,   "",    "Limits Calibration (ll then lc) (switch stops)", 3 },
  { "ll", handlerLL,   "",    "Limit Lower [1st] (switch stops)", 3 },
  { "lu", handlerLU,   "",    "Limit Upper [2nd] (switch stops)", 3 },
  { "ls", handlerLS,   "",    "Limits Span (move upper then lower)", 3},

  { "?",  handlerHelp, "",    "Help", 4 },
};

MD_cmdProcessor CP(Serial, cmdTable, ARRAY_SIZE(cmdTable));

void handlerHelp(char* param)
{
  CP.help();
  Serial.print(F("\n\n"));
}

void setup(void) 
{
  Serial.begin(57600);
  Serial.print(F("\n[MD_Stepper Torque Test]\nEnsure lines are terminated by newline\nE-Stop switch any time\n\n== CALIBRATE FIRST =="));

  stack.clear();

  swRun.begin();
  swEStop.begin();

  S.begin();
  S.setStepMode(MD_Stepper::HALF);
  S.setSpeed(CRAWL_SPEED);  // set something as default

  CP.begin();
  handlerHelp(nullptr);
}

void loop(void) 
{
#if !ENABLE_AUTORUN
  S.run();
#endif

  // handle the e-STOP first
  if (swEStop.read() == MD_UISwitch::KEY_PRESS)
  {
    Serial.print(F("\nE_STOP!!"));
    S.stop();
    stack.clear();
    runningMode = IDLE;
    stage = INIT;
  }

  // now handle how we are running this test at the moment
  if (runningMode == IDLE)
    CP.run();
  else
    switch (runningMode)
    {
    case IDLE:
      break;

    case RUN_TEST:
    {
      static uint16_t s;

      switch (stage)
      {
      default:
        s = S.getSpeed();
        S.setSpeed(DROP_SPEED);
        S.moveHome();
        S.start();
        Serial.print(F("\nTest cycle @ ")); Serial.print(s); Serial.print(F(" steps/sec"));
        stage = MOVE_TO_LL;
        break;

      case MOVE_TO_LL:
        if (S.getPosition() == 0)
        {
          S.setSpeed(s);
          Serial.print(F(" - press switch to start"));
          stage = WAIT_START;
        }
        break;

      case WAIT_START:
        if (swRun.read() == MD_UISwitch::KEY_PRESS)
        {
          S.move(limitTop);
          S.start();
          stage = RAISE_LOAD;
        }
        break;

      case RAISE_LOAD:
        if (S.getPosition() == limitTop)
        {
          Serial.print(F(" - end"));
          stage = INIT;
          runningMode = stack.pop();
        }
        break;
      }
    }
    break;

    case RUN_UL:
    {
      switch (stage)
      {
      default:
        Serial.print(F("\nUpper Limit - press switch to set"));  
        S.setDirection(DIR_UP);
        S.setSpeed(CRAWL_SPEED);
        S.start();
        stage = RUN;
        break;

      case RUN:
        if (swRun.read() == MD_UISwitch::KEY_PRESS)
        {
          S.stop();
          limitTop = S.getPosition();
          stage = INIT;
          runningMode = stack.pop();
        }
        break;
      }
    }
    break;

    case RUN_LL:
    {
      switch (stage)
      {
      default:
        Serial.print(F("\nLower Limit - press switch to set"));  
        S.setDirection(DIR_DOWN);
        S.setSpeed(CRAWL_SPEED);
        S.start();
        stage = RUN;
        break;

      case RUN:
        if (swRun.read() == MD_UISwitch::KEY_PRESS)
        {
          S.stop();
          S.resetPosition();
          stage = INIT;
          runningMode = stack.pop();
        }
        break;
      }
    }
    break;

    case SPAN_LIMITS:
    {
      static enum { INIT, TO_UP, TO_DOWN } stage = INIT;

      switch (stage)
      {
      case INIT:
        Serial.print(F("\nSpan Limit up"));   
        S.setDirection(DIR_UP);
        S.setSpeed(DROP_SPEED);
        S.start();
        stage = TO_UP;
        break;

      case TO_UP:
        if (S.getPosition() >= limitTop)
        {
          S.stop();
          S.moveHome();
          S.start();
          stage = TO_DOWN;
          Serial.print(F(" - down"));
        }
        break;

      case TO_DOWN:
        if (S.getPosition() == 0)
        {
          Serial.print(F(" - end"));
          stage = INIT;
          runningMode = stack.pop();
        }
        break;
      }
    }
  }
}


