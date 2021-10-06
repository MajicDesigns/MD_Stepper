/*
 Example sketch for the MD_Stepper library.

 Tests all the calls for the library by allowing
 them to be invoked through a command line interface
 with input and output to the serial monitor.

 Dependencies:
 MD_cmdProcessor library found at http://github.com/MajicDesigns/MD_cmdProcessor 
            or the IDE library manager.
*/

#include <MD_cmdProcessor.h>
#include <MD_Stepper.h>

const uint8_t INA1 = 2;
const uint8_t INA2 = 3;
const uint8_t INB1 = 4;
const uint8_t INB2 = 5;

const uint8_t PIN_BUSY_SIGNAL = 255;

MD_Stepper S(INA1, INA2, INB1, INB2);

#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))

// handler function prototypes
void handlerHelp(char* param);

char* mode2sz(void)
{
  static const PROGMEM char SZ_FULL[] = "FULL";
  static const PROGMEM char SZ_WAVE[] = "WAVE";
  static const PROGMEM char SZ_HALF[] = "HALF";
  static char sz[8];

  switch (S.getStepMode())
  {
  case MD_Stepper::FULL: strcpy_P(sz, SZ_FULL); break;
  case MD_Stepper::WAVE: strcpy_P(sz, SZ_WAVE); break;
  case MD_Stepper::HALF: strcpy_P(sz, SZ_HALF); break;
  }

  return(sz);
}

// handler functions
void handlerX(char* param)
{
  switch (param[0])
  {
  case '1': S.start(); Serial.print(F("\nStart")); break;
  case '0': S.stop();  Serial.print(F("\nStop"));  break;
  }
}

void handlerS(char* param)
{
  uint16_t s = strtoul(param, nullptr, 0);

  S.setSpeed(s);
  Serial.print(F("\nSpeed: "));
  Serial.print(S.getSpeed());
}

void handlerM(char* param)
{
  int16_t d = strtol(param, nullptr, 0);

  S.move(d);
  Serial.print(F("\nMove: "));
  Serial.print(S.moveToGo());
}

void handlerMP(char* param)
{
  Serial.print(F("\nTo Go: "));
  Serial.print(S.moveToGo());
}

void handlerMH(char* param)
{
  S.moveHome();
  Serial.print(F("\nMove Home"));
}

void handlerD(char* param)
{
  bool b = (param[0] == '1');

  S.setDirection(b);
  Serial.print(F("\nDirection: "));
  Serial.print(S.isForward() ? 'F' : 'R');
}

void handlerMO(char* param)
{
  switch (toupper(param[0]))
  {
  case 'F': S.setStepMode(MD_Stepper::FULL); break;
  case 'W': S.setStepMode(MD_Stepper::WAVE); break;
  case 'H': S.setStepMode(MD_Stepper::HALF); break;
  }

  Serial.print(F("\nStepper Mode: "));
  Serial.print(mode2sz());
}

void handlerLE(char* param)
{
  bool b = (param[0] == '1');

  S.enableMotorLock(b);
  Serial.print(F("\nLock: "));
  Serial.print(S.getMotorLock() ? F("on") : F("off"));
}

void handlerLT(char* param)
{
  uint8_t s = strtoul(param, nullptr, 0);

  S.setMotorLockTime(s);
  Serial.print(F("\nLock Time: "));
  Serial.print(S.getMotorLockTime());
}

void handlerP(char* param)
{
  if (param[0] == '0')
    S.resetPosition();
  
  Serial.print(F("\nPosn: "));
  Serial.print(S.getPosition());
}

void handlerG(char* param)
{
  Serial.print(F("\n\n== Status"));
  Serial.print(F("\nRaw Status: "));
  {
    uint8_t rs = S.getRawStatus();

    for (int8_t i = 7; i >= 0; i--)
      Serial.print((rs >> i) & 1);
  }
  Serial.print(F("\nSpeed: "));
  Serial.print(S.getSpeed());
  Serial.print(F("\nPosition: "));
  Serial.print(S.getPosition());
  Serial.print(F("\nDirection: "));
  Serial.print(S.isForward() ? 'F' : 'R');
  Serial.print(F("\nStep Mode: "));
  Serial.print(mode2sz());
  Serial.print(F("\n\n== Options"));
  Serial.print(F("\nMotor Lock: "));
  Serial.print(S.getMotorLock() ? F("on") : F("off"));
  Serial.print(F("\nLock Time: "));
  Serial.print(S.getMotorLockTime());
  Serial.print(F("\nMotor Locked: "));
  Serial.print(S.isMotorLocked() ? F("yes") : F("no"));
  Serial.print(F("\n\n"));
}

const MD_cmdProcessor::cmdItem_t PROGMEM cmdTable[] =
{
  { "?",  handlerHelp, "",    "Help", 0 },
  { "h",  handlerHelp, "",    "Help", 0 },

  { "s",  handlerS,    "v",   "Speed set to v full step/sec", 2 },
  { "m",  handlerM,   "d",    "Move +/- d full steps", 2 },
  { "mh", handlerMH,  "",     "Move Home", 2 },
  { "mp", handlerMP,   "",    "display Move Position to-go", 2 },

  { "le", handlerLE,   "n",   "motor Lock enable (n=1) or disable (n=0)", 3 },
  { "lt", handlerLT,   "t",   "motor Lock release Time (t=0..255) 0.1 sec units", 3 },

  { "d",  handlerD,    "n",   "Direction fwd (n=1) or rev (n=0)", 4 },
  { "g",  handlerG,    "",    "Get all values", 4 },
  { "mo", handlerMO,   "x",   "stepping MOde [f=Full, w=Wave, h=Half]", 4 },
  { "p",  handlerP,    "[0]", "Position report (blank) or reset (0)", 4 },
  { "r",  handlerX,    "[n]", "Run start (1) or stop (0)", 4 },
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
  Serial.print(F("\n[MD_Stepper Test]\nEnsure lines are terminated by newline\n"));

  S.begin(PIN_BUSY_SIGNAL);
  CP.begin();
  handlerHelp(nullptr);
}

void loop(void)
{
  S.run();
  CP.run();
}