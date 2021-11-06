#pragma once
/**
\mainpage Stepper motor control library

This library provides services to control unipolar and bipolar stepper 
motors through appropriate intermediate hardware drivers. It is intended
for control of motors used in robotic applications running at modest 
step frequencies (ie up to low KHz rates).

Motion can be enabled in a polled mode for generic architectures or 
from an interrupt timer for AVR architectures, using either AVR 
timer 1 or 2 (selectable at compile time).

## Motor Connections

The motor control interface is 4 digital output pins specified in the 
class constructor. These pins are connected to external motor control
hardware to provide the appropriate H-bridge connections for control
bipolar motors) or current switching (unipolar motors).

The pins are labeled in pairs A1, A2, B1, B2 for the 2 motor coils
as shown in the figure below.

![Motor Connection Diagram] (Motor_Connections.png "Motor Connections")

### Using 28BYJ-48 with ULN2003 driver

![28BYJ-48] (Motor_28BYJ-48.png "28BYJ-48 Connections")

The stepper motor is a unipolar stepper with coil wire pairs BLU/YLW, 
ORN/PNK and common RED. For the software the I/O pin wiring is 
INA1 to IND, INA2/INB, INB1/INC, and INB2/INA. This motor can be 
modified be a bipolar motor - 
https://ardufocus.com/howto/28byj-48-bipolar-hw-mod/ (verified works).

## Important Notes
- By enabling ENABLE_AUTORUN, this library is limited to AVR
architectures and motor stepping functions will be invoked by a 
timer ISR, effectively driving the motor stepping more consistently 
and as a background process.

- With ENABLE_AUTORUN enabled, this library uses AVR TIMER1 or TIMER2
to implement interrupt driven clock timing. TIMER0 is used by the Arduino 
millis() clock, TIMER1 is commonly used by the Servo library and TIMER2 
by the Tone library. Change USE_TIMER (defined at the top of the header 
file) to select which timer is enabled in the library code.

- This library has been tested on Arduino Uno and Nano (ie, 328P processor).

- TIMERn is a global resource, so each concurrent class instance is driven 
from the same TIMERn interrupt. The constant MAX_INSTANCE is used to limit
the global maximum for instances allowed to be processed by the same interrupt.

- ISR_FREQUENCY is used to set the TIMERn ISR frequency. This affects the 
max number of steps/s to drive the motor. Higher frequency means more interrupts 
and less time to do other things (including other interrupts).

See Also
- \subpage pageRevisionHistory
- \subpage pageDonation
- \subpage pageCopyright

\page pageDonation Support the Library
If you like and use this library please consider making a small donation
using [PayPal](https://paypal.me/MajicDesigns/4USD)

\page pageCopyright Copyright
Copyright (C) 2021 Marco Colli. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

\page pageRevisionHistory Revision History
Nov 2021 ver 1.1.1
- Fixed rare problem with move() and drive() modes confused

Oct 2021 ver 1.1.0
- Changed ISR handling for memory and code efficiency
- Implemented direct I/O for speed

Sep 2021 ver 1.0.2
- Standardized status field methods
- Added PositionControlDual, TorqueTest examples
- Added setVelocity() - speed & direction

Sep 2021 ver 1.0.1
- Fixed small issues, eliminated maxSpeed concepts
- Clearer documentation for required connections
- Added SpeedContol and PositionControl examples

Sep 2021 ver 1.0.0
- Initial release
*/

#include <Arduino.h>

/**
 * \file
 * \brief Main header file and class definition for the MD_Stepper library.
 */

#ifndef ENABLE_AUTORUN
#define ENABLE_AUTORUN 0    ///< enable/disable autorun mode using TIMER ISR
#endif
#if ENABLE_AUTORUN
#define USE_TIMER 1         ///< Set to use hardware TIMER1 or TIMER2 (1 or 2)
#endif

#ifndef STPDEBUG
#define STPDEBUG 0         ///< turns library debug output on/off (AUTORUN should be off if DEBUG is on)
#endif

#if STPDEBUG
#define PRINTFSM(s)  do { Serial.print('\n'); Serial.print(_pin[0].id); Serial.print(F("=>")); Serial.print(F(s)); } while (false)
#define PRINTS(s)    Serial.print(F(s))
#define PRINT(s,v)   do { Serial.print(F(s)); Serial.print(v); } while (false)
#define PRINTX(s,v)  do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false)
#define PRINTB(s,v)  do { Serial.print(F(s)); Serial.print(F("0b")); Serial.print(v, BIN); } while (false)
#else
#define PRINTFSM(s)
#define PRINTS(s)
#define PRINT(s,v)
#define PRINTX(s,v)
#define PRINTB(s,v)
#endif

/**
 * Core object for the MD_PWM library
 */
class MD_Stepper
{
public:
 //--------------------------------------------------------------
 /** \name Structures, Enumerated Types and Constants.
  * @{
  */
 /**
  * Enumerated type for Stepper motor motion profile
  *
  * Specifies the sequence for powering the motor windings 
  * during rotation.
  */
  typedef enum 
  { 
    FULL = 0,   ///< Full step pattern. 4 full steps: AB-BC-CD-DA
    WAVE = 1,   ///< Wave pattern. 4 wave steps: A-B-C-D
    HALF = 2    ///< Half step pattern. 8 half steps: A-AB-B-BC-C-CD-D-DA 
  } stepMode_t;

  /** @} */

 //--------------------------------------------------------------
 /** \name Class constructor and destructor.
  * @{
  */
 /**
  * Class Constructor
  *
  * Instantiate a new instance of the class.
  *
  * The main function for the core object is to set the internal
  * setup Parameters and other default values.
  * 
  * The pins are passed as one pair for coil A and one pair for coil B.
  *
  * \param inA1 pin connected to hardware driver Coil A one side.
  * \param inA2 pin connected to hardware driver Coil A other side.
  * \param inB1 pin connected to hardware driver Coil B one side.
  * \param inB2 pin connected to hardware driver Coil B other side.
  */
  MD_Stepper(uint8_t inA1, uint8_t inA2, uint8_t inB1, uint8_t inB2);

 /**
  * Class Destructor.
  *
  * Release any allocated memory and clean up anything else.
  *
  * If all the instances of the class are closed, then the ISR is
  * disconnected and the timer is stopped.
  */
  ~MD_Stepper(void);

  /** @} */

 //--------------------------------------------------------------
 /** \name Methods for core object control.
  * @{
  */
 /**
  * Initialize the object.
  *
  * Initialize the object data. This needs to be called during setup()
  * to set items that cannot be done during object creation.
  *
  * If this is the first instance of this class, then the ISR TIMER code
  * is initialized and the frequency of the hardware timer is set.
  * Subsequent instances do not affect the timer frequency.
  * 
  * The busy pin can be specified as an option parameter.
  *
  * \sa setBusyPin()
  *
  * \param busyPin the optional busy pin.
  */
  void begin(uint8_t busyPin = 0xff);

  /**
   * Start movement.
   *
   * Start the motor moving using the currently configured motion parameters.
   * 
   * If a move() was specified, this will take run priority until it concludes.
   * If no move() was specified or it has completed (ie, the move distance is 
   * 0) then free running is started until stop() is invoked.
   * 
   * \sa setSpeed(), move(), run(), stop()
   */
  inline void start(void) { _runState = STP_INIT; }

  /**
   * Stop movement.
   *
   * Stop the motor moving immediately. This stops free running motion and will 
   * immediately terminate motion set up using move().
   *
   * \sa move(), run(), start()
   */
  inline void stop(void) { _runState = STP_STOP; }

  /**
   * Run motion.
   *
   * This needs to be called every iteration through loop() to run all 
   * the required Stepper management functions.
   * 
   * Note that if AUTORUN is not enabled, this will also step the motors.
   * If AUTORUN is enabled, the motors will be stepped through a timer interrupt
   * and managed via this call.
   * 
   * \sa start(), stop()
   * 
   * \return true if a step was taken during the method invocation.
   */
  void run(void);

  /** @} */

 //--------------------------------------------------------------
 /** \name Methods for motion parameter management.
  * @{
  */

 /**
  * Set the current motor direction.
  *
  * Set the current rotation direction for the motor. By convention the 
  * forward direction is 'true' and the reverse is 'false'.
  *
  * Note that forward and reverse have no inherent meaning other than they
  * are in opposite rotational directions.
  *
  * \sa getDirection()
  *
  * \param fwd sets the motor to 'forward' if true, 'reverse' if false
  */
  inline void setDirection(bool fwd) { if (fwd) flagSet(_status, S_FWD); else flagClr(_status, S_FWD); }

 /**
  * Reset the current step counter.
  *
  * The library counts the number of steps it moves (+1 for forward motion, -1
  * for reverse motion) into a cumulative counter. This method sets the current 
  * step count to zero, setting a new 'home' position.
  *
  * \sa getPosition()
  */
  inline void resetPosition(void) { _stepCount = 0; }

 /**
  * Gets the current step counter.
  *
  * The library counts the number of steps it moves (+1 for forward motion, -1
  * for reverse motion) into a cumulative counter. This method returns the current
  * step count.
  *
  * The number of steps proportional to the current motor running mode (ie, HALF
  * stepping creates double the number of steps as FULL or WAVE stepping to for the 
  * same physical distance).
  *
  * \sa resetPosition()
  * 
  * \return the current step position counter
  */
  inline int32_t getPosition(void)  { return(_stepCount); }

 /**
  * Gets the current speed setting.
  *
  * \sa setSpeed(), setMaxSpeed()
  *
  * \return the running speed in steps/sec
  */
  inline uint16_t getSpeed(void) { return(_speedSet); }

 /**
  * Set the running speed.
  *
  * The motor speed can be set to any value but speeds higher than 
  * the motor maximum attainable speed will result in it stalling. 
  * Setting the motor speed to 0 has the same effect as a stop().
  * 
  * The new speed takes effect at the next motor step (immediately if running 
  * or at the next start()).
  * 
  * As the speed is specified in steps/sec, the current motor stepping mode 
  * has an effect. HALF stepping results in half the physical speed of a FULL 
  * or WAVE stepping for the same numeric setting, so when the mode is changed
  * all items mesaured in steps or steps per unit time are adjusted 
  * appropriately.
  *
  * \sa getSpeed(), setMode()
  *
  * \param s the required running speed in steps/sec
  */
  void setSpeed(uint16_t s);

/**
  * Set the velocity (speed and direction).
  *
  * Sets the new velocity. positive velocity is forward movement, negative
  * velocity reverse movement. This is a wrapper for setDirection() and 
  * setSpeed().
  *
  * As velocity is specified in steps/sec.
  *
  * \sa setSpeed(), setDirection()
  *
  * \param v the required velocity in steps/sec
  */
  void setVelocity(int16_t v) { setDirection(v > 0); if (v < 0) v = -v;  setSpeed(v); }

 /**
  * Set the number of steps to move.
  *
  * The motor can be set to move a specific number of steps and then stop. This
  * method sets up the number of steps for the next start(). 
  *
  * If the number of steps is positive the motion is in the forward direction; 
  * negative for reverse motion. Any change in direction cause by move() remains 
  * in effect at the end of the motor movement.
  *
  * To cancel a move() set the number of steps to 0.
  * 
  * The move is specified as the number of steps in the current motor mode (ie, HALF
  * stepping requires double the number of steps as FULL or WAVE stepping to move the 
  * same physical distance).
  *
  * \sa setSpeed(), moveToGo()
  *
  * \param dist the number of steps (distance) required running speed in steps/sec
  */
  void move(int32_t dist);

 /**
  * Set the number of steps to move.
  *
  * The motor can be set to move a specific number of steps and then stop. This
  * method sets up the number of steps for the next start().
  *
  * If the number of steps is positive the motion is in the forward direction;
  * negative for reverse motion. Any change in direction cause by move() remains
  * in effect at the end of the motor movement.
  *
  * To cancel a move() set the number of steps to 0.
  *
  * The move is specified as the number of steps in the current motor mode (ie, HALF
  * stepping requires double the number of steps as FULL or WAVE stepping to move the
  * same physical distance).
  *
  * \sa setSpeed(), moveToGo()
  *
  * \return the number of steps required running speed in steps/sec
  */
  inline uint32_t moveToGo(void) { return(_moveCountSet); }

 /**
  * Move back the home position.
  *
  * The motor is set up to move back to its home position, defined as position with 
  * step count 0. It is equivalent to move(-getPosition()). 
  *
  * \sa move(), moveToGo()
  */
  inline void moveHome(void) { move(-getPosition()); }

  /** @} */

 //--------------------------------------------------------------
 /** \name Methods for status and hardware options management.
  * @{
  */
 /**
  * Check if motor is running.
  *
  * Check if motors are currently running. This method is useful to check when
  * move() has completed their motions. The output on the 'busy pin', if enabled,
  * is the same as this status.
  *
  * \sa begin(), setBusyPin()
  *
  * \return true if the motor is running
  */
  inline bool isBusy(void) { return(flagChk(_status, S_BUSY)); }

  /**
   * Check if autorun is enabled.
   *
   * Check if autorun is enabled. This required the compile time switch to be 
   * set and the autorun initialization in begin() to have succeeded. If autorun 
   * the run() method will be automatically invoked on an interrupt timer and does
   * not need to be called from loop().
   *
   * \sa \ref ENABLE_AUTORUN
   *
   * \return true if the autorun is enabled
   */
  inline bool isAutoRun(void) { return(flagChk(_status, S_AUTORUN)); }
  
 /**
  * Check if motor direction is forward.
  *
  * Returns true if the current motor direction is forward. By convention the forward
  * direction is 'true' and the reverse is 'false'.
  * 
  * Note that forward and reverse have no inherent meaning other than they 
  * are in opposite rotational directions.
  * 
  * \sa setDirection()
  * 
  * \return true if the motor is moving in the 'forward' direction, false otherwise
  */
  inline bool isForward(void)   { return(flagChk(_status, S_FWD)); }

 /**
  * Check if motor is currently locked.
  *
  * Returns true if current condition of the motors is stopped and motor locked.
  * 
  * \sa setMotorLock(), getMotorLock()
  * 
  * \return true if the motor is currently locked, false otherwise
  */
  inline bool isMotorLocked(void)   { return(flagChk(_status, S_LOCKED) && !isBusy()); }

 /**
  * Check if next/current motion is a move.
  *
  * Returns true if the motion control is set for or currently executing a move() and
  * not free running. If the move is currently happening then isBusy() will also be true.
  * 
  * \sa isBusy(), move()
  * 
  * \return true if the motion is move and not free running.
  */
  inline bool isMoveEnabled(void)   { return(flagChk(_status, S_RUNMOVE)); }

  /**
  * Set pin for hardware busy status.
  *
  * If this pin is specified, it will mirror the status returned by isBusy() at the
  * output pin. The output is set HIGH when the motor is running.
  *
  * As an alternative the pin can also be specified in the begin() method.
  *
  * Set 0xff (255) to disable this pin (the default).
  *
  * \sa isBusy()
  *
  * \param pin the pin number to use (0xff to disable)
  */
  inline void setBusyPin(uint8_t pin) { _pinBusy = pin; if (_pinBusy != 0xff) pinMode(_pinBusy, OUTPUT); }

 /**
  * Get the current motor stepping mode.
  *
  * Returns the current motor stepping mode as one of the stepMode_t enumerated values.
  *
  * \sa setStepMode()
  *
  * \return the current motor stepping mode
  */
  inline stepMode_t getStepMode(void) { return(_stepMode); }

 /**
  * Get the current motor stepping mode.
  *
  * Sets the new motor stepping mode to one of the stepMode_t enumerated values.
  * 
  * The change from a full step mode (FULL, WAVE) to HALF step mode, and vice versa,
  * creates a doubling/halving adjustment to current counts and speeds that are defined
  * so that they remain consistent with the new steps generated.
  * 
  * The library default stepping mode is HALF.
  *
  * \sa getStepMode(), \ref stepMode_t
  *
  * \param mode the new motor stepping mode
  */
  void setStepMode(stepMode_t mode);

 /**
  * Get the current motor lock setting.
  *
  * Returns the currently configured motor lock setting.
  *
  * \sa enableMotorLock(), setMotorLockTime()
  *
  * \return true if the motor remained lock when stopped
  */
  inline bool getMotorLock(void) { return(flagChk(_options, O_LOCKED)); }

 /**
  * Enable the motor lock setting.
  *
  * When the motor is not being moved, the coils can be left energized 
  * for maximum holding torque or turned off to reduce power consumption 
  * and motor heating. This method changes the setting between locked 
  * (true) and unlocked (false).
  *
  * If set to unlock, when the motor stops it remains locked for the time 
  * specified by setMotorLockTime() before the motor coils are released.
  * 
  * If the motor is set to unlock while it is not running, it will be 
  * unlocked after the lock time expires. Locking while stopped will lock 
  * the motor the next time it runs and stops.
  *
  * The library default is for the motors to unlock.
  *
  * \sa setMotorLockTime(), getMotorLock().
  *
  * \param b true if the motor is to remain locked when stopped.
  */
  void enableMotorLock(bool b);

 /**
  * Get the current motor lock hold time.
  *
  * Returns the currently configured motor lock time in units of
  * 0.1 seconds.
  *
  * \sa setMotorLockTime(), enableMotorLock().
  *
  * \return motor lock time as multiples of 0.1 sec (100 ms)
  */
  inline uint8_t getMotorLockTime(void) { return(_timeLockRelease / 100); }

 /**
  * Set the motor lock hold time.
  *
  * When the motor is not being moved, the coils can be left energized
  * for maximum holding torque or turned off to reduce power consumption
  * and motor heating. If set to unlock, it will remain locked for the
  * period of time specified by this method before the motor coils are
  * released.
  *
  * This setting is specified in units of 0.1 seconds (0 to 25.5 seconds).
  * The library default is 2 seconds.
  *
  * \sa getMotorLockTime(), enableMotorLock().
  *
  * \param t motor lock time as multiples of 0.1 sec (100 ms)
  */
  void setMotorLockTime(uint8_t t) { _timeLockRelease = t * 100; }

 /**
  * Get the internal status byte.
  *
  * The library maintains internal status bit field of status information.
  * This method returns the bit field as a byte, mainly for debugging purposes. 
  * 
  * The meaning of each status bit is defined in the class' private section and 
  * can be obtained separately using one of the is*() methods.
  *
  * \sa getDirection(), isBusy(), isMoveEnabled() isMotorLocked(), isForward()
  * \sa isAutoRun()
  *
  * \return the status byte
  */
  inline uint8_t getRawStatus(void) { return(_status); }

  /** @} */

private:
  // Step pattern tables
  static const PROGMEM uint8_t stepFull[];
  static const PROGMEM uint8_t stepWave[];
  static const PROGMEM uint8_t stepHalf[];
  static const uint8_t* stepTableLookup[];

  // direct pin I/O structure
  typedef struct
  {
    uint8_t id;     // pin id (number)
    volatile uint8_t *reg;   // pin's direct register
    uint8_t mask;   // mask for pin within register
  } pinDirect_t;

  volatile enum { STP_IDLE, STP_INIT, STP_RUN, STP_STOP } _runState = STP_INIT;

  static const uint8_t MAX_PINS = 4;            // number of control pins per motor

  pinDirect_t _pin[MAX_PINS];    // output pins [A..D]
  uint8_t _pinBusy;              // hardware 'busy' pin; 255 if not used
  
  uint32_t _speedSet;            // set speed in steps/sec valid for current mode setting

  volatile int32_t  _stepCount;    // current steps and direction since last resetPosition()
  volatile uint32_t _moveCountSet; // move pulses target value and to-go counter

  volatile uint32_t _stepTick;   // time between each motor step at _speedSet speed, in microseconds
  volatile uint32_t _timeMark;   // generic time marker for FSM. This could be in milli- or micro-seconds in context

  uint16_t _timeLockRelease;     // set time in milliseconds before pins are released

  stepMode_t _stepMode;          // type of stepper movement 
  volatile int8_t _stepCur = 0;  // current element of step sequence table
  volatile const uint8_t* _stepTable;     // the currently active step table

  // Define options register and bit positions
  volatile uint8_t _options;     // options register
  const uint8_t O_LOCKED = 0;    // leave motor locked when stopped stopped

  // Define status register and bit positions
  volatile uint8_t _status;      // status flags
  const uint8_t S_BUSY = 0;      // motor currently rotation (busy)
  const uint8_t S_RUNMOVE = 1;   // running in move mode, otherwise in free run mode
  const uint8_t S_FWD = 2;       // current motion is in forward direction
  const uint8_t S_LOCKED = 3;    // motor currently locked
  const uint8_t S_AUTORUN = 4;   // autorun initialized and running

  inline bool flagChk(volatile uint8_t f, uint8_t bit) { return((f & _BV(bit)) != 0); }
  inline void flagSet(volatile uint8_t &f, uint8_t bit) { f |= _BV(bit); }
  inline void flagClr(volatile uint8_t &f, uint8_t bit) { f &= ~_BV(bit); }

  void calcStepTick(uint16_t spd);
#if !ENABLE_AUTORUN
  void runStepISR(void);
#endif

#if ENABLE_AUTORUN
private: 
  static const uint16_t ISR_FREQUENCY = 2048;        ///< frequency for TIMER ISR
#if USE_TIMER == 1
  static const uint32_t TIMER_RESOLUTION = 65535;    ///< Timer1 is 16 bit
#elif USE_TIMER == 2
  static const uint32_t TIMER_RESOLUTION = 256;      ///< Timer2 is 8 bit
#endif
  
  inline void setTimerMode(void);   // set TIMER mode
  inline void attachISR(void);      // attach to TIMER ISR
  inline void detachISR(void);      // detach from TIMER ISR
  inline void stopTimer(void);      // stop the timer
  void disableInstance(void);       // enable autorun instance
  bool enableInstance(void);        // enable autorun instance

public:
 //--------------------------------------------------------------
 /** \name AUTORUN and ISR management related. NOT for Applications!
  * @{
  */
  static const uint8_t MAX_INSTANCE = 4;  ///< Maximum concurrent instances

  static bool _bInitialised;          ///< ISR - Global vector initialization flag
  static volatile uint8_t _instCount; ///< ISR - Number of instances currently configured
  static MD_Stepper* _cbInstance[];   ///< ISR - Callback instance handle per pin slot

 /**
  * ISR - Step the motor
  *
  * NOT FOR END USER APPLICATIONS!
  *
  * Runs the motor steps and keeps counts when AUTORUN is enabled.
  */
  void runStepISR(void);

 /**
  * ISR - Set the timer frequency for the
  *
  * NOT FOR END USER APPLICATIONS!
  * 
  * Manipulates the CPU registers to set the appropriate clock counters
  * to achieve the desired output frequency.
  * 
  * \param freq the desired frequency.
  */ 
    void setFrequency(uint32_t freq);
  /** @} */
#endif
};


