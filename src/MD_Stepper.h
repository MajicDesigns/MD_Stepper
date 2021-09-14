#pragma once
/**
\mainpage Stepper motor control library

This library provide control of unipolar and bipolar stepper motors
through the appropriate hardware drivers.

Important Notes:
- This library uses AVR TIMER1 or TIMER2 to implement the interrupt
driven clock. TIMER0 is used by the Arduino millis() clock, TIMER1
is commonly used by the Servo library and TIMER2 by the Tone library.
Change USE_TIMER (defined at the top of the header file) to select
which timer is enabled in the library code.

- This library has been tested on Arduino Uno and Nano (ie, 328P processor).
It should work on other AVR chips (eg, MEGA) with slight modifications but
it will not work on non-AVR architectures without some extensive rework.


# Implementation

This library implements user defined frequency PWM output for any digital pin
software limited to MAX_FREQUENCY Hz.

The TIMERn is set for 255 times this frequency (eg, 200Hz becomes 51kHz). This
causes the TIMERn interrupt routine to be called 255 times for each PWM cycle
and, depending on where it is in the cycle, allows the software to set the
digital output pin to LOW or HIGH, thus creating the desired PWM signal.
This is illustrated below.

![PWM Timing Diagram] (PWM_Timing.png "PWM Timing Diagram")

The duty cycle can be changed very smoothly by changing the set point at which
the digital transition occurs. The new duty cycle takes effect at the next
PWM digital transition.

TIMERn is a global resource, so each object instance of class is driven from the
same TIMERn interrupt. The constant MAX_PWM_PIN is used to set limits the
global maximum for instances allowed to be processed by the interrupt.

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
Sep 2021 ver 1.0.0
- Initial release
*/

#include <Arduino.h>

/**
 * \file
 * \brief Main header file and class definition for the MD_Stepper library.
 */

#ifndef STPDEBUG
#define STPDEBUG 0      ///< 1 turns library debug output on
#endif

#if STPDEBUG
#define PRINTS(s)    Serial.print(F(s))
#define PRINT(s,v)   do { Serial.print(F(s)); Serial.print(v); } while (false)
#define PRINTX(s,v)  do { Serial.print(F(s)); Serial.print(F("0x")); Serial.print(v, HEX); } while (false)
#define PRINTB(s,v)  do { Serial.print(F(s)); Serial.print(F("0b")); Serial.print(v, BIN); } while (false)
#else
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
  * shared variables and timers to default values.
  *
  * \param inA pin connected to hardware driver INA or IN1 input (Coil A).
  * \param inB pin connected to hardware driver INB or IN2 input (Coil A).
  * \param inC pin connected to hardware driver INC or IN3 input (Coil B).
  * \param inD pin connected to hardware driver IND or IN4 input (Coil B).
  * \param speedMax the maximum speed the motor can work at in steps/second.
  */
  MD_Stepper(uint8_t inA, uint8_t inB, uint8_t inC, uint8_t inD, uint16_t speedMax = SPEEDMAX_DEFAULT);

 /**
  * Class Destructor.
  *
  * Release any allocated memory and clean up anything else.
  *
  * If all the instances of the class are closed, then the ISR is
  * disconnected and the timer is stopped.
  */
  ~MD_Stepper(void) {}

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
  * If this is the first instance of this class, then the ISR is code
  * is initialized and the frequency of the hardware timer is set.
  * Subsequent instances do not affect the timer frequency.
  * 
  * The busy pin can also be specifed as an option parameter.
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
  inline void start(void) { _runState = INIT; }

  /**
   * Stop movement.
   *
   * Stop the motor moving imediately. This stops free running motion and will 
   * immediately terminate motion set up using move().
   *
   * \sa move(), run(), start()
   */
  inline void stop(void) { _runState = STOP; }

  /**
   * Run motion.
   *
   * This is called every iteration through loop() to run all the required
   * Stepper management functions.
   * 
   * \sa start(), stop()
   * 
   * \return true if a step was taken during the method invocation.
   */
  bool run(void);

  /** @} */

 //--------------------------------------------------------------
 /** \name Methods for motion parameter management.
  * @{
  */

 /**
  * Get the current motor direction.
  *
  * Returns the current direction of the motor. By convention the forward
  * direction is 'true' and the reverse is 'false'.
  * 
  * Note that forward and reverse have no inherent meaning other than they 
  * are in opposite rotational directions.
  * 
  * \sa setDirection()
  * 
  * \return true if the motor is moving in the 'forward' direction
  */
  inline bool getDirection(void)   { return(flagChk(_status, S_FWD)); }

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
  * for reverse motion) into a cumulative counter. This method retuns the current
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
  * Gets the configured maximum speed.
  *
  * \sa setMaxSpeed(), MD_Stepper()
  *
  * \return the current configured maximum speed
  */
  inline uint16_t getMaxSpeed(void) { return(_speedMax); }

 /**
  * Gets the configured maximum speed.
  *
  * Motors can be practically run up to a maximum speed (steps/sec). The library
  * can be configured with this speed to allow upper-bound checks. This method
  * sets the maximum speed for the instance of the library. The setting can also 
  * be set as an optional parameter in the class constructor.
  *
  * As the speed is specified in steps/sec, the current motor stepping mode
  * has an effect. HALF stepping results in half the physical speed of a FULL
  * or WAVE stepping for the same numeric setting.
  *
  * \sa getMaxSpeed(), MD_Stepper()
  *
  * \param s the maximum speed in steps/sec
  */
  void setMaxSpeed(uint16_t s);


 /**
  * Gets the configured running speed.
  *
  * \sa setSpeed(), setMaxSpeed()
  *
  * \return the running speed in steps/sec
  */
  inline uint16_t getSpeed(void) { return(_speedSet); }

 /**
  * Set the running speed.
  *
  * The motor speed can be set to any value between 0 and the configured maximum
  * speed. Speeds higher than the maximum are capped at the maximum. Setting the 
  * motor speed to 0 has the same effect as a stop().
  * 
  * The new speed takes effect at the next motor step (immediately if running 
  * or at the next start()).
  * 
  * As the speed is specified in steps/sec, the current motor stepping mode 
  * has an effect. HALF stepping results in half the physical speed of a FULL 
  * or WAVE stepping fore the same numeric setting.
  *
  * \sa getSpeed(), setMaxSpeed()
  *
  * \param s the required running speed in steps/sec
  */
  void setSpeed(uint16_t s);

 /**
  * Set the number of steps to move.
  *
  * The motor can be set to move a specific number of steps and then stop. This
  * method sets up the number of steps for the next start(). 
  *
  * If the number of stps is positive the motion is in the forward direction; 
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
  * If the number of stps is positive the motion is in the forward direction;
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
 /** \name Methods for hardware and options management.
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
  * The library default stepping mode is FULL.
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
  * When the motor is not being moved, the coils can be left energised 
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
  * When the motor is not being moved, the coils can be left energised
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
  * Get the internal ststus byte.
  *
  * The library maintains internal status bit field of status information.
  * This method returns the bit field as a byte, mainly for debugging purposes. 
  * 
  * The meaning of each status bit is defined in the class' private section.
  *
  * \sa getDirection(), isBusy(), enableMotorLock(), 
  *
  * \return the status byte
  */
  inline uint8_t getRawStatus(void) { return(_status); }

  /** @} */
private:
  enum { IDLE, INIT, RUN, STOP } _runState = INIT;

  static const uint16_t SPEEDMAX_DEFAULT = 1000;// maximum full steps/second
  static const uint8_t MAX_PINS = 4;            // max number of control pins

  uint8_t _pinBusy;              // hardware 'busy' pin; 255 if not used
  uint8_t _in[MAX_PINS];         // output pins [A..D]

  uint32_t _speedMax;            // maximum speed in full step (full steps/sec)
  uint32_t _speedSet;            // set speed in full steps (full steps/sec)
  int32_t  _stepCount;           // current steps since last resetPosition()
  uint32_t _moveCountSet;        // move pulses target value and to-go counter

  uint16_t _stepTick;            // time between each motor step at _speedSet speed, in microseconds
  uint32_t _timeMark;            // generic time marker for FSM. This could be in milli- or micro-seconds in context

  uint16_t _timeLockRelease;     // set time in milliseconds before pins are released

  stepMode_t _stepMode;          // type of stepper movement 
  int8_t _seqCur = 0;            // current element of step sequence table

  // Define options register and bit positions
  uint8_t _options;              // options register
  const uint8_t O_LOCKED = 0;    // leave motor locked when stopped stopped

  // Define status register and bit positions
  uint8_t _status;               // status flags
  const uint8_t S_BUSY = 0;      // motor currently rotation (busy)
  const uint8_t S_RUNMOVE = 1;   // running in move mode, otherwise in free run mode
  const uint8_t S_FWD = 2;       // current motion is in forward direction
  const uint8_t S_LOCKED = 3;    // motor currently locked

  inline bool flagChk(uint8_t f, uint8_t bit) { return((f & _BV(bit)) != 0); }
  inline void flagSet(uint8_t &f, uint8_t bit) { f |= _BV(bit); }
  inline void flagClr(uint8_t &f, uint8_t bit) { f &= ~_BV(bit); }

  void calcStepTick(uint16_t spd);
  bool runFSM(void);
  void singleStep(void);
};


