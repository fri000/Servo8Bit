/*
 Servo8Bit.cpp - Interrupt driven Servo library for the Attiny45 and Attiny85 that uses an 8 bit timer.
 Version 0.6
 Copyright (c) 2011 Ilya Brutman.  All right reserved.

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
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


/*

  A servo is activated by creating an instance of the Servo8Bit class passing the desired pin to the attach() method.
  The servos are pulsed in the background using the value most recently written using the write() method


  The methods are:

   Servo8Bit - Class for manipulating servo motors connected to Attiny pins.

   attach(pin)           - Attaches a servo motor to an i/o pin.
   attach(pin, min, max) - Attaches to a pin setting min and max values in microseconds
                           default min is 544, max is 2400

   write()               - Sets the servo angle in degrees.  (invalid angle that is valid as pulse in microseconds is treated as microseconds)
   writeMicroseconds()   - Sets the servo pulse width in microseconds
   read()                - Gets the last written servo pulse width as an angle between 0 and 180.
   readMicroseconds()    - Gets the last written servo pulse width in microseconds.
   attached()            - Returns true if there is a servo attached.
   detach()              - Stops an attached servos from pulsing its i/o pin.
 */

#ifndef Servo8Bit_h
#define Servo8Bit_h

#include <inttypes.h>


//Options
//pick one, comment out the other one out:
//#define USE_TIMER0
#define USE_TIMER1


class Servo8Bit
{
public:
  Servo8Bit();
  //TODO: create destructor
  uint8_t attach(uint8_t pin);              // attach the given pin to the next free channel, returns channel number or 0 if failure
  uint8_t attach(uint8_t pin, uint16_t newMin, uint16_t newMax); // as above but also sets min and max values for writes.
  void detach();

  void write(uint16_t value);               // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
  void writeMicroseconds(uint16_t value);   // Write pulse width in microseconds
  uint16_t read();                          // returns current pulse width as an angle between 0 and 180 degrees
  uint16_t readMicroseconds();              // returns current pulse width in microseconds for this servo
  bool attached();                          // return true if this servo is attached, otherwise false

private:
   //private constants
   static const uint16_t kDefaultMinimalPulse = 544;
   static const uint16_t kDefaultMaximumPulse = 2400;

   //private variables
   uint8_t  myServoNumber;                  // Our ID number that we get from the ServoSequencer after we register with it
   uint8_t  invalidServoNumber;             // value that represents and invalid servo number. This is set only once.
   uint16_t myMin;                          // minimum pulse length that corresponds to the angle of 0 degrees
   uint16_t myMax;                          // maximum pulse length that corresponds to the angle of 180 degrees

   //our own map function, so that we don't have to get it from some library
   long map(long x, long in_min, long in_max, long out_min, long out_max);
};

#endif
