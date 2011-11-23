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


//TODO: Add an explaination of how this driver works. Include plenty of ASCII diagrams.

#include "Servo8Bit.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//There two things defined in this file
// *The Servo Sequencer - which is the actual driver that generates pulses
// *The Servo class - which provides the Arduino-like interface



#ifdef USE_TIMER0
    #define TCNTn   TCNT0
    #define OCRnx   OCR0A
    #define OCFnx   OCF0A
    #define OCIEnx  OCIE0A
#endif

#ifdef USE_TIMER1
    #define TCNTn   TCNT1
    #define OCRnx   OCR1A
    #define OCFnx   OCF1A
    #define OCIEnx  OCIE1A
#endif

// Trim Duration is about the total combined time spent inside the Compare Match ISR
// This time is in timer ticks, where each tick is always 8 microseconds.
#define TRIM_DURATION 4


//This is the driver class responsible for generating the servo control pulses
// This class is purely static.
class ServoSequencer
{
public:
    //=============================================================================
    // Servo Sequencer public functions
    //=============================================================================
    static uint8_t  registerServo();
    //              Reserves a slot for a servo in the pulse sequence

    static void     deregisterServo(uint8_t servoSlotNumber);
    //              Frees an occupied slot

    static void     setServoPulseLength(uint8_t servoNumber, uint16_t newLengthInMicroseconds);
    //              Updates the pulse length of a servo to a new value

    static uint16_t getServoPulseLength(uint8_t servoNumber);
    //              returns the pulse length of a servo in microseconds

    static void     setServoPin(uint8_t servoNumber, uint8_t newPin);
    //              Updates the pin that will be pulsed

    static void     enableDisableServo(uint8_t servoNumber, bool servoShouldBeEnabled);
    //              Enable or disable a servo in a slot. A pulse is not generated for a servo slot that is disabled.

    static bool     isEnabled(uint8_t servoNumber);
    //              returns true is the servo is enabled

    inline static void timerCompareMatchISR();
    //              Handles a timer compare match.
    //              Should only be called when the timer compare match interrupt fires.

    //=============================================================================
    // Servo Sequencer public constants
    //=============================================================================
    static const uint8_t kInvalidServoIndex = 0xFF;



private:
    //=============================================================================
    // Servo Sequencer types
    //=============================================================================
    //This enum defines the states of the Servo Sequencer
    enum SequencerState_t
    {
        WAITING_FOR_512_MARK,
        WAITING_TO_SET_PIN_LOW,
        WAITING_FOR_2048_MARK,
        WAITING_TO_SET_PIN_HIGH
    };

    //This struct defines a single servo slot
    struct ServoEntry
    {
        uint8_t  pulseLengthInTicks;    //length of pulse in ticks after offset is applied
        uint8_t  pin;                   //which pin to pulse on portB
        bool     enabled;               //True when this servo should be pulsed
        bool     slotOccupied;          //True when this servo entry is allocated to a servo
    };

    //=============================================================================
    // Servo Sequencer private variables
    //=============================================================================
    static const    uint16_t          kMaxNumberOfServosSupported = 5;  //The number of servos to support. See NOTE1 below.
    static volatile SequencerState_t  state;                            //The current state of the driver
    static          bool              timerIsSetup;                     //True if the timer used by this driver was configured
    static          bool              servoArrayIsInited;               //True if the servo Registry array was initialized with default values.
    static          ServoEntry        servoRegistry[kMaxNumberOfServosSupported]; //The array of servo slots
    static volatile uint8_t           servoIndex;                       //The index of the current servo slot we are working with.
                                                                        //With 5 servos, we go through the whole servoRegistry every 20 milliseconds exactly.

    //=============================================================================
    // Servo Sequencer private functions
    //=============================================================================
    //This is a purely static class. Disallow making instances of this class.
    ServoSequencer();              //private constructor
    static void servoTimerSetup(); //Configures the timer used by this driver
    static void setupTimerPrescaler(); //helper function to setup the prescaler
    static void initServoArray();  //sets default values to each element of the servoRegistry array

};//end ServoSequencer



//=============================================================================
// Servo Sequencer static variables initialization
//=============================================================================
volatile ServoSequencer::SequencerState_t ServoSequencer::state         = ServoSequencer::WAITING_TO_SET_PIN_HIGH;
         bool                             ServoSequencer::timerIsSetup  = false;
         bool                             ServoSequencer::servoArrayIsInited = false;
volatile uint8_t                          ServoSequencer::servoIndex    = 0;
         ServoSequencer::ServoEntry       ServoSequencer::servoRegistry[kMaxNumberOfServosSupported];
         //TODO: Add the rest of the class variables here for better organization?


//NOTE1:
// With 5 servos, each servo is pulsed exactly every 20 milliseconds. You can increase this number to have
// the driver support more servos. The servos are pulsed in sequence and each servo takes up 4 ms of time.
// So if you change this number to 6 then each servo will be pulsed every 24 milliseconds.



//=============================================================================
// FUNCTION:    bool registerServo()
//
// DESCRIPTION: Reserves a slot for a servo in the pulse sequence.
//              There is a limited number of slots.
//
// INPUT:       Nothing
//
// RETURNS:     On success returns the slot number.
//              If no free slot is found returns kInvalidServoIndex.
//=============================================================================
uint8_t ServoSequencer::registerServo()
{

    if(servoArrayIsInited == false)
    {
        initServoArray();
    }
    else
    {
        //the servo array is already inited. Do nothing.
        //It needs to be setup only once. We do it when the first servo is registered.
    }


    //find a free slot in the servo registry
    for(uint8_t i = 0; i < kMaxNumberOfServosSupported; i++)
    {
        if(servoRegistry[i].slotOccupied == false)
        {
            //found a free slot.
            servoRegistry[i].slotOccupied = true;
            //return the slot number
            return i;
        }
        else
        {
            //this slot is not free, check the next one.
        }
    }
    //no free slots were found.
    return kInvalidServoIndex;
}//end registerServo


//=============================================================================
// FUNCTION:    bool deregisterServo(uint8_t servoSlotNumber)
//
// DESCRIPTION: Frees up a slot in the pulse sequence.
//
// INPUT:       servoSlotNumber - The slot number to deallocate.
//
// RETURNS:     Nothing
//=============================================================================
void ServoSequencer::deregisterServo(uint8_t servoSlotNumber)
{
    //make sure we got a valid slot number
    if(servoSlotNumber < kMaxNumberOfServosSupported)
    {
        servoRegistry[servoSlotNumber].enabled      = false;
        servoRegistry[servoSlotNumber].slotOccupied = false;
    }
    else
    {
        //we got a slot number that is out of range. Do nothing.
    }
}//end deregisterServo





//=============================================================================
// FUNCTION:    void setServoPulseLength(uint8_t servoNumber, uint16_t newLengthInMicroseconds)
//
// DESCRIPTION: Updates a servo slots with a new pulse length.
//              Only works on allocated slots.
//
// INPUT:       servoNumber - the slot number to update
//              newLengthInMicroseconds - the new pulse length to set
//
// RETURNS:     Nothing
//=============================================================================
void ServoSequencer::setServoPulseLength(uint8_t servoNumber, uint16_t newLengthInMicroseconds)
{
    //make sure we got a valid slot number and the slot is registered to a servo
    if( (servoNumber < kMaxNumberOfServosSupported      ) &&
        (servoRegistry[servoNumber].slotOccupied == true)   )
    {
        //Convert the servo pulse length into timer ticks.
        //Each timer tick is 8 microseconds.
        int16_t newLengthInClockTicks = newLengthInMicroseconds / 8;
        //subtract the pulse offset
        newLengthInClockTicks -= 64;

        //make sure the length of this pulse is within the acceptable range
        if( (newLengthInClockTicks > -1) && (newLengthInClockTicks < 256) )
        {
            servoRegistry[servoNumber].pulseLengthInTicks = static_cast<uint8_t>(newLengthInClockTicks);
            //Programing note: If pulseLengthInTicks is ever changed to be larger than 1 byte in size then
            //                 interrupts would need to be disabled when updating it to a new value.
        }
        else
        {
            //The new pulse length is too short or long than what we can generate
        }
    }
    else
    {
        //Servo number is out of range or is not allocate to a servo. Do nothing.
    }
}//end setServoPulseLength



//=============================================================================
// FUNCTION:    void getServoPulseLength(uint8_t servoNumber)
//
// DESCRIPTION: Gets the pulse length of a servo slot.
//              Only works on allocated slots.
//
// INPUT:       servoNumber - which slot to get the pulse length from
//
// RETURNS:     The pulse length in microseconds
//=============================================================================
uint16_t ServoSequencer::getServoPulseLength(uint8_t servoNumber)
{
    uint16_t pulseLength = 0;

    //make sure we got a valid slot number and the slot is registered to a servo
    if( (servoNumber < kMaxNumberOfServosSupported      ) &&
        (servoRegistry[servoNumber].slotOccupied == true)   )
    {
        pulseLength = (servoRegistry[servoNumber].pulseLengthInTicks * 8) + 64;
    }
    else
    {
        //Servo number is out of range or is not allocate to a servo. Do nothing.
    }

    return pulseLength;
}//end getServoPulseLength


//=============================================================================
// FUNCTION:    void setServoPin(uint8_t servoNumber, uint8_t newPin)
//
// DESCRIPTION: Sets which pin on PortB should be pulsed for a servo slot.
//
// INPUT:       servoNumber - which servo slot to update
//              newPin - which pin on portB should be pulsed
//
// RETURNS:     Nothing
//
//=============================================================================
void ServoSequencer::setServoPin(uint8_t servoNumber, uint8_t newPin)
{
    //make sure we got a valid slot number and the slot is registered to a servo
    if( (servoNumber < kMaxNumberOfServosSupported      ) &&
        (servoRegistry[servoNumber].slotOccupied == true)   )
    {

        servoRegistry[servoNumber].pin = newPin;

    }
    else
    {
        //Servo number is out of range or is not allocate to a servo. Do nothing.
    }
}//end setServoPin


//=============================================================================
// FUNCTION:    void enableDisableServo(uint8_t servoNumber, bool servoShouldBeEnabled)
//
// DESCRIPTION: Enabled or disables an allocated servo slot.
//              An enabled slot generates a pwm wave on its pin.
//              A disabled slot does not.
//
// INPUT:       servoNumber - which servo slot to update
//              servoShouldBeEnabled - true to enable the servo slot
//                                     false to disable the servo slot
//
// RETURNS:     Nothing
//
//=============================================================================
void ServoSequencer::enableDisableServo(uint8_t servoNumber, bool servoShouldBeEnabled)
{
    //make sure we got a valid slot number and the slot is registered to a servo
    if( (servoNumber < kMaxNumberOfServosSupported      ) &&
        (servoRegistry[servoNumber].slotOccupied == true)   )
    {
        if(servoShouldBeEnabled == true)
        {
            //if this is the very first servo we are enabling then configure the servo timer
            if( timerIsSetup == false)
            {
                servoTimerSetup();
                timerIsSetup = true;
            }
            else
            {
                //The timer is already setup. Do nothing.
                //It needs to be setup only once. We do it when the first servo is enabled.
                //We setup the timer as late as possible. This allows this servo library
                //to be more compatible with various frameworks written for the attiny45/85,
                //which typically configure all the timers to their liking on start up.
                //Configuring our timer late allows us to overwrite these settings.
            }

            //enable the servo. Its pulse will now be outputed on its pin.
            servoRegistry[servoNumber].enabled = true;
        }
        else
        {
            //disable the servo. Its pulse will cease to be generated.
            servoRegistry[servoNumber].enabled = false;
            //TODO: set this servo pin low, if it is high.
            //      Actually, ideally the pulse should finish by itself
            //      forcing the pin low will generate a weird length pulse for the servo
            //      Need to add some sort of "disable pending" status
        }
    }
    else
    {
        //Servo number is out of range or is not allocate to a servo. Do nothing.
    }
}//end enableDisableServo



//=============================================================================
// FUNCTION:    bool isEnabled(uint8_t servoNumber)
//
// DESCRIPTION: Determines if a servo slot is enabled.
//
// INPUT:       servoNumber - which servo slot to check
//
// RETURNS:     true if the slot is enabled
//              false if it is disabled or not allocated to a servo.
//
//=============================================================================
bool ServoSequencer::isEnabled(uint8_t servoNumber)
{
    //make sure we got a valid slot number and the slot is registered to a servo
    if( (servoNumber < kMaxNumberOfServosSupported      ) &&
        (servoRegistry[servoNumber].slotOccupied == true)   )
    {
        return servoRegistry[servoNumber].enabled;
    }
    else
    {
        //Servo number is out of range or is not allocate to a servo.
        //So therefore it is not enabled.
        return false;
    }
}//end isEnabled


//=============================================================================
// FUNCTION:    void servoTimerSetup()
//
// DESCRIPTION: Sets up the timer used by this servo driver
//              No servo pulses can be generated until this function is called.
//
// INPUT:       Nothing
//
// RETURNS:     Nothing
//
//=============================================================================
void ServoSequencer::servoTimerSetup()
{
    //set up the timer prescaler based on which timer was selected and our F_CPU clock
    setupTimerPrescaler();

    // Enable Output Compare Match Interrupt
    TIMSK |= (1 << OCIEnx);

    //reset the counter to 0
    TCNTn  = 0;
    //set the compare value to any number larger than 0
    OCRnx = 255;
    // Enable global interrupts
    sei();

    /*
    TCNT0 - The Timer/Counter
    OCR0A and OCR0B - Output Compare Registers
    TIFR0 - Timer Interrupt Flag Register
    TIMSK - Timer Interrupt Mask Register
    TCCR0B Timer/Counter Control Register B
    */

}//end servoTimerSetup


//=============================================================================
// FUNCTION:    void setupTimerPrescaler()
//
// DESCRIPTION: Helper function that sets up the timer prescaller based on what
//              timer is selected and the F_CPU frequence.
//
// INPUT:       Nothing
//
// RETURNS:     Nothing
//
//=============================================================================
void ServoSequencer::setupTimerPrescaler()
{
    #ifdef USE_TIMER0
        //reset the Timer Counter Control Register to its reset value
        TCCR0B = 0;

        #if F_CPU == 8000000L
            //set counter0 prescaler to 64
            //our FCLK is 8mhz so this makes each timer tick be 8 microseconds long
            TCCR0B &= ~(1<< CS02); //clear
            TCCR0B |=  (1<< CS01); //set
            TCCR0B |=  (1<< CS00); //set

        #elif F_CPU == 1000000L
            //set counter0 prescaler to 8
            //our F_CPU is 1mhz so this makes each timer tick be 8 microseconds long
            TCCR0B &= ~(1<< CS02); //clear
            TCCR0B |=  (1<< CS01); //set
            TCCR0B &= ~(1<< CS00); //clear
        #else
            //unsupported clock speed
            //TODO: find a way to have the compiler stop compiling and bark at the user
        #endif
    #endif


    #ifdef USE_TIMER1
        //reset the Timer Counter Control Register to its reset value
        TCCR1 = 0;

        #if F_CPU == 8000000L
            //set counter1 prescaler to 64
            //our F_CPU is 8mhz so this makes each timer tick be 8 microseconds long
            TCCR1 &= ~(1<< CS13); //clear
            TCCR1 |=  (1<< CS12); //set
            TCCR1 |=  (1<< CS11); //set
            TCCR1 |=  (1<< CS10); //set

        #elif F_CPU == 1000000L
            //set counter1 prescaler to 8
            //our F_CPU is 1mhz so this makes each timer tick be 8 microseconds long
            TCCR1 &= ~(1<< CS13); //clear
            TCCR1 |=  (1<< CS12); //set
            TCCR1 &= ~(1<< CS11); //clear
            TCCR1 &= ~(1<< CS10); //clear
        #else
            //unsupported clock speed
            //TODO: find a way to have the compiler stop compiling and bark at the user
        #endif
    #endif
}//end setupTimerPrescaler


//=============================================================================
// FUNCTION:    void initServoArray()
//
// DESCRIPTION: Sets default values to each element of the servoRegistry array
//
// INPUT:       Nothing
//
// RETURNS:     Nothing
//
//=============================================================================
void ServoSequencer::initServoArray()
{
    //init the Servo Registry array
    for(uint8_t i = 0; i < kMaxNumberOfServosSupported; ++i)
    {
        servoRegistry[i].pulseLengthInTicks = 128;
        servoRegistry[i].pin = 0;
        servoRegistry[i].enabled = false;
        servoRegistry[i].slotOccupied = false;
    }

    servoArrayIsInited = true;
}//end initServoArray


//=============================================================================
// FUNCTION:    void timerCompareMatchISR()
//
// DESCRIPTION: Interrupt service routine for timer0 compare A match.
//              This is where the magic happens.
//
// INPUT:       Nothing
//
// RETURNS:     Nothing
//=============================================================================
void ServoSequencer::timerCompareMatchISR()
{
    switch (state)
    {
    case WAITING_TO_SET_PIN_HIGH:
        //go to the next servo in the registry
        ++servoIndex;
        //if we are the end of the registry, go to the beginning of it
        if(servoIndex == kMaxNumberOfServosSupported)
        {
            servoIndex = 0;
        }
        else
        {
            //we are not at the end, leave the servo index as is
        }

        //if this servo is enabled set the pin high
        if( servoRegistry[servoIndex].enabled == true )
        {
            PORTB |= (1 << servoRegistry[servoIndex].pin);
        }
        else
        {
            //This servo position is not enabled, don't manipulate the pin
        }

        //reset the counter to 0
        TCNTn  = 0;
        //set the compare value to 64 (512 us). This is the constant pulse offset.
        OCRnx = 64 - TRIM_DURATION; //trim off 4 ticks (32us), this is about the total combined time we spent inside this ISR;
        //update our state
        state = WAITING_FOR_512_MARK;
        break;


    case WAITING_FOR_512_MARK:
        //set the compare value to the additional amount of timer ticks the pulse should last
        OCRnx = servoRegistry[servoIndex].pulseLengthInTicks;
        //update our state
        state = WAITING_TO_SET_PIN_LOW;

        //reset the counter to 0
        TCNTn  = 0;

        //Did we just set OCRnx to zero?
        if(OCRnx == 0)
        {
           //Since we are setting OCRnx and TCNTn to 0 we are not going to get an interrupt
           //until the counter overflows and goes back to 0.
           //set the counter its highest value, to have it overflow right away.
           TCNTn = 0xFF;
           //This will cause this interrupt to fire again almost immediately (at the next timer tick)
        }
        else
        {
            //otherwise we need to clear the OCF0A flag because it is possible that the
            //counter value incremented and matched the output compare value while this
            //function was being executed
            TIFR = (1 << OCF0A);  // write logical 1 to the OCF0A flag to clear it
                                  // also have to write 0 to all other bits for this to work.
        }
        break;


    case WAITING_TO_SET_PIN_LOW:
        //if this servo is enabled set the pin low
        if( servoRegistry[servoIndex].enabled == true )
        {
            PORTB &= ~(1 << servoRegistry[servoIndex].pin);
        }
        else
        {
            //This servo position is not enabled, don't manipulate the pin
        }

        //check if the length of this pulse is 2048 microseconds or longer
        if( (64 + servoRegistry[servoIndex].pulseLengthInTicks) > 255 )
        {
            //This pulse length has passed the 2048 us mark, so we skip state WAITING_FOR_2048_MARK
            //update state
            state = WAITING_TO_SET_PIN_HIGH;
            //set the compare value to the amount of time (in timer ticks) we need to wait to reach
            //4096 microseconds mark
            //which is 512 minus the total pulse length. (resulting number will be between 0 and 255 inclusive)
            OCRnx = 512 - (64 + servoRegistry[servoIndex].pulseLengthInTicks);
        }
        else
        {
            //This pulse length has not reached the 2048 us mark, therefor we have to get to that mark first
            //update state
            state = WAITING_FOR_2048_MARK;
            //set OCRnx to the amount of time (in timer ticks) we have to wait to reach this mark
            //which is 255 minus the total pulse length
            OCRnx = 255 - (64 + servoRegistry[servoIndex].pulseLengthInTicks);
        }

        //reset the counter to 0
        TCNTn  = 0;

        break;

    case WAITING_FOR_2048_MARK:
        //update state
        state = WAITING_TO_SET_PIN_HIGH;
        //reset the counter to 0
        TCNTn  = 0;
        //set the compare value to the longest length of time, 255 ticks, or 2040 microseconds
        //This will take us to the ~4096 microsecond mark,
        //at which point the cycle starts again with the next servo slot.
        OCRnx = 255;
        break;
    }//end switch
}//end timerCompareMatchISR




//=============================================================================
// Non Memeber Functions
//=============================================================================

//only define this ISR if we are using TIMER0
#ifdef USE_TIMER0
//=============================================================================
// FUNCTION:    Interrupt service routine for timer0 compare A match
//
// DESCRIPTION: AVR Libc provided function that is vectored into when the
//              timer0 compare A match interrupt fires.
//
// INPUT:       Nothing
//
// RETURNS:     Nothing
//=============================================================================
ISR(TIM0_COMPA_vect)
{
    ServoSequencer::timerCompareMatchISR();
}//end ISR TIM0_COMPA_vect
#endif



//only define this ISR if we are using TIMER1
#ifdef USE_TIMER1
//=============================================================================
// FUNCTION:    Interrupt service routine for timer1 compare A match
//
// DESCRIPTION: AVR Libc provided function that is vectored into when the
//              timer0 compare A match interrupt fires.
//
// INPUT:       Nothing
//
// RETURNS:     Nothing
//=============================================================================
ISR(TIM1_COMPA_vect)
{
    ServoSequencer::timerCompareMatchISR();
}//end ISR TIM0_COMPA_vect
#endif

































//=============================================================================
// Servo Class Functions
//=============================================================================

//=============================================================================
// FUNCTION:    constructor
//
// DESCRIPTION: Constructor
//              Also registers with the ServoSequencer.
//
// INPUT:       Nothing
//
// RETURNS:     Nothing
//
//=============================================================================
Servo8Bit::Servo8Bit()
:invalidServoNumber(ServoSequencer::kInvalidServoIndex),
 myServoNumber(invalidServoNumber),
 myMin(kDefaultMinimalPulse),
 myMax(kDefaultMaximumPulse)
{
    myServoNumber = ServoSequencer::registerServo();
}//end constructor


//=============================================================================
// FUNCTION:    void attach(uint8_t pin)
//
// DESCRIPTION: Attaches a servo motor to an i/o pin on Port B.
//
// INPUT:       pin - which pin on portB to attach to
//
// RETURNS:     The servo number of this servo.
//
//=============================================================================
uint8_t Servo8Bit::attach(uint8_t pin)
{
    //make sure we have a valid servo number. If it's invalid then exit doing nothing.
    if(myServoNumber == invalidServoNumber) return 0;

    //LIMITATION: this servo class only works with PORTB, which is the only port
    //on the attiny45 and attiny85

    //valid pin values are between 0 and 5, inclusive.
    if( pin <= 5 )
    {
        DDRB |= (1<<pin); //set pin as output
        //set the servo pin
        ServoSequencer::setServoPin(myServoNumber, pin);
        //enable the servo to start outputing the pwm wave
        ServoSequencer::enableDisableServo(myServoNumber, true);
    }
    else
    {
        //bad pin value. do nothing.
    }

    return myServoNumber;

}//end attach





//=============================================================================
// FUNCTION:    uint8_t attach(uint8_t pin, uint16_t min, uint16_t max)
//
// DESCRIPTION: Attaches a servo motor to an i/o pin on Port B and also sets
//              the minimum and maximum pulse length values for this servo.
//
// INPUT:       pin - which pin on portB to attach to
//              min - minimum pulse length to use
//              max - maximum pulse length to use
//
// RETURNS:
//
//=============================================================================
uint8_t Servo8Bit::attach(uint8_t pin, uint16_t newMin, uint16_t newMax)
{
    myMin = newMin;
    myMax = newMax;
    return attach(pin);
}//end attach with min/max


//=============================================================================
// FUNCTION:    void detach()
//
// DESCRIPTION: Stops an attached servos from pulsing its i/o pin.
//
// INPUT:       Nothing
//
// RETURNS:     Nothing
//
//=============================================================================
void Servo8Bit::detach()
{
    ServoSequencer::deregisterServo(myServoNumber);
    myServoNumber = invalidServoNumber;
}


//=============================================================================
// FUNCTION:    void write(uint16_t value)
//
// DESCRIPTION: Sets the servo angle in degrees.
//              invalid angle that is valid as pulse in microseconds is
//              treated as microseconds.
//
// INPUT:       value - position for the servo to move to
//
// RETURNS:     Nothing
//
//=============================================================================
void Servo8Bit::write(uint16_t value)
{
    //make sure we have a valid servo number. If it's invalid then exit doing nothing.
    if(myServoNumber == invalidServoNumber) return;

    //for now, only accept angles, and angles that are between 0 and 200 degrees
    if( value > 180 )
    {
        //treat this number as microseconds
        writeMicroseconds( value );
    }
    else
    {
        //treat this number as degrees
        uint16_t servoPulseLengthInUs = map(value, 0, 180, myMin, myMax);
        writeMicroseconds( servoPulseLengthInUs );
    }
}//end write


//=============================================================================
// FUNCTION:    void writeMicroseconds(uint16_t value)
//
// DESCRIPTION: Sets the servo pulse width in microseconds
//
// INPUT:       value - the pulse width of the servo pulse in microseconds
//
// RETURNS:     Nothing
//
//=============================================================================
void Servo8Bit::writeMicroseconds(uint16_t value)
{
    //make sure we have a valid servo number. If it's invalid then exit doing nothing.
    if(myServoNumber == invalidServoNumber) return;

    ServoSequencer::setServoPulseLength(myServoNumber, value );
}//end writeMicroseconds





//=============================================================================
// FUNCTION:    uint16_t readMicroseconds()
//
// DESCRIPTION: Gets the last written servo pulse width in microseconds.
//
// INPUT:       Nothing
//
// RETURNS:     The pulse width in microseconds
//
//=============================================================================
uint16_t Servo8Bit::readMicroseconds()
{
    //make sure we have a valid servo number. If it's invalid then exit doing nothing.
    if(myServoNumber == invalidServoNumber) return 0;

    return ServoSequencer::getServoPulseLength(myServoNumber);
}//end readMicroseconds


//=============================================================================
// FUNCTION:    uint16_t read()
//
// DESCRIPTION: Gets the last written servo pulse width as an angle between 0 and 180.
//
// INPUT:       Nothing
//
// RETURNS:     Angle between 0 and 180
//
//=============================================================================
uint16_t Servo8Bit::read()
{
    //make sure we have a valid servo number. If it's invalid then exit doing nothing.
    if(myServoNumber == invalidServoNumber) return 0;

    uint16_t servoPulseLengthInUs = readMicroseconds();
    uint16_t servoPositionInDegrees = map(servoPulseLengthInUs, myMin, myMax, 0, 180);
    return servoPositionInDegrees;
}//end read


//=============================================================================
// FUNCTION:    bool attached()
//
// DESCRIPTION: Returns true if there is a servo attached.
//
// INPUT:       Nothing
//
// RETURNS:     true, if attached
//              false, otherwise
//
//=============================================================================
bool Servo8Bit::attached()
{
    //make sure we have a valid servo number. If it's invalid then exit doing nothing.
    if(myServoNumber == invalidServoNumber) return false;

    return ServoSequencer::isEnabled(myServoNumber);
}//end attached


//=============================================================================
// FUNCTION:    long map(long x, long in_min, long in_max, long out_min, long out_max)
//
// DESCRIPTION: Our own map function, so that we don't have to get it from some library.
//              Re-maps a number from one range to another.
//              Does not constrain values to within the range.
//
// INPUT:       x       - value to map
//              in_min  - from low
//              in_max  - from high
//              out_min - to low
//              out_max - to high
//
// RETURNS:     re-maped value
//
//=============================================================================
long Servo8Bit::map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}//end map
