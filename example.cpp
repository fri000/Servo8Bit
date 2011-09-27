// Servo8Bit Example code
// Ilya Brutman

#include "Servo8Bit.h"

void delay(uint16_t milliseconds);  //forward declaration to the delay function






int main()
{
    Servo8Bit myServo;  //create a servo object.
                        //a maximum of five servo objects can be created

    myServo.attach(1);  //attach the servo to pin PB1


    myServo.write(0);   //rotate to the 0 degree position
    delay(2000);        //wait 2 seconds

    myServo.write(180); //rotate to the 180 degree position
    delay(2000);        //wait 2 seconds

    myServo.write(90);  //rotate to the center (90 degree) position
    delay(2000);        //wait 2 seconds


    //sweep the servo
    while(1)
    {
        for(int pos = 0; pos < 180; pos++)  // goes from 0 degrees to 180 degrees
        {                                   // in steps of 1 degree
            myServo.write(pos);             // tell servo to go to position in variable 'pos'
            delay(15);                      // waits 15ms for the servo to reach the position
        }

        for(int pos = 180; pos > 1; pos--)  // goes from 180 degrees to 0 degrees
        {
            myServo.write(pos);             // tell servo to go to position in variable 'pos'
            delay(15);                      // waits 15ms for the servo to reach the position
        }
    }
}









void delayMicroseconds(uint16_t us)
{
#if F_CPU >= 16000000L
    // for the 16 MHz clock on most Arduino boards

    // for a one-microsecond delay, simply return.  the overhead
    // of the function call yields a delay of approximately 1 1/8 us.
    if (--us == 0)
        return;

    // the following loop takes a quarter of a microsecond (4 cycles)
    // per iteration, so execute it four times for each microsecond of
    // delay requested.
    us <<= 2;

    // account for the time taken in the preceeding commands.
    us -= 2;
#else
    // for the 8 MHz internal clock on the ATmega168

    // for a one- or two-microsecond delay, simply return.  the overhead of
    // the function calls takes more than two microseconds.  can't just
    // subtract two, since us is unsigned; we'd overflow.
    if (--us == 0)
        return;
    if (--us == 0)
        return;

    // the following loop takes half of a microsecond (4 cycles)
    // per iteration, so execute it twice for each microsecond of
    // delay requested.
    us <<= 1;

    // partially compensate for the time taken by the preceeding commands.
    // we can't subtract any more than this or we'd overflow w/ small delays.
    us--;
#endif

    // busy wait
    __asm__ __volatile__ (
        "1: sbiw %0,1" "\n\t" // 2 cycles
        "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
    );
}//end delayMicroseconds



void delay(uint16_t milliseconds)
{
    for(uint16_t i = 0; i < milliseconds; i++)
    {
        delayMicroseconds(1000);
    }
}//end delay
