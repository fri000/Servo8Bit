// Servo8Bit Example code
// Ilya Brutman

#include "Servo8Bit.h"

Servo8Bit myServo;

void setup()
{
  //create a servo object.
  //a maximum of five servo objects can be created
  myServo.attach(1);  //attach the servo to pin PB1
}

void loop() {
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



