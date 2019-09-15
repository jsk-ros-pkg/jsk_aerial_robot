/*
J.Teda 21/04/2013

Exsample of how to run Dynamixel in WHEEL mode - tell servo to move continues at speed

Robotis e-Manual ( http://support.robotis.com )
 
*/

#include <Dynamixel_Serial.h>       // Library needed to control Dynamixal servo

#define SERVO_ID 0x01               // ID of which we will set Dynamixel too 
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 100000    // Baud rate speed which the Dynamixel will be set too (57600)

void setup(){
 delay(1000);                                                           // Give time for Dynamixel to start on power-up

Dynamixel.begin(SERVO_SET_Baudrate, SERVO_ControlPin);        // We now need to set Ardiuno to the new Baudrate speed 
Dynamixel.setMode(SERVO_ID, WHEEL, 0, 0);                               // set mode to WHEEL 
  Dynamixel.wheel(SERVO_ID,RIGHT,0x00);
}


void loop(){
  
Dynamixel.setMaxTorque(SERVO_ID, 0x2FF);                     // Set Dynamixel to max torque limit
/*
  Dynamixel.wheel(SERVO_ID,LEFT,0x3FF);              // Comman for Wheel mode, Move left at max speed 
  delay(4000); 
  
  Dynamixel.wheel(SERVO_ID,RIGHT,0x3FF);          // Comman for Wheel mode, Move right at 50 speedd 
  delay(4000);
*/
  delay(1000);
  

}
