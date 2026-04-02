/*
 Basic control and commands for the PCA9548A/TCA9548A I2C multiplexer
*/
#include <Wire.h>
#define MUX_ADDR 0x70 //7-bit unshifted default I2C Address

//Enables a specific port number
bool enableMuxPort(byte portNumber)
{
  if(portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if(!Wire.available()) return(false); //Error
  byte settings = Wire.read();

  //Set the wanted bit to enable the port
  settings |= (1 << portNumber);

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return(true);
}

//Disables a specific port number
bool disableMuxPort(byte portNumber)
{
  if(portNumber > 7) portNumber = 7;

  //Read the current mux settings
  Wire.requestFrom(MUX_ADDR, 1);
  if(!Wire.available()) return(false); //Error
  byte settings = Wire.read();

  //Clear the wanted bit to disable the port
  settings &= ~(1 << portNumber);

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(settings);
  Wire.endTransmission();

  return(true);
}
