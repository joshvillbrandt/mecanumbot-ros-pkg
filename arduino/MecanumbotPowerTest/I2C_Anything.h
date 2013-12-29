// Written by Nick Gammon
// May 2012
// http://forum.arduino.cc/index.php/topic,104732.0.html
// modified for the I2c library by Josh Villbrandt

#include <Arduino.h>

template <typename T> int I2C_readAnything(T& value)
  {
  Serial.print("bytes available: ");
  Serial.println(I2c.available());
    //byte * p = (byte*) &value;
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof value; i++) {
          *p++ = I2c.receive();
        }
    return i;
  }  // end of I2C_readAnything
