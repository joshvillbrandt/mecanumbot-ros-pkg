/*
  MecanumbotPowerTest.ino - Reads telemetry over I2C.
  Created by Josh Villbrandt (http://javconcepts.com/).
  Created in December 2013.
  Released into the public domain.
*/

#include <I2C.h>
#include "I2C_Anything.h"

const byte PWR_ADDRESS = 42;

void setup()
{
  // Start I2C
  I2c.begin();
  I2c.timeOut(15); // ms
  
  // Start serial
  Serial.begin(57600);
  
  I2c.scan();
}

void loop()
{
  float e_wall, e_batt1, e_batt2, e_bus, i_bus;
  byte pwr_bit;
  
  I2c.read((uint8_t)PWR_ADDRESS, (uint8_t)21);
  
  I2C_readAnything(e_wall);
  I2C_readAnything(e_batt1);
  I2C_readAnything(e_batt2);
  I2C_readAnything(e_bus);
  I2C_readAnything(i_bus);
  I2C_readAnything(pwr_bit);
  
//  I2C_readAnything(PWR_ADDRESS, 5, e_wall);
//  I2C_readAnything(PWR_ADDRESS, 1, e_batt1);
//  I2C_readAnything(PWR_ADDRESS, 2, e_batt2);
//  I2C_readAnything(PWR_ADDRESS, 3, e_bus);
//  I2C_readAnything(PWR_ADDRESS, 4, i_bus);
//  I2C_readAnything(PWR_ADDRESS, 5, pwr_bit);
  
  Serial.print("e_wall: ");
  Serial.println(e_wall);
  Serial.print("e_batt1: ");
  Serial.println(e_batt1);
  Serial.print("e_batt2: ");
  Serial.println(e_batt2);
  Serial.print("e_bus: ");
  Serial.println(e_bus);
  Serial.print("i_bus: ");
  Serial.println(i_bus, 4);
  
  Serial.print("wall_avail: ");
  Serial.println(bitRead(pwr_bit, 7));
  Serial.print("batt1_avail: ");
  Serial.println(bitRead(pwr_bit, 6));
  Serial.print("batt2_avail: ");
  Serial.println(bitRead(pwr_bit, 5));
  Serial.print("wall_active: ");
  Serial.println(bitRead(pwr_bit, 4));
  Serial.print("batt1_active: ");
  Serial.println(bitRead(pwr_bit, 3));
  Serial.print("batt2_active: ");
  Serial.println(bitRead(pwr_bit, 2));
  Serial.print("power_switch: ");
  Serial.println(bitRead(pwr_bit, 1));
  Serial.print("external_switch: ");
  Serial.println(bitRead(pwr_bit, 0));
    
  delay(500);
  Serial.println();
}
