/*
  MD25.cpp - Library for the Devantech MD25 motor controller.
  Created by Josh Villbrandt (http://javconcepts.com/), July 7, 2012.
  Released into the public domain.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "../I2C/I2C.h"
#include "MD25.h"


/*
* Constructors
*
* default address is B0 << 1 i.e. 0x58 as a 7 bit address for the wire lib
*/
MD25::MD25()
{
     MD25(0xB0 >> 1);
}

MD25::MD25(byte i2cAddress)
{
     this->i2cAddress = i2cAddress;
     
     I2c.begin();
     I2c.timeOut(5); // ms
}


/*
* Public Methods
*/

// Gets
int MD25::getEncoder1()
{
   return readEncoderArray(encoderOneReg);
}

int MD25::getEncoder2()
{
   return readEncoderArray(encoderTwoReg);
}

long MD25::getSoftwareVersion()
{
   return readRegisterByte(softwareVerReg);
}

float MD25::getBatteryVolts()
{
     return readRegisterByte(voltReg)/10.0;
}

byte MD25::getAccelerationRate()
{
   return readRegisterByte(accRateReg);
}

byte MD25::getMotor1Speed()
{
   return readRegisterByte(speed1Reg);
}

byte MD25::getMotor2Speed()
{
   return readRegisterByte(speed2Reg);
}

byte MD25::getMotor1Current()
{
   return readRegisterByte(current1Reg);
}

byte MD25::getMotor2Current()
{
   return readRegisterByte(current2Reg);
}

byte MD25::getMode()
{
   return readRegisterByte(modeReg);
}

// Sets
void MD25::resetEncoders()
{
   static byte command[] = { cmdReg, 0x20 };
   sendWireCommand(command, 2);
}

void MD25::enableSpeedRegulation()
{
   static byte command[] = { cmdReg, 0x31 };
   sendWireCommand(command, 2);
}

void MD25::disableSpeedRegulation()
{
   static byte command[] = { cmdReg, 0x30 };
   sendWireCommand(command, 2);
}

void MD25::enableTimeout()
{
   static byte command[] = { cmdReg, 0x33 };
   sendWireCommand(command, 2);
}

void MD25::disableTimeout()
{
   static byte command[] = { cmdReg, 0x32 };
   sendWireCommand(command, 2);
}

void MD25::setMotorsSpeed(byte speed)
{
   setMotor1Speed(speed);
   setMotor2Speed(speed);
}

void MD25::setMotor1Speed(byte speed)
{
   setMotorSpeed(speed1Reg, speed);
}

void MD25::setMotor2Speed(byte speed)
{
   setMotorSpeed(speed2Reg, speed);
}

void MD25::stopMotor1()
{
   setMotor1Speed(stopSpeed);
}

void MD25::stopMotor2()
{
   setMotor2Speed(stopSpeed);
}

void MD25::stopMotors()
{
   stopMotor1();
   stopMotor2();
}

void MD25::setMode(byte mode)
{
   static byte command[] = { modeReg, 0x00 };
   command[1] = mode;
   sendWireCommand(command, 2);
}

void MD25::setAccelerationRate(byte rate)
{
   static byte command[] = { accRateReg, 0x05 };
   command[1] = rate;
   sendWireCommand(command, 2);
}

void MD25::changeAddress(byte newAddress)
{
     static byte command[] = { cmdReg, 0x0A };
     command[1] = 0x0A;
     sendWireCommand(command, 2);
     delay(6);
     command[1] = 0xAA;
     sendWireCommand(command, 2);
     delay(6);
     command[1] = 0xA5;
     sendWireCommand(command, 2);
     delay(6);
     command[1] = newAddress;
     sendWireCommand(command, 2);
     delay(6);
}


/*
* Private Methods
*/

void MD25::setMotorSpeed(byte motor, byte speed)
{
   static byte command[] = { 0x00, stopSpeed };
   command[0] = motor;
   command[1] = speed;
   sendWireCommand(command, 2);
}

byte MD25::readRegisterByte(byte reg)
{
  I2c.read(i2cAddress, reg, (byte)1);
  return I2c.receive();
}

int MD25::readEncoderArray(byte reg)
{
  I2c.read(i2cAddress, reg, (byte)4);
  int position = 0;
  position = I2c.receive() << 8 << 8 << 8;
  position |= I2c.receive() << 8 << 8;
  position |= I2c.receive() << 8;
  position |= I2c.receive();
  return position;
}

void MD25::sendWireCommand(byte bytes[], byte numBytes)
{
   I2c.write(i2cAddress, bytes[0], bytes + 1, numBytes);
}
