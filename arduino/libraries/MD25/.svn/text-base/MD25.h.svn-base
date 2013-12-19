/*
  MD25.h - Library for the Devantech MD25 motor controller.
  Created by Josh Villbrandt (http://javconcepts.com/), July 7, 2012.
  Released into the public domain.
*/

#ifndef MD25_h
#define MD25_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class MD25
{
  public:
    MD25();
    MD25(byte i2cAddress);
    void resetEncoders();
    int getEncoder1();
    int getEncoder2();
    void setMotorsSpeed(byte speed);
    void setMotor1Speed(byte speed);
    void setMotor2Speed(byte speed);
    void stopMotor1();
    void stopMotor2();
    void stopMotors();
    long getSoftwareVersion();
    float getBatteryVolts();
    void changeAddress(byte newAddress);
    byte getAccelerationRate();
    byte getMotor1Current();
    byte getMotor2Current();
    byte getMotor1Speed();
    byte getMotor2Speed();
    byte getMode();
    void enableSpeedRegulation();
    void disableSpeedRegulation();
    void enableTimeout();
    void disableTimeout();
    void setMode(byte mode);
    void setAccelerationRate(byte rate);

  private:
    void setMotorSpeed(byte motor, byte speed);
    byte readRegisterByte(byte reg);
    int readEncoderArray(byte reg);
    void sendWireCommand(byte bytes[], byte numBytes);

    byte i2cAddress;

    static byte const cmdReg		= 0x10;  // command register
    static byte const speed1Reg		= 0x00;  // speed to first motor
    static byte const speed2Reg		= 0x01;  // speed to second motor
    static byte const encoderOneReg	= 0x02;  // motor encoder 1 (first byte)
    static byte const encoderTwoReg	= 0x06;  // motor encoder 2 (first byte)
    static byte const voltReg		= 0x0A;  // battery volts
    static byte const current1Reg	= 0x0B;  // motor 1 current
    static byte const current2Reg	= 0x0C;  // motor 2 current
    static byte const softwareVerReg= 0x0D;  // software version
    static byte const accRateReg	= 0x0E;  // acceleration rate
    static byte const modeReg		= 0x0F;  // mode of operation
    static byte const stopSpeed		= 0x88;  // 0 velocity
};

#endif
