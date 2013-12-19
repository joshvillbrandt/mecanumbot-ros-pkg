/*
  Mecanum.h - Library for Mecanum wheels with two Devantech MD25 motor controllers.
  Created by Josh Villbrandt (http://javconcepts.com/), July 18, 2012.
  Released into the public domain.
*/

#ifndef Mecanum_h
#define Mecanum_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "../MD25/MD25.h"

// for a 16 bit signed int, in second half of storage if abs(counts) > COUNT_OVERFLOW
#define COUNT_OVERFLOW 16374

// assuming max 2 m/s with 100mm wheels and 360 counts / rotation and 1 second between function calls ~ 1080 counts
// the smaller the number, the more likely we are to catch a reset by the more likely it is a false positive
#define COUNT_RESET 200

// total size of int
#define INT_MAX 65535

// get sign of a number
#define sign(a) (min(1, max(-1, a)))

class Mecanum
{
  public:
    Mecanum();
    void cmd_vel(byte x, byte y, byte xy);
    void getEncoderCounts(float*);

  private:
	MD25 frontController;
	MD25 backController;
	
	int lastEncoderCounts[4];
};

#endif
