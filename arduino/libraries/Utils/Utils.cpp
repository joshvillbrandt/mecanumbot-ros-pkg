/*
  Utils.cpp - Library for measuring Arduino CPU usage and free memory.
  Created by Josh Villbrandt (http://javconcepts.com/), April 7, 2013.
  Released into the public domain.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Utils.h"


/*
* Constructors
*
* default address is B0 << 1 i.e. 0x58 as a 7 bit address for the wire lib
*/
Utils::Utils()
{
    this->counter = 0;
}


/*
* Public Methods
*/

void Utils::cpuIdle()
{
    this->counter++;
}

unsigned long Utils::freeCpu()
{
    unsigned long temp = this->counter;
    this->counter = 0;
    return temp;
}

byte Utils::cpuUsage(unsigned long idle_cpu)
{
    return (idle_cpu - this->freeCpu())*100/idle_cpu;
}

unsigned long Utils::measureIdleUsage(unsigned int period)
{
    unsigned long idleCounter = 0;
    unsigned long idleTimer = millis() + period;
    while(millis() < idleTimer)
    {
        idleCounter++;
    }
    return idleCounter;
}

int Utils::freeMem() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

byte Utils::memUsage(int total_mem)
{
    return (total_mem - this->freeMem())/(total_mem/100);
}
