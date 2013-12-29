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

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define SRAM_SIZE 8192
#elif defined(__AVR_ATmega328P__)
#define SRAM_SIZE 2048
#else
#define SRAM_SIZE 1024
#endif


/*
* Constructors
*
* default address is B0 << 1 i.e. 0x58 as a 7 bit address for the wire lib
*/
Utils::Utils()
{
    this->counter = 0;
    this->idle_counter = 0;
}


/*
* Public Methods
*/

void Utils::cpuIdle()
{
    this->counter++;
    delay(1);
}

unsigned long Utils::freeCpu()
{
    unsigned long temp = this->counter;
    this->counter = 0;
    return temp;
}

float Utils::cpuUsage(unsigned int period)
{
    // is ticket of counter is 1ms so counter is the total free time we had during period
    return (float)(period - this->freeCpu()) / (float)period;
}

unsigned long Utils::measureIdleUsage(unsigned int period)
{
    this->idle_counter = 0;
    unsigned long idle_timer = millis() + period;
    while(millis() < idle_timer)
    {
        this->idle_counter++;
    }
    return this->idle_counter;
}

int Utils::freeMem() {
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

float Utils::memUsage()
{
    return (float)(SRAM_SIZE - this->freeMem()) / (float)SRAM_SIZE;
}
