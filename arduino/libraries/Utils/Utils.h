/*
  Utils.h - Library for measuring Arduino CPU usage and free memory.
  Created by Josh Villbrandt (http://javconcepts.com/), April 7, 2013.
  Released into the public domain.
*/

#ifndef UTILS_h
#define UTILS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Utils
{
    public:
        Utils();
        void cpuIdle();
        unsigned long freeCpu();
        float cpuUsage(unsigned int period);
        unsigned long measureIdleUsage(unsigned int period);
        static int freeMem();
        float memUsage();

    private:
        unsigned long counter; // this number is usually smaller than 500k for a 16MHz Arduino
        unsigned long idle_counter;
};

#endif
