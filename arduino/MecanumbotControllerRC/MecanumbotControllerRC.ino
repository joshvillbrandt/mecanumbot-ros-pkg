/*
  MecanumControllerRC - Testing the Mecanum bot through a standard hobby RC system.
  Created by Josh Villbrandt (http://javconcepts.com/), July 19, 2012.
  Released into the public domain.
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include <MD25.h>
#include <Mecanum.h>

// Mecanum
Mecanum controller;

// RC
int ch1; // aileron (RX ch 1) => turning
int ch2; // throttle (RX ch 3) => forward/backward
int ch3; // rudder (RX ch 4) => straffing
byte ch1b;
byte ch2b;
byte ch3b;
int pwmMin = 1100;
int pwmMax = 1915;
int pwmDead = 35; // really half of the dead band
int pwmCenter;

void setup() {
  // Mecanum
  controller = Mecanum();
  
  // RC
  pwmCenter = (pwmMax + pwmMin) / 2;
  pinMode(3, INPUT); // Set our input pins as such
  pinMode(5, INPUT);
  pinMode(6, INPUT);
}

void loop() {
  // RC
  ch1 = pulseIn(3, HIGH, 25000); // Read the pulse width of 
  ch2 = pulseIn(5, HIGH, 25000); // each channel
  ch3 = pulseIn(6, HIGH, 25000);
  
  if(ch1 > (pwmCenter - pwmDead) && ch1 < (pwmCenter + pwmDead)) ch1b = 128;
  else ch1b = map(ch1, pwmMin, pwmMax, 255, 0);
  
  if(ch2 > (pwmCenter - pwmDead) && ch2 < (pwmCenter + pwmDead)) ch2b = 128;
  else ch2b = map(ch2, pwmMin, pwmMax, 0, 255);
  
  if(ch3 > (pwmCenter - pwmDead) && ch3 < (pwmCenter + pwmDead)) ch3b = 128;
  else ch3b = map(ch3, pwmMin, pwmMax, 255, 0);
  
  // Mecanum
  controller.cmd_vel(ch2b, ch3b, ch1b);
}
