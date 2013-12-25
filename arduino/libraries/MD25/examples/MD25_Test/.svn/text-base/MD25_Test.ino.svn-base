/*
  MD25_Test.ino - Example for the MD25 library.
  Created by Josh Villbrandt (http://javconcepts.com/), July 7, 2012.
  Released into the public domain.
*/

#include <Wire.h>
#include <MD25.h>

MD25 controller;

void setup() {
  controller = MD25(0xB0 >> 1);
  Wire.begin();
}

void loop() {
  controller.setMotor1Speed(255);
  delay(1000);
}