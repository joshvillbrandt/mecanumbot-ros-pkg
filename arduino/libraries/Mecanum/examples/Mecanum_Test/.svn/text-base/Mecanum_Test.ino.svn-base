/*
  MD25_Test.ino - Example for the MD25 library.
  Created by Josh Villbrandt (http://javconcepts.com/), July 7, 2012.
  Released into the public domain.
*/

#include <Wire.h>
#include <MD25.h>
#include <Mecanum.h>

Mecanum controller;

void setup() {
  controller = Mecanum();
  
  // execute circle
  controller.cmd_vel_norm(255, 128, 0);
  delay(300);
  controller.cmd_vel_norm(255, 0, 0);
  delay(300);
  controller.cmd_vel_norm(128, 0, 0);
  delay(300);
  controller.cmd_vel_norm(0, 0, 0);
  delay(300);
  controller.cmd_vel_norm(0, 128, 0);
  delay(300);
  controller.cmd_vel_norm(0, 255, 0);
  delay(300);
  controller.cmd_vel_norm(128, 255, 0);
  delay(300);
  controller.cmd_vel_norm(255, 255, 0);
  delay(300);
  controller.cmd_vel_norm(128, 128, 0);
}

void loop() {
}
