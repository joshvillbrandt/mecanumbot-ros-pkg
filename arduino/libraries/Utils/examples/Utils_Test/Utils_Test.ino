/*
  Utils_Test.ino - Example for the Utils library.
  Created by Josh Villbrandt (http://javconcepts.com/), April 7, 2013.
  Released into the public domain.
*/

#include <Utils.h>

#define LOOP_PERIOD 1000 // 1 Hz
#define LED_PIN 13

Utils utils;
unsigned long idle_cpu = 0;
int total_mem = 2048; // Arduino Uno
unsigned long loop_timer = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN,  OUTPUT);
  
  // Print memory test
  Serial.print("Free memory then (bytes): ");
  Serial.println(Utils::freeMem());
  String test = "arduino";
  Serial.print("Free memory now (bytes): ");
  Serial.println(Utils::freeMem());
  
  // Check idle usage
  Serial.println("Beginning idle CPU usage check...");
  idle_cpu = Utils::measureIdleUsage(3*LOOP_PERIOD)/3;
  Serial.print("CPU idle counts: ");
  Serial.println(idle_cpu);
  Serial.println("");
}

void loop() {
  // Run 1Hz actions
  if((millis() - loop_timer) > LOOP_PERIOD) {
    // Some action
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200); // simulate a lengthy action
    
    // Print stats?
    Serial.print("CPU usage (%): ");
    Serial.println(utils.cpuUsage(idle_cpu));
    Serial.print("Mem usage (%): ");
    Serial.println((total_mem - utils.freeMem())/(total_mem/100)); //utils.memUsage(total_mem));
    
    // reset loop timer
    loop_timer = loop_timer + LOOP_PERIOD;
  }
  
  // keep track of an idle CPU
  utils.cpuIdle();
}