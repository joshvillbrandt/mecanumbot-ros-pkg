/*
  MecanumbotPower.ino - Controls power input for the Mecanumbot.
  Created by Josh Villbrandt (http://javconcepts.com/).
  Created in October 2012. Last updated December 2013.
  Released into the public domain.
*/

// Set up I2C
#include <Wire.h>
#include "I2C_Anything.h"

#define DEBUG false
#define PC_STARTUP true
#define REF_5V 4.973 // V
#define ADC_COUNTS 1023.0 // counts
#define ADC_V_MULT ((REF_5V / ADC_COUNTS) / 0.24) // counts to V
#define ADC_I_MULT ((REF_5V / ADC_COUNTS) * (1000.0 / (0.002 * 82500))) // counts to A
#define MIN_VOLT 10.5 // V
#define TURN_OFF_DEBOUNCE 1000 // ms
#define LED_RATE 100 // ms
#define LED_DUTY 65 // ms
const byte MY_ADDRESS = 42;

// Define I/O pins
#define EXT_SW 4
#define EXT_PC 8
byte ledPins[] = {10, 9, 11}; //wall, batt1, batt2
byte fetPins[] = {7, 6, 5}; //wall, batt1, batt2
byte voltagePins[] = {A2, A0, A1, A3, A7}; //wall, batt1, batt2, VIN, PWRSW
byte currentPins[] = {A6};

// State variables
float voltageReadings[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float currentReadings[] = {0.0};
boolean availableSource[] = {true, false, false};
boolean activeSource[] = {true, false, false};
long loopStart = 0;
long loopEnd = 0;
boolean startup = true;
boolean pwrswState = false;
boolean lastPwrswState = false;
long pwrswDebounceTime = 0;
boolean stayPowered = true;

void setup() {
  if(DEBUG) Serial.begin(57600);
  
  // Start I2C
  Wire.begin(MY_ADDRESS);
  Wire.onRequest(telemetryCallback);
  
  // set external interface pins
  pinMode(EXT_SW, INPUT);
  pinMode(EXT_PC, OUTPUT);
  
  // set fet outputs
  for(int i = 0 ; i < 3; i++) {
    pinMode(fetPins[i], OUTPUT);
  }
}

void loop() {
  if(DEBUG) loopStart = micros();
  
  // voltage inputs
  for(int i = 0; i < 5; i++) {
    int sensorValue = analogRead(voltagePins[i]);
    voltageReadings[i] = sensorValue * ADC_V_MULT;
    
    // power source calcs
    if(i <= 2) {
      availableSource[i] = (voltageReadings[i] >= MIN_VOLT);
    }
  }
  
  // current inputs
  for(int i = 0; i < 1; i++) {
    int sensorValue = analogRead(currentPins[i]);
    currentReadings[i] = sensorValue * ADC_I_MULT;
  }
  
  // choose active source
  for(int i = 0 ; i < 3; i++) activeSource[i] = false;
  if(availableSource[0]) activeSource[0] = true; // prefer the wall outlet
  else {
    int voltageMin = 99;
    int indexMin = -1;
    for(int i = 1 ; i < 3; i++) {
      if(availableSource[i] && voltageReadings[i] < voltageMin) {
        voltageMin = voltageReadings[i];
        indexMin = i;
      }
    }
    if(indexMin >= 0) activeSource[indexMin] = true;
    else stayPowered = false; // forcing a clean shutdown, otherwise the uC jitters
  }
  // TODO: this should really have a persistance value and not change source right away (when multiple are present)
  
  // set active source
  for(int i = 0 ; i < 3; i++) {
    // control MOSFET
    if(activeSource[i] && (stayPowered || pwrswState)) digitalWrite(fetPins[i], HIGH);
    else digitalWrite(fetPins[i], LOW);
    
    // set LED
    if(!stayPowered) analogWrite(ledPins[i], 0); // so we know the switch is listening
    else if(activeSource[i] && (stayPowered || pwrswState)) {
      // fancy active indication
      if(millis() % LED_RATE < LED_DUTY) analogWrite(ledPins[i], 255);
      else analogWrite(ledPins[i], 0);
    }
    else if(availableSource[i]) analogWrite(ledPins[i], 127);
    else analogWrite(ledPins[i], 1);
  }
  
  // external PC control
  if(startup && millis() > 3000 && millis() < 4000) digitalWrite(EXT_PC, HIGH);
  else digitalWrite(EXT_PC, LOW);
  if(millis() > 4000) startup = false;
  
  // turn off board
  pwrswState = (!startup && voltageReadings[4] < 5.0); // voltageReadings[4] is not calibrated correctly
  if(pwrswState != lastPwrswState) pwrswDebounceTime = millis();
  if(pwrswState && (millis() - pwrswDebounceTime) > TURN_OFF_DEBOUNCE) stayPowered = false;
  lastPwrswState = pwrswState;
  
  // Debug
  if(DEBUG) {
    loopEnd = micros() - loopStart;
    
    Serial.println("MecanumPower Debug");
    
    Serial.print("e_wall:  \t");
    Serial.print(voltageReadings[0]);
    Serial.println(" V");
    
    Serial.print("e_batt1: \t");
    Serial.print(voltageReadings[1]);
    Serial.println(" V");
    
    Serial.print("e_batt2: \t");
    Serial.print(voltageReadings[2]);
    Serial.println(" V");
    
    Serial.print("e_bus:   \t");
    Serial.print(voltageReadings[3]);
    Serial.println(" V");
    
    Serial.print("i_bus:   \t");
    Serial.print(currentReadings[0], 4);
    Serial.println(" A");
    
    Serial.print("pwr_sw:  \t");
    Serial.println(pwrswState);
    
    Serial.print("ext_sw:  \t");
    Serial.println(digitalRead(EXT_SW));
    
    Serial.print("loop tm: \t");
    Serial.print(loopEnd);
    Serial.println(" us");
    
    delay(500);
    Serial.println();
  }
}

void telemetryCallback()
{
  // build power bit (wall_avail, batt1_avail, batt2_avail, wall_active, batt1_active, batt2_active, power_switch, external_switch)
  byte pwr_bit;
  bitWrite(pwr_bit, 7, availableSource[0]);
  bitWrite(pwr_bit, 6, availableSource[1]);
  bitWrite(pwr_bit, 5, availableSource[2]);
  bitWrite(pwr_bit, 4, activeSource[0]);
  bitWrite(pwr_bit, 3, activeSource[1]);
  bitWrite(pwr_bit, 2, activeSource[2]);
  bitWrite(pwr_bit, 1, pwrswState);
  bitWrite(pwr_bit, 0, digitalRead(EXT_SW));

  byte buffer[21];
  unsigned int buffer_position;
  buffer_position = addToBuffer(buffer, buffer_position, voltageReadings[0]); // e_wall
  buffer_position = addToBuffer(buffer, buffer_position, voltageReadings[1]); // e_batt1
  buffer_position = addToBuffer(buffer, buffer_position, voltageReadings[2]); // e_batt2
  buffer_position = addToBuffer(buffer, buffer_position, voltageReadings[3]); // e_bus
  buffer_position = addToBuffer(buffer, buffer_position, currentReadings[0]); // i_bus
  buffer_position = addToBuffer(buffer, buffer_position, pwr_bit);
  
  Wire.write(buffer, 21);
}
