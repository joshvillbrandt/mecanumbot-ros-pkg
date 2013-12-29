/*
  MecanumPower.ino - Controls power input for the Mecanumbot.
  Created by Josh Villbrandt (http://javconcepts.com/), 10/12-1/13.
  Released into the public domain.
*/

// Set up I2C
//#include <Wire.h>

#define DEBUG false
#define PC_STARTUP true
#define REF_5V 4.973 // V
#define ADC_COUNTS 1023.0 // counts
#define ADC_V_MULT ((REF_5V / ADC_COUNTS) / 0.24) // counts to V
#define ADC_I_MULT ((REF_5V / ADC_COUNTS) * (1000.0 / (0.002 * 82500))) // counts to A
#define MIN_VOLT 10.5 // V
#define TURN_OFF_DEBOUNCE 1500 // ms
#define LED_RATE 100 // ms
#define LED_DUTY 65 // ms

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
    if(activeSource[i] && (stayPowered || pwrswState)) {
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
  
  // TODO: send I2C message to master if asked for
  
  // Debug
  if(DEBUG) {
    loopEnd = micros() - loopStart;
    
    Serial.println("MecanumPower Debug");
    
    Serial.print("e_batt1: \t");
    Serial.print(voltageReadings[1]);
    Serial.println(" V");
    
    Serial.print("e_wall:  \t");
    Serial.print(voltageReadings[0]);
    Serial.println(" V");
    
    Serial.print("e_batt2: \t");
    Serial.print(voltageReadings[2]);
    Serial.println(" V");
    
    Serial.print("e_vin:   \t");
    Serial.print(voltageReadings[3]);
    Serial.println(" V");
    
    Serial.print("i_vin:   \t");
    Serial.print(currentReadings[0]);
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
