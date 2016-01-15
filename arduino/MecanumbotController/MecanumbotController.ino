/*
  MecanumbotController.ino - Mecanumbot low-level ROS driver.
  Created by Josh Villbrandt (http://javconcepts.com/), July 19, 2012.
  Released into the public domain.
*/

#include "Arduino.h"

// Health
#include <I2C.h>
#include <I2c_BMP085.h>
#include <Utils.h>
const byte PWR_ADDRESS = 42;
#define PRESSURE_SL 101600 // Pa (=1016mb http://w1.weather.gov/obhistory/KLAX.html)

// Mecanum
#include <MD25.h>
#include <Mecanum.h>
#define COUNTS_TO_METERS (0.6283185 / 360.0)

// ROS
#include <ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <mecanumbot/RobotHazardsEnable.h>
#include <mecanumbot/RobotHealth.h>

// LEDs
#include "LPD8806.h"
#include "SPI.h"
#define NUM_LEDS 16
#define LED_DATA_PIN 8
#define LED_CLOCK_PIN 9
#define COLOR_WHITE  strip.Color(127, 127, 127)
#define COLOR_RED    strip.Color(127,   0,   0)
#define COLOR_YELLOW strip.Color(127,  95,   0) // color correction
#define COLOR_GREEN  strip.Color(  0, 127,   0)
#define COLOR_CYAN   strip.Color(  0, 127, 127)
#define COLOR_BLUE   strip.Color(  0,   0, 127)
#define COLOR_VIOLET strip.Color(127,   0, 127)
#define COLOR_OFF    0

// Loop Periods
#define ODOM_PERIOD 50 // 20 Hz
#define WATCHDOG_PERIOD 500 // 2 Hz
#define HEALTH_PERIOD 1000 // 1 Hz

// Global Variables
Utils utils;
Mecanum controller;
ros::NodeHandle  nh;
std_msgs::Float32MultiArray enc_msg;
ros::Publisher enc_pub("encoders", &enc_msg);
mecanumbot::RobotHealth health_msg;
ros::Publisher health_pub("robot_health", &health_msg);
unsigned long health_timer = 0;
unsigned long odom_timer = 0;
unsigned long watchdog_timer = 0;
boolean hazards_enabled = false;
byte lights = 4; // bitmask LSB:[internal, front, hazards, top in, top out, blank, blank, blank]
LPD8806 strip = LPD8806(NUM_LEDS, LED_DATA_PIN, LED_CLOCK_PIN);
I2c_BMP085 bmp;

// convert to motor
// TODO: move this into the Mecanum library
#define LINEAR_X_MAX_VEL 1.1 // m/s
#define LINEAR_Y_MAX_VEL 1.3 // m/s
#define ANGULAR_Z_MAX_VEL 2.8 // rad/s
#define DEADZONE 5 // in bytes around 128
byte convertToMotor(float value, float maxValue) {
  value = constrain(value, -1.0 * maxValue, maxValue);
  byte r = map(value*256.0, -1.0 * maxValue * 256.0, maxValue * 256.0, 0.0, 256.0);
  if(value >= maxValue) r = 255; // i guess i don't understand how map works
  
  // enforce deadzone around 128
  if(r >= 128-DEADZONE && r <= 128+DEADZONE) r = 128;
  
  return r;
}

// Mecanum cmd_vel Callback    
void vel_callback(const geometry_msgs::Twist& cmdvel) {
  if(hazards_enabled) {
    controller.cmd_vel(
      convertToMotor(cmdvel.linear.x, LINEAR_X_MAX_VEL),
      convertToMotor(cmdvel.linear.y, LINEAR_Y_MAX_VEL),
      convertToMotor(cmdvel.angular.z, ANGULAR_Z_MAX_VEL)
    );
  }
  else {
    stopMotors();
  }
  
  watchdog_timer = millis();
}
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", &vel_callback);

// Hazards Enable Callback
void hazards_callback(const mecanumbot::RobotHazardsEnableRequest& req, mecanumbot::RobotHazardsEnableResponse& res) {
  // handle request
  hazards_enabled = req.enable;
  if(!hazards_enabled) {
    stopMotors();
  }
  
  // generate response
  res.success = true;
}
ros::ServiceServer<mecanumbot::RobotHazardsEnableRequest, mecanumbot::RobotHazardsEnableResponse> hazard_srv("robot_hazards_enable", &hazards_callback);

// set multiple pixels
void setPixles(byte start, byte num, uint32_t c) {
  for(byte i=start; i<start+num; i++) strip.setPixelColor(i, c);
  strip.show();
}
void setPixle(byte i, uint32_t c) {
  strip.setPixelColor(i, c);
  strip.show();
}

float readFloat() {
  float value;
  byte* p = (byte*)(void*)&value;
  for(unsigned int i = 0; i < sizeof value; i++) {
    *p++ = I2c.receive();
  }
  return value;
}

void stopMotors() {
  controller.cmd_vel(128, 128, 128);
}

void setup()
{
  // Get baseline CPI utilization
  utils.measureIdleUsage(HEALTH_PERIOD);

  // Start I2C
  I2c.begin();
  I2c.timeOut(5); // ms
  // the lower the timeout, the less CPU we waste when the e-stop is enabled
  
  // Start up the LED strip, turn them all off
  strip.begin();
  strip.show();
  setPixles(0, NUM_LEDS, COLOR_OFF);
  
  // Mecanum Setup
  controller = Mecanum();
  controller.cmd_vel(128, 128, 128);
  
  // ROS Setup
  nh.initNode();
  nh.subscribe(vel_sub);
  nh.advertise(enc_pub);
  nh.advertise(health_pub);
  nh.advertiseService(hazard_srv);
  while(!nh.connected()) nh.spinOnce();
  odom_timer = millis();
  enc_msg.data = (float *)malloc(sizeof(float)*4);
  enc_msg.data_length = 4;
  
  // Start BMP sensor
  if(!bmp.begin()) {
    nh.logerror("Could not find a valid BMP085 sensor, check wiring!");
  }
  
  // ready to go! default color state w/ ROS connected
  nh.loginfo("MecanumController startup complete");
  
  // turn on headlights
  setPixle(2, COLOR_WHITE);
  setPixle(3, COLOR_WHITE);
  setPixle(6, COLOR_WHITE);
  setPixle(7, COLOR_WHITE);
  
  // turn on interior light
  setPixles(0, 2, COLOR_WHITE);
}

void loop()
{
  // auto stop motors if we haven't received a cmd_vel message in a while
  if((millis() - watchdog_timer) > WATCHDOG_PERIOD) {
    stopMotors();
    watchdog_timer = millis();
  }
  
  // send odom
  if((millis() - odom_timer) > ODOM_PERIOD) {
    // grab data from encoders
    controller.getEncoderCounts(enc_msg.data); // returned as x, y, theta
    unsigned long currentTime = millis();
    enc_msg.data[3] = (float)(currentTime - odom_timer) / 1000;
    
    // publish data
    enc_pub.publish(&enc_msg);
    if((currentTime - ODOM_PERIOD) > (odom_timer + ODOM_PERIOD)) {
      odom_timer = currentTime;
    }
    else {
      odom_timer = odom_timer + ODOM_PERIOD;
    }
  }
  
  // health timer
  if((millis() - health_timer) > HEALTH_PERIOD) {
    // microcontroller health
    health_msg.cpu_used = utils.cpuUsage(HEALTH_PERIOD); // percentage
    health_msg.mem_used = utils.memUsage(); // percentage
    
    // power board health
    I2c.read((uint8_t)PWR_ADDRESS, (uint8_t)21);
    int avail = I2c.available();
    health_msg.e_wall = readFloat();
    health_msg.e_batt1 = readFloat();
    health_msg.e_batt2 = readFloat();
    health_msg.e_bus = readFloat();
    health_msg.i_bus = readFloat();
    byte pwr_bit = I2c.receive();
    health_msg.wall_avail = bitRead(pwr_bit, 7);
    health_msg.batt1_avail = bitRead(pwr_bit, 6);
    health_msg.batt2_avail = bitRead(pwr_bit, 5);
    health_msg.wall_active = bitRead(pwr_bit, 4);
    health_msg.batt1_active = bitRead(pwr_bit, 3);
    health_msg.batt2_active = bitRead(pwr_bit, 2);
    health_msg.power_switch = bitRead(pwr_bit, 1);
    health_msg.external_switch = bitRead(pwr_bit, 0);
    
    // update LEDs to reflect wall power state
    if(health_msg.wall_active) {
      setPixle(4, COLOR_BLUE);
      setPixle(5, COLOR_BLUE);
    }
    else {
      setPixle(4, COLOR_OFF);
      setPixle(5, COLOR_OFF);
    }
    
    // temperature and pressure
    health_msg.t_internal = bmp.readTemperature();
    health_msg.p_internal = bmp.readPressure();
    health_msg.altitude = bmp.readAltitude(PRESSURE_SL);
    
    // publish
    health_pub.publish(&health_msg);
    health_timer = millis();
  }
  
  // set motor status LEDs
  byte led_sequence = (millis() % 1000) / 125;
  if(hazards_enabled) {
    if(led_sequence == 0 || led_sequence == 2) {
      setPixles(12, 4, COLOR_GREEN);
    }
    else {
      setPixles(12, 4, COLOR_OFF);
    }
  }
  else {
    if(led_sequence == 0 || led_sequence == 1 || led_sequence == 2 || led_sequence == 3) {
      setPixles(12, 4, COLOR_RED);
    }
    else {
      setPixles(12, 4, COLOR_OFF);
    }
  }
  
  // send / receive ROS messages
  nh.spinOnce();
  // ros likes us to wait at least 1ms between loops - this is accounted for in utils.cpuIdle() below
  
  // keep track of an idle CPU
  utils.cpuIdle(); // sleeps 1ms
}
