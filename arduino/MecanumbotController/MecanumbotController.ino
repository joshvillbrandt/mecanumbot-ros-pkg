/*
  MecanumbotController.ino - Mecanumbot low-level ROS driver.
  Created by Josh Villbrandt (http://javconcepts.com/), July 19, 2012.
  Released into the public domain.
*/

#include "Arduino.h"

// Health
#include "../I2C/I2C.h"
#include <Utils.h>
#define POWER_ADDRESS 5

// Mecanum
#include <I2C.h>
#include <MD25.h>
#include <Mecanum.h>
#define COUNTS_TO_METERS (0.6283185 / 360.0)

// ROS
#include <ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <mecanumbot/LightControl.h>
#include <mecanumbot/RobotHealth.h>

// LEDs
#include "LPD8806.h"
#include "SPI.h"
#define NUM_LEDS 16
#define LED_DATA_PIN 8
#define LED_CLOCK_PIN 9
#define FLASH_MODE_1 50
#define FLASH_MODE_2 150
#define FLASH_MODE_3 190
#define FLASH_MODE_4 500
#define COLOR_WHITE strip.Color(127, 127, 127)
#define COLOR_RED strip.Color(127,   0,   0)
#define COLOR_YELLOW strip.Color(127, 127,   0)
#define COLOR_GREEN strip.Color(  0, 127,   0)
#define COLOR_CYAN strip.Color(  0, 127, 127)
#define COLOR_BLUE strip.Color(  0,   0, 127)
#define COLOR_VIOLET strip.Color(127,   0, 127)
#define COLOR_OFF 0

// Control
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
byte lights = 4; // bitmask LSB:[internal, front, hazards, top in, top out, blank, blank, blank]
byte flash_mode = 1;
LPD8806 strip = LPD8806(NUM_LEDS, LED_DATA_PIN, LED_CLOCK_PIN);

// convert to motor
// TODO: move this into the Mecanum library
#define LINEAR_X_MAX_VEL 1.1 // m/s
#define LINEAR_Y_MAX_VEL 1.3 // m/s
#define ANGULAR_Z_MAX_VEL 2.8 // rad/s
#define DEADZONE 13 // in bytes around 128
byte convertToMotor(float value, float maxValue) {
  value = constrain(value, -1.0 * maxValue, maxValue);
  byte r = map(value*256.0, -1.0 * maxValue * 256.0, maxValue * 256.0, 0.0, 256.0);
  if(value >= maxValue) r = 255; // i guess i don't understand how map works
  
  // enforce deadzone around 128
  if(r >= 128-DEADZONE && r <= 128+DEADZONE) r = 128;
  
  return r;
}

// Mecanum cmd_vel Callback    
void vel_callback( const geometry_msgs::Twist& cmdvel) {
  controller.cmd_vel(
    convertToMotor(cmdvel.linear.x, LINEAR_X_MAX_VEL),
    convertToMotor(cmdvel.linear.y, LINEAR_Y_MAX_VEL),
    convertToMotor(cmdvel.angular.z, ANGULAR_Z_MAX_VEL)
  );
  
  watchdog_timer = millis();
}
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", &vel_callback);

// LightControl Callback    
void light_callback( const mecanumbot::LightControl& lightctrl) {
  // set internal lights
  uint8_t val = min(lightctrl.internal_brightness, 127);
  setPixles(0, 2, strip.Color(val, val, val));
  
  // set head lights
  val = min(lightctrl.forward_brightness, 127);
  setPixle(2, strip.Color(val, val, val));
  setPixle(7, strip.Color(val, val, val));
  val = max(lightctrl.forward_brightness-128, 0);
  setPixles(3, 4, strip.Color(val, val, val));
  
  // set side lights
  if(lightctrl.mood_color == 1) setPixles(12, 4, COLOR_GREEN);
  else if(lightctrl.mood_color == 2) setPixles(12, 4, COLOR_YELLOW);
  else if(lightctrl.mood_color == 3) setPixles(12, 4, COLOR_RED);
  else setPixles(12, 4, COLOR_OFF);
}
ros::Subscriber<mecanumbot::LightControl> light_sub("light_control", &light_callback);

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
  float f = I2c.receive() << 24 | I2c.receive() << 16 | I2c.receive() << 8 | I2c.receive();
  return f;
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
  nh.advertise(enc_pub);
  nh.advertise(health_pub);
  nh.subscribe(vel_sub);
  nh.subscribe(light_sub);
  while(!nh.connected()) nh.spinOnce();
  odom_timer = millis();
  enc_msg.data = (float *)malloc(sizeof(float)*4);
  enc_msg.data_length = 4;

  // Get baseline cpu usage
  
  // default color state w/ ROS connected
  setPixle(2, COLOR_WHITE);
  setPixle(7, COLOR_WHITE);

  // ready to go!
  nh.loginfo("MecanumController startup complete");
}

void loop()
{
  // auto stop motors if we haven't received a cmd_vel message in a while
  if((millis() - watchdog_timer) > WATCHDOG_PERIOD) {
    controller.cmd_vel(128, 128, 128);
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
    if((currentTime - ODOM_PERIOD) > (odom_timer + ODOM_PERIOD))
      odom_timer = currentTime;
    else odom_timer = odom_timer + ODOM_PERIOD;
  }
  
  // health timer
  if((millis() - health_timer) > HEALTH_PERIOD) {
    // microcontroller health
    health_msg.cpu_used = utils.cpuUsage(HEALTH_PERIOD); // percentage
    health_msg.mem_used = utils.memUsage(); // percentage
    
    // power board health
    //I2c.read((byte)POWER_ADDRESS, (byte)21);
    // health_msg.pwr_bit = I2c.receive();
    // health_msg.e_wall = readFloat();
    // health_msg.e_batt1 = readFloat();
    // health_msg.e_batt2 = readFloat();
    // health_msg.e_bus = readFloat();
    // health_msg.i_bus = readFloat();
    
    // temperature and pressure
    //todo
    
    // publish
    health_pub.publish(&health_msg);
    health_timer = millis();
  }
  
  // send / receive ROS messages
  nh.spinOnce();
  // ros likes us to wait at least 1ms between loops - this is accounted for in utils.cpuIdle() below
  
  // keep track of an idle CPU
  utils.cpuIdle(); // sleeps 1ms
}

//char log_msg[10];
//sprintf(log_msg, "ram: %d", memoryTest());
//nh.loginfo(log_msg);
