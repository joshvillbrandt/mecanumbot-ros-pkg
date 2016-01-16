Mecanumbot Work Log
===================

Entries listed from newest to oldest.

## 15 January 2016, Josh Villbrandt

* created a "hazards enable" service on the arduino to enable and disable motor power
* the mecanumbot boots up with hazards disabled and red flashing LEDs
* when hazards are enabled, green LEDs flash
* four headlight LEDs automatically turn on when a connection to the ROS master is made
* the two front, middle LEDs turn blue when wall power is being used
* updated the teleop_xbox node to call the hazards enable service when the A and B buttons are pressed
* an occasional timing delay in the LED flashes can be noted; this is because the timing is being controlled via the main loop of the arduino
* noted that the laser data sometimes appears to drop every other point (observed in rviz); todo: see if the length of the scan message changes
* fixed the Kinect and updated setup instructions for it
* it has been noted that the phantom wheel spins only start happening after a connection is established between the arduino and ROS

## 14 January 2016, Josh Villbrandt

* initial expirements with hector slam with and without the encoder odometry
* the odometry is not turned for turning on carpet, so the map looks a bit messed up
* when trying hector slam without odometry, the map -> base_link transform is not being published for some reason

## 13 January 2016, Josh Villbrandt

* upgraded the mecanumbot to use ROS Jade
* discovered that my Neato XV-11 Lidar is more stable when the spin motor is powered with a higher voltage; now using 4V instead of 3V
* hacked in a 15 Ohm resistor off of a 5V line to provide the 4V

## 1 January 2014, Josh Villbrandt

* phatom wheel spins still happening
 * noted that phantom wheel spins always seem to be full speed
 * having a faster cmd_vel publish rate mitigates the affect, because the full-speed command is quickly overwritten by the legitimate command
 * this maybe be related to why the wheels spin on power up some time.
* still also have phantom LED flashes, although way less frequently
 * this doesn't have anything to do with I2C, but is commanded by the same Arduino that sends the motor commands
 * maybe a memory problem on the Arduino? But this doesn't seem like it is the case...

## 29 December 2013, Josh Villbrandt

* added barometer telemetry in robot_health
* refactored documentation
* evaluated I2C bus pull up resistors
 * http://www.dsscircuits.com/articles/effects-of-varying-i2c-pull-up-resistors.html
 * R_pull_up_min = (Vcc - 0.4V)/3mA
  * this is a 1.5k resistor for 5V logic
 * R_pull_up_max = 1us / C_b (C_b = bus capacitance)
  * C_arduino is 10pF from the datasheet, so I'm assuming C_b is 50 pF since I also have four slaves
  * this equates to a 20k max pull up resistor
* also a good read: http://www.robot-electronics.co.uk/acatalog/I2C_Tutorial.html
* the Mecanumbot currently uses a 10k resistor each on the clock and data lines
* hooked up an oscope to the i2c bus and saw:
 * 100 kHz on average - spot on
 * 640mV peal to peak - this should be 5V
 * squareness of signal was pretty round for most of cycle - bad but not terrible
  * this matches the 10k resistor in the dsscircuits testing
  * would like to try 2.2k resistors instead
 * i2c bus falls flat when motor e-stop is enable meaning there is no comm even for to the barometer and the power board
  * i already knew this and can live with this...
 * slight dip in the clock line during the start signal... maybe the phantom wheel spins are coming from this dipping
 * changed resistance to 1.8k, 2.2k, and 2.8k and that had no affect (still wheel movements)
* other I2C experiments
 * removed power board and barometer from the bus - no affect (wheels still have occasional momentary jerk)
 * removing either SDA of SCL line from the primary microcontroller (the i2c master) stops all ghost wheel movements (but obviously stops all legitimate comm too)
  * tried hooking primary uC up via different wires to the other MD25 - no affect
  * maybe slave address colisions? 58, 59, 42, and 119
  * maybe try a different arduino? (need another arduino mega...)
  * maybe an interupt problem? maybe ROS vs I2C library?

## 28 December 2013, Josh Villbrandt

* finally routed board power telemetry through the primary arduino and then to ROS

## 24 December 2013, Josh Villbrandt

* ball tracker now uses HSV space for color filtering

## 15 December 2013, Josh Villbrandt

* created a tf average frame

## 8 December 2013, Josh Villbrandt

* migrated ball tracker to hydro

## 30 November 2013, Josh Villbrandt

* migrating repository to hydro
* teleop_xbox works in hydro

## 18 March 2013, Josh Villbrandt

* starting move to ROS Groovy / catkin build system
* starting move to github over svn.javconcepts.com for ROS and Arduino code
* files verified in catkin and Groovy:
 * extra/mecanumbot.rviz
 * extra/mecanumbot.urdf
 * launch/core.launch
 * launch/rviz.launch
 * src/odometry_publisher.cpp

## 12 December 2012, Josh Villbrandt

* updated xv_11_laser_driver with the following time_increment change:

    //scan->time_increment = motor_speed/good_sets/1e8;
    scan->time_increment = 10.6667 / (motor_speed / good_sets); //average time increment
    // JAV corrected this based on https://github.com/Xevel/NXV11/wiki
    // motor speed comes to use in 64*rpm
    // time_increment = 1 / (avg_motor_speed / 64 / 60) / 360

Still seeing a bit of wiggle when rotating the robot. This seems to indicate some lag in the odometry transform.

## 18 November 2012, Josh Villbrandt

* trying out hector slam

    cd ~/ros
    svn co http://tu-darmstadt-ros-pkg.googlecode.com/svn/branches/electric/hector_common/
    rosmake hector_common --rosdep-install
    svn co http://tu-darmstadt-ros-pkg.googlecode.com/svn/branches/electric/hector_slam/
    rosmake hector_slam --rosdep-install

## 17 November 2012, Josh Villbrandt

* updated to the more official laser driver which seems to have a lot less lag
 * still lags a bit rotationally, but not traslationally
* install the laser scan matcher to try and get odom that way

    cd ~/ros
    git clone git://github.com/ccny-ros-pkg/scan_tools.git
    rosmake scan_tools --rosdep-install
    roslaunch laser_scan_matcher demo.launch

## 3 November 2012, Josh Villbrandt

* found some websites that could help with the I2C blocking the rest of the MecanumController Arduino program
 * wire lib with timeout - http://dsscircuits.com/articles/arduino-i2c-master-library.html
 * another wire lib with timeout - https://groups.google.com/forum/?fromgroups=#!topic/ukhas/2n10YIKzudI https://github.com/whitestarballoon/Com-Controller/tree/SessionInitiationFixing/Libraries/WSWire
 * a way to scan the bus to see if the slave is present - http://todbot.com/blog/2009/11/29/i2cscanner-pde-arduino-as-i2c-bus-scanner/
* converted MD25 code to use the I2C library with timeout
* added a cmd_vel watchdog to MecanumController to zero the motors when no message has been received in a while

## 31 October 2012, Josh Villbrandt

* Velocity Calibration was preformed. Video of the Mecanumbot moving was taken with an iPhone 5 and analyzed with Avidemux.

| movement | t1 | t2 | dt | d/dt | units | d/dt | units |
| --- | --- | --- | --- | --- | --- | --- | --- |
| turning 1 rotation | 1.900 | 4.100 | 2.200 | 2.856 | rad/s |  |  |
| forward 4 ft | 1.233 | 2.633 | 1.400 | 0.350 | ft/s | 1.148 | m/s |
| sideways 4 ft | 1.666 | 3.266 | 1.600 | 0.400 | ft/s | 1.312 | m/s |

## 30 October 2012, Josh Villbrandt

* created launch file for gmapping, testing running gmapping - everything works!!
* tested out running nodes on a remote machine with a shared roscore master

## 29 October 2012, Josh Villbrandt

* finally understand the conflict between running odom publisher and cmd_vel subscriber at the same time on the Arduino - memory
* rosserial odom example already consumes all but ~100 bytes of RAM
* using custom "encoders" message (4 floats) and a node on the PC to convert to proper odom messages
* updated urdf origin to be at the center of the robot instead of the front because turning in place was showing up as turning about the front
* I finally have everything I need to run the nav stack! Woohoo!!!!!!!!

## 27 October 2012, Josh Villbrandt

* resolved the Arduino startup (upower) conflict

## 26 October 2012, Josh Villbrandt

* checked out most of the power board and wrote the MecanumPower Arduino program

## 23 October 2012, Josh Villbrandt

* MecanumController finally runs the motors from the Wiimote! Hurray!!!!
* TODO: move convertToMotor into the Mecanum Arduino library
* note that I still can't run odometry concurrently with the cmd_vel subscriber; not sure why
* updated teleop_wiimote LEDs

## 20 October 2012, Josh Villbrandt

* filled out teleop_wiimote
* add teleop.launch to start the wiimote node and the teleop_wiimote node
memory

## 16 October 2012, Josh Villbrandt

* added the laser driver to the repo - this is a copy (with removal of one program) of https://github.com/JBot/smart-robotics-ros-pkg/tree/master/neato_xv11_lds
* fixed the URDF to rotate the laser a tad
* verified the /dev/laser custom USB rule
* updated the start.launch file to properly start the laser
* updated the rviz config file to include the laser scan
* will probably decrease the "not seen" value in the laser driver - the value of 2600 is just weird
 * however, this value might tell mean to SLAM that there is in fact empty space from 0 to "not seen" value which is reasonable assumption up until this range

## 13 October 2012, Josh Villbrandt

* The thinkpenguin.com bluetooth dongle was dead on arrival, however the customer support was very good and the agreed to send me out another one on Monday. Hopefully that one will work...
* Got two of my Neato Laser USB bored populated. They work with one fix - the laser logic needs 3.3V and not 3.0V, so a jumper to the 3.3 line needs to be attached.
* Instructions from http://www.ros.org/wiki/xv_11_laser_driver/Tutorials/Running%20the%20XV-11%20Node don't seem to work for laser firmware V2.6.
* Forum thread http://forums.trossenrobotics.com/showthread.php?4936 and code from https://github.com/JBot/smart-robotics-ros-pkg/tree/master/neato_xv11_lds worked however.
* Laser data in ROS with my board, woohoo!
* Next up is a piggy-back node that removes the four screws in the field of view.

## 5 October 2012, Josh Villbrandt

* created the teleop_wiimote node
* the bluetooth adapter i had from an old Microsoft mouse didn't work in Ubuntu, ordered a new dongle from thinkpenguin.com
* ROS / Arduino serial communication is continueing to give me lots of problems.

## 22 September 2012, Josh Villbrandt

* found problem between ROS and RC Arduino code.. using pulseIn() takes about 30% CPU for each channel reading in (as noted here: http://diydrones.com/profiles/blog/show?id=705844:BlogPost:38418)
* trying a lower timeout of 20000 (dsudo lsof /dev/ttyUSB0efault is 1000000us)
* will probably have to move to an interupt/timer based RC library like http://arduino.cc/playground/Code/ReadReceiver
* need to look at aeroquad or ardupilot for ready-made RC library
* I might simply not support RC driving from now on - long term goal is just to have a remote running through ROS anyway
 * benefit is that is less code stored on the Arduino
 * downside is that I can manually drive the robot without the computer being on

## 26 August 2012, Josh Villbrandt

* updated to new readme.html format
* started MecanumController arduino sketch
 * outputting fake odometry
 * successfully listening to the cmd_vel topic

## 25 August 2012, Josh Villbrandt

* created ball_tracker node to follow a red ball
* created a udev rule for the Mecanumbot Arduino
* added the rosserial_python command to start.launch

## 17 August 2012, Josh Villbrandt

* set up mecanumbot.urdf and start.launch
