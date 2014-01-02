Mecanumbot Todo List
====================

## Mecanumbot Core

* BUG: occasional loss of comm (~once every 5 minutes) with MecanumbotController arduino 
 * sometimes does not recover when sending lots of commands via the remote
 * sometimes does not connect at all!
 * monitor arduino cpu and mem with rqt plot
* BUG: occasional "hick-ups" in motor controller - wheels spin for a moment without command
 * suspect this is bad resistors on the I2C bus, check with oscope
* BUG: laser data goes totally wack every few seconds... this is something with the new hydro driver
* BUG: xbox controller udev rule doesn't always work
* FEATURE: move installation dependencies to the package.xml file and provide a method to auto-install
* FEATURE: start ros on system startup

## Robot LEDs
* BUG: occasional random `light_control` messages, or so it seems anyway
* FEATURE: change `light_control` to an action instead of a continuous message
* FEATURE: add support for the top plate LEDs in `light_control`
* change message/action format:
 * unit8 strand (0=all, 1=headlights, 2=internal, 3=sides, 4=topdeck)
 * unit8 pattern (0=off, 1=on, 2=flashing on/off, 3=quick double flash, 4= in to out/front to back/cylon, 5 = ou to in/back to front)
 * unit8 color (0=white, 1=red, 2=orange, 3=yellow, 4=green, 5=cyan, 6=blue, 7=purple, 8=magenta, 9=rainbow)
 * unit8 brightness (0=0%, 255=100%)

## Teleop Xbox Controller

* set up `teleop_xbox` as a safety controller - only publish `cmd_vel` from xbox controller when xbox controller is enable
* include some motion kill switch
* toggle target following
* new key bindings:
 * D-pad up/down - toggle headlight brightness
 * D-pad left/right - toggle other light modes
 * B (red) - emergency stop; disable motors and flash lights red
 * A (green) - enable motors
 * X (blue) - enable controller cmd_vel output
 * Y (yellow) - enable target following

## Ball Tracker

* FEATURE: try using a Kalman filter to better track a ball
* FEATURE: throttle point cloud (http://wiki.ros.org/topic_tools/throttle)
* FEATURE: calibrate Kinect using [this guide](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration?action=show&redirect=openni_camera%252Fcalibration)
* FEATURE: register and identify targets between frames
* FEATURE: experiment with [difference of normals segmentation](http://pointclouds.org/documentation/tutorials/don_segmentation.php)
* learn from [turtlebot-follower](https://github.com/turtlebot/turtlebot_apps/blob/hydro/turtlebot_follower/src/follower.cpp) app:
 * use nodelet to avoid pointcloud serialization
 * use service to start and stop follow
 * dynamic reconfigure looks pretty easy here!
