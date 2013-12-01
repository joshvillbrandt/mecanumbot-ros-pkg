mecanumbot
==========

A ROS package for the Mecanumbot robot.

## Setup

Complete the following steps to install the mecanumbot package on a fresh robot. If you are not familiar with ROS, please complete the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) before continuing.

1. Install Ubuntu 12.04
1. Set the hostname and verify avahi MDNS
1. [Install ROS Hyrdro](http://wiki.ros.org/hydro/Installation/Ubuntu)
1. [Create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
1. Clone and build the mecanumbot package

    cd somewhere
    git clone https://github.com/joshvillbrandt/mecanumbot.git

1. Set up the custom USB rules

## Usage

The core mecanumbot code is started with `roslaunch mecanumbot core.launch`. This will bring up the tranform frames, the laser scanner, the kinect, and the rosserial client to the Arduino onboard the mecanumbot.

Launch rviz with the custom mecanumbot config with 'roslaunch mecanumbot rviz.launch'. To do this on a remote machine, be sure to run `export ROS_MASTER_URI=http://jvillbrandt-robot:11311` in your bash session first.

You can control the mecanumbot with a wireless Xbox remote by running `roslaunch mecanumbot teleop_xbox.launch`. To install have the mecanumbot autonomously follow a red ball, run TODO and TODO instead.
