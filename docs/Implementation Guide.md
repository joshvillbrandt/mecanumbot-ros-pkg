Mecanumbot Implementation Guide
===============================

This file discusses the implementation of various parts of the Mecanumbot package.

## High-Level Design Notes

### Control
The MecanumbotController Arduino sketch combines subscribes to the cmd_vel topic and translates and translates the message for the Mecanum library. The Mecanum library in turn send speed commands to each EMG30 motor on two Devantech MD25 motor contollers. The MecanumbotController also sends back odometry and responds to LED sequence commands.

## File-Specific Notes

### mecanumbot.urdf
The origin of the base_link frame for the mecanumbot is taken to be the center of the robot at the base of the ground. (This is 0.163m behind the front of the robot.) Following ROS Enhancement Proposal 103, x is forward, y is left, and z is up.
