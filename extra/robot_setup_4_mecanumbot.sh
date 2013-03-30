#!/bin/bash
####################################
#
# Step 4: Clone mecanumbot packages
#
####################################

# checkout the mecanumbot package

# check out the XV-11 laser package
cd ~/rosbuild_ws
wstool set cwru-ros-pkg --git https://github.com/cwru-robotics/cwru-ros-pkg.git
#rosws update cwru-ros-pkg # this doesn't work with this repo for some reason
git clone https://github.com/cwru-robotics/cwru-ros-pkg.git
rosmake xv_11_laser_driver common_rosdeps
