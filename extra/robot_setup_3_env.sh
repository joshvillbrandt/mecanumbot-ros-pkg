#!/bin/bash
####################################
#
# Step 3: Setup ROS environment
#
####################################

# setup ros groovy
source /opt/ros/groovy/setup.bash

# setup catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make

# setup rosbuild workspace
mkdir ~/rosbuild_ws
cd ~/rosbuild_ws
rosws init . ~/catkin_ws/devel

# save setup information to ~/.bashrc
echo "" >> ~/.bashrc
echo "# ROS Groovy, Mecanumbot" >> ~/.bashrc
echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
echo "source ~/rosbuild_ws/setup.bash" >> ~/.bashrc
#source ~/catkin_ws/devel/setup.bash # this is used if we only have the catkin_ws without the rosbuild_ws
