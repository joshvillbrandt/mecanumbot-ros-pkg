#!/bin/bash
####################################
#
# Step 2: Install ROS Groovy on Ubuntu 12.04 LTS
#
####################################

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y ros-groovy-desktop-full ros-groovy-openni-camera ros-groovy-openni-launch ros-groovy-robot-model ros-groovy-robot-model-visualization ros-groovy-rosserial
