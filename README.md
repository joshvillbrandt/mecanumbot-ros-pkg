mecanumbot
==========

A ROS package for the Mecanumbot robot.

## Setup

Complete the following steps to install the mecanumbot package on a new robot. If you are not familiar with ROS, please complete the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) before continuing.

### Install Ubuntu 12.04

Use your favorite method to install a fresh copy of Ubuntu Precise. After install, modify `/etc/hostname` and `/etc/hosts` to point to a new hostname such as `jvillbrandt-robot`. You'll also want to verify that Avahi MDNS installed correctly by running `ping jvillbrandt-robot.local` from another machine on the local network.

### Install ROS

Follow the [ROS Hydro Install Guide](http://wiki.ros.org/hydro/Installation/Ubuntu) to get ROS up an running. This boils down to:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install ros-hydro-desktop-full

In addition to the standard desktop package, you'll want to install the Point Cloud Library for the mecanumbot.

    sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
    sudo apt-get update
    sudo apt-get install libpcl-all
    sudo apt-get install ros-hydro-pcl-ros

To complete the install, source the ROS bash file. You'll probably want to stick this in your bashrc file as well.

    source /opt/ros/hydro/setup.bash

### Create a Workspace

Create a Catkin workspace by following the [create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) guide on ros.org. This boils down to:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

Just like with the primary ROS install, you'll want to source the newly created bash setup file as well as adding it to your bashrc file.

    source ~/catkin_ws/devel/setup.bash

### Clone the Mecanumbot Package

    cd ~/catkin_ws/src
    git clone https://github.com/joshvillbrandt/mecanumbot.git

TODO: Set up the custom USB rules

## Usage

The core mecanumbot code is started with `roslaunch mecanumbot core.launch`. This will bring up the tranform frames, the laser scanner, the kinect, and the rosserial client to the Arduino onboard the mecanumbot.

Launch rviz with the custom mecanumbot config with 'roslaunch mecanumbot rviz.launch'. To do this on a remote machine, be sure to run `export ROS_MASTER_URI=http://jvillbrandt-robot:11311` in your bash session first.

You can control the mecanumbot with a wireless Xbox remote by running `roslaunch mecanumbot teleop_xbox.launch`. To install have the mecanumbot autonomously follow a red ball, run TODO and TODO instead.
