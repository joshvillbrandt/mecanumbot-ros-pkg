mecanumbot
==========

A ROS package for the Mecanumbot robot.

The Mecanumbot robot is a robotic development platform designed and built by Josh Villbrandt. It features a holonomic drive platform with mecanum wheels as well as a 2D laser scanner and an RGB-D camera. Research goals in localization, navigation, and point cloud manipulation. Learn more about the Mecanumbot at [JAV Concepts](http://javconcepts.com/tag/mecanumbot/).

* [Setup](#setup)
* [Usage](#usage)
* [Extra](#extra)
* [Releases](#releases)
* [Todo List](#todo-list)

## Setup

Complete the following steps to install the mecanumbot package on a new robot. If you are not familiar with ROS, please complete the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) before continuing.

### Install Ubuntu 12.04

Use your favorite method to install a fresh copy of Ubuntu Precise. After the install, you'll want to update the OS and install some additional packages.

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install git-core gnome-session-fallback compizconfig-settings-manager

You'll also want to give your robot a custom hostname so that you can easily find it on the network. Modify `/etc/hostname` and `/etc/hosts` to point to a hostname such as `jvillbrandt-robot`. Restart the robot with `sudo shutdown -r now` and verify that Avahi mDNS installed correctly by running `ping jvillbrandt-robot.local` from another machine on the local network.

### Install ROS

If you have earlier versions of ROS installed, it might be best to uninstall them first. If you have problems with the subsequent install statements, try using aptitude instead of apt-get to resolve dependencies.

    sudo apt-get purge ros-groovy-*

Follow the [ROS Hydro Install Guide](http://wiki.ros.org/hydro/Installation/Ubuntu) to get ROS up an running. This boils down to:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install ros-hydro-desktop-full

In addition to the standard desktop package, you'll want to install a few other packages that the mecanumbot package depends on. The first few lines of this are from the [PCL Ubuntu install page](http://pointclouds.org/downloads/linux.html).

    sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
    sudo apt-get update
    sudo apt-get install libpcl-all
    sudo apt-get install ros-hydro-pcl-ros ros-hydro-joy ros-hydro-openni-camera ros-hydro-openni-launch ros-hydro-rosserial-arduino ros-hydro-rosserial ros-hydro-robot-upstart ros-hydro-rqt-robot-plugins

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

Set up unique identifiers for USB devices by linking to custom device rules.

    cd /etc/udev/rules.d/
    sudo ln -s ~/ros/mecanumbot/99-usb-serial.rules 99-usb-serial.rules

### Set up the XV-11 Laser

There was recently an XV-11 laser driver package added to Hydro, but it seems useless at the moment. (Try `sudo apt-get install ros-hydro-xv-11-laser-driver` to install it.) Until that works, you can run the following lines to get the laser set up.

    cd ~/catkin_ws/src
    git clone https://github.com/joshvillbrandt/xv_11_laser_driver.git

## Usage

After the initial install or after making any code changes, the mecanumbot package needs to be compiled. You can do this by running:

    cd ~/catkin_ws
    catkin_make

The core mecanumbot code is started with `roslaunch mecanumbot core.launch`. This will bring up the tranform frames, the laser scanner, the kinect, and the rosserial client to the Arduino onboard the mecanumbot.

Launch rviz with the custom mecanumbot config with 'roslaunch mecanumbot rviz.launch'. To do this on a remote machine, be sure to run `export ROS_MASTER_URI=http://jvillbrandt-robot:11311` in your bash session first.

You can control the mecanumbot with a wireless Xbox remote by running `roslaunch mecanumbot teleop_xbox.launch`. To install have the mecanumbot autonomously follow a red ball, run TODO and TODO instead.

### Auto-start ROS

We can automatically launch the `core.launch` launch file by using upstart. Clearpath Robotics made a nice little package called [robot_upstart](https://github.com/clearpathrobotics/robot_upstart) to help with this. With this package installed, simply run the following command:

    rosrun robot_upstart install mecanumbot/launch/core.launch

## Extra

### New udev Rules

Check out [this syntax guide](http://www.reactivated.net/writing_udev_rules.html#syntax) for creating new udev rules. To identify properties of currently plugged in devices, try a command like this:

    udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -n1

Rules are automatically ran at startup. To automatically reload the rules without restarting, run `sudo udavadm trigger`.

### Installing Arduino Code

The following steps will allow you to update the onboard Arduino.

    mkdir ~/sketchbook
    cd ~/sketchbook
    git clone https://github.com/joshvillbrandt/MecanumbotController
    cd ~/sketchbook/libraries
    rm -rf ros_lib
    rosrun rosserial_arduino make_libraries.py .
    sudo apt-get install arduino arduino-core 

Now open the [MecanumbotController](https://github.com/joshvillbrandt/MecanumbotController) sketch in the Arduino IDE, select board==Arduino Mega 2560 and the correct serial port (try `ls -l /dev | grep USB` and look for `controller`) and click the upload button.

### Fix upower / Arduino Startup Bug

There seems to be a bug with the FTDI chip and the Ubuntu power saving component, upower. We can tell upower not to care about ttyUSB devices so that we can see the FTDI chip on startup. The following notes are taken from http://ten.homelinux.net/productivity/recipes/Arduino%20does%20not%20see%20ttyUSB0.

    sudo vi /lib/udev/rules.d/95-upower-wup.rules
    and look for an entry like:

    SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A80?????", ENV{UPOWER_VENDOR}="Watts Up, Inc.", ENV{UPOWER_PRODUCT}="Watts Up? Pro", ENV{UP_MONITOR_TYPE}="wup"

    If present, comment the line (prepend the line by a "#")

    sudo /etc/init.d/udev restart

    sudo killall upowerd

    sudo lsof /dev/ttyUSB0
    should now report nothing
    Now, Arduino should give access to the USB interface. 

More information [here](http://arduino.cc/forum/index.php?topic=104492.15 and http://bugs.debian.org/cgi-bin/bugreport.cgi?bug=586751).

## Releases

This package uses [semantic versioning](http://semver.org/).

### [v0.2.0](https://github.com/joshvillbrandt/Library/releases/tag/v0.2.0)

Updating for Hydro. New ball tracking with target follower.

### [v0.1.0](https://github.com/joshvillbrandt/Library/releases/tag/v0.1.0)

Last working version of the mecanumbot package for ROS Groovy.

## Todo List

* BUG: occasional loss of comm (~once every 5 minutes) with MecanumbotController arduino 
 * sometimes does not recover
* BUG: occasional "hick-ups" in motor controller - wheels spin for a moment without command
 * suspect this is bad resistors on the I2C bus, check with oscope
* BUG: laser data goes totally wack every few seconds... this is something with the new hydro driver
* BUG: occasional random `light_control` messages
* FEATURE: follow a red ball
* FEATURE: move installation dependencies to the package.xml file and provide a method to auto-install 
