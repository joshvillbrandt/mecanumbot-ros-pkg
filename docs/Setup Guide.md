Mecanumbot Setup Guide
======================

Complete the following steps to install the mecanumbot package on a new robot. If you are not familiar with ROS, please complete the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) before continuing.

## Install Ubuntu 12.04

Use your favorite method to install a fresh copy of Ubuntu Precise. After the install, you'll want to update the OS and install some additional packages.

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get install git-core gnome-session-fallback compizconfig-settings-manager

You'll also want to give your robot a custom hostname so that you can easily find it on the network. Modify `/etc/hostname` and `/etc/hosts` to point to a hostname such as `jvillbrandt-robot`. Restart the robot with `sudo shutdown -r now` and verify that Avahi mDNS installed correctly by running `ping jvillbrandt-robot.local` from another machine on the local network.

## Install ROS

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
    sudo apt-get install ros-hydro-pcl-ros ros-hydro-joy ros-hydro-openni-camera ros-hydro-openni-launch ros-hydro-rosserial-arduino ros-hydro-rosserial ros-hydro-robot-upstart ros-hydro-rqt-robot-plugins screen

To complete the install, source the ROS bash file. You'll probably want to stick this in your `.bashrc` file as well.

    source /opt/ros/hydro/setup.bash

## Create a Workspace

Create a Catkin workspace by following the [create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) guide on ros.org. This boils down to:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

Just like with the primary ROS install, you'll want to source the newly created bash setup file as well as adding it to your bashrc file.

    source ~/catkin_ws/devel/setup.bash

## Clone the Mecanumbot Package

    cd ~/catkin_ws/src
    git clone https://github.com/joshvillbrandt/mecanumbot.git

Set up unique identifiers for USB devices by linking to custom device rules.

    cd /etc/udev/rules.d/
    sudo ln -s ~/catkin_ws/src/mecanumbot/extra/99-usb-serial.rules 99-usb-serial.rules

## Set up the XV-11 Laser

There was recently an XV-11 laser driver package added to Hydro, but it seems useless at the moment. (Try `sudo apt-get install ros-hydro-xv-11-laser-driver` to install it.) Until that works, you can run the following lines to get the laser set up.

    cd ~/catkin_ws/src
    git clone https://github.com/joshvillbrandt/xv_11_laser_driver.git

After the initial install or after making any code changes, the mecanumbot package needs to be compiled. You can do this by running:

    cd ~/catkin_ws
    catkin_make

## Extra

These setup steps aren't always necessary.

### New udev Rules

Check out [this syntax guide](http://www.reactivated.net/writing_udev_rules.html#syntax) for creating new udev rules. To identify properties of currently plugged in devices, try a command like this:

    udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -n1

Rules are automatically ran at startup. To automatically reload the rules without restarting, run `sudo udavadm trigger`.

### Installing Arduino Code

The following steps will allow you to update the onboard Arduino.

    mv ~/sketchbook ~/sketchbook_old
    ln -s ~/catkin_ws/src/mecanumbot/arduino ~/sketchbook
    cd ~/sketchbook/libraries
    rm -rf ros_lib # just in case
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

### Pair the Wiimote

Run the following steps to start the wiimote node. The following steps are from http://www.ros.org/wiki/wiimote/Tutorials/StartingWiimoteNode.

    sudo apt-get install python-rosdep
    rosdep install wiimote
    rosmake wiimote
    rosrun wiimote wiimote_node.py

With a new Electric install, the libcwiid library will complain about an unknown symbol clock_gettime. The problem is with Electric's bundled version of the library. Run the following steps to install the stock library. From http://answers.ros.org/question/28084/problem-in-wiimote_nodepy/.

    sudo apt-get install libcwiid1 libcwiid-dev
    cd /opt/ros/electric/stacks/joystick_drivers/cwiid/cwiid/lib/
    sudo mkdir old
    sudo mv libcwiid.s* old
    sudo ln -s /usr/lib/libcwiid.so libcwiid.so
    sudo ln -s /usr/lib/libcwiid.so.1 libcwiid.so.1
    sudo ln -s /usr/lib/libcwiid.so.1.0 libcwiid.so.1.0

I also installed everything on this page: https://help.ubuntu.com/community/CWiiD.
