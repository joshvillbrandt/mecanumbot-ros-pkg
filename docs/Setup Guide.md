# Mecanumbot Setup Guide

Complete the following steps to install the mecanumbot package on a new robot. If you are not familiar with ROS, please complete the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) before continuing.

## Install Ubuntu 14.04

Use your favorite method to install a fresh copy of Ubuntu Trusty. Pay special attention to the hostname you choose during installation - this will be the hostname of your robot.

After installtion completes, install these additional, required packages:

```bash
sudo apt-get update
sudo apt-get install -y git openssh-server screen
```

## Optional Environment Setup

The following packages are optional and are intended for a development workstation:

```bash
# old gnome (don't forget to enable "Static Application Switcher" for alt-tab)
sudo apt-get install -y gnome-session-fallback compiz compiz-plugins-extra compizconfig-settings-manager

# sublime 3
sudo add-apt-repository ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get install -y sublime-text-installer

# zsh
sudo apt-get install -y zsh
chsh -s $(which zsh)
sh -c "$(wget https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"

# scm_breeze
git clone git://github.com/ndbroadbent/scm_breeze.git ~/.scm_breeze
~/.scm_breeze/install.sh
source ~/.zshrc
```

## Install ROS

If you have earlier versions of ROS installed, it might be best to uninstall them first. If you have problems with the subsequent install statements, try using aptitude instead of apt-get to resolve dependencies.

```
sudo apt-get purge ros-indigo-*
```

Follow the [ROS Jade Install Guide](http://wiki.ros.org/jade/Installation/Ubuntu) to get ROS up an running. This boils down to:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install -y ros-jade-desktop-full
sudo rosdep init
rosdep update
```

In addition to the standard desktop package, you'll want to install a few other packages that the mecanumbot package depends on. The first few lines of this are from the [PCL Ubuntu install page](http://pointclouds.org/downloads/linux.html).

```bash
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install -y libpcl-all ros-jade-pcl-ros ros-jade-joy ros-jade-openni-camera ros-jade-openni-launch ros-jade-rosserial-arduino ros-jade-rosserial ros-jade-robot-upstart ros-jade-rqt-robot-plugins ros-jade-hector-slam
```

To complete the install, source the ROS bash file. You'll probably want to stick this in your `~/.zshrc` file as well.

```bash
# for bash
echo "source /opt/ros/jade/setup.bash" >> ~/.bashrc
source ~/.bashrc

# for zsh
echo "source /opt/ros/jade/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

## Create a Workspace

Create a Catkin workspace by following the [create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) guide on ros.org. This boils down to:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```

Just like with the primary ROS install, you'll want to source the newly created bash setup file as well as adding it to your `~/.zshrc` file.

```bash
echo "source ~/catkin_ws/devel/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

## Clone the Mecanumbot Package

```bash
git clone https://github.com/joshvillbrandt/mecanumbot-ros-pkg.git ~/catkin_ws/src/mecanumbot
cd ~/catkin_ws
catkin_make
```

Set up unique identifiers for USB devices by linking to custom device rules and make sure users are a part of the dialup group

```bash
sudo cp ~/catkin_ws/src/mecanumbot/extra/99-usb-serial.rules /etc/udev/rules.d/

# to immediately reload the rules without restarting
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

## Set up the Kinect

The instructions were copied from the TurtleBot documentation at http://learn.turtlebot.com/2015/02/01/5/.

```bash
mkdir ~/kinectdriver
cd ~/kinectdriver
git clone https://github.com/avin2/SensorKinect
cd SensorKinect/Bin/
tar xvjf SensorKinect093-Bin-Linux-x64-v5.1.2.1.tar.bz2
cd Sensor-Bin-Linux-x64-v5.1.2.1/
sudo ./install.sh
```

## Set up the XV-11 Laser

```bash
cd ~/catkin_ws/src
git clone https://github.com/rohbotics/xv_11_laser_driver.git
sudo adduser $(whoami) dialout
cd ~/catkin_ws
catkin_make
```

## Set up upstart

To automatically launch ROS on startup:

```bash
rosrun robot_upstart install mecanumbot/launch/core.launch
```

Manually control the upstart job with these:

```bash
sudo service mecanumbot start
sudo service mecanumbot stop
```


Check the log like this:

```bash
sudo tail /var/log/upstart/mecanumbot.log -n 30
```

## Extra

These setup steps aren't always necessary.

### New udev Rules

Check out [this syntax guide](http://www.reactivated.net/writing_udev_rules.html#syntax) for creating new udev rules. To identify properties of currently plugged in devices, try a command like this:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}' | head -n1
```

Rules are automatically ran at startup. To automatically reload the rules without restarting, run:

```bash
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

### Installing Arduino Code

The following steps will allow you to update the onboard Arduino. Note that you must run `catkin_make` prior to this step in order to generate headers for custom message types.

```bash
mv ~/sketchbook ~/sketchbook_old
ln -s ~/catkin_ws/src/mecanumbot/arduino ~/sketchbook
cd ~/sketchbook/libraries
rm -rf ros_lib # just in case
rosrun rosserial_arduino make_libraries.py .
sudo apt-get install -y arduino arduino-core
```

Now open the [MecanumbotController](https://github.com/joshvillbrandt/MecanumbotController) sketch in the Arduino IDE, select board==Arduino Mega 2560 and the correct serial port (try `ls -l /dev | grep USB` and look for `arduino`) and click the upload button.

### Fix upower / Arduino Startup Bug

There seems to be a bug with the FTDI chip and the Ubuntu power saving component, upower. We can tell upower not to care about ttyUSB devices so that we can see the FTDI chip on startup. The following notes are taken from http://ten.homelinux.net/productivity/recipes/Arduino%20does%20not%20see%20ttyUSB0.

```bash
sudo vi /lib/udev/rules.d/95-upower-wup.rules
```

And look for an entry like:

> SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A80?????", ENV{UPOWER_VENDOR}="Watts Up, Inc.", ENV{UPOWER_PRODUCT}="Watts Up? Pro", ENV{UP_MONITOR_TYPE}="wup"

If present, comment the line (prepend the line by a "#") and then:

```bash
sudo /etc/init.d/udev restart

sudo killall upowerd

sudo lsof /dev/ttyUSB0
# should now report nothing
# Now, Arduino should give access to the USB interface.
```

More information [here](http://arduino.cc/forum/index.php?topic=104492.15 and http://bugs.debian.org/cgi-bin/bugreport.cgi?bug=586751).

### Pair the Wiimote

Run the following steps to start the wiimote node. The following steps are from http://www.ros.org/wiki/wiimote/Tutorials/StartingWiimoteNode.

```bash
sudo apt-get install -y python-rosdep
rosdep install wiimote
rosmake wiimote
rosrun wiimote wiimote_node.py
```

With a new Electric install, the libcwiid library will complain about an unknown symbol clock_gettime. The problem is with Electric's bundled version of the library. Run the following steps to install the stock library. From http://answers.ros.org/question/28084/problem-in-wiimote_nodepy/.

```bash
sudo apt-get install -y libcwiid1 libcwiid-dev
cd /opt/ros/electric/stacks/joystick_drivers/cwiid/cwiid/lib/
sudo mkdir old
sudo mv libcwiid.s* old
sudo ln -s /usr/lib/libcwiid.so libcwiid.so
sudo ln -s /usr/lib/libcwiid.so.1 libcwiid.so.1
sudo ln -s /usr/lib/libcwiid.so.1.0 libcwiid.so.1.0
```

I also installed everything on this page: https://help.ubuntu.com/community/CWiiD.
