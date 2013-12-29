Mecanumbot Usage Guide
======================

The core mecanumbot code is started with `roslaunch mecanumbot core.launch`. This will bring up the transform publisher, the laser scanner, and the rosserial client to the Arduino onboard the mecanumbot.

Launch rviz with the custom mecanumbot config with 'roslaunch mecanumbot rviz.launch'. To do this on a remote machine, be sure to run `export ROS_MASTER_URI=http://jvillbrandt-robot:11311` in your bash session first.

You can control the mecanumbot with a wireless Xbox remote by running `roslaunch mecanumbot teleop_xbox.launch`. To instead have the mecanumbot autonomously follow a red ball, run TODO and TODO instead.

## Using Screen

During the development, it is often easiest to manually launch the pertinent parts of the robot instead of starting everything on bootup. Using `screen` over SSH is the preferred manor of manual launching since it is robust against disconnects. To start the robot with this method, open a terminal on another computer (not the robot) and type the following:

    ssh -Y jvillbrandt-robot.local
    screen
    roslaunch mecanumbot core.launch
    # [ctrl-a] [c]
    roslaunch mecanumbot kinect.launch
    # [ctrl-a] [c]
    # other stuff here
    # [ctrl-a] [n] # go to the next window

If you drop the connection to the robot, reconnect with this:

    screen -ls
    screen -r DESIRED_SCREEN_ID

To learn more about screen, check out [this tutorial](http://www.rackaid.com/resources/linux-screen-tutorial-and-how-to/).

## Auto-start ROS

We can automatically launch the `core.launch` launch file by using upstart. Clearpath Robotics made a nice little package called [robot_upstart](https://github.com/clearpathrobotics/robot_upstart) to help with this. With this package installed, simply run the following command:

    rosrun robot_upstart install mecanumbot/launch/core.launch

Just for my own reference, a second auto-start pacakge called [ros-system-daemon-hydro](https://github.com/TurtleBot-Mfg/ros-system-daemon-hydro) is available in the turtlebot repos.
