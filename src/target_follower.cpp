/*
 * File: mecanumbot/src/target_follower.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: December 2013
 * Description: This application publishes a cmd_vel message to follow a target.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>



#include <geometry_msgs/Twist.h>
#include <mecanumbot/LightControl.h>

class TargetFollower
{
    public:
        TargetFollower();
        void spin();

    private:
        ros::NodeHandle nh;
        tf::TransformListener listener;
};


TargetFollower::TargetFollower()
{
}

void TargetFollower::spin()
{
    while(ros::ok())
    {
        // check for transform from robot to target
        tf::StampedTransform transform;
	    try {
	        listener.lookupTransform("/base_link", "/target", ros::Time(0), transform);

	        // create the cmd_vel message
	        ROS_INFO_STREAM("transform: x=" << transform.getOrigin().x() << ", y=" << transform.getOrigin().y());
	    }
	    catch(tf::TransformException ex) {
	        ROS_ERROR("%s",ex.what());
	    }

        // sleep to keep CPU happy
        ros::Duration(0.1).sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_follower");
    TargetFollower target_follower;
    target_follower.spin();
}
