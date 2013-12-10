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
#include <math.h>

// constrain function
// http://stackoverflow.com/questions/8941262/constrain-function-port-from-arduino
template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if(x < a) return a;
    else if(b < x) return b;
    else return x;
}

class TargetFollower
{
    public:
        TargetFollower();
        void spin();

    private:
        ros::NodeHandle nh;
        ros::Publisher vel_pub;
        tf::TransformListener listener;

        double follow_distance, linear_x_scale, linear_x_max, angular_z_scale, angular_z_max;
        geometry_msgs::Twist msg;
};

TargetFollower::TargetFollower()
{
    // load parameters
    ros::NodeHandle nh_priv("~");
    nh_priv.param("follow_distance", follow_distance, 1.0); // m
    nh_priv.param("linear_x_scale", linear_x_scale, 0.5);
    nh_priv.param("linear_x_max", linear_x_max, 1.0); // assumed symmetric around 0
    nh_priv.param("angular_z_scale", angular_z_scale, 2.0);
    nh_priv.param("angular_z_max", angular_z_max, 2.0);; // assumed symmetric around 0
    
    // lets show em what we got
    ROS_INFO_STREAM("param follow_distance: " << follow_distance);
    ROS_INFO_STREAM("param linear_x_scale: " << linear_x_scale);
    ROS_INFO_STREAM("param linear_x_max: " << linear_x_max);
    ROS_INFO_STREAM("param angular_z_scale: " << angular_z_scale);
    ROS_INFO_STREAM("param angular_z_max: " << angular_z_max);

    // connects subs and pubs
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // static message parameters
    msg.linear.y = 0; // m/s
}

void TargetFollower::spin()
{
    while(ros::ok())
    {
    	// default cmd_vel
        msg.linear.x = 0.0; // m/s
        msg.angular.z = 0.0; // m/s

        // check for transform from robot to target
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/base_link", "/target", ros::Time(0), transform);
            //ROS_INFO_STREAM("transform: x=" << transform.getOrigin().x() << ", y=" << transform.getOrigin().y());

            // calculate the position error
            double linear_x_error = transform.getOrigin().x() - follow_distance;
            double angular_z_error = 0.0;
            if(transform.getOrigin().x() != 0)
            	angular_z_error = tan(transform.getOrigin().y() / transform.getOrigin().x());

            // create the cmd_vel message
            msg.linear.x = linear_x_scale * linear_x_error; // m/s
            msg.linear.x = constrain(msg.linear.x, -1.0*linear_x_max, linear_x_max);
            msg.angular.z = angular_z_scale * angular_z_error; // rad/s
            msg.angular.z = constrain(msg.angular.z, -1.0*angular_z_max, angular_z_max);
        }
        catch(tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        // publish the message
        vel_pub.publish(msg);

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
