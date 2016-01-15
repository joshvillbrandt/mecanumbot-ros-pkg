/*
 * File: mecanumbot/src/teleop_xbox.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: October 2012 - March 2013
 * Description: This application produces a twist message from an Xbox controller joy message.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mecanumbot/RobotHazardsEnable.h>

class TeleopXbox
{
    public:
        TeleopXbox();
        void spin();

    private:
        void callHazardsEnable(bool enabled);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

        ros::NodeHandle nh;
        ros::Subscriber joy_sub;
        ros::Publisher vel_pub;
        ros::ServiceClient hazards_cli;
        
        int linear_x_axis, linear_y_axis, angular_z_axis;
        int linear_y_left_button, linear_y_right_button, boost_button, enable_button, disable_button;
        double linear_x_scale, linear_y_scale, angular_z_scale, preboost_scale, force_pub_rate;

        ros::Time last_pub_time;
        ros::Duration force_pub_period;
        geometry_msgs::Twist msg;
        int previous_enable_button_state, previous_disable_button_state;
};

TeleopXbox::TeleopXbox():
    previous_enable_button_state(0),
    previous_disable_button_state(0)
{
    // load parameters
    ros::NodeHandle nh_priv("~");
    nh_priv.param("linear_x_scale", linear_x_scale, 0.1); // scaling from integer 10 (1g in m/s/s); 0.1 -> 1m/s
    nh_priv.param("linear_y_scale", linear_y_scale, 0.8);
    nh_priv.param("angular_z_scale", angular_z_scale, 0.2);
    nh_priv.param("preboost_scale", preboost_scale, 1.0);
    nh_priv.param("force_pub_rate", force_pub_rate, 10.0); // Hz
    nh_priv.param("linear_x_axis", linear_x_axis, -1);
    nh_priv.param("linear_y_axis", linear_y_axis, -1);
    nh_priv.param("angular_z_axis", angular_z_axis, -1);
    nh_priv.param("linear_y_left_button", linear_y_left_button, -1);
    nh_priv.param("linear_y_right_button", linear_y_right_button, -1);
    nh_priv.param("boost_button", boost_button, -1);
    nh_priv.param("enable_button", enable_button, -1);
    nh_priv.param("disable_button", disable_button, -1);
    
    // lets show em what we got
    ROS_INFO_STREAM("param linear_x_scale: " << linear_x_scale);
    ROS_INFO_STREAM("param linear_y_scale: " << linear_y_scale);
    ROS_INFO_STREAM("param angular_z_scale: " << angular_z_scale);
    ROS_INFO_STREAM("param preboost_scale: " << preboost_scale);
    ROS_INFO_STREAM("param force_pub_rate: " << force_pub_rate);
    ROS_INFO_STREAM("param linear_x_axis: " << linear_x_axis);
    ROS_INFO_STREAM("param linear_y_axis: " << linear_y_axis);
    ROS_INFO_STREAM("param angular_z_axis: " << angular_z_axis);
    ROS_INFO_STREAM("param linear_y_left_button: " << linear_y_left_button);
    ROS_INFO_STREAM("param linear_y_right_button: " << linear_y_right_button);
    ROS_INFO_STREAM("param boost_button: " << boost_button);
    ROS_INFO_STREAM("param enable_button: " << enable_button);
    ROS_INFO_STREAM("param disable_button: " << disable_button);

    // connects subs and pubs
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopXbox::joyCallback, this);
    hazards_cli = nh.serviceClient<mecanumbot::RobotHazardsEnable>("robot_hazards_enable");

    // always enable hazards if no enable button is set
    if(enable_button < 0) {
        callHazardsEnable(true);
    }

    // intial message that might be pushed out by the force_pub_rate
    msg.linear.x = 0; // m/s
    msg.linear.y = 0; // m/s
    msg.angular.z = 0; // rad/s

    // setup 
    last_pub_time = ros::Time::now();
    force_pub_period = ros::Duration(1.0 / force_pub_rate);
}

void TeleopXbox::spin()
{
    while(ros::ok())
    {
        // xbox controller doesn't transmit if value is the same, force publishing the last value
        if(ros::Time::now() > last_pub_time + force_pub_period) {
            vel_pub.publish(msg);
            last_pub_time = ros::Time::now();
        }
        
        // call all waiting callbacks
        ros::Duration(0.05).sleep(); // this sets a crude max rate of 20Hz
        ros::spinOnce();
    }
}

void TeleopXbox::callHazardsEnable(bool enable)
{
    // generate request
    mecanumbot::RobotHazardsEnable srv;
    srv.request.enable = enable;

    // send request
    if(hazards_cli.call(srv)) {
        if(enable) {
            ROS_INFO("Hazards enabled");
        }
        else {
            ROS_INFO("Hazards disabled");
        }
    }
    else {
        ROS_ERROR("Failed to update hazards");
    }
}

void TeleopXbox::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // enable buton
    if(enable_button >= 0) {
        if(joy->buttons[enable_button] == 1 && previous_enable_button_state == 0) {
            callHazardsEnable(true);
        }
        previous_enable_button_state = joy->buttons[enable_button];
    }

    // disable buton
    if(disable_button >= 0) {
        if(joy->buttons[disable_button] == 1 && previous_disable_button_state == 0) {
            callHazardsEnable(false);
        }
        previous_disable_button_state = joy->buttons[disable_button];
    }

    // generate cmd_vel
    msg.linear.x = 0; // m/s
    msg.linear.y = 0; // m/s
    msg.angular.z = 0; // rad/s

    if(linear_x_axis >= 0) msg.linear.x = linear_x_scale * joy->axes[linear_x_axis]; // m/s
    if(linear_y_axis >= 0) msg.linear.y = linear_y_scale * joy->axes[linear_y_axis]; // m/s
    else if(linear_y_left_button >= 0 && joy->buttons[linear_y_left_button] == 1) msg.linear.y = linear_y_scale; // m/s
    else if(linear_y_right_button >= 0 && joy->buttons[linear_y_right_button] == 1) msg.linear.y = -1 * linear_y_scale; // m/s
    if(angular_z_axis >= 0) msg.angular.z = -1 * angular_z_scale * joy->axes[angular_z_axis]; // rad/s
    
    if(boost_button >= 0 && joy->buttons[boost_button] == 0) {
        msg.linear.x = msg.linear.x * preboost_scale;
        msg.linear.y = msg.linear.y * preboost_scale;
        msg.angular.z = msg.angular.z * preboost_scale;
    }

    // publis the message
    vel_pub.publish(msg);
    last_pub_time = ros::Time::now();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_xbox");
    TeleopXbox teleop_xbox;
    teleop_xbox.spin();
}

