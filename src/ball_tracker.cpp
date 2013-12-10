/*
 * File: mecanumbot/src/ball_tracker.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: December 2013
 * Description: This application extracts a red ball from a point cloud.
 */

// Hydro tips:
// http://wiki.ros.org/pcl/Tutorials#line-197
// http://wiki.ros.org/hydro/Migration#PCL

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <mecanumbot/LightControl.h>
#include "cloud_helpers.cpp"
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

class BallTracker
{
    public:
        BallTracker();
        void spin();

    private:
        void cloudCallback(const pcl::PCLPointCloud2ConstPtr& cloud_in);

        ros::NodeHandle nh;
        ros::Publisher cloud_pub;
        ros::Subscriber cloud_sub;
        //ros::Publisher light_pub;
        // broadcaster
        // rviz marker
        
        // bool enabled, flipped;
        // int linear_x_axis, linear_y_axis, angular_z_axis;
        // int linear_y_left_button, linear_y_right_button, boost_button, enable_button;
        // double linear_x_scale, linear_y_scale, angular_z_scale, preboost_scale, force_pub_rate;

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        cloud_helpers::Color red;
        // geometry_msgs::Twist msg;
        // mecanumbot::LightControl light_msg;
        // ros::Time last_pub_time;
        // ros::Duration force_pub_period;
};


BallTracker::BallTracker()//:
    // enabled(false),
    // flipped(false)
{
    // load parameters
    ros::NodeHandle nh_priv("~");
    // nh_priv.param("linear_x_scale", linear_x_scale, 0.1); // scaling from integer 10 (1g in m/s/s); 0.1 -> 1m/s
    // nh_priv.param("linear_y_scale", linear_y_scale, 0.8);
    // nh_priv.param("angular_z_scale", angular_z_scale, 0.2);
    // nh_priv.param("preboost_scale", preboost_scale, 1.0);
    // nh_priv.param("force_pub_rate", force_pub_rate, 10.0); // Hz
    // nh_priv.param("linear_x_axis", linear_x_axis, -1);
    // nh_priv.param("linear_y_axis", linear_y_axis, -1);
    // nh_priv.param("angular_z_axis", angular_z_axis, -1);
    // nh_priv.param("linear_y_left_button", linear_y_left_button, -1);
    // nh_priv.param("linear_y_right_button", linear_y_right_button, -1);
    // nh_priv.param("boost_button", boost_button, -1);
    // nh_priv.param("enable_button", enable_button, -1);
    // if(enable_button < 0) enabled = true;
    
    // lets show em what we got
    // ROS_INFO_STREAM("param linear_x_scale: " << linear_x_scale);
    // ROS_INFO_STREAM("param linear_y_scale: " << linear_y_scale);
    // ROS_INFO_STREAM("param angular_z_scale: " << angular_z_scale);
    // ROS_INFO_STREAM("param preboost_scale: " << preboost_scale);
    // ROS_INFO_STREAM("param force_pub_rate: " << force_pub_rate);
    // ROS_INFO_STREAM("param linear_x_axis: " << linear_x_axis);
    // ROS_INFO_STREAM("param linear_y_axis: " << linear_y_axis);
    // ROS_INFO_STREAM("param angular_z_axis: " << angular_z_axis);
    // ROS_INFO_STREAM("param linear_y_left_button: " << linear_y_left_button);
    // ROS_INFO_STREAM("param linear_y_right_button: " << linear_y_right_button);
    // ROS_INFO_STREAM("param boost_button: " << boost_button);
    // ROS_INFO_STREAM("param enable_button: " << enable_button);
    
    // Define red color
    red.r_u = 100;
    red.r_s = 24;
    red.g_u = 25;
    red.g_s = 25;
    red.b_u = 15;
    red.b_s = 15;

    // connects subs and pubs
    cloud_sub = nh.subscribe("cloud_in", 1, &BallTracker::cloudCallback, this);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
    // light_pub = nh.advertise<mecanumbot::LightControl>("light_control", 1);

    // intial message that might be pushed out by the force_pub_rate
    // msg.linear.x = 0; // m/s
    // msg.angular.z = 0; // rad/s
    // msg.linear.y = 0; // m/s
}

void BallTracker::spin()
{
    while(ros::ok())
    {
        // xbox controller doesn't transmit if value is the same, force publishing the last value
        // if(ros::Time::now() > last_pub_time + force_pub_period) {
  //           vel_pub.publish(msg);
        //     last_pub_time = ros::Time::now();
        // }
        
        // call all waiting callbacks
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
}

void BallTracker::cloudCallback(const pcl::PCLPointCloud2ConstPtr& cloud_in)
{
    // enable / disable cmd_vel commands
    // if(enable_button >= 0) {
    //     if(joy->buttons[enable_button] == 0) flipped = false;
    //     if(joy->buttons[enable_button] == 1 && !flipped) {
    //         enabled = !enabled;
    //         flipped = true;
    //     }
    // }
    
    // // generate cmd_vel from accelerometers
    // msg.linear.x = 0; // m/s
    // msg.angular.z = 0; // rad/s
    // msg.linear.y = 0; // m/s
    // if(enabled) {
    //     if(linear_x_axis >= 0) msg.linear.x = linear_x_scale * joy->axes[linear_x_axis]; // m/s
    //     if(linear_y_axis >= 0) msg.linear.y = linear_y_scale * joy->axes[linear_y_axis]; // m/s
    //     else if(linear_y_left_button >= 0 && joy->buttons[linear_y_left_button] == 1) msg.linear.y = linear_y_scale; // m/s
    //     else if(linear_y_right_button >= 0 && joy->buttons[linear_y_right_button] == 1) msg.linear.y = -1 * linear_y_scale; // m/s
    //     if(angular_z_axis >= 0) msg.angular.z = -1 * angular_z_scale * joy->axes[angular_z_axis]; // rad/s
        
    //     if(boost_button >= 0 && joy->buttons[boost_button] == 0) {
    //         msg.linear.x = msg.linear.x * preboost_scale;
    //         msg.linear.y = msg.linear.y * preboost_scale;
    //         msg.angular.z = msg.angular.z * preboost_scale;
    //     }
    // }
    // vel_pub.publish(msg);
    // last_pub_time = ros::Time::now();
    
    // // lights
    // light_msg.forward_brightness = 255 - ((joy->axes[5] + 1) * 255 / 2);
    // if(joy->buttons[2] == 1) light_msg.internal_brightness = 127;
    // else light_msg.internal_brightness = 0;
    // if(joy->buttons[1] == 1) light_msg.mood_color = 3;
    // else if(joy->buttons[3] == 1) light_msg.mood_color = 2;
    // else if(joy->buttons[0] == 1) light_msg.mood_color = 1;
    // else light_msg.mood_color = 0;
    // light_pub.publish(light_msg);

    // downsample
    // pcl::PCLPointCloud2::Ptr cloud_downsampled (new pcl::PCLPointCloud2);
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud(cloud_in);
    // sor.setLeafSize(0.05, 0.05, 0.05);
    // sor.filter(*cloud_downsampled);

    // convert
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_filtered);

    // Pick out red points
    pcl::PointIndices::Ptr redPoints = cloud_helpers::filterColor(cloud_filtered, red);
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(redPoints);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);

    // Statistical outlier filter (Useful for carpet)
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.01); // smaller is more restrictive
    sor.filter(*cloud_filtered);
    
    // Segment by distance
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    clusters = cloud_helpers::segmentByDistance(cloud_filtered);
    ROS_INFO_STREAM("clusters: " << clusters.size());
    if(clusters.size() > 0) cloud_filtered = clusters[0];

    // convert
    pcl::PCLPointCloud2::Ptr cloud_out (new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud_filtered, *cloud_out);

    // publish
    cloud_pub.publish(cloud_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_tracker");
    BallTracker ball_tracker;
    ball_tracker.spin();
}
