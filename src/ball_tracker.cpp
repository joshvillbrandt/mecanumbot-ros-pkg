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
#include <visualization_msgs/Marker.h>
#include "cloud_helpers.cpp"
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
// PCL-ROS auto type conversions
#include <pcl_ros/point_cloud.h>

class BallTracker
{
    public:
        BallTracker();
        void spin();

    private:
        void cloudCallback(const pcl::PCLPointCloud2ConstPtr& cloud_in);

        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub;
        ros::Publisher marker_pub;
        ros::Publisher light_pub;
        // broadcaster
        
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        cloud_helpers::Color red;
        mecanumbot::LightControl light_msg;
        visualization_msgs::Marker marker;
};


BallTracker::BallTracker()
{
    // load parameters
    ros::NodeHandle nh_priv("~");
    // nh_priv.param("linear_y_scale", linear_y_scale, 0.8);
    
    // lets show em what we got
    // ROS_INFO_STREAM("param linear_y_scale: " << linear_y_scale);
    
    // Define red color
    red.r_u = 105;
    red.r_s = 24;
    red.g_u = 20;
    red.g_s = 25;
    red.b_u = 15;
    red.b_s = 15;

    // connects subs and pubs
    cloud_sub = nh.subscribe("cloud_in", 1, &BallTracker::cloudCallback, this);
    cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("cloud_out", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);
    light_pub = nh.advertise<mecanumbot::LightControl>("light_control", 1);
    
    // static marker values
    marker.ns = "targets";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 0.16;
    marker.scale.y = 0.16;
    marker.scale.z = 0.16;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    
    // static light message values
    light_msg.forward_brightness = 0;
    light_msg.internal_brightness = 0;
}

void BallTracker::spin()
{
    while(ros::ok())
    {
        // call all waiting callbacks
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
}

void BallTracker::cloudCallback(const pcl::PCLPointCloud2ConstPtr& cloud_in)
{

    // convert
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_filtered);

    // pick out red points
    pcl::PointIndices::Ptr redPoints = cloud_helpers::filterColor(cloud_filtered, red);
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(redPoints);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);

    // statistical outlier filter (Useful for carpet)
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.01); // smaller is more restrictive
    sor.filter(*cloud_filtered);
    
    // segment by distance
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    clusters = cloud_helpers::segmentByDistance(cloud_filtered);
    ROS_INFO_STREAM("clusters: " << clusters.size());
    if(clusters.size() > 0) cloud_filtered = clusters[0];

    // publish filtered cloud for debugging
    cloud_pub.publish(*cloud_filtered);
    
    // Update rviz marker
    if(clusters.size() > 0) {
	    uint8_t r, g, b;
	    pcl::PointXYZRGB avg = cloud_helpers::averageCloud(cloud_filtered, r, g, b);
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.position.x = avg.x + 0.03; // fits the cloud better, because camera isn't calibrated
	    marker.pose.position.y = avg.y;
	    marker.pose.position.z = avg.z + 0.04; // add a couple cm since we only see the front
    }
    else marker.action = visualization_msgs::Marker::DELETE;
    marker.header = pcl_conversions::fromPCL(cloud_filtered->header);
    marker_pub.publish(marker);

    // publish lights to indicate status
    if(clusters.size() > 0) light_msg.mood_color = 1;
    else light_msg.mood_color = 3;
    light_pub.publish(light_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_tracker");
    BallTracker ball_tracker;
    ball_tracker.spin();
}
