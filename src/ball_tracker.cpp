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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <mecanumbot/LightControl.h>
#include <visualization_msgs/Marker.h>
#include "cloud_helpers.cpp"
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
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
        tf::TransformBroadcaster broadcaster;
        
        pcl::ExtractIndices<pcl::PointXYZHSV> extract;
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
    // convert to RGB from input
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_in2);

    // convert to HSV for processing
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZHSV>);
    cloud_helpers::PointCloudXYZRGBtoXYZHSV(*cloud_in2, *cloud_filtered);

    // pick out red points
    pcl::PointIndices::Ptr redPoints = cloud_helpers::filterByHue(cloud_filtered, 350, 20);
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(redPoints);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);

    // filter saturation
    pcl::PassThrough<pcl::PointXYZHSV> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("s");
    pass.setFilterLimits (0.8, 1.0);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);

    // filter value
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("v");
    pass.setFilterLimits (0.1, 0.95);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);

    // statistical outlier filter (Useful for carpet)
    pcl::StatisticalOutlierRemoval<pcl::PointXYZHSV> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.01); // smaller is more restrictive
    sor.filter(*cloud_filtered);
    
    // segment by distance
    std::vector<pcl::PointIndices> cluster_indices;
    cluster_indices = cloud_helpers::segmentByDistance(cloud_filtered);
    ROS_INFO_STREAM("clusters: " << cluster_indices.size());
    //std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr> clusters;
    //if(clusters.size() > 0) cloud_filtered = clusters[0];

    // chose one cluster as the target


    
    // // Update rviz marker and transform
    // marker.header = pcl_conversions::fromPCL(cloud_filtered->header);
    // if(clusters.size() > 0) {
    //     // find cloud average
    //     uint8_t r, g, b;
    //     pcl::PointXYZHSV avg = cloud_helpers::averageCloud(cloud_filtered, r, g, b);
    //     avg.x += 0.03; // m; adjust x position because my kinect isn't calibrated
    //     avg.z += 0.04; // m; adjust z because we are only seeing the front portion

    //     // broadcast transform
    //     tf::Transform transform;
    //     transform.setOrigin( tf::Vector3(avg.x, avg.y, avg.z) );
    //     transform.setRotation( tf::Quaternion(0, 0, 0) );
    //     broadcaster.sendTransform(tf::StampedTransform(transform, marker.header.stamp, marker.header.frame_id, "target"));

    //     // update marker properties
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = avg.x;
    //     marker.pose.position.y = avg.y;
    //     marker.pose.position.z = avg.z;
    // }
    // else marker.action = visualization_msgs::Marker::DELETE;
    // marker_pub.publish(marker);

    // publish filtered cloud for debugging
    if(cloud_pub.getNumSubscribers() > 0) {
        // convert to RGB for output
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_helpers::PointCloudXYZHSVtoXYZRGB(*cloud_filtered, *cloud_out);

        // ROS_INFO_STREAM("cloud size: " << cloud_filtered->points.size());
        // ROS_INFO_STREAM("average hue: " << cloud_helpers::calculateHue(cloud_filtered));
        cloud_pub.publish(*cloud_out);
    }

    // publish markers
    if(marker_pub.getNumSubscribers() > 0) {
        // cloud
        marker = cloud_helpers::getCloudMarker(cloud_filtered, cluster_indices[0]);
        marker.header = pcl_conversions::fromPCL(cloud_filtered->header);
        marker.pose.orientation.w = 1;
        marker.ns = "targets";
        marker.id = 1;//current_marker_id_++;
        marker_pub.publish(marker);

        // extract cloud
        pcl::PointIndices::Ptr cluster_indices_ptr (new pcl::PointIndices(cluster_indices[0]));
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(cluster_indices_ptr);
        extract.setNegative(false);
        extract.filter(*target_cloud);

        // label
        pcl::PointXYZHSV avg = cloud_helpers::getCloudAverage(target_cloud);
        visualization_msgs::Marker marker_label;
        marker_label.ns = "target_labels";
        marker_label.id = marker.id;
        marker_label.action = visualization_msgs::Marker::ADD;
        marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_label.lifetime = ros::Duration();
        marker_label.text = "#" + boost::to_string(marker_label.id);
        marker_label.header = pcl_conversions::fromPCL(cloud_filtered->header);
        marker_label.pose.position.x = avg.x;
        marker_label.pose.position.y = avg.y - 0.15;
        marker_label.pose.position.z = avg.z;
        //marker_label.color = marker.color;
        marker_label.color.r = marker_label.color.g = marker_label.color.b = marker_label.color.a = 1.0;
        ROS_INFO_STREAM("label: " << avg.x << ", " << avg.y << ", " << avg.z);
        marker_label.scale.z = 0.1;
        marker_pub.publish(marker_label);
    }

    // // publish lights to indicate status
    // if(clusters.size() > 0) light_msg.mood_color = 1;
    // else light_msg.mood_color = 3;
    // light_pub.publish(light_msg);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_tracker");
    BallTracker ball_tracker;
    ball_tracker.spin();
}
