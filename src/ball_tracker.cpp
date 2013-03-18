/*
 * File: mecanumbot/src/ball_tracker.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: August 2012
 * Description: This application uses tracks a red ball in a point cloud. The position is published as a tf to "target_object."
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "CloudHelpers.cpp"
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

CloudHelpers cloud_helpers;
sensor_msgs::PointCloud2 ros_cloud;
pcl::ExtractIndices<pcl::PointXYZRGB> extract;
ros::Publisher pub_debug;
Color red;
visualization_msgs::Marker marker;

void processCloud(const sensor_msgs::PointCloud2ConstPtr& input)
{
	ROS_INFO("processCloud");

	// ROS to PCL
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	fromROSMsg(*input, *cloud);
	
	/*
	// Separate largest contiguous plane (floor) via RANSAC
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.01);
	seg.setOptimizeCoefficients (true); // optional

	// Filter out surface
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*cloud);
	
	// Remove anything past surface
	pcl::PointIndices::Ptr behindPlane = cloud_helpers.filterBehindPlane(cloud, coefficients);
	extract.setInputCloud (cloud);
	extract.setIndices (behindPlane);
	extract.setNegative (false);
	extract.filter (*cloud);
	*/
	
	// Pick out red points
	pcl::PointIndices::Ptr redPoints = cloud_helpers.filterColor(cloud, red);
	extract.setInputCloud (cloud);
	extract.setIndices (redPoints);
	extract.setNegative (false);
	extract.filter (*cloud);

	// Statistical outlier filter (Useful for carpet)
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (0.1);
	sor.filter (*cloud);
	
	// Segment by distance
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
	clusters = cloud_helpers.segmentByDistance(cloud);
	if(clusters.size() > 0) cloud = clusters[0];
	//std::cerr << "clusters: " << clusters.size() << std::endl;
	
	// Publish debug cloud
	toROSMsg(*cloud, ros_cloud);
	pub_debug.publish(ros_cloud);
	
	// Publish ball location information
	//TODO
	
	// Update rviz marker
	uint8_t r, g, b;
	pcl::PointXYZRGB avg = cloud_helpers.averageCloud(cloud, r, g, b);
	marker.pose.position.x = avg.x + 0.03; // fits the bloud better, not sure why
	marker.pose.position.y = avg.y;
	marker.pose.position.z = avg.z + 0.04; // add a couple cm since we only see the front

	return;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "ball_tracker");
	ros::NodeHandle nh;
	ROS_INFO("Starting up ball tracker...");

	ros::Rate r(10); // Hz

	// Create a ROS broadcaster for the target transform
	tf::TransformBroadcaster broadcaster;

	// Create a ROS publisher for the markers
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("target_object", 1);

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/camera/rgb/points", 1, processCloud);

	// Create a ROS publisher for the output point cloud
	pub_debug = nh.advertise<sensor_msgs::PointCloud2> ("ball_tracker/debug", 1);
	
	// Build Marker
	marker.header.frame_id = "openni_depth_optical_frame";
	marker.ns = "targets";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.scale.x = 0.16;
	marker.scale.y = 0.16;
	marker.scale.z = 0.16;
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.8f;
	
	// Define red color
	red.r_u = 100;
	red.r_s = 24;
	red.g_u = 25;
	red.g_s = 25;
	red.b_u = 15;
	red.b_s = 15;

	while(nh.ok()) {
		ros::spinOnce();
		
		// openni_camera to target_object
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 0.0, 0)
			),
			ros::Time::now(),"openni_camera", "target_object")
		);
		
		// publish a marker to view in rviz
		marker.header.stamp = ros::Time::now();
		marker_pub.publish(marker);
		
		r.sleep();
	}
}
