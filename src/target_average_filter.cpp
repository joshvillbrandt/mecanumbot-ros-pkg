/*
 * File: mecanumbot/src/target_average_filter.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: December 2013
 * Description: Averages a transform over a certain window.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <list>

class TargetFilter
{
    public:
        TargetFilter();
        void spin();

    private:
        tf::TransformBroadcaster broadcaster;
        tf::TransformListener listener;

        std::string parent_frame, target_frame, target_filtered_frame;
        ros::Duration window_size, pub_period;

        ros::Time last_pub_time;
        std::list<tf::StampedTransform> transforms;
};

TargetFilter::TargetFilter()
{
    // temporary variable
    double window_size_in, pub_rate;

    // load parameters
    ros::NodeHandle nh_priv("~");
    nh_priv.param<std::string>("parent_frame", parent_frame, "/base_link");
    nh_priv.param<std::string>("target_frame", target_frame, "/target");
    nh_priv.param<std::string>("target_filtered_frame", target_filtered_frame, "/target_filtered");
    nh_priv.param("window_size", window_size_in, 1.0); // s
    nh_priv.param("pub_rate", pub_rate, 10.0); // Hz
    
    // lets show em what we got
    ROS_INFO_STREAM("param parent_frame: " << parent_frame);
    ROS_INFO_STREAM("param target_frame: " << target_frame);
    ROS_INFO_STREAM("param target_filtered_frame: " << target_filtered_frame);
    ROS_INFO_STREAM("param window_size: " << window_size_in);
    ROS_INFO_STREAM("param pub_rate: " << pub_rate);

    // setup 
    last_pub_time = ros::Time::now();
    window_size = ros::Duration(window_size_in);
    pub_period = ros::Duration(1.0 / pub_rate);
}

void TargetFilter::spin()
{
    while(ros::ok())
    {
        if(ros::Time::now() > last_pub_time + pub_period) {
            ros::Time now = ros::Time::now();

            // clear out stale transforms
            while(transforms.front().stamp_ < (now - window_size)) transforms.pop_front();

            // grab the latest transform
            try {
                tf::StampedTransform latest;
                listener.lookupTransform(parent_frame, target_frame, ros::Time(0), latest);
                transforms.push_back(latest);
            }
            catch(tf::TransformException ex) {
                //ROS_ERROR("%s",ex.what());
            }

            // compute the average
            tf::Vector3 origin_average = tf::Vector3(0, 0, 0);
            for(std::list<tf::StampedTransform>::iterator it=transforms.begin(); it != transforms.end(); ++it) {
                origin_average += it->getOrigin();
            }

            // publish the average
            if(transforms.size() > 0) {
                tf::Transform average;
                average.setOrigin( origin_average / transforms.size());
                average.setRotation( tf::Quaternion(0, 0, 0) );

                tf::StampedTransform latest = transforms.back(); // get last element
                broadcaster.sendTransform(tf::StampedTransform(average, latest.stamp_, latest.frame_id_, target_filtered_frame));
                last_pub_time = now;
            }
        }
        else ros::Duration(0.1).sleep();
        
        // call all waiting callbacks
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_average_filter");
    TargetFilter target_filter;
    target_filter.spin();
}
