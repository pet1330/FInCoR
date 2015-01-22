#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

ros::Publisher pub;
/*
void callback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& right_image) {

}
*/
int main(int argc, char** argv) {
/*
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    message_filters::Subscriber<Image> left_sub(nh, "/left/rgb/image_color", 1);
    message_filters::Subscriber<Image> right_sub(nh, "/right/rgb/image_color", 1);
    pub = nh.advertise<Image>("tranformed",1);
    typedef message_filters::sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
    */
}
