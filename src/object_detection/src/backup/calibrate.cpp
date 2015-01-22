#include <ros/ros.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void left_callback(const PointCloud::ConstPtr& msg)
{
  printf ("LEFT Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
   // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

void right_callback(const PointCloud::ConstPtr& msg)
{
  printf ("RIGHT Cloud: width = %d, height = %d\n", msg->width, msg->height);
 // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber left_sub = nh.subscribe<PointCloud>("/left/depth_registered/points", 1, left_callback);
  ros::Subscriber right_sub = nh.subscribe<PointCloud>("/right/depth_registered/points", 1, right_callback);
  ros::spin();
}