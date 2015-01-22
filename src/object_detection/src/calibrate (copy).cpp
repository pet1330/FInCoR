#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& left_pc, const PointCloud::ConstPtr& right_pc) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*left_pc,*left, indices);
    pcl::removeNaNFromPointCloud(*right_pc,*right, indices);
    //BOOST_FOREACH (const pcl::PointXYZ& pt, right->points)
   // {
   // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
   // }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(left);
    icp.setInputTarget(right);
    pcl::PointCloud<pcl::PointXYZ> Final;
    std::cout << "===HERE===" << std::endl;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;

    message_filters::Subscriber<PointCloud> left_sub(nh, "/left/depth_registered/points", 1);
    message_filters::Subscriber<PointCloud> right_sub(nh, "/right/depth_registered/points", 1);
    typedef message_filters::sync_policies::ApproximateTime<PointCloud, PointCloud> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
}
