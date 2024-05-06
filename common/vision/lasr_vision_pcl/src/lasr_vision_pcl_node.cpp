#include "lasr_vision_pcl/lasr_vision_pcl.h"

#include <ros/ros.h>

PCLNode::PCLNode(ros::NodeHandle &nh) : nh_(nh) {}

bool PCLNode::segmentPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud)
{
    return false;
}

bool PCLNode::segmentPlaneService(lasr_vision_pcl::SegmentPlane::Request &req, lasr_vision_pcl::SegmentPlane::Response &res)
{
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lasr_vision_pcl");
    ros::NodeHandle nh;

    PCLNode pcl_node(nh);

    ros::spin();

    return 0;
}