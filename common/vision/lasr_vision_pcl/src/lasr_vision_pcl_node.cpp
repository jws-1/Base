#include "lasr_vision_pcl/lasr_vision_pcl.h"

#include <ros/ros.h>

PCLNode::PCLNode(ros::NodeHandle &nh) : nh_(nh)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lasr_vision_pcl");
    ros::NodeHandle nh;

    PCLNode pcl_node(nh);

    ros::spin();

    return 0;
}