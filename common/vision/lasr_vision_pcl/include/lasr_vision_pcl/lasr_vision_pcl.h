
#include <ros/ros.h>

class PCLNode
{

public:
    PCLNode(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;

private:
    bool segmentPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud);

public:
    bool segmentPlaneService(lasr_vision_pcl::SegmentPlane::Request &req, lasr_vision_pcl::SegmentPlane::Response &res);
};