#include "lasr_vision_pcl/SegmentPlane.h"
#include "lasr_vision_pcl/RemoveOutliers.h"

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PCLNode
{

public:
    PCLNode(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    ros::ServiceServer segment_plane_service_;
    ros::ServiceServer remove_outliers_service_;

private:
    bool segmentPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud);
    bool removeOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud);

public:
    bool segmentPlaneService(lasr_vision_pcl::SegmentPlane::Request &req, lasr_vision_pcl::SegmentPlane::Response &res);
    bool removeOutliersService(lasr_vision_pcl::RemoveOutliers::Request &req, lasr_vision_pcl::RemoveOutliers::Response &res);
};