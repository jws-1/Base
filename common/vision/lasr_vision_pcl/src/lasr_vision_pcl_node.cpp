#include "lasr_vision_pcl/lasr_vision_pcl.h"

#include <ros/ros.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

PCLNode::PCLNode(ros::NodeHandle &nh) : nh_(nh)
{
    segment_plane_service_ = nh_.advertiseService("segment_plane", &PCLNode::segmentPlaneService, this);
    ROS_INFO_STREAM(ros::this_node::getName() << " is ready.");
}

bool PCLNode::segmentPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud)
{
    ROS_INFO("Performing planar segmentation on pointcloud with %lu points", in_cloud->points.size());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(in_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given pointcloud.");
        return false;
    }

    ROS_INFO("Model coefficients: %f %f %f %f\n", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    ROS_INFO("Model inliers: %lu\n", inliers->indices.size());

    // Copy the original cloud to the output cloud
    *out_cloud = *in_cloud;

    // Set inliers to nan
    for (auto i : inliers->indices)
    {
        out_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
        out_cloud->points[i].y = std::numeric_limits<float>::quiet_NaN();
        out_cloud->points[i].z = std::numeric_limits<float>::quiet_NaN();
    }

    ROS_INFO("Output cloud has %lu points\n", out_cloud->points.size());

    return true;
}

bool PCLNode::segmentPlaneService(lasr_vision_pcl::SegmentPlane::Request &req, lasr_vision_pcl::SegmentPlane::Response &res)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert the input pointcloud to PCL format
    pcl::fromROSMsg(req.cloud, *in_cloud);

    // Perform the planar segmentation
    if (!segmentPlane(in_cloud, out_cloud))
    {
        ROS_ERROR("Failed to segment plane");
        return false;
    }

    // Convert the output pointcloud to ROS format
    pcl::toROSMsg(*out_cloud, res.cloud_segmented);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lasr_vision_pcl");
    ros::NodeHandle nh("~");

    PCLNode pcl_node(nh);

    ros::spin();

    return 0;
}