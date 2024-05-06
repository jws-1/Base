#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import PointCloud2, LaserScan

from lasr_vision_pcl.srv import SegmentPlane, RemoveOutliers

rospy.init_node("plane_segment_pub")
pub = rospy.Publisher("/plane_segment", PointCloud2, queue_size=10)
rospy.wait_for_service("lasr_vision_pcl/segment_plane")
rospy.wait_for_service("lasr_vision_pcl/remove_outliers")
segment_plane = rospy.ServiceProxy("lasr_vision_pcl/segment_plane", SegmentPlane)
remove_outliers = rospy.ServiceProxy("lasr_vision_pcl/remove_outliers", RemoveOutliers)

# scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=10)


def segment(pcl):
    segmented_pcl = segment_plane(pcl)
    filtered_pcl = remove_outliers(segmented_pcl.cloud_segmented)
    pub.publish(filtered_pcl.cloud_filtered)

    # scan = rospy.wait_for_message("/scan_filtered", LaserScan)
    # scan_pcl = rospy.wait_for_message("/rgbd_scan", LaserScan)

    # combine the two scans


sub = rospy.Subscriber(
    "/xtion/depth_registered/points", PointCloud2, segment, queue_size=10
)

rospy.spin()
