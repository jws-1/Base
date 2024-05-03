#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import PointCloud2

from tiago_kcl_moveit_grasp.srv import SegmentPlane

rospy.init_node("plane_segment_pub")
pub = rospy.Publisher("/plane_segment", PointCloud2, queue_size=10)

segment_plane = rospy.ServiceProxy("pcl_object_segmenter/segment_plane", SegmentPlane)


def segment(pcl):
    segmented_pcl = segment_plane(pcl)
    pub.publish(segmented_pcl.cloud_out)


sub = rospy.Subscriber(
    "/xtion/depth_registered/points", PointCloud2, segment, queue_size=10
)

rospy.spin()
