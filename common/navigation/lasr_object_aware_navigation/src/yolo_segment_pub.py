#!/usr/bin/env python3

import rospy
import numpy as np
import ros_numpy as rnp
import cv2
from lasr_vision_msgs.srv import YoloDetection3D
from sensor_msgs.msg import PointCloud2

rospy.init_node("yolo_segment_pub")

yolo = rospy.ServiceProxy("yolov8/detect3d", YoloDetection3D)
rospy.wait_for_service("yolov8/detect3d")

pcl_pub = rospy.Publisher("/pcl_segmented", PointCloud2, queue_size=1)

OBJECTS = ["banana"]


def pcl_to_cv2(pcl_msg):
    pcl = np.frombuffer(pcl_msg.data, dtype=np.uint8)
    pcl = pcl.reshape(pcl_msg.height, pcl_msg.width, 32)

    frame = np.frombuffer(pcl_msg.data, dtype=np.uint8)
    frame = frame.reshape(pcl_msg.height, pcl_msg.width, 32)
    frame = frame[:, :, 16:19].copy()
    frame = np.ascontiguousarray(frame, dtype=np.uint8)

    return frame


def mask(pcl_msg, detection):
    frame = pcl_to_cv2(pcl_msg)
    contours = np.array(detection.xyseg).reshape(-1, 2)
    mask_ = np.zeros(frame.shape[:2], np.uint8)
    cv2.fillPoly(mask_, [contours], 255)
    return mask_


def detect_objects_and_segment(pcl_msg):
    resp = yolo(pcl_msg, "yolov8n-seg.pt", 0.1, 0.1)
    masks = []
    for detection in resp.detected_objects:
        if detection.name in OBJECTS:
            masks.append(mask(pcl_msg, detection))

    if not masks:
        mask_ = np.zeros((pcl_msg.height, pcl_msg.width), np.uint8)
    else:
        mask_ = masks[0]
        for m in masks[1:]:
            mask_ = cv2.bitwise_or(mask_, m)

    print(mask_.shape)
    pcl_np = rnp.point_cloud2.pointcloud2_to_array(pcl_msg).copy()

    negative_indices = np.where(mask_ == 0)
    print(negative_indices[0].shape, negative_indices[1].shape)
    for x, y in zip(*negative_indices):
        pcl_np["x"][x, y] = np.nan
        pcl_np["y"][x, y] = np.nan
        pcl_np["z"][x, y] = np.nan

    pcl_cropped = rnp.point_cloud2.array_to_pointcloud2(
        pcl_np, stamp=pcl_msg.header.stamp, frame_id=pcl_msg.header.frame_id
    )
    pcl_pub.publish(pcl_cropped)


sub = rospy.Subscriber(
    "/xtion/depth_registered/points",
    PointCloud2,
    detect_objects_and_segment,
    queue_size=10,
)

rospy.spin()
