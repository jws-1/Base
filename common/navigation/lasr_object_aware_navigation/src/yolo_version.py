#!/usr/bin/env python3

import rospy

from lasr_vision_msgs.srv import YoloDetection3D

yolo = rospy.ServiceProxy("yolov8/detect3d", YoloDetection3D)

"""
Detect objects
"""
