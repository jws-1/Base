#!/usr/bin/env python3

import rospy

from lasr_vision_msgs.srv import (
    StartTracker,
    StartTrackerRequest,
    StartTrackerResponse,
    UpdateTracker,
    UpdateTrackerRequest,
    UpdateTrackerResponse,
    YoloDetection,
    YoloDetectionRequest,
    YoloDetectionResponse,
)

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from sensor_msgs.msg import Image

rospy.init_node("test_tracking")

start_tracker = rospy.ServiceProxy("start_tracker", StartTracker)
update_tracker = rospy.ServiceProxy("update_tracker", UpdateTracker)
stop_tracker = rospy.ServiceProxy("stop_tracker", Empty)
yolo = rospy.ServiceProxy("yolov8/detect", YoloDetection)

start_tracker(StartTrackerRequest(30, True, "mobilenet", ""))

prev_time = rospy.Time.now()
time_between_updates = rospy.Duration(1.0)


def create_yolo_req(img_msg):
    return YoloDetectionRequest(
        img_msg,
        "yolov8n-seg.pt",
        0.7,
        0.5,
    )


def image_callback(msg):
    global prev_time
    global time_between_updates
    if rospy.Time.now() - prev_time < time_between_updates:
        return
    prev_time = rospy.Time.now()
    yolo_req = create_yolo_req(msg)
    yolo_resp = yolo(yolo_req)
    update_tracker(
        UpdateTrackerRequest(
            msg, [det for det in yolo_resp.detected_objects if det.name == "person"]
        )
    )


image_sub = rospy.Subscriber("usb_cam/image_raw", Image, image_callback)

rospy.spin()

stop_tracker(EmptyRequest())
