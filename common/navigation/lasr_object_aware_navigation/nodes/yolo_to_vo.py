#!/usr/bin/env python3

import rospy

from lasr_vision_msgs.srv import (
    YoloDetection3D,
    YoloDetection3DRequest,
    YoloDetection3DResponse,
)

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from typing import Union, List, Tuple, Dict

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
import rosparam
import numpy as np
import tf2_ros as tf2
import tf2_geometry_msgs  # noqa
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
import cv2_pcl


class YOLOtoVO:

    _yolo: rospy.ServiceProxy
    _yolo_model: str
    _confidence: float
    _nms: float
    _classes: Union[None, List[str]]
    _start_srv: rospy.Service
    _stop_srv: rospy.Service
    _pcl_sub: Union[None, rospy.Subscriber]
    _tf_buffer: tf2.Buffer
    _tf_listener: tf2.TransformListener
    _cached_objects: List[Dict[str, Union[List[PointStamped], rospy.Time]]]
    _processing: bool
    _timer: rospy.Timer

    def __init__(
        self,
        yolo_model: str = "yolov8n-seg.pt",
        confidence: float = 0.3,
        nms: float = 0.3,
        classes: Union[None, List[str]] = None,
    ):

        self._yolo_model = yolo_model
        self._confidence = confidence
        self._nms = nms
        self._classes = classes

        self._yolo = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
        self._yolo.wait_for_service()

        self._start_srv = rospy.Service("/yolo_to_vo/start", Trigger, self._start)
        self._stop_srv = rospy.Service("/yolo_to_vo/stop", Trigger, self._stop)

        self._pcl_sub = None

        self._tf_buffer = tf2.Buffer()
        self._tf_listener = tf2.TransformListener(self._tf_buffer)

        self._cached_objects = []

        self._processing = False
        self._timer = rospy.Timer(rospy.Duration(1), self._timer_cb)

    def _start(self, _: TriggerRequest):
        self._pcl_sub = rospy.Subscriber(
            "/xtion/depth_registered/points", PointCloud2, self._pcl_cb
        )
        return TriggerResponse(success=True)

    def _tf_points(
        self, points: List[PointStamped], source_frame: str, target_frame: str
    ):
        trans = self._tf_buffer.lookup_transform(
            target_frame, source_frame, rospy.Time(0)
        )

        return [do_transform_point(point, trans) for point in points]

    def _pcl_cb(self, msg: PointCloud2):
        rospy.loginfo("Received point cloud")
        detections = self._yolo(
            YoloDetection3DRequest(msg, "yolov8x-seg.pt", 0.3, 0.3, False, "")
        )
        if self._classes is not None:
            detections.detected_objects = [
                detection
                for detection in detections.detected_objects
                if detection.name in self._classes
            ]

        for detection in detections.detected_objects:
            contours_3d = cv2_pcl.seg_to_3d_contours(
                msg, np.array(detection.xyseg), msg.height, msg.width
            )
            contours_3d = contours_3d[~np.isnan(contours_3d).any(axis=1)]
            contours_points = [
                PointStamped(header=msg.header, point=Point(*point))
                for point in contours_3d
            ]
            contours_points = self._tf_points(
                contours_points, msg.header.frame_id, "map"
            )
            self._cached_objects.append(
                {
                    "contours_points": contours_points,
                    "stamp": msg.header.stamp,
                }
            )

    def _timer_cb(self, _):
        if not self._processing:
            self._processing = True
            num_objects = len(self._cached_objects)
            self._cached_objects = [
                obj
                for obj in self._cached_objects
                if (rospy.Time.now() - obj["stamp"]).to_sec() < 30.0
            ]
            rospy.loginfo(
                f"Removed {num_objects - len(self._cached_objects)} objects from cache"
            )

        if rosparam.list_params("/mmap"):
            rosparam.delete_param("mmap")

        mmap_dict = {"vo": {"submap_0": dict()}, "numberOfSubMaps": 1}
        vo_vertex_count = 0
        object_count = 0

        for obj in self._cached_objects:
            vo = f"vo_{vo_vertex_count}"
            for point in obj["contours_points"]:
                mmap_dict["vo"]["submap_0"][vo] = [
                    "submap_0",
                    f"object_{object_count}",
                    point.point.x,
                    point.point.y,
                    point.point.z,
                ]
                vo_vertex_count += 1
            object_count += 1

        rosparam.upload_params("mmap", mmap_dict)
        self._processing = False

    def _stop(self, _: TriggerRequest):
        if self._pcl_sub is None:
            return TriggerResponse(success=False)
        self._pcl_sub.unregister()
        self._pcl_sub = None
        return TriggerResponse(success=True)


if __name__ == "__main__":
    rospy.init_node("yolo_to_vo")
    yolo_to_vo = YOLOtoVO(classes=["banana"])
    rospy.spin()
