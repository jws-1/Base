#!/usr/bin/env python3
import smach
import rospy
import numpy as np
import ros_numpy as rnp
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from common_math import pcl_msg_to_cv2, seg_to_centroid
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest
from cv_bridge3 import CvBridge, cv2
from pal_startup_msgs.srv import StartupStart, StartupStop
import rosservice
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from collections import Counter

from lasr_shapely import LasrShapely
shapely = LasrShapely()

OBJECTS = ["cup", "mug", "bowl"]

class CheckOrder(smach.State):
    def __init__(self, voice_controller, yolo, tf, pm, context):
        smach.State.__init__(self, outcomes=['correct', 'incorrect'])
        self.voice_controller = voice_controller
        self.detect = yolo
        self.tf = tf
        self.play_motion_client = pm
        self.context = context
        self.bridge = CvBridge()
        self.prev_order = None

        service_list = rosservice.get_service_list()
        # This should allow simulation runs as well, as i don't think the head manager is running in simulation
        if "/pal_startup_control/stop" in service_list:
            self.stop_head_manager = rospy.ServiceProxy("/pal_startup_control/stop", StartupStop)
            self.start_head_manager = rospy.ServiceProxy("/pal_startup_control/start", StartupStart)

    def estimate_pose(self, pcl_msg, detection):
        centroid_xyz = seg_to_centroid(pcl_msg, np.array(detection.xyseg))
        centroid = PointStamped()
        centroid.point = Point(*centroid_xyz)
        centroid.header = pcl_msg.header
        tf_req = TfTransformRequest()
        tf_req.target_frame = String("map")
        tf_req.point = centroid
        response = self.tf(tf_req)
        return np.array([response.target_point.point.x, response.target_point.point.y, response.target_point.point.z])

    def execute(self, userdata):
        result = self.stop_head_manager.call("head_manager")

        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(pm_goal)
        order = self.context.current().order

        counter_corners = rospy.get_param(f"/counter/cuboid")

        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.bridge.cv2_to_imgmsg(cv_im)
        detections = self.detect(img_msg, "yolov8n-seg.pt", 0.5, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, det)) for det in detections.detected_objects if det.name in OBJECTS]
        satisfied_points = shapely.are_points_in_polygon_2d(counter_corners, [[pose[0], pose[1]] for (_, pose) in detections]).inside
        given_order = [detections[i][0].name for i in range(0, len(detections)) if satisfied_points[i]]
        self.context.current().given_order = given_order

        res = self.start_head_manager.call("head_manager", '')

        if sorted(order) == sorted(given_order):
            return 'correct'
        else:
            if self.prev_order == given_order:
                rospy.sleep(rospy.Duration(5.0))
                return 'incorrect'

            missing_items = list((Counter(order) - Counter(given_order)).elements())
            missing_items_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in
                                              Counter(missing_items).items()]).replace(', ', ', and ',
                                                                                       len(missing_items) - 2)
            invalid_items = list((Counter(given_order) - Counter(order)).elements())
            invalid_items_string = ', '.join([f"{count} {item if count == 1 else item+'s'}" for item, count in
                                              Counter(invalid_items).items()]).replace(', ', ', and ',
                                                                                       len(invalid_items) - 2)
            if not len(invalid_items):
                self.voice_controller.sync_tts(
                    f"You didn't give me {missing_items_string} which I asked for. Please correct the order.")
            elif not len(missing_items):
                self.voice_controller.sync_tts(
                    f"You have given me {invalid_items_string} which I didn't ask for. Please correct the order.")
            else:
                self.voice_controller.sync_tts(
                    f"You have given me {invalid_items_string} which I didn't ask for, and didn't give me {missing_items_string} which I asked for. Please correct the order.")
            self.prev_order = given_order
            rospy.sleep(rospy.Duration(5.0))
            return 'incorrect'
