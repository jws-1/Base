#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

class GuidePerson(smach.State):
    def __init__(self, context):
        smach.State.__init__(self, outcomes=['done'])
        self.context = context

    def perform_detection(self, pcl_msg, polygon, filter, model):
        cv_im = pcl_msg_to_cv2(pcl_msg)
        img_msg = self.context.bridge.cv2_to_imgmsg(cv_im)
        detections = self.context.yolo(img_msg, model, 0.5, 0.3)
        detections = [(det, self.estimate_pose(pcl_msg, det)) for det in detections.detected_objects if det.name in filter]
        rospy.loginfo(f"All: {[(det.name, pose) for det, pose in detections]}")
        rospy.loginfo(f"Boundary: {polygon}")
        satisfied_points = self.context.shapely.are_points_in_polygon_2d(polygon, [[pose[0], pose[1]] for (_, pose) in detections]).inside
        detections = [detections[i] for i in range(0, len(detections)) if satisfied_points[i]]
        rospy.loginfo(f"Filtered: {[(det.name, pose) for det, pose in detections]}")
        return detections

    def execute(self, userdata):
        empty_tables = [(label, rospy.get_param(f"/tables/{label}")) for label, table in self.context.tables.items() if table["status"] == "ready"]
        label, table = empty_tables[0]
        self.context.current_table = label
        position, orientation = table["location"]["position"], table["location"]["orientation"]
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))

        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)

        self.context.voice_controller.sync_tts("Please be seated, I will wait for you to sit down!")

        self.person_polygon = rospy.get_param(f"/tables/{self.context.current_table}/persons_cuboid")

        motions = ["look_left", "look_right"]
        customer_seated = False
        while not customer_seated:
            for motion in motions:
                pm_goal = PlayMotionGoal(motion_name=motion, skip_planning=True)
                self.context.play_motion_client.send_goal_and_wait(pm_goal)
                pcl_msg = rospy.wait_for_message("/xtion/depth_registered/points", PointCloud2)
                customer_seated = len(self.perform_detection(pcl_msg, self.person_polygon, ["person"], self.context.YOLO_person_model)) > 0
                if customer_seated:
                    break

        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)

        self.context.tables[label]["status"] = "needs serving"
        return 'done'
