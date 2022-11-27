#!/usr/bin/env python3

import actionlib
import rospy
from tiago_controllers.helpers import is_running, is_terminated
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped


class BaseController:
    def __init__(self):
        self._goal_sent = False
        self._client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._client.wait_for_server()

    def cancel_goal(self):
        if self._goal_sent is True:
            state = self._client.get_state()
            if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
                self._client.cancel_goal()
            while True:
                if (self._client.get_state() in [GoalStatus.PREEMPTED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                                                 GoalStatus.RECALLED, GoalStatus.SUCCEEDED]):
                    break
                rospy.sleep(0.5)
                self._goal_sent = False

    def get_current_pose(self):
        msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        x = round(msg.pose.pose.position.x, 2)
        y = round(msg.pose.pose.position.y, 2)
        quat = msg.pose.pose.orientation
        return x, y, quat

    def is_running(self):
        return is_running(self._client)

    def get_status(self):
        return self._client.get_state()

    def is_active(self):
        return self._client.get_state() == GoalStatus.PENDING or self._client.get_state() == GoalStatus.ACTIVE

    def is_terminated(self):
        return is_terminated(self._client)

    def __to_pose(self, pose, done_cb=None):
        if pose:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = pose

            rospy.loginfo("base is going to (%.2f, %.2f, %.2f) pose", pose.position.x, pose.position.y, pose.position.z)

            self._goal_sent = True
            self._client.send_goal(goal, done_cb=done_cb)

    def async_to_pose(self, pose):
        self.__to_pose(pose)

    def get_client(self):
        return self._client

    def sync_to_pose(self, pose, wait=60):
        self.__to_pose(pose)
        done = self._client.wait_for_result(rospy.Duration(wait))
        self._goal_sent = False

        state = self._client.get_state()
        if done and state == GoalStatus.SUCCEEDED:
            return True
        return state


from geometry_msgs.msg import Twist
from math import radians


class CmdVelController:
    def __init__(self):
        self._vel_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.1)  # wait for publisher to activate

    def rotate(self, angular_velocity: int, angle: int, clockwise: bool):
        vel_msg = Twist()
        angular_velocity = radians(angular_velocity)
        angle = radians(angle)
        if clockwise:
            vel_msg.angular.z = -abs(angular_velocity)
        else:
            vel_msg.angular.z = abs(angular_velocity)

        curr_ang = 0.0
        t0 = rospy.Time.now().to_sec()

        while curr_ang < angle:
            self._vel_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            curr_ang = angular_velocity * (t1 - t0)

    def linear_movement(self, speed: float, distance: float, is_forward: bool):
        vel_msg = Twist()
        if is_forward:
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        curr_dist = 0
        t0 = rospy.Time.now().to_sec()
        while curr_dist < distance:
            t1 = rospy.Time.now().to_sec()
            curr_dist = curr_dist + speed * (t1 - t0)
            self._vel_pub.publish(vel_msg)