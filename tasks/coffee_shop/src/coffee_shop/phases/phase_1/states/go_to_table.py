#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from coffee_shop.core import Table

class GoToTable(smach.State):
    def __init__(self, base_controller, voice_controller, context):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller
        self.context = context

    def execute(self, userdata):
        robot_x, robot_y = self.base_controller.get_pose()
        closest_table = self.context.closest(robot_x, robot_y, Table.Status.UNVISITED)
        self.voice_controller.sync_tts(f"I am going to {closest_table.idx}")
        position, orientation = closest_table.position, closest_table.orientation
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.context.visit(closest_table, Table.Status.VISITING)
        return 'done'
