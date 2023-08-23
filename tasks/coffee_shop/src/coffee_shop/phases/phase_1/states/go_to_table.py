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
        unvisited_tables = self.context.unvisited()
        robot_x, robot_y = self.base_controller.get_pose()
        closest_table = min(unvisited_tables, key=lambda table: np.linalg.norm(table.location["position"]["x"] - robot_x, table.location["position"]["y"] - robot_y))
        self.voice_controller.sync_tts(f"I am going to {closest_table.idx}")
        position, orientation = closest_table.location["position"], closest_table.location["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        closest_table.visit(Table.Status.VISITING)
        self.context._current = closest_table
        return 'done'
