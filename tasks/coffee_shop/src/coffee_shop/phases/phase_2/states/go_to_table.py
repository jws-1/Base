#!/usr/bin/env python3
import smach
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from coffee_shop.core import Table

class GoToTable(smach.State):
    
    def __init__(self, base_controller, voice_controller, context):
        smach.State.__init__(self, outcomes=['done', 'skip'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller
        self.context = context

    def execute(self, userdata):
        tables_need_serving = self.context.needs_serving()
        if not tables_need_serving:
            return 'skip'
        table = tables_need_serving[0]
        self.voice_controller.sync_tts(f"I am going to {table.idx}, which needs serving")
        position = table.position
        orientation = table.orientation
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.context.visit(table, Table.Status.SERVING)
        return 'done'
1