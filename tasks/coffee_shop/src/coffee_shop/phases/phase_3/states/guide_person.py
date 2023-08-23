#!/usr/bin/env python3
import smach
import rospy
from lasr_voice.voice import Voice
from coffee_shop.core import Table
from geometry_msgs.msg import Pose, Point, Quaternion

class GuidePerson(smach.State):
    def __init__(self, base_controller, voice_controller, context):
        smach.State.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller
        self.context = context

    def execute(self, userdata):
        ready = self.context.ready()
        table = ready[0]
        position, orientation = table.location["position"], table.location["orientation"]
        self.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.voice_controller.sync_tts("Please be seated!")
        table.status = Table.Status.NEEDS_SERVING
        return 'done'