#!/usr/bin/env python3
import smach
import numpy as np
from play_motion_msgs.msg import PlayMotionGoal
import json
import rospy

class LoadOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def execute(self, userdata):
        location = rospy.get_param("/counter/load_location")
        position = location["position"]
        orientation = location["orientation"]
        self.context.voice_controller.sync_tts("That's the right order, I'll position myself so that it can be loaded")
        self.context.base_controller.sync_to_pose(Pose(position=Point(**position), orientation=Quaternion(**orientation)))
        self.context.base_controller.rotate(np.pi)
        pm_goal = PlayMotionGoal(motion_name="load_unload", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.voice_controller.sync_tts("I'll give you some time to load the order...")
        rospy.sleep(rospy.Duration(10.0))
        pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.context.play_motion_client.send_goal_and_wait(pm_goal)
        self.context.start_head_manager("head_manager", '')
        return 'done'
