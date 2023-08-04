#!/usr/bin/env python3
import smach
import rospy

class CheckOrder(smach.State):
    def __init__(self, voice_controller):
        smach.State.__init__(self, outcomes=['correct', 'incorrect'])
        self.voice_controller = voice_controller

    def execute(self, userdata):
        order = rospy.get_param(f"/tables/{rospy.get_param('current_table')}/order")

        """
        Determine the items
        """
        given_order = []
        rospy.set_param(f"/tables/{rospy.get_param('current_table')}/given_order", given_order)
        return 'correct' if sorted(order) == sorted(given_order) else 'incorrect'