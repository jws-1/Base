#!/usr/bin/env python3
import smach
import rospy

class LoadOrder(smach.State):

    def __init__(self, base_controller, voice_controller, pm):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.base_controller = base_controller
        self.voice_controller = voice_controller
        self.pm = pm

    def execute(self, userdata):
        return 'done'