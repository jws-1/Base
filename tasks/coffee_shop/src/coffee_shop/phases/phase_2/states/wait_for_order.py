#!/usr/bin/env python3
import smach
import json
import rospy
from play_motion_msgs.msg import PlayMotionGoal
import difflib
from std_msgs.msg import Empty

class WaitForOrder(smach.State):

    def __init__(self, context):
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.context = context

    def listen(self):
        resp = self.context.speech(True)
        if not resp.success:
            self.context.voice_controller.sync_tts(self.context.get_random_retry_utterance())
            return self.listen()
        resp = json.loads(resp.json_response)
        rospy.loginfo(resp)
        return resp

    def affirm(self):
        resp = self.listen()
        if resp["intent"]["name"] not in ["affirm", "deny"]:
            self.context.voice_controller.sync_tts(self.context.get_random_retry_utterance())
            return self.affirm()
        return resp["intent"]["name"] == "affirm"

    def execute(self, userdata):
        if self.context.tablet:
            self.context.voice_controller.sync_tts("Please press 'ready' when you are ready for me to check the order.")
            rospy.wait_for_message("/tablet/ready", Empty)
            return 'done'
        else:
            pm_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
            self.context.play_motion_client.send_goal_and_wait(pm_goal)
            while True:
                rospy.sleep(rospy.Duration(5.0))
                self.context.voice_controller.sync_tts("Is the order ready to be checked? Please answer with yes or no.")
                if self.affirm():
                    return 'done'
