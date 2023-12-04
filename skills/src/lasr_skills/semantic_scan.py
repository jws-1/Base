#!/usr/bin/env python3

import rospy
import smach
#from smach_ros import SimpleActionState
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


from lasr_skills import DetectObjects

class SemanticScan(smach.StateMachine):


    class ProcessDetections(smach.State):

        def __init__(self):

            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['bulk_detections', 'detections'], output_keys=['bulk_detections'])

        def execute(self, userdata):
            userdata.bulk_detections.extend(userdata.detections.detected_objects)
            return 'succeeded'

    class PlayMotion(smach.State):

        def __init__(self):

            smach.State.__init__(self, outcomes=['succeeded', 'finished'], input_keys=['motions'], output_keys=['motions'])
            self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
            self.play_motion_client.wait_for_server()

        def execute(self, userdata):
            if not userdata.motions:
                return 'finished'
            motion = userdata.motions.pop(0)
            play_motion_goal = PlayMotionGoal(motion_name=motion, skip_planning=True)
            self.play_motion_client.send_goal_and_wait(play_motion_goal)
            return 'succeeded'


    def __init__(self):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['image_topic', 'semantic_location', 'filter'], output_keys=['bulk_detections'])

        with self:
            
            self.userdata.bulk_detections = []
            self.userdata.motions = rospy.get_param(f"/{self.userdata.semantic_location}/motions")
            # motions = rospy.get_param(f"/{self.userdata.semantic_location}/motions")

            # def motion_goal_cb():
            #     if motions:
            #         motion_name = motions.pop(0)
            #         play_motion_goal = PlayMotionGoal(motion_name=motion_name, skip_planning=True)
            #         return play_motion_goal
            #     return None

            smach.StateMachine.add('PLAY_MOTION', self.PlayMotion(), transitions={'succeeded' : 'DETECT_OBJECTS', 'finished' : 'succeeded'})
            #smach.StateMachine.add('PLAY_MOTION', SimpleActionState('play_motion', PlayMotionAction, goal_cb=motion_goal_cb), transitions={'succeeded' : 'DETECT_OBJECTS', 'aborted' : 'failed'})
            smach.StateMachine.add('DETECT_OBJECTS', DetectObjects(), transitions={'succeeded' : 'PROCESS_DETECTIONS', 'failed' : 'failed'})
            smach.StateMachine.add('PROCESS_DETECTIONS', self.ProcessDetections(), transitions={'succeeded' : 'PLAY_MOTION', 'failed' : 'failed'})

if __name__ == "__main__":
    rospy.init_node("test_semantic_scan")
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    with sm:

        sm.userdata.semantic_location = "bedroom"
        sm.userdata.motions = ["look_down", "look_left", "look_right"]
        sm.userdata.filter = ["fork"]
        sm.userdata.image_topic = "/xtion/rgb/image_raw"

        smach.StateMachine.add('SEMANTIC_SCAN', SemanticScan(), transitions={'succeeded':'succeeded','failed':'failed'})


        outcome = sm.execute()