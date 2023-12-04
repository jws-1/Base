#!usr/bin/env python3

import rospy
import smach
from smach_ros import SimpleActionState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


from lasr_skills import DetectObjects

class SemanticScan(smach.StateMachine):


    class ProcessDetections(smach.State):

        def __init__(self):

            smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['bulk_detections', 'detections'], output_keys=['bulk_detections'])

        def execute(self, userdata):
            userdata.bulk_detections.extend(userdata.detections)
            return 'succeeded'


    def __init__(self):

        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['image_topic', 'semantic_location'], output_keys=['bulk_detections'])

        with self:

            motions = rospy.get_param(f"/{self.userdata.semantic_location}/motions")

            def motion_goal_cb():
                if motions:
                    motion_name = motions.pop(0)
                    play_motion_goal = PlayMotionGoal(motion_name=motion_name, skip_planning=True)
                    return play_motion_goal
                return None

            smach.StateMachine.add('PLAY_MOTION', SimpleActionState('play_motion', PlayMotionAction, goal_cb=motion_goal_cb), transitions={'succeeded' : 'DETECT_OBJECTS', 'aborted' : 'failed'})
            smach.StateMachine.add('DETECT_OBJECTS', DetectObjects(), transitions={'succeeded' : 'PROCESS_DETECTIONS', 'failed' : 'failed'})
            smach.StateMachine.add('PROCESS_DETECTIONS', self.ProcessDetections(), transitions={'succeeded' : 'PLAY_MOTION', 'failed' : 'failed'})

if __name__ == "__main__":
    rospy.init_node("test_semantic_scan")
    sm = SemanticScan()
    sm.userdata.filter = ["fork"]
    sm.userdata.image_topic = "/xtion/rgb/image_raw"
    outcome = sm.execute()