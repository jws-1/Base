#!/usr/bin/env python3
import rospy
import smach
from lasr_skills import SemanticScan, GoToSemanticLocation
from lasr_voice import Voice
from collections import defaultdict
rospy.init_node("find_object_in_room")


class StartIteration(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'not_finished'], input_keys=['locations'], output_keys=['location', 'motions'])
    
    def execute(self, userdata):
        if not userdata.locations:
            return 'finished'
        userdata.location = userdata.locations.pop(0)
        userdata.motions = rospy.get_param(f"/{semantic_location}/{userdata.location}/motions")
        return 'not finished'

class EndIteration(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['bulk_detections', 'detections_in_room', 'location'], output_keys=['detections_in_room'])

    def execute(self, userdata):
        userdata.detections_in_room[userdata.location] = userdata.bulk_detections

class Finish(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['detections_in_room'], output_keys=['detections_in_room'])
        self.voice = Voice()

    def execute(self, userdata):
        final_detections = defaultdict(list)
        for location, detections in userdata.detections_in_room.items():
            final_detections[location]
            for detection in detections:
                final_detections[location].append(detection.name)

        for location in final_detections.keys():
            self.voice.sync_tts(f"I found the following objects in {location}: {' '.join(list(set(final_detections[location])))}")

sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

"""
Find the cutlery in the development room
"""

import rosparam
semantic_location = "development_room"
rosparam.upload_params(semantic_location, {"table1" : {"location": {}, "motions": []}, "table2" : {"location" : {}, "motions": []}})

sm.userdata.locations = rospy.get_param(f"/{semantic_location}").keys()
sm.userdata.filter = ["fork", "spoon"]
sm.userdata.image_topic = "/xtion/rgb/image_raw"
sm.userdata.detections_in_room = {}

with sm:
    smach.StateMachine.add('START_ITERATION', StartIteration(), transitions={'finished' : 'FINISH' ,'not_finished' : 'GO_TO_SEMANTIC_LOCATION'})
    smach.StateMachine.add('GO_TO_SEMANTIC_LOCATION', GoToSemanticLocation(), transitions={'succeeded' : 'SEMANTIC_SCAN', 'failed' : 'failed'})
    smach.StateMachine.add('SEMANTIC_SCAN', SemanticScan(), transitions={'succeeded' : 'PROCESS', 'failed' : 'failed'})
    smach.StateMachine.add('END_ITERATION', EndIteration(), transitions={'succeeded' : 'START_ITERATION'})
    smach.StateMachine.add('FINISH', Finish(), transitions={'succeeded': 'succeeded'})
sm.execute()