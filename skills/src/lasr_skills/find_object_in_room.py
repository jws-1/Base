#!/usr/bin/env python3
import rospy
import smach
from lasr_skills import SemanticScan, GoToSemanticLocation
from lasr_voice import Voice
from collections import defaultdict
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal

rospy.init_node("find_object_in_room")

semantic_location = "development_room"

class StartIteration(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'not_finished'], input_keys=['locations', 'room'], output_keys=['location', 'motions'])
    
    def execute(self, userdata):
        print(userdata.locations)
        if not userdata.locations:
            return 'finished'
        location = userdata.locations.pop(0)
        userdata.location = f"/{userdata.room}/{location}"
        userdata.motions = rospy.get_param(f"/{userdata.room}/{location}/motions")
        return 'not_finished'

class EndIteration(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['bulk_detections', 'detections_in_room', 'location'], output_keys=['detections_in_room', 'bulk_detections'])
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.play_motion_client.wait_for_server()

    def execute(self, userdata):
        userdata.detections_in_room[userdata.location] = userdata.bulk_detections
        userdata.bulk_detections = []
        play_motion_goal = PlayMotionGoal(motion_name="back_to_default", skip_planning=True)
        self.play_motion_client.send_goal_and_wait(play_motion_goal)
        return 'succeeded'

class Finish(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['detections_in_room'], output_keys=['detections_in_room'])
        self.voice = Voice()

    def execute(self, userdata):
        final_detections = defaultdict(list)
        for location, detections in userdata.detections_in_room.items():
            for detection in detections:
                final_detections[location].append(detection.name)

        rospy.loginfo(final_detections)
        self.voice.sync_tts("I'm done searching the room.")
        for location in final_detections.keys():
            objects = list(set(final_detections[location]))
            if objects:
                self.voice.sync_tts(f"I found the following objects in {location}: {' '.join(objects)}")
        return 'succeeded'

sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

"""
Find the cutlery in the development room
"""

import rosparam

rosparam.upload_params(
    semantic_location, 
    {
        "table1" : {
            "location" : {
                "position" : {"x" : 3.60559579428,  "y": 0.0779423496259, "z": 0.0},
                "orientation" : {"x": 0.0, "y": 0.0, "z": 0.0363086961051, "w": 0.999340621904}
            },
            "motions": ["check_table"]
        },
        "table2" : {
            "location" : {
                "position" : {"x" : 2.54642828685,  "y": -3.66341021552, "z": 0.0},
                "orientation" : {"x": 0.0, "y": 0.0, "z": -0.672521603535, "w": 0.740077491064}
            },
            "motions": ["check_table"]
        },
        "table3" : {
            "location" : {
                "position" : {"x" : 0.963071182341,  "y": 1.60594601259, "z": 0.0},
                "orientation" : {"x": 0.0, "y": 0.0, "z": 0.76758370603, "w": 0.640948714201}
            },
            "motions": ["check_table"]
        }
    }
)


sm.userdata.room = semantic_location
sm.userdata.locations = list(rospy.get_param(f"/{semantic_location}").keys())
print(sm.userdata.locations)
sm.userdata.filter = ["fork", "spoon", "cup"]  
sm.userdata.image_topic = "/xtion/rgb/image_raw"
sm.userdata.detections_in_room = {}

with sm:
    smach.StateMachine.add('START_ITERATION', StartIteration(), transitions={'finished' : 'FINISH' ,'not_finished' : 'GO_TO_SEMANTIC_LOCATION'})
    smach.StateMachine.add('GO_TO_SEMANTIC_LOCATION', GoToSemanticLocation(), transitions={'succeeded' : 'SEMANTIC_SCAN', 'failed' : 'failed'})
    smach.StateMachine.add('SEMANTIC_SCAN', SemanticScan(), transitions={'succeeded' : 'END_ITERATION', 'failed' : 'failed'})
    smach.StateMachine.add('END_ITERATION', EndIteration(), transitions={'succeeded' : 'START_ITERATION'})
    smach.StateMachine.add('FINISH', Finish(), transitions={'succeeded': 'succeeded'})
sm.execute()