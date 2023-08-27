#!/usr/bin/env python3
import smach
from coffee_shop.phases.phase_2.states import MakeOrder, CheckOrder, InvalidateOrder, LoadOrder
import rospy
from lasr_voice import Voice
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform, TfTransformRequest
from lasr_speech.srv import Speech
from tiago_controllers import BaseController
import rosservice
from lasr_shapely import LasrShapely

HAS_HEAD_MANAGER = "pal_startup_control/stop" in rosservice.get_service_list()

try:
    from pal_startup_msgs.srv import StartupStart, StartupStop
    HAS_HEAD_MANAGER &= True
except ImportError:
    HAS_HEAD_MANAGER = False
    pass

if __name__ == "__main__":
    rospy.init_node("test_check_table")
    sm = smach.StateMachine(outcomes=['end'])
    base_controller = BaseController()
    voice_controller = Voice()
    play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    play_motion_client.wait_for_server(rospy.Duration(15.0))
    rospy.wait_for_service("/yolov8/detect", rospy.Duration(15.0))
    yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
    rospy.wait_for_service("/tf_transform", rospy.Duration(15.0))
    tf = rospy.ServiceProxy("/tf_transform", TfTransform)
    shapely = LasrShapely()
    rospy.wait_for_service("/lasr_speech/transcribe_and_parse")
    speech = rospy.ServiceProxy("/lasr_speech/transcribe_and_parse", Speech)

    if HAS_HEAD_MANAGER:
        rospy.wait_for_service("/pal_startup_control/start")
        start_head_manager = rospy.ServiceProxy("/pal_startup_control/start", StartupStart)
        rospy.wait_for_service("/pal_startup_control/stop")
        stop_head_manager = rospy.ServiceProxy("/pal_startup_control/stop", StartupStop)
    else:
        start_head_manager = lambda *args: None
        stop_head_manager = lambda *args: None

    with sm:
        sm.add('MAKE_ORDER', MakeOrder(voice_controller), transitions={'done' : 'CHECK_ORDER'})
        sm.add('CHECK_ORDER', CheckOrder(yolo, tf, play_motion_client, shapely, start_head_manager, stop_head_manager), transitions={'correct': 'LOAD_ORDER', 'incorrect': 'INVALIDATE_ORDER'})
        sm.add('INVALIDATE_ORDER', InvalidateOrder(voice_controller), transitions={'done': 'CHECK_ORDER'})
        sm.add('LOAD_ORDER', LoadOrder(base_controller, voice_controller, play_motion_client, speech), transitions={'done':'end'})

    sm.execute()
    rospy.signal_shutdown("down")
