#!/usr/bin/env python3

import rospy
from input_string_to_location.srv import StringToLocation, StringToLocationRequest
from std_msgs.msg import String
from lasr_interaction_server.srv import SpeechInteraction, Speech

def client(location: String):
    rospy.wait_for_service('/string_to_location')
    try:
        req = StringToLocationRequest()
        req.location = location
        str_to_pos = rospy.ServiceProxy('string_to_location', StringToLocation)
        resp = str_to_pos(req)
        return resp.is_reached
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
if __name__ == '__main__':
    rospy.init_node("string_to_location_client", anonymous=True)
    speech_interaction = rospy.ServiceProxy("/lasr_interaction_server/speech_interaction", SpeechInteraction)
    speak = rospy.ServiceProxy("/lasr_interaction_server/text_interaction", Speech)
    try:
        while not rospy.is_shutdown():
            location = speech_interaction("go_to_location", "ask_location").result
            print(location)
            if not location:
                speak("I didn't get that. Please can you repeat?", True)
            else:
                speak(f"Do you want me to go to {location}?", True)
                correct = speech_interaction("go_to_location", "confirm_location").result == "yes"
                if correct:
                    speak(f"Going to {location}")
                    client(location)
    except rospy.ROSInterruptException:
        pass