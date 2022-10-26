#!/usr/bin/env python3
import rospy

rospy.init_node("test")

from lasr_interaction_server.srv import SpeechInteraction, TextInteraction

speech = rospy.ServiceProxy("/lasr_interaction_server/speech_interaction", SpeechInteraction)
print(speech("go_to_location", "ask_location"))

# text = rospy.ServiceProxy("/lasr_interaction_server/text_interaction", TextInteraction)
# print(text("go_to_location", "ask_location", "Go to 6.02."))

