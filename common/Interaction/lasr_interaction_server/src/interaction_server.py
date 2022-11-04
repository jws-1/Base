#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
from lasr_interaction_server.srv import SpeechInteraction, SpeechInteractionResponse, TextInteraction, TextInteractionResponse, Speech, SpeechResponse
from lasr_dialogflow.srv import DialogflowAudio, DialogflowText
from pal_interaction_msgs.msg import TtsGoal, TtsAction, TtsActionGoal
import actionlib


class InteractionServer():

    def __init__(self):
        
        self.dialogflow_process_audio = rospy.ServiceProxy("/dialogflow_server/process_audio", DialogflowAudio)
        self.dialogflow_process_text = rospy.ServiceProxy("/dialogflow_server/process_text", DialogflowText)

        self.speech_interaction_srv = rospy.Service("/lasr_interaction_server/speech_interaction", SpeechInteraction, self.speech_interaction)
        self.text_interaction_srv = rospy.Service("/lasr_interaction_server/text_interaction", TextInteraction, self.text_interaction)
    

        self.speech_client = actionlib.SimpleActionClient("/tts", TtsAction)

        self.speak_srv = rospy.Service("/lasr_interaction_server/speak", Speech, self.speak)

    def speech_interaction(self, req):
        
        response = SpeechInteractionResponse()

        dialogflow_response = self.dialogflow_process_audio(req.task, req.action)
    
        response.result, response.success = dialogflow_response.result, dialogflow_response.success

        return response

    def text_interaction(self, req):

        response = TextInteractionResponse()

        dialogflow_response = self.dialogflow_process_text(req.task, req.action, req.query_text)
    
        response.result, response.success = dialogflow_response.result, dialogflow_response.success

        return response

    def speak(self, req):
        self.speech_client.wait_for_server()
        goal = TtsGoal()
        goal.rawtext.text = req.text
        goal.rawtext.lang_id = 'en_GB'
        self.speech_client.send_goal(goal)
        if req.wait:
            self.speech_client.wait_for_result()
        return SpeechResponse()

if __name__ == "__main__":
    rospy.init_node("lasr_interaction_server")
    interaction_server = InteractionServer()
    rospy.spin()