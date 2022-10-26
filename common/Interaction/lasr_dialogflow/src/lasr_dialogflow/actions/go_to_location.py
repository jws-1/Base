#!/usr/bin/env python3

import rospy
from lasr_dialogflow.actions import BaseAction
from std_msgs.msg import String
from google.api_core.exceptions import DeadlineExceeded


class GoToLocationAction(BaseAction):
    
    def __init__(self, project_id, microphone_stream, rate, chunk_sz, df_lang_id="en"):
        super(GoToLocationAction, self).__init__(project_id, microphone_stream, rate, chunk_sz, df_lang_id=df_lang_id)

        self.actions = {
            "ask_location" : self.ask_location,
            "confirm_location": self.confirm_location
        }

    def ask_location(self, use_mic=False, text=None):
        if not use_mic and text:
            return self.handle_response(self.text_in_context(text, context="GetLocation"), "GetLocation")
                
        elif use_mic:
            return self.handle_response(self.listen_in_context("GetLocation"), "GetLocation")

    def confirm_location(self, use_mic=False, text=None):
        print(use_mic, text)
        if not use_mic and text:
            return self.handle_response(self.text_in_context(text, context="ConfirmLocation"), "ConfirmLocation")
        elif use_mic:
            return self.handle_response(self.listen_in_context("ConfirmLocation"), "ConfirmLocation")

    def handle_response(self, response, context):
        if not response:
            print(response)
            return None
        try:
            if context == "GetLocation":
                return response.query_result.parameters["location"]
            elif context == "ConfirmLocation":
                return response.query_result.parameters["yesno"]
        except ValueError:
            return None