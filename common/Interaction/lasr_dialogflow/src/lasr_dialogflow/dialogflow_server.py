#!/usr/bin/env python3
import rospy
from lasr_dialogflow.srv import DialogflowAudio, DialogflowAudioResponse, DialogflowText, DialogflowTextResponse
from lasr_dialogflow.actions import ReceptionistAction, GoToLocationAction
import json
import os
from std_msgs.msg import String

import pyaudio

class DialogflowServer():

    def __init__(self):
        
        self.config = {
            "robocup_receptionist" : {
                "project_id" : "robocup-receptionist-evnn",
                "cls" : ReceptionistAction
            },
            "go_to_location" : {
                "project_id" : "go-to-location-rmty",
                "cls" : GoToLocationAction
            }
        }

        pa = pyaudio.PyAudio()

        info = pa.get_host_api_info_by_index(0)
        for i in range(info['deviceCount']):
                if pa.get_device_info_by_host_api_device_index(0, i)['maxInputChannels'] > 0:
                    print("Input Device id ", i, " - ", pa.get_device_info_by_host_api_device_index(0, i)['name'])

        print("Please enter a device index to use: ", end='')
        self.device = int(input())

        print(f"Using device index {self.device}, {pa.get_device_info_by_host_api_device_index(0, self.device)['name']}")

        self.audio_srv = rospy.Service("/dialogflow_server/process_audio", DialogflowAudio, self.process_audio)
        self.text_srv = rospy.Service("/dialogflow_server/process_text", DialogflowText, self.process_text)

    def process_audio(self, req):

        response = DialogflowAudioResponse()

        project_id = self.config[req.task]["project_id"]
        task = self.config[req.task]["cls"](project_id, device=self.device)
        result = task.actions[req.action](use_mic=True)

        if result:
            response.result = result
            response.success = True

        return response

    def process_text(self, req):

        response = DialogflowTextResponse()

        project_id = self.config[req.task]["project_id"]
        task = self.config[req.task]["cls"](project_id)
        result = task.actions[req.action](use_mic=False, text=req.query_text)

        if result:
            response.result = result
            response.success = True

        return response

if __name__ == "__main__":
    rospy.init_node("dialogflow_server")
    dialogflow_server = DialogflowServer()
    rospy.spin()