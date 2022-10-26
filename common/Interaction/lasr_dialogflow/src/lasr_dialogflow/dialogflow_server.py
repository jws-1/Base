#!/usr/bin/env python3
import rospy
from lasr_dialogflow.srv import DialogflowAudio, DialogflowAudioResponse, DialogflowText, DialogflowTextResponse
from lasr_dialogflow.actions import ReceptionistAction, GoToLocationAction
import json
import os
from std_msgs.msg import String

import pyaudio


CHUNK_SIZE = 4096

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

        self.audio_interface = pyaudio.PyAudio()

        info = self.audio_interface.get_host_api_info_by_index(0)
        for i in range(info['deviceCount']):
                if self.audio_interface.get_device_info_by_host_api_device_index(0, i)['maxInputChannels'] > 0:
                    print("Input Device id ", i, " - ", self.audio_interface.get_device_info_by_host_api_device_index(0, i)['name'])

        print("Please enter a device index to use: ", end='')
        self.device = int(input())
        self.sample_rate = int(self.audio_interface.get_device_info_by_index(self.device)["defaultSampleRate"])
        print(f"Using device index {self.device}, {self.audio_interface.get_device_info_by_host_api_device_index(0, self.device)['name']}, sample rate is {self.sample_rate}, frames per buffer is {CHUNK_SIZE}")

        self.microphone_stream = self.audio_interface.open(rate=self.sample_rate, channels=1, input_device_index=self.device, format=pyaudio.paInt16, input=True, frames_per_buffer=CHUNK_SIZE, start=False)
        self.audio_srv = rospy.Service("/dialogflow_server/process_audio", DialogflowAudio, self.process_audio)
        self.text_srv = rospy.Service("/dialogflow_server/process_text", DialogflowText, self.process_text)

    def process_audio(self, req):

        response = DialogflowAudioResponse()

        project_id = self.config[req.task]["project_id"]
        task = self.config[req.task]["cls"](project_id, self.microphone_stream, self.sample_rate, CHUNK_SIZE)
        result = task.actions[req.action](use_mic=True)

        if result:
            response.result = result
            response.success = True

        return response

    def process_text(self, req):

        response = DialogflowTextResponse()

        project_id = self.config[req.task]["project_id"]
        task = self.config[req.task]["cls"](project_id, self.microphone_stream, self.sample_rate, CHUNK_SIZE)
        result = task.actions[req.action](use_mic=False, text=req.query_text)

        if result:
            response.result = result
            response.success = True

        return response

if __name__ == "__main__":
    rospy.init_node("dialogflow_server")
    dialogflow_server = DialogflowServer()
    rospy.spin()