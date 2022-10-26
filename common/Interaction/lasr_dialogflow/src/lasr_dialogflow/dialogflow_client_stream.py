#!/usr/bin/env python3

import os
import pyaudio
from google.cloud import dialogflow_v2 as dialogflow
import rospkg

CHUNK_SIZE = 4096

class DialogflowClientStream():

    def __init__(self, project_id, session_id, language_code="en-GB", audio_encoding=dialogflow.types.AudioEncoding.AUDIO_ENCODING_LINEAR_16, sample_rate=None, input_device=None):

        self.audio_interface = pyaudio.PyAudio()
        self.input_device = input_device

        if self.input_device is None:
            self.sample_rate = int(self.audio_interface.get_default_input_device_info()["defaultSampleRate"])
        else:
            self.sample_rate = int(self.audio_interface.get_device_info_by_index(self.input_device)["defaultSampleRate"])

        credentials_path = os.path.join(rospkg.RosPack().get_path('lasr_dialogflow'), 'config', '{}.json'.format(project_id))


        self.session_client = dialogflow.SessionsClient.from_service_account_file(credentials_path)
        self.session_path = self.session_client.session_path(project_id, session_id)
        self.audio_config = dialogflow.InputAudioConfig(
            audio_encoding=audio_encoding,
            language_code=language_code,
            sample_rate_hertz=self.sample_rate,
        )
        
        self.stop_requested = False
        # self.microphone_stream = None

    def detect_intent(self, query_params):

        self.stop_requested = False

        self.microphone_stream = self.audio_interface.open(rate=self.sample_rate, channels=1, input_device_index=self.input_device, format=pyaudio.paInt16, input=True, frames_per_buffer=CHUNK_SIZE)
        print("listening")
        def request_generator(self):
            query_input = dialogflow.QueryInput(audio_config=self.audio_config)

            # The first request contains the configuration.
            yield dialogflow.StreamingDetectIntentRequest(
                session=self.session_path, query_input=query_input, single_utterance=True, query_params=query_params
            )

            for chunk in self.audio_generator():
                yield dialogflow.StreamingDetectIntentRequest(input_audio=chunk)


        return self.session_client.streaming_detect_intent(requests=request_generator(self))



    #     self.session_client = dialogflow.SessionsClient.from_service_account_file(credentials_path)
    #     self.session_path = self.session_client.session_path(project_id, session_id)
    #     self.audio_config = dialogflow.types.session_pb2.InputAudioConfig(
    #         language_code=language_code,
    #         audio_encoding=audio_encoding,
    #         sample_rate_hertz=self.sample_rate,
    #     )

    # def response_generator(self, query_params=None):
    #     s = self.session_client.streaming_detect_intent(requests=self.requests_generator(query_params))
    #     return s
    
    # def requests_generator(self, query_params=None):
    #     print(query_params)
    #     self.stop_requested = False
    #     self.microphone_stream = self.audio_interface.open(rate=self.sample_rate, channels=1, input_device_index=self.input_device, format=pyaudio.paInt16, input=True, frames_per_buffer=CHUNK_SIZE)


    #     query_input = dialogflow.types.session_pb2.QueryInput(audio_config=self.audio_config)

    #     # First request is for config
    #     yield dialogflow.types.session_pb2.StreamingDetectIntentRequest(
    #         session=self.session_path,
    #         query_input=query_input,
    #         query_params=query_params,
    #         single_utterance=True
    #     )

    #     print("Listening")
    #     for chunk in self.audio_generator():
    #         if self.stop_requested:
    #             break
    #         yield dialogflow.types.session_pb2.StreamingDetectIntentRequest(input_audio=chunk)
        

    # def text_request(self, text, query_params=None):
    #     text_input = dialogflow.types.session_pb2.TextInput(text=text, language_code="en-GB")
    #     query_input = dialogflow.types.session_pb2.QueryInput(text=text_input)
    #     response = self.session_client.detect_intent(session=self.session_path, query_input=query_input, query_params=query_params)

    #     return response

    def audio_generator(self):
        while not self.stop_requested:
            chunk = self.microphone_stream.read(CHUNK_SIZE)
            if chunk:
                yield chunk

    def stop(self):
        self.stop_requested = True
        if self.microphone_stream:
            self.microphone_stream.stop_stream()
            self.microphone_stream.close()
        print("Stopped listening")
    
    # def __del__(self):
    #     self.stop()
    #     self.audio_interface.terminate()