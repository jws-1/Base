#!/usr/bin/env python3

import os
import pyaudio
from google.cloud import dialogflow_v2 as dialogflow
import rospkg


class DialogflowClientStream():

    def __init__(self, project_id, session_id, microphone_stream, rate, chunk_sz, language_code="en-GB", audio_encoding=dialogflow.types.AudioEncoding.AUDIO_ENCODING_LINEAR_16):

        self.microphone_stream = microphone_stream
        self.chunk_sz = chunk_sz
        credentials_path = os.path.join(rospkg.RosPack().get_path('lasr_dialogflow'), 'config', '{}.json'.format(project_id))

        self.session_client = dialogflow.SessionsClient.from_service_account_file(credentials_path)
        self.session_path = self.session_client.session_path(project_id, session_id)
        self.audio_config = dialogflow.InputAudioConfig(
            audio_encoding=audio_encoding,
            language_code=language_code,
            sample_rate_hertz=rate,
        )
        
        self.stop_requested = False

    def detect_intent(self, query_params):

        self.stop_requested = False

        print("listening")
        self.microphone_stream.start_stream()

        def request_generator(self):
            query_input = dialogflow.QueryInput(audio_config=self.audio_config)
            print(query_params)

            # The first request contains the configuration.
            yield dialogflow.StreamingDetectIntentRequest(
                session=self.session_path, query_input=query_input, single_utterance=True, query_params=query_params
            )

            for chunk in self.audio_generator():
                yield dialogflow.StreamingDetectIntentRequest(input_audio=chunk)

        return self.session_client.streaming_detect_intent(requests=request_generator(self))


    def text_request(self, text, query_params=None):
        text_input = dialogflow.types.TextInput(text=text, language_code="en-GB")
        query_input = dialogflow.types.QueryInput(text=text_input)
        # help(self.session_client.detect_intent)
        request = dialogflow.DetectIntentRequest(
            session=self.session_path,
            query_input=query_input,
            query_params=query_params
        )
        response = self.session_client.detect_intent(request=request)
        print(response)
        return response

    def audio_generator(self):
        while not self.stop_requested:
            chunk = self.microphone_stream.read(self.chunk_sz)
            if chunk is not None:
                yield chunk

    # def __del__(self):
    #     self.stop()
    #     self.audio_interface.terminate()


    def stop(self):
        self.stop_requested = True
        self.microphone_stream.stop_stream()
            # self.microphone_stream.close()
            # self.audio_interface.terminate()
        print("Stopped listening")
    
    # def __del__(self):
    #     self.stop()
    #     self.audio_interface.terminate()