#!/usr/bin/env python3

from lasr_dialogflow.dialogflow_client_stream import DialogflowClientStream
import uuid
from google.cloud import dialogflow_v2 as dialogflow
import rospy

class BaseAction():

    def __init__(self, project_id, df_lang_id="en", device=None):

        self.project_id = project_id
        self.df_lang_id = df_lang_id
        self.attempts = 0
        self.max_attempts = 3
        self.streaming_client = None

        self.session_id = uuid.uuid4()

        self.streaming_client = DialogflowClientStream(
            self.project_id,
            self.session_id,
            language_code=self.df_lang_id,
            input_device=device
        )
    
    def stop(self):
        self.streaming_client.stop()
    
    def listen_in_context(self, context=None):
        query_params = None
        response = None
        if context:
            query_params = dialogflow.types.QueryParameters(contexts=[self.get_context(context)])
            print(query_params)
        for response in self.streaming_client.detect_intent(query_params):
            print(response)
            if response.recognition_result.message_type == dialogflow.types.StreamingRecognitionResult.MessageType.END_OF_SINGLE_UTTERANCE:
                print("end of utterance")
                # self.stop()
        return response

    def text_in_context(self, text, context=None):
        query_params = None
        if context:
            query_params = dialogflow.types.QueryParameters(contexts=[self.get_context(context)])
        
        return self.streaming_client.text_request(text, query_params=query_params)

    def get_context(self, context, lifespan=1):
        return dialogflow.types.Context(name=dialogflow.ContextsClient.context_path(self.project_id, self.session_id, context),
                              lifespan_count=lifespan)