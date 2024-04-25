#!/usr/bin/env python3

import rospy
import smach
from smach import UserData
from typing import List, Any, Dict, Union
from lasr_vision_msgs.srv import TorchFaceFeatureDetectionDescription
from sensor_msgs.msg import Image
from lasr_skills import DescribePeople
import json


class GetGuestAttributes(smach.StateMachine):

    class HandleGuestAttributes(smach.State):
        def __init__(self, guest_id: str):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["people", "guest_data"],
                output_keys=["guest_data"],
            )

            self._guest_id: str = guest_id

        def execute(self, userdata: UserData) -> str:
            if len(userdata.people) == 0:
                return "failed"
            userdata.guest_data[self._guest_id]["attributes"] = json.loads(
                userdata.people[0]["features"]
            )["attributes"]
            return "succeeded"

    def __init__(
        self,
        guest_id: str,
    ):

        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["guest_id", "guest_data"],
            output_keys=["guest_data"],
        )
        self._guest_id: str = guest_id
        self._attribute_service: rospy.ServiceProxy = rospy.ServiceProxy(
            "/torch/detect/face_features", TorchFaceFeatureDetectionDescription
        )

        with self:

            smach.StateMachine.add(
                "GET_GUEST_ATTRIBUTES",
                DescribePeople(),
                transitions={
                    "succeeded": "HANDLE_GUEST_ATTRIBUTES",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "HANDLE_GUEST_ATTRIBUTES",
                self.HandleGuestAttributes(self._guest_id),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )


if __name__ == "__main__":
    rospy.init_node("test_get_guest_attributes")
    sm = GetGuestAttributes("guest1")
    sm.userdata.guest_data = {"guest1": {}}
    sm.execute()
    print(sm.userdata.guest_data)
