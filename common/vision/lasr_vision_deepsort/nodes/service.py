#!/usr/bin/env python3
import rospy

from lasr_vision_msgs.srv import (
    StartTracker,
    StartTrackerRequest,
    StartTrackerResponse,
    UpdateTracker,
    UpdateTrackerRequest,
    UpdateTrackerResponse,
)

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from deep_sort_realtime.deepsort_tracker import DeepSort

from typing import Union

import lasr_vision_deepsort as deepsort


class DeepSORT:

    _tracker: Union[DeepSort, None] = None

    def __init__(self):
        self._tracker = None

    def start_tracker(self, req: StartTrackerRequest) -> StartTrackerResponse:
        """
        Sets up the tracker with the given parameters
        """
        self._tracker = deepsort.create_tracker(req)

        return StartTrackerResponse()

    def update_tracker(self, req: UpdateTrackerRequest) -> UpdateTrackerResponse:
        if self._tracker is None:
            raise rospy.ServiceException(
                "Tracker has not been started. Call start_tracker first"
            )

        return deepsort.update_tracker(req, self._tracker)

    def stop_tracker(self, req: EmptyRequest) -> EmptyResponse:
        self._tracker = None
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("lasr_vision_deepsort")

    deepsort = DeepSORT()
    rospy.loginfo("DeepSORT service started")

    rospy.Service("start_tracker", StartTracker, deepsort.start_tracker)
    rospy.Service("update_tracker", UpdateTracker, deepsort.update_tracker)
    rospy.Service("stop_tracker", Empty, deepsort.stop_tracker)

    rospy.spin()
