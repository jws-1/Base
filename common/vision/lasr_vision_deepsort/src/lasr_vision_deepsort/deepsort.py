from deep_sort_realtime.deepsort_tracker import DeepSort

from lasr_vision_msgs.srv import (
    StartTrackerRequest,
    UpdateTrackerRequest,
    UpdateTrackerResponse,
)

import cv2_img
import rospy
from lasr_vision_msgs.msg import Detection

from typing import Union

import cv2


def create_tracker(req: StartTrackerRequest) -> DeepSort:
    """
    Sets up the tracker with the given parameters
    """
    return DeepSort(
        max_age=req.max_age,
        embedder=req.embedder,
        embedder_model_name=req.torchreid_embedder or None,
        polygon=req.polygon,
    )


def update_tracker(
    req: UpdateTrackerRequest,
    tracker: DeepSort,
    debug_publisher: Union[rospy.Publisher, None] = None,
) -> UpdateTrackerResponse:

    cv_im = cv2_img.msg_to_cv2_img(req.image)

    if tracker.polygon:
        detections = [
            [detection.xyseg for detection in req.detections],
            [detection.name for detection in req.detections],
            [detection.confidence for detection in req.detections],
        ]
    else:
        detections = [
            (detection.xywh, detection.confidence, detection.name)
            for detection in req.detections
        ]
    tracks = tracker.update_tracks(detections, frame=cv_im.copy())

    response = UpdateTrackerResponse()

    for track in tracks:
        detection = Detection()
        detection.name = track.track_id
        detection.confidence = track.det_conf
        detection.xywh = track.to_ltwh().astype(int).tolist()

        x, y, w, h = detection.xywh

        cv2.rectangle(cv_im, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            cv_im,
            f"{track.track_id}",
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

    if debug_publisher is not None:
        debug_publisher.publish(cv2_img.cv2_img_to_msg(cv_im))

    return response
