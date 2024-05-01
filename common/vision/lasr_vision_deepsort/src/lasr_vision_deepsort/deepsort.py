from deep_sort_realtime.deepsort_tracker import DeepSort

from lasr_vision_msgs.srv import (
    StartTrackerRequest,
    UpdateTrackerRequest,
    UpdateTrackerResponse,
)


def create_tracker(req: StartTrackerRequest) -> DeepSort:
    """
    Sets up the tracker with the given parameters
    """
    return DeepSort(
        max_age=req.max_age,
        embedder=req.embedder,
        embedder_name=req.torchreid_embedder or None,
        polygon=req.polygon,
    )


def update_tracker(
    req: UpdateTrackerRequest, tracker: DeepSort
) -> UpdateTrackerResponse:
    if tracker.polygon:
        detections = [
            (detection.xyseg, detection.confidence, detection.name)
            for detection in req.detections
        ]
    else:
        detections = [
            (detection.xywh, detection.confidence, detection.name)
            for detection in req.detections
        ]
    tracks = tracker.update_tracks(detections)
    return UpdateTrackerResponse()
