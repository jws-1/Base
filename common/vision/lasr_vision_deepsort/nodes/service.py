#!/usr/bin/env python3
import rospy

from lasr_vision_msgs.srv import StartTracker, UpdateTracker, StopTracker


class DeepSORT:
    def __init__(self):
        pass

    def start_tracker(self, req):
        """
        Sets up the tracker with the given parameters
        """
        pass

    def update_tracker(self, req):
        """
        Updates the tracker with the given detections, and returns tracked objects in the frame
        """
        pass

    def stop_tracker(self, req):
        """
        Stops the tracker
        """
        pass


if __name__ == "__main__":
    rospy.init_node("lasr_vision_deepsort")

    deepsort = DeepSORT()
    rospy.loginfo("DeepSORT service started")

    rospy.Service("start_tracker", StartTracker, deepsort.start_tracker)
    rospy.Service("update_tracker", UpdateTracker, deepsort.update_tracker)
    rospy.Service("stop_tracker", StopTracker, deepsort.stop_tracker)

    rospy.spin()
