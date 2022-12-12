#!/usr/bin/env python3
import rospy

# for debugging

from cv_bridge3 import CvBridge
from cv_bridge3 import cv2
from lasr_perception_server.srv import DetectImage, OneDetectionImage

import os, random
import rospkg
from sensor_msgs.msg import Image
from lasr_perception_server.msg import Detection
from std_msgs.msg import String

IMAGES = 1

def continuous_perception_publisher():
    """
    This function is responsible for the continuous perception of the robot.
    It is called every time the robot is in the idle state.
    It is responsible for the continuous perception of the robot.
    :return:
    """
    # define a publisher for continuous perception
    continuous_perception_pub = rospy.Publisher('/continuous_perception', String, queue_size=10)
    rate = rospy.Rate(3)
    imgs = []
    print(' i initialised the pub')

    while not rospy.is_shutdown():
        # call the perception server
        det = rospy.ServiceProxy("lasr_perception_server/detect_objects_images", OneDetectionImage)
        # im = rospy.wait_for_message('/usb_cam/image_raw', Image)
        im = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        imgs.append(im)
        resp = det(im, "coco", 0.7, 0.3, "person", 'known_people').detected_objects
        imgs.clear()
        print(resp)
        resp = [r.name for r in resp]
        # publish the result
        for r in set(resp):
            continuous_perception_pub.publish(r)
        rate.sleep()



if __name__ == "__main__":
    rospy.init_node("continuous_perception_publisheear", anonymous=True)
    rospy.loginfo("initialising the continuous perception publisher")
    try:
        continuous_perception_publisher()
    except rospy.ROSInterruptException:
        pass
