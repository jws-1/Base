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
introduced_people = {}

def continuous_perception_publisher():
    global introduced_people
    """
    This function is responsible for the continuous perception of the robot.
    It is called every time the robot is in the idle state.
    It is responsible for the continuous perception of the robot.
    :return:
    """
    # define a publisher for continuous perception
    continuous_perception_pub = rospy.Publisher('/continuous_perception', String, queue_size=10)
    print(' i initialised the pub')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # call the perception server
        det = rospy.ServiceProxy("lasr_perception_server/detect_objects_images", OneDetectionImage)
        # im = rospy.wait_for_message('/usb_cam/image_raw', Image)
        if rospy.get_published_topics(namespace='/usb_cam'):
            topic = '/usb_cam/image_raw'
        else:
            topic = '/xtion/rgb/image_raw'
        im = rospy.wait_for_message(topic, Image)
        resp = det(im, "coco", 0.7, 0.3, "person", 'known_people', rospy.Time.now().secs)
        print(resp, 'resp')
        for human in resp.detected_objects:
            if human.name not in introduced_people:
                introduced_people[human.name] = resp.timestamp
                continuous_perception_pub.publish(human.name)
        if introduced_people:
            print('introduced_people', introduced_people)
            for human in list(introduced_people.keys()):
                if rospy.Time.now().secs - introduced_people[human] > 10:
                    del introduced_people[human]
                    print('del published', introduced_people)
        print('____________________SLEEPING____________________')
        rate.sleep()



if __name__ == "__main__":
    rospy.init_node("continuous_perception_publisheear", anonymous=True)
    rospy.loginfo("initialising the continuous perception publisher")
    try:
        continuous_perception_publisher()
    except rospy.ROSInterruptException and rospy.service.ServiceException and rospy.exceptions.ROSInterruptException:
        pass
