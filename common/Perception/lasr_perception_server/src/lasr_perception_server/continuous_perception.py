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
    global spotted_people
    """
    This function is responsible for the continuous perception of the robot.
    It is called every time the robot is in the idle state.
    It is responsible for the continuous perception of the robot.
    :return:
    """
    # define a publisher for continuous perception
    continuous_perception_pub = rospy.Publisher('/continuous_perception', String, queue_size=10)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        # call the perception server
        det = rospy.ServiceProxy("lasr_perception_server/detect_objects_images", OneDetectionImage)
        if rospy.get_published_topics(namespace='/usb_cam'):
            topic = '/usb_cam/image_raw'
        else:
            topic = '/xtion/rgb/image_raw'
        im = rospy.wait_for_message(topic, Image)
        resp = det(im, "coco", 0.9, 0.8, "person", 'known_people', rospy.Time.now().secs)
        print(resp, 'resp')

        for human in resp.detected_objects:
            if human.name not in introduced_people.keys():
                introduced_people[human.name] = resp.timestamp
                print('publishing', human.name)
                continuous_perception_pub.publish(human.name)
                rospy.set_param('known_people', introduced_people)
                print(rospy.get_param('known_people'))
        if introduced_people:
            print('introduced_people', introduced_people, 'before deleting')
            for human in list(introduced_people.keys()):
                if rospy.Time.now().secs - introduced_people[human] > 5:
                    try:
                        rospy.delete_param('known_people/' + human)
                    except KeyError:
                        pass
                    try:
                        del introduced_people[human]
                        print('del published', introduced_people)
                    except KeyError:
                        pass
            # print('____________________SLEEPING____________________')
        rate.sleep()



if __name__ == "__main__":
    rospy.init_node("continuous_perception_publisheear", anonymous=True)
    rospy.loginfo("initialising the continuous perception publisher")
    try:
        continuous_perception_publisher()
    except rospy.ROSInterruptException and rospy.service.ServiceException and rospy.exceptions.ROSInterruptException:
        pass
