#!/usr/bin/env python3
import rospy
import os
import rospkg
from lasr_voice.voice import Voice
from tiago_controllers.helpers import get_pose_from_param, is_running
from sensor_msgs.msg import Image
from cv_bridge3 import CvBridge, cv2
from lasr_perception_server.srv import DetectImage
from models.controllers import Controllers
from lasr_perception_server import continuous_perception_publisher
from lasr_perception_server.msg import Detection
from std_msgs.msg import String


class SimpleMeetGreet:
    def __init__(self):
        self.continuous_perception_sub = None
        self.controllers = Controllers()
        #
        # if rospy.get_published_topics(namespace='/xtion'):
        #     print('in xtion')
        #     self.topic = '/xtion/rgb/image_raw'
        # else:
        #     print('in usb')
        #     self.topic = '/usb_cam/image_raw'

        self.map_points = ['/point1', '/point2']  # pos on map

    # def find_person(self):
    #     imgs = []
    #     rate = rospy.Rate(3)
    #     for i in range(IMAGES):
    #         im = rospy.wait_for_message('/usb_cam/image_raw', Image)
    #         imgs.append(im)
    #         # imgs.append(rospy.wait_for_message(self.topic, Image))
    #         rate.sleep()
    #
    #     det = rospy.ServiceProxy("lasr_perception_server/detect_objects_image", DetectImage)
    #     return resp

    def handle_findings(self, detections):
        names = []
        counter = 0
        if len(detections)> 0:
            for human in detections:
                print('human name', human.name)
                if human.name == 'person':
                    counter = counter + 1
                else:
                    self.voice.sync_tts("Hi" +str(human.name))

                    print('hi, ', human.name)
            if counter > 0:
                # self.voice.sync_tts(' there are new people. I have not met ' + str(counter) + 'people')
                if counter == 1:
                    # self.voice.sync_tts("there are new people. I have not met you before")
                    print(' there are new people. I have not met you before')
                else:
                    # self.voice.sync_tts(' there are new people. I have not met ' + str(counter) + 'people')
                    print(' there are new people. I have not met ' + str(counter) + 'people')


    def perception_cb(self,data):
        # define a callback for continuous perception
        if data.data != 'person':
            # self.voice.sync_tts("Hi" + str(data.data))
            print('hi, ', str(data.data))
        else:
            print('I dont known anyone')
            # self.voice.sync_tts('I dont known anyone')

    def main(self):
        # pos = get_pose_from_param('/door')
        # self.controllers.base_controller.sync_to_pose(pos)
        while not rospy.is_shutdown():
            self.continuous_perception_sub = rospy.Subscriber('/continuous_perception', String, self.perception_cb)
            rospy.sleep(5)
        # for i in range(4):
        #     resp = self.find_person()
        #     if len(resp) > 0:
        #         self.handle_findings(resp)
        #         return

        # self.voice.sync_tts("Hi, i don't know anyone")
        # print("Hi, i don't know anyone")
        # print('I dont known anyone')




if __name__ == '__main__':
    rospy.init_node("simple_meet_and_greet", anonymous=True)
    test = SimpleMeetGreet()
    try:
        test.main()
    except rospy.ROSInterruptException:
        pass
