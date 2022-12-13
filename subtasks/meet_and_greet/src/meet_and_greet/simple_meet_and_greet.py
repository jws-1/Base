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

previous_name = ""
class SimpleMeetGreet:
    def __init__(self):
        self.continuous_perception_sub = None
        self.voice = Voice()


    def perception_cb(self,data):
        global previous_name
        # define a callback for continuous perception
        # print(data, 'data')
        if data.data != 'person' and data.data != previous_name:
            print(rospy.get_param('known_people'))
            self.voice.sync_tts("Good afternoon" + str(data.data))
            print('hi, ', str(data.data))
        # else:
        #     print('I dont known anyone')
            # self.voice.sync_tts('I dont known anyone')
        previous_name = data.data

    def main(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.continuous_perception_sub = rospy.Subscriber('/continuous_perception', String, self.perception_cb)
            rate.sleep()
            # rospy.sleep(5)




if __name__ == '__main__':
    rospy.init_node("simple_meet_and_greet", anonymous=True)
    test = SimpleMeetGreet()
    try:
        test.main()
    except rospy.ROSInterruptException:
        pass
