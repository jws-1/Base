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
introduced_people = {}
previous_stranger_time = 0


class SimpleMeetGreet:
    def __init__(self):
        self.continuous_perception_sub = None
        self.voice = Voice()


    def perception_cb(self,data):
        global previous_name, previous_stranger_time
        # define a callback for continuous perception
        # print(data, 'data')
        if data.data != 'person' and data.data not in introduced_people.keys():
        # if data.data != 'person' and data.data != previous_name:
            # if data.data != 'person':
            print(rospy.get_param('known_people'))
            self.voice.sync_tts("Good afternoon" + str(data.data))
            try:
                rospy.delete_param('known_people/' + data.data)
            except KeyError:
                pass
            print('hi, ', str(data.data))
            introduced_people[data.data] = rospy.Time.now().secs
        elif data.data == 'person' and rospy.Time.now().secs - previous_stranger_time > 5:
            self.voice.sync_tts('Hi, Stranger')
            previous_stranger_time = rospy.Time.now().secs
        previous_name = data.data




    def main(self):
        print('main')
        # rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            self.continuous_perception_sub = rospy.Subscriber('/continuous_perception', String, self.perception_cb)
            if introduced_people:
                print(introduced_people, 'introduced_people')
                for person in list(introduced_people.keys()):
                    print(person, 'person')
                    print(rospy.Time.now().secs - introduced_people[person], 'rospy.Time.now().secs - introduced_people[person]')
                    if rospy.Time.now().secs - introduced_people[person] > 5:
                        # try:
                        del introduced_people[person]
                        # except:
                        #     pass
            # rate.sleep()
            # rospy.sleep(5)




if __name__ == '__main__':
    rospy.init_node("simple_meet_and_greet", anonymous=True)
    test = SimpleMeetGreet()
    try:
        test.main()
    except rospy.ROSInterruptException:
        pass
