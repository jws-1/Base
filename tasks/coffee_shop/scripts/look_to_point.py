#!/usr/bin/env python3
from tiago_controllers import HeadController
import rospy

rospy.init_node("test")
hc = HeadController()

hc.