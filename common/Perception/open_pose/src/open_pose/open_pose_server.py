#!/usr/bin/env python3

import rospy
import rospkg
from std_msgs.msg import String

from open_pose.srv import DetectKeypoints, DetectKeypointsResponse
from open_pose.open_pose_net import OpenPose

import os
from cv_bridge3 import CvBridge
from urllib.request import urlopen
from shutil import copyfileobj

PROTO_PATH = os.path.join(rospkg.RosPack().get_path('open_pose'), 'nets', 'pose_deploy_linevec.prototxt')
COCO_WEIGHTS_PATH = os.path.join(rospkg.RosPack().get_path('open_pose'), 'models', 'coco', 'pose_iter_440000.caffemodel')

PROTO_ROOT = os.path.join(rospkg.RosPack().get_path('open_pose'), 'nets')
COCO_ROOT = os.path.join(rospkg.RosPack().get_path('open_pose'), 'models', 'coco')

PROTO = os.path.join(PROTO_ROOT, 'pose_deploy_linevec.prototxt')
COCO_WEIGHTS = os.path.join(COCO_ROOT, 'pose_iter_440000.caffemodel')

PROTO_URL = "https://github.com/foss-for-synopsys-dwc-arc-processors/synopsys-caffe-models/raw/master/caffe_models/openpose/caffe_model/pose_deploy_linevec.prototxt?download="
COCO_WEIGHTS_URL = "https://github.com/foss-for-synopsys-dwc-arc-processors/synopsys-caffe-models/raw/master/caffe_models/openpose/caffe_model/pose_iter_440000.caffemodel?download="

class OpenPoseServer():

    def __init__(self):
        self.open_pose = OpenPose(PROTO_PATH, COCO_WEIGHTS_PATH)
        self.bridge = CvBridge()
    
    def __call__(self, req):
        keypoints_json = self.open_pose(self.bridge.imgmsg_to_cv2_np(req.image), True)
        print(keypoints_json)
        return DetectKeypointsResponse(keypoints_json)

if __name__ == "__main__":
    rospy.init_node("open_pose")

    if not os.path.exists(PROTO_ROOT):
        os.makedirs(PROTO_ROOT, exist_ok=True)

    if not os.path.exists(PROTO_PATH):
        rospy.loginfo("Downloading Net...")
        with urlopen(PROTO_URL) as f_in, open(PROTO_PATH, 'wb') as f_out:
            copyfileobj(f_in, f_out)
        rospy.loginfo("Downloaded Net...")

    if not os.path.exists(COCO_ROOT):
        os.makedirs(COCO_ROOT, exist_ok=True)

    if not os.path.exists(COCO_WEIGHTS):
        rospy.loginfo("Downloading model trained on coco...")
        with urlopen(COCO_WEIGHTS_URL) as f_in, open(COCO_WEIGHTS, 'wb') as f_out:
            copyfileobj(f_in, f_out)
        rospy.loginfo("Downloaded model trained on coco.")

    server = OpenPoseServer()
    service = rospy.Service('detect_keypoints', DetectKeypoints, server)
    rospy.loginfo("Open Pose Service is ready.")
    rospy.spin()
