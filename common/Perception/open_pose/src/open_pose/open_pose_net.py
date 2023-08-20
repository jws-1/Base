#!/usr/bin/env python3

import cv2
import numpy as np
import json

class OpenPose():

    def __init__(self, proto_path, weights_path):
        self.net = cv2.dnn.readNetFromCaffe(proto_path, weights_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        
        # COCO Output format
        self.labels = ['Nose', 'Neck', 'R-Sho', 'R-Elb', 'R-Wr', 'L-Sho', 'L-Elb', 'L-Wr', 'R-Hip', 'R-Knee', 'R-Ank', 'L-Hip', 'L-Knee', 'L-Ank', 'R-Eye', 'L-Eye', 'R-Ear', 'L-Ear']
        self.POSE_PAIRS = [[1,2], [1,5], [2,3], [3,4], [5,6], [6,7],
                    [1,8], [8,9], [9,10], [1,11], [11,12], [12,13],
                    [1,0], [0,14], [14,16], [0,15], [15,17],
                    [2,17], [5,16] ]

        self.pafs = [[31,32], [39,40], [33,34], [35,36], [41,42], [43,44],
                [19,20], [21,22], [23,24], [25,26], [27,28], [29,30],
                [47,48], [49,50], [53,54], [51,52], [55,56],
                [37,38], [45,46]]

    def __call__(self, image, jsonify=False):
        net_output = self.forward(image)
        keypoints = self.detect_keypoints(net_output, image)

        keypoints_dict = {k : v for k, v in zip(self.labels, keypoints)}

        if jsonify:
            class NumpyEncoder(json.JSONEncoder):
                def default(self, obj):
                    if isinstance(obj, np.float32):
                        return float(obj)
                    return json.JSONEncoder.default(self, obj)
            return json.dumps(keypoints_dict, cls=NumpyEncoder)
        else:
            return keypoints

    def forward(self, image):
        in_height = 368
        in_width = int((in_height/image.shape[0])*image.shape[1])
        inpBlob = cv2.dnn.blobFromImage(image, 1.0 / 255, (in_width, in_height),
                            (0, 0, 0), swapRB=False, crop=False)
        self.net.setInput(inpBlob)
        return self.net.forward()
    
    def detect_keypoints(self, net_output, image):
        idx = 0
        detected_keypoints = []
        for n in range(len(self.POSE_PAIRS)):
            
            prob_map = net_output[0,n,:,:]
            prob_map = cv2.resize(prob_map, (image.shape[1], image.shape[0]))
            keypoints = []
            for keypoint in self.get_keypoints(prob_map):
                keypoints.append(keypoint + (idx,))
                idx+=1
            detected_keypoints.append(keypoints)
        return detected_keypoints


    def get_keypoints(self, prob_map, threshold=0.1):

        keypoints = []

        gauss_map = cv2.GaussianBlur(prob_map, (3, 3), 0, 0)
        mask = np.uint8(gauss_map > threshold)

        for contour in cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]:

            blob = np.zeros_like(mask)
            blob = cv2.fillConvexPoly(blob, contour, 1)

            masked_prob_map = gauss_map * blob
            max_loc = cv2.minMaxLoc(masked_prob_map)[3]

            keypoints.append(max_loc + (prob_map[max_loc[1], max_loc[0]],))
        
        return keypoints
