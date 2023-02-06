#!/usr/bin/env python3

from recognise_people.srv import RecognisePeople,  RecognisePeopleResponse
import numpy as np
import rospy
from cv_bridge3 import CvBridge
from lasr_perception_server.msg import Detection

from cv_bridge3 import cv2
import random, os, rospkg
from sensor_msgs.msg import Image

class RecognisePeopleServer():
    """
    A Server for recognising known people using yolo and opencv
    """

    def __init__(self):

        self.face_detections = []
        self.yolo_detections = []
        self.bridge = CvBridge()


     #todo: get known people form rosparam or another funct

    def recogniser(self, req):
        response = RecognisePeopleResponse()

        self.yolo_detections = req.detected_objects_yolo
        self.face_detections = req.detected_objects_opencv
        detection_map = {}
        i = 0
        if len(self.yolo_detections) < 1 and len(self.face_detections) < 1:
            rospy.loginfo('empty resp')
            return response


        image = rospy.wait_for_message('/xtion/rgb/image_raw', Image)
        image = self.bridge.imgmsg_to_cv2(image)

        # yolo_map = {[] for msg in self.yolo_detections}
        yolo_map = {i: [] for i in range(len(self.yolo_detections))}

        for i, yolo in enumerate(self.yolo_detections):
            for face in self.face_detections:
                x1, y1, x2, y2 = face.xywh
                x3, y3, x4, y4 = yolo.xywh
                x4 = x3 + x4
                y4 = y3 + y4

                # if x1 < x3 and x2 > x4 and y1 < y3 and y2 > y4:
                if x3< x1 < x2 < x4 and y3 < y1 < y2 < y4:
                    yolo_map[i].append(face)

        for yolo, faces in yolo_map.items():
            body_x = (self.yolo_detections[yolo].xywh[0] + self.yolo_detections[yolo].xywh[2]) / 2 # center of the body in the x axis
            min_dist = np.inf
            best_face = None
            for face in faces:
                x1, y1, x2, y2 = face.xywh
                head_x = (x1 + x2) / 2
                dist = abs(head_x - body_x)
                if dist < min_dist:
                    min_dist = dist
                    best_face = face
            if best_face is not None:
                if best_face.confidence > 0.95:
                    response.detected_objects.append(Detection(name=best_face.name, confidence=best_face.confidence, xywh=self.yolo_detections[yolo].xywh))







        # get bb of person using face bb and body bb
        # maps name -> face(name, confidence, xywh) and yolo(name, conf, xywh)
        # for yolo in self.yolo_detections:
        #     for face in self.face_detections:
        #         x1, y1, x2, y2 = face.xywh
        #         # print(yolo, 'the yoloooo ')
        #         im = image.copy()
        #         cv2.rectangle(im, (x1, y1), (x2, y2),
        #                       (0, 0, 255), 2)
        #         cv2.rectangle(im, (yolo.xywh[0], yolo.xywh[1]), (yolo.xywh[0]+yolo.xywh[2], yolo.xywh[1]+yolo.xywh[3]),
        #                       (0, 0, 255), 2)
        #         # cv2.imshow('image', im)
        #         # cv2.waitKey(0)
        #
        #         head_x = (x1 + x2) / 2
        #         body_x = (yolo.xywh[0] + yolo.xywh[2]) / 2 # center of the body in the x axis
        #         # self.image_show(yolo.name, yolo.confidence,yolo.xywh, i)
        #         i = i+1
        #



                # if abs(head_x - body_x) < 50:
                #     if not face.name in detection_map.keys():
                #         # print('detection map', face, yolo)
                #         detection_map[face.name] = (face, yolo)
                #         rospy.loginfo("yolo detection here, bb overlapped.")
                #         break
                #     # if newer detection has bigger confidence
                #     if yolo.confidence > detection_map[face.name][1].confidence:
                #         detection_map[face.name] = (face, yolo)
                # else:
                #     print(face.name, face.xywh, yolo.xywh)

        # print()
        # print('-'* 40)
        # print(detection_map, 'detection map\n')
        # print(len(self.yolo_detections), '+'* 40)
        # print(len(self.face_detections), '-'* 40)
        # print()
        # print('-'* 40)
        # print(detection_map, 'detection map\n')

        # make confidence smooth
        # print(detection_map, 'detection map')
        # for name, (face, yolo) in detection_map.items():
        #     # if face.confidence > 0.7:
        #     response.detected_objects.append(Detection(name=face.name, confidence=face.confidence, xywh=yolo.xywh))
        #
        # # return response
        #
        #
        #
        #
        # if len(self.face_detections) >= len(self.yolo_detections) and len(self.face_detections) > 0:
        #     # print('face detections are more than yolo detections')
        #     for face in self.face_detections:
        #         # Append detection
        #         if face.confidence > 0.5:
        #             response.detected_objects.append(Detection(name=face.name, confidence=face.confidence,
        #                                                  xywh=face.xywh))
        # elif len(self.face_detections) < 1 and len(self.yolo_detections) > 0:
        #     # print('yolo detections are more than face detections')
        #     for person in self.yolo_detections:
        #         # Append detection.
        #         if person.confidence > 0.5:
        #             response.detected_objects.append(
        #                 Detection(
        #                     name=person.name,
        #                     confidence=person.confidence,
        #                     xywh=person.xywh
        #                 )
        #             )
        # else:
        #     response = []

        # print(response, '~'*40)
        return response

    def image_show(self, name, proba, dim, i):
        x1, y1, x2, y2 = dim
        path_output = os.path.join(rospkg.RosPack().get_path('face_detection'), "output")
        image = cv2.imread(path_output + "/images/random.jpg",0)
        # draw the bounding box of the face along with the associated
        # probability
        text = "{}: {:.2f}%".format(name, proba * 100)
        y = y1 - 10 if y1 - 10 > 10 else y1 + 10
        cv2.rectangle(image, (x1, y1), (x2+x1, y2+y1),
                      (0, 0, 255), 2)
        cv2.putText(image, text, (x1, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        # show the output image

        bridge = CvBridge()
        # cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        cv2.imwrite(path_output + "/images/random" + str(i+1)+".jpg", image)


if __name__ == "__main__":
    rospy.init_node("recognise_people_server")
    server = RecognisePeopleServer()
    service = rospy.Service('recognise_people_server', RecognisePeople, server.recogniser)
    rospy.loginfo("Recognise People Service initialised")
    rospy.spin()
