#!/usr/bin/env python3
import rospy
from cv_bridge3 import CvBridge, cv2
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from coffee_shop.srv import TfTransform, TfTransformRequest
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from lasr_object_detection_yolo.srv import YoloDetection

rospy.init_node("raycast")
ci = rospy.wait_for_message("/xtion/rgb/camera_info", CameraInfo)
tf = rospy.ServiceProxy("/tf_transform", TfTransform)
camera = PinholeCameraModel()
camera.fromCameraInfo(ci)
bridge = CvBridge()
marker_pub = rospy.Publisher("/person", Marker, queue_size=100)

detect = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
import numpy as np
while True:
    try:
        img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
        detections = detect(img_msg, "yolov8n-seg.pt", 0.5, 0.3).detected_objects
        detections = [det for det in detections if det.name == "person"]

        """
        Missing step is to ensure vectors are relative to the cardinal orientation of the head...
        """


        person1 = detections[0]
        person2 = detections[1]
        px1, py1 = np.mean(np.array(person1.xyseg).reshape(-1, 2), axis=0)
        px2, py2 = np.mean(np.array(person2.xyseg).reshape(-1, 2), axis=0)
        ray1 = camera.projectPixelTo3dRay((px1, py1))
        ray2 = camera.projectPixelTo3dRay((px2, py2))
        vec1 = np.array([ray1[0], ray1[1], 1.0])
        vec2 = np.array([ray2[0], ray2[1], 1.0])
        mid = (vec1 + vec2) / 2


        # Create a marker message
        marker = Marker()
        marker.id = 0
        marker.header.frame_id = img_msg.header.frame_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        # Define start and end points of the ray
        start_point = Point()
        start_point.x = 0.0
        start_point.y = 0.0
        start_point.z = 0.0
        end_point = Point(*vec1)
        marker.points = [start_point, end_point]
        marker_pub.publish(marker)


        # Create a marker message
        marker = Marker()
        marker.id = 1
        marker.header.frame_id = img_msg.header.frame_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        # Define start and end points of the ray
        start_point = Point()
        start_point.x = 0.0
        start_point.y = 0.0
        start_point.z = 0.0
        end_point = Point(*vec2)
        marker.points = [start_point, end_point]
        marker_pub.publish(marker)


        # Calculate the dot product
        dot = np.dot(vec1, vec2)

        # Calculate the magnitudes of the vectors
        magnitude1 = np.linalg.norm(vec1)
        magnitude2 = np.linalg.norm(vec2)

        # Calculate the cosine of the angle
        cosine_angle = dot / (magnitude1 * magnitude2)

        # Calculate the angle in radians
        angle_radians = np.arccos(cosine_angle)

        # Convert radians to degrees
        angle_degrees = np.degrees(angle_radians)

        # Create a marker message
        marker = Marker()
        marker.id = 2
        marker.header.frame_id = img_msg.header.frame_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = f"{round(angle_degrees, 2)}°"
        marker.action = Marker.ADD
        marker.color.a = 1.0
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position = Point(*mid)
        marker_pub.publish(marker)
    except KeyboardInterrupt:
        break
rospy.spin()