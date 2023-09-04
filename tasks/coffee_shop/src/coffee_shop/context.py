import rospy
import rosparam
from tiago_controllers import BaseController, HeadController
from lasr_voice import Voice
from play_motion_msgs.msg import PlayMotionAction
from lasr_object_detection_yolo.srv import YoloDetection
from coffee_shop.srv import TfTransform
from lasr_shapely import LasrShapely
from lasr_speech.srv import Speech
import actionlib
import yaml
from visualization_msgs.msg import Marker



class Context:

    def __init__(self, config_path):
        self.base_controller = BaseController()
        self.head_controller = HeadController()
        self.voice_controller = Voice()
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
        self.yolo = rospy.ServiceProxy('/yolov8/detect', YoloDetection)
        self.tf = rospy.ServiceProxy("/tf_transform", TfTransform)
        self.shapely = LasrShapely()
        self.speech = rospy.ServiceProxy("/lasr_speech/transcribe_and_parse", Speech)

        self.tables = dict()

        with open(config_path, "r") as fp:
            data = yaml.safe_load(fp)

        self.tables = {
            table: {"status" : "unvisited", "people": list(), "order": list()} for table in data["tables"].keys()
        }

        self.target_objects = data["objects"]

        self.current_table = None
        self.new_customer_pose = None

        self._people_pose_pub = rospy.Publisher("/people_poses", Marker, queue_size=100)
        self._people_idx = 0
        self._object_pose_pub = rospy.Publisher("/object_poses", Marker, queue_size=100)
        self._objects_idx = 0

    @staticmethod
    def _create_point_marker(idx, x, y, z, frame_id, r, g, b):
        marker_msg = Marker()
        marker_msg.header.frame_id = frame_id
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.id = idx
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = z
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        marker_msg.color.a = 1.0
        marker_msg.color.r = r
        marker_msg.color.g = g
        marker_msg.color.b = b
        return marker_msg

    def publish_person_pose(self, x, y, z, frame_id):
        self._people_pose_pub.publish(self._create_point_marker(self._people_idx, x, y, z, frame_id, 1.0, 0.0, 0.0))
        self._people_idx += 1

    def publish_object_pose(self, x, y, z, frame_id):
        self._object_pose_pub.publish(self._create_point_marker(self._objects_idx, x, y, z, frame_id, 0.0, 1.0, 0.0))
        self._objects_idx += 1