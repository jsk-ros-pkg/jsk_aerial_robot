'''
 Refactored by li-jinjie on 25-3-19.
'''
import rospy
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from abc import ABC, abstractmethod

from geometry_msgs.msg import PoseStamped
from util import check_first_data_received, check_topic_subscription


##########################################
# one-to-one mapping of all instantiation classes required.
##########################################

class PoseBase(ABC):
    def __init__(self, object_name: str, topic_name: str, msg_type):
        """
        Base class for subscribing to pose_msg information.
        :param topic_name: The name of the topic to subscribe to.
        :param msg_type: The message type for pose_msg data.
        """
        self.object_name = object_name
        self.pose_msg = None

        self.subscriber = self._sub_topic(topic_name, msg_type)

        check_first_data_received(self, "pose_msg", self.object_name)

    @check_topic_subscription
    def _sub_topic(self, topic_name, msg_type):
        rospy.loginfo(f"Subscribed to {topic_name}")
        return rospy.Subscriber(topic_name, msg_type, self._pose_msg_callback, queue_size=3)

    def _pose_msg_callback(self, msg):
        """
        Callback function to process incoming pose_msg messages.
        Should be implemented by subclasses.
        :param msg: The message containing pose_msg data.
        """
        self.pose_msg = msg

    def get_pose_msg(self):
        """
        Return pose_msg information from the message.
        :return: Extracted pose_msg data.
        """
        return self.pose_msg


class HandPose(PoseBase):
    def __init__(self):
        super().__init__(object_name="Hand", topic_name="/hand/mocap/pose", msg_type=PoseStamped)


class ArmPose(PoseBase):
    def __init__(self):
        super().__init__(object_name="Arm", topic_name="/arm/mocap/pose", msg_type=PoseStamped)


class DronePose(PoseBase):
    def __init__(self, robot_name):  # Note the data type is odometry
        super().__init__(object_name="Drone", topic_name=f"/{robot_name}/uav/cog/odom", msg_type=Odometry)

    def _pose_msg_callback(self, msg):
        self.pose_msg = msg.pose


class Glove:
    def __init__(self):
        self.object_name = "Glove"
        self.control_mode = None
        self.control_mode_sub = self._sub_topic("/hand/control_mode", UInt8, self._glove_data_callback)

        self._check_data_initialized()

    @check_topic_subscription
    def _sub_topic(self, topic_name, msg_type, callback_func):
        rospy.loginfo(f"Subscribed to {topic_name}")
        return rospy.Subscriber(topic_name, msg_type, callback_func, queue_size=3)

    def _check_data_initialized(self):
        """
        Waits until the pose_msg is initialized. Logs a message repeatedly
        until a valid pose_msg is received.
        """
        while not rospy.is_shutdown() and self.control_mode is None:
            rospy.loginfo(f"Waiting for {self.object_name}'s data...")
            rospy.sleep(0.2)
        if self.control_mode is not None:
            rospy.loginfo(f"{self.object_name}'s data received for the first time")

    def _glove_data_callback(self, msg: UInt8):
        """
        Callback function to process incoming messages.
        Should be implemented by subclasses.
        :param msg: The message containing glove data.
        """
        self.control_mode = msg.data

    def get_control_mode(self):
        """
        Return pose_msg information from the message.
        :return: Extracted glove data.
        """
        return self.control_mode
