"""
Refactored by li-jinjie on 25-3-19.
"""

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
    def __init__(self, param_name="/hand/control_mode", default_value=1):
        """
        Initializes the ControlModeManager.

        :param param_name: The name of the ROS parameter to manage.
        :param default_value: The default value if the parameter does not exist.
        """
        self.param_name = param_name

        # Ensure the parameter is set in ROS if not already present
        if not rospy.has_param(self.param_name):
            rospy.set_param(self.param_name, default_value)
            rospy.loginfo(f"Initialized {self.param_name} with default value: {default_value}")

        self._control_mode = self.get_control_mode()

    def get_control_mode(self):
        """
        Retrieves the current value of the control mode parameter.

        :return: The current control mode value.
        """
        return rospy.get_param(self.param_name)

    @staticmethod
    def set_control_mode(new_mode: int):
        """
        Updates the control mode parameter.

        :param new_mode: The new control mode value to set.
        """
        rospy.set_param("/hand/control_mode", new_mode)

    def print_current_mode(self):
        """
        Prints the current control mode.
        """
        mode = self.get_control_mode()
        print(f"Current control mode: {mode}")
