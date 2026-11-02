#!/uer/bin/env python

import rospy
import aerial_robot_msgs.msg import FlightNav, PoseControlPid
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest
from IPython import embed
import math
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, Empty, Int8, Int16, UInt8
import tf
import time 

class KinikunInterface:
    def __init__(self):
        #ros messages
        self.cog_odom_ = Odometry()
        self.flight_state_ = UInt8()
        self.pid_ = PoseControlPid()
        self.joint_state_ = JointState()
        self.joy_ = Joy()

        #parameters for task
        self.robot_ns = rospy.get_namespace()

        #ros publishers and subscribers
        self.tf_lister_ = tf.TransformListener()
        self.joints_ctrl_pub_ = rospy.Publiser('joints_ctrl', JointState, queue_size=1)
        self.uav_nav_pub_ = rospy.Publisher('uav_nav', FlightNav, queue_size=1)

        self.start_pub_ = rospy.Publisher('teleop_command/start', Empty, queue_size=1)
        self.takeoff_pub_ = rospy.Publisher('teleop_command/takeoff', Empty, queue_size=1)
        self.land_pub_ = rospy.Publisher('teleop_command/land', Empty, queue_size=1)
        self.force_landing_pub_ = rospy.Publisher('teleop_command/force_landing',Empty, queue_size=1)
        self.halt_pub_ = rospy.Publisher('teleop_command/halt', Empty, queue_size=1)
        rospy.Subscriber("debug/pose/pid", PoseControlPid, self.pidCallback)
        rospy.Subscriber('flight_state', UInt8, self.flightStateCallback)
        rospy.Subscriber('force_skip', Empty, self.forceSkipCallback)
        rospy.Subscriber('joy', Joy, self.joyCallback)
        rospy.Subscriber('joint_states', JointState, self.jointStateCallback)
        rospy.Subscriber('cog_odom', Odometry, self.cogOdomCallback)

        rospy.wait_for_service("controller/nplot/set_parameters")
        self.dynamic_reconfigure_client = rospy.ServiceProxy("controller/nplot/set_parameters", Reconfigure)
        rospy.wait_for_service('add_extra_module')
        self.add_extra_module_client = rospy.ServiceProxy('add_extra_module', AddExtraModule)

        time.sleep(2.0)
        print("created" + self.robot_ns + "interface")

        def start(self, sleep = 1.0):
            self.start_pub_.publish()
            rospy.sleep(sleep)
        def takeoff(self):
            self.takeoff_pub_.publish()
        def land(self):
            self.land_pub_.publish()
        def halt(self):
            self.halt_pub_.publish()
        def forceLanding(self):
            self.force_landing_pub_.publish()
        def cogOdomCallback(self, msg):
            self.cog_odom_ = msg
        def gteCogOdom(self):
            return self.cog_odom_
        def getCogPostion(self):
            return np.array([self.cog_odom_.pose.pose.position.x, self.cog_odom_.pose.pose.position.y, self.cog_odom_.pose.pose.position.z])
        def getCogOrientation(self):
            return np.array([self.cog_odom_.pose.pose.orientation.x, self.cog_odom_.pose.pose.orientation.y, self.cog_odom_.pose.pose.orientation.z, self.cog_odom_.pose.pose.orientation.w])
        def getCogRotationMatrix(self):
            return np.array(tf.transformations.quaternion_matrix(self.getCogQuaternion()))[0:3, 0:3]

        def flightStateCallback(self, msg):
            self.flight_state_ = msg.data
        def getFlightState(self):
            return self.flight_state_
        def pidCallback(self, msg):
            self.pid_ = msg
        def getPid(self):
            return self.pid_      


