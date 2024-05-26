#!/usr/bin/env python

import rospy
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from geometry_msgs.msg import PoseStamped
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import  Empty,UInt8
import tf
import time

class DeltaInterface:
    def __init__(self):
        # ros messages
        self.cog_odom_ = Odometry()
        self.flight_state_ = UInt8()
        self.pid_ = PoseControlPid()
        self.joint_state_ = JointState()
        self.joy_ = Joy()

        ## flight states
        self.ARM_OFF_STATE = 0
        self.START_STATE = 1
        self.ARM_ON_STATE = 2
        self.TAKEOFF_STATE = 3
        self.LAND_STATE = 4
        self.HOVER_STATE = 5
        self.STOP_STATE = 6

        # parameters for task
        self.robot_ns = rospy.get_namespace()
        self.force_skip_ = False

        # ros subscriber and publisher
        self.tf_listener_ = tf.TransformListener()
        self.joints_ctrl_pub_ = rospy.Publisher('joints_ctrl', JointState, queue_size = 1)
        self.uav_nav_pub_ = rospy.Publisher("uav/nav", FlightNav, queue_size=1)

        self.start_pub_ = rospy.Publisher('teleop_command/start', Empty, queue_size = 1)
        self.takeoff_pub_ = rospy.Publisher('teleop_command/takeoff', Empty, queue_size = 1)
        self.land_pub_ = rospy.Publisher('teleop_command/land', Empty, queue_size = 1)
        self.force_landing_pub_ = rospy.Publisher('teleop_command/force_landing', Empty, queue_size = 1)
        self.halt_pub_ = rospy.Publisher('teleop_command/halt', Empty, queue_size = 1)

        rospy.Subscriber("debug/pose/pid", PoseControlPid, self.pidCallback)
        rospy.Subscriber('flight_state', UInt8, self.flightStateCallback)
        rospy.Subscriber("joint_states", JointState, self.jointStateCallback)
        rospy.Subscriber("joy", Joy, self.joyCallback)
        rospy.Subscriber("uav/cog/odom", Odometry, self.cogOdomCallback)

        time.sleep(2.0)
        rospy.logwarn("created " + self.robot_ns + " interface")


    def start(self, sleep = 1.0):
        self.start_pub_.publish()
        rospy.sleep(sleep)
    def takeoff(self):
        self.takeoff_pub_.publish()
    def land(self):
        self.land_pub_.publish()
    def forceLanding(self):
        self.force_landing_pub_.publish()
    def halt(self):
        self.halt_pub_.publish()

    def cogOdomCallback(self, msg):
        self.cog_odom_ = msg
    def getCogOdom(self):
        return self.cog_odom_

    def flightStateCallback(self, msg):
        self.flight_state_ = msg.data
    def getFlightState(self):
        return self.flight_state_

    def pidCallback(self, msg):
        self.pid_ = msg
    def getPID(self):
        return self.pid_
    def posControlConverged(self, pos=[0, 0, 0], thresh=0.05):
        if math.sqrt((pos[0] - self.cog_odom_.pose.pose.position.x)**2 + (pos[1] - self.cog_odom_.pose.pose.position.y)**2 + (pos[2] - self.cog_odom_.pose.pose.position.z)**2) < thresh:
            return True
        else:
            return False
    def getPosControlError(self, pos=[0, 0, 0]):
        return [pos[0] - self.cog_odom_.pose.pose.position.x, pos[1] - self.cog_odom_.pose.pose.position.y, pos[2] - self.cog_odom_.pose.pose.position.z]

    def yawControlConverged(self, thresh=0.1):
        if(abs(self.pid_.yaw.err_p) < thresh):
            return True
        else:
            return False

    def setTargetPos(self, x, y, z):
        msg = FlightNav()
        msg.control_frame = msg.WORLD_FRAME
        msg.target = msg.COG
        msg.pos_xy_nav_mode = msg.POS_MODE
        msg.target_pos_x = x
        msg.target_pos_y = y
        msg.pos_z_nav_mode = msg.POS_MODE
        msg.target_pos_z = z
        self.uav_nav_pub_.publish(msg)

    def setTargetYaw(self, yaw):
        msg = FlightNav()
        msg.yaw_nav_mode = msg.POS_MODE
        msg.target_yaw = yaw
        self.uav_nav_pub_.publish(msg)

    def getTransform(self, parent, child):
        try:
            (trans, rot) = self.tf_listener_.lookupTransform(self.robot_ns + parent, self.robot_ns + child, rospy.Time(0))
            return (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)

    def sendJointState(self, joint_state_msg):
        self.joints_ctrl_pub_.publish(joint_state_msg)

    def jointStateCallback(self, msg):
        self.joint_state_ = msg

    def joyCallback(self, msg):
        self.joy_ = msg

    def getForceSkipFlag(self):
        return self.force_skip_
    def setForceSkipFlag(self, flag):
        self.force_skip_ = flag

if __name__ == "__main__":
    rospy.init_node("delta_interface")
    node = DeltaInterface()
    rospy.spin()
