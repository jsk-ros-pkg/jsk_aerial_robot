import rospy
import actionlib
import hydrus_xi_transformation_planning.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
import numpy as np
import ros_numpy as ros_np
from tf.transformations import *
from std_msgs.msg import Empty
from actionlib_msgs.msg import GoalStatus

class HydrusXiInterface:
    def __init__(self):
        self.joint_state_sub_ = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self.jointStateCallback)
        self.cog_odom_sub_ = rospy.Subscriber('/uav/cog/odom', Odometry, self.cogOdomCallback)
        self.baselink_odom_sub_ = rospy.Subscriber('/uav/baselink/odom', Odometry, self.baselinkOdomCallback)
        self.nav_pub_ = rospy.Publisher('/uav/nav', FlightNav, queue_size = 1)
        self.start_pub_ = rospy.Publisher('/teleop_command/start', Empty, queue_size = 1)
        self.takeoff_pub_ = rospy.Publisher('/teleop_command/takeoff', Empty, queue_size = 1)
        self.land_pub_ = rospy.Publisher('/teleop_command/land', Empty, queue_size = 1)

        self.joint_state_ = None
        self.cog_odom_ = None
        self.baselink_odom_ = None
        self.joint_trajectory_client_ = actionlib.SimpleActionClient('transformation_planner', hydrus_xi_transformation_planning.msg.JointTrajectoryAction)

    def setJointAngle(self, target_joint_state, gimbal_velocity_limit = 3.0, execute_time = 10.0, interpolation_times = 30):
        self.joint_trajectory_client_.wait_for_server()
        goal = hydrus_xi_transformation_planning.msg.JointTrajectoryGoal()
        goal.joint_state = target_joint_state
        goal.gimbal_velocity_limit = gimbal_velocity_limit #rad/s
        goal.execute_time = execute_time
        goal.interpolation_times = interpolation_times
        self.joint_trajectory_client_.send_goal(goal)

    def preemptSetJointAngle(self):
        self.joint_trajectory_client_.cancel_goal()

    def getClientState(self):
        return self.joint_trajectory_client_.get_state()

    def waitForSetJointAngleResult(self):
        self.joint_trajectory_client_.wait_for_result()

    def jointStateCallback(self, msg):
        self.joint_state_ = msg

    def cogOdomCallback(self, msg):
        self.cog_odom_ = msg

    def baselinkOdomCallback(self, msg):
        self.baselink_odom_ = msg

    def navigation(self, nav_msg):
        self.nav_pub_.publish(nav_msg)

    def getJointState(self):
        return self.joint_state_

    def getBaselinkOdom(self):
        return self.baselink_odom_

    def getCogOdom(self):
        return self.cog_odom_

    def start(self):
        self.start_pub_.publish()

    def takeOff(self):
        self.takeoff_pub_.publish()

    def startAndTakeOff(self):
        self.start()
        rospy.sleep(0.5)
        self.takeOff()

    def land(self):
        self.land_pub_.publish()

    def getBaselinkPos(self):
        return ros_np.numpify(self.baselink_odom_.pose.pose.position)

    def getBaselinkRot(self):
        return quaternion_matrix(ros_np.numpify(self.baselink_odom_.pose.pose.orientation))

    def getBaselinkRPY(self):
        return euler_from_quaternion(ros_np.numpify(self.baselink_odom_.pose.pose.orientation))

    def getBaselinkLinearVel(self):
        return ros_np.numpify(self.baselink_odom_.twist.twist.linear)

    def getCogPos(self):
        return ros_np.numpify(self.cog_odom_.pose.pose.position)

    def getCogRot(self):
        return quaternion_matrix(ros_np.numpify(self.cog_odom_.pose.pose.orientation))

    def getCogRPY(self):
        return euler_from_quaternion(ros_np.numpify(self.cog_odom_.pose.pose.orientation))

    def getCogLinearVel(self):
        return ros_np.numpify(self.cog_odom_.twist.twist.linear)

    def noNavigation(self):
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
        self.navigation(nav_msg)
