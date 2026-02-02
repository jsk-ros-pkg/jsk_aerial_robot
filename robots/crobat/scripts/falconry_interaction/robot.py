#!usr/bin/env python
import rospy
from std_msgs.msg import Empty, Int64
from geometry_msgs.msg import Pose
from variables import *

class Flapper:
    def __init__(self):
        self.state_sub = rospy.Subscriber('/state', Int64,
                                           self.state_callback)
        self.start_pub = rospy.Publisher('/crobat/teleop_command/start', Empty)
        self.takeoff_pub = rospy.Publisher('/crobat/teleop_command/takeoff', Empty)
        self.land_pub = rospy.Publisher('/crobat/teleop_command/land', Empty)
        self.halt_pub = rospy.Publisher('/crobat/teleop_command/halt', Empty)
        self.palm_land_pub = rospy.Publisher('palm_land', Pose)
        self.state = RobotState.START

    def state_callback(self, msg):
        self.state = msg.data
        
    def takeoff(self):
        self.start_pub.publish()
        rospy.sleep(2)
        self.takeoff_pub.publish()
        self.state = RobotState.TAKEOFF
        rospy.sleep(3)
        self.state = RobotState.HOVER

    def land(self):
        self.land_pub.publish()

    def palm_land(self, yaw):
        self.land_pub.publish()
        self.state = RobotState.STOP

    def stop(self):
        self.halt_pub.publish()
