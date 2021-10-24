#!/usr/bin/env python

import sys
import time
import rospy

from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import UInt8, Empty
from sensor_msgs.msg import Joy

class Demo(object):

    def __init__(self):

        self.ARM_OFF_STATE = 0
        self.START_STATE = 1
        self.ARM_ON_STATE = 2
        self.TAKEOFF_STATE = 3
        self.LAND_STATE = 4
        self.HOVER_STATE = 5
        self.STOP_STATE = 6

        self.flight_state = self.ARM_OFF_STATE
        self.joy_flag = False

        self.flight_state_sub = rospy.Subscriber('dragon/flight_state', UInt8, self.flightStateCb)
        self.joy_sub = rospy.Subscriber('dragon/joy', Joy, self.joyCb)
        self.start_pub = rospy.Publisher('dragon/teleop_command/start', Empty, queue_size = 1)
        self.takeoff_pub = rospy.Publisher('dragon/teleop_command/takeoff', Empty, queue_size = 1)
        self.land_pub = rospy.Publisher('dragon/teleop_command/land', Empty, queue_size = 1)
        self.force_landing_pub = rospy.Publisher('dragon/teleop_command/force_landing', Empty, queue_size = 1)
        self.rot_pub = rospy.Publisher('dragon/final_target_baselink_rpy', Vector3Stamped, queue_size = 1)


        rospy.sleep(1)

    def joyCb(self, msg):

        self.joy_flag = True
        self.joy_sub.unregister()
        
    def flightStateCb(self, msg):

        self.flight_state = msg.data

    def main(self):

        if not self.joy_flag:
            rospy.logerr("Cannot receive joy message")
            return
        
        # motor arm
        self.start_pub.publish(Empty())

        t = rospy.get_time()
        while not rospy.is_shutdown():

            if self.flight_state == self.ARM_ON_STATE:
                rospy.loginfo("motor is armed")
                break;

            if rospy.get_time() - t > 2.0:
                rospy.logerr("cannot arm motor")
                return
            
            rospy.sleep(1.0)


        # takeoff
        self.takeoff_pub.publish(Empty())

        while not rospy.is_shutdown():

            if self.flight_state == self.HOVER_STATE:
                rospy.loginfo("reach hover state")
                break;

            rospy.loginfo("wait for hovering")
            rospy.sleep(1.0)

        rospy.sleep(3.0)

        # vertical pose
        rospy.loginfo("vertical pose")
        msg = Vector3Stamped()
        msg.vector.x = -1.57
        msg.vector.y = -0.78
        self.rot_pub.publish(msg)

if __name__=="__main__":
    rospy.init_node("vertical_demo")

    demo = Demo()
    demo.main()
