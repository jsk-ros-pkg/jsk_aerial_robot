#!/usr/bin/env python
from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty, Int16
from aerial_robot_msgs.msg import FlightNav
import rosgraph

init_msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

u: standing mode
j: rolling mode
k: down mode

p: Locomotion mode
;: Manipulation mode

Flying mode: cog, Manipulation mode: end effector
w: +x
s: -x
[: +z
]: -z

Please don't have caps lock on.
CTRL+c to quit\n
---------------------------
"""

class keyboardCommand():
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.robot_ns = rospy.get_param("~robot_ns", "");
        print(init_msg)

        if not self.robot_ns:
            master = rosgraph.Master('/rostopic')
            try:
                _, subs, _ = master.getSystemState()
            except socket.error:
                raise ROSTopicIOException("Unable to communicate with master!")

            teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
            if len(teleop_topics) == 1:
                robot_ns = teleop_topics[0].split('/teleop')[0]

        self.ground_navigation_mode = 0
        self.ground_motion_mode = 0

        ns = robot_ns + "/teleop_command"
        self.land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        self.start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        self.force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        self.nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=1)

        self.ground_navigation_command_pub = rospy.Publisher(robot_ns + "/ground_navigation_command", Int16, queue_size=1)
        self.ground_motion_command_pub = rospy.Publisher(robot_ns + "/ground_motion_command", Int16, queue_size=1)

        ground_navigation_ack_sub = rospy.Subscriber(robot_ns + "/ground_navigation_ack", Int16, self.groundNavigationAckCallback)
        ground_motion_ack_sub = rospy.Subscriber(robot_ns + "/ground_motion_ack", Int16, self.groundMotionAckCallback)

        self.xy_vel   = rospy.get_param("xy_vel", 0.2)
        self.z_vel    = rospy.get_param("z_vel", 0.2)
        self.yaw_vel  = rospy.get_param("yaw_vel", 0.2)

        try:
            while(True):
                nav_msg = FlightNav()
                nav_msg.control_frame = FlightNav.WORLD_FRAME
                nav_msg.target = FlightNav.COG

                key = self.getKey()
                msg = ""

                if key == 'l':
                    self.land_pub.publish(Empty())
                    msg = "send land command"
                if key == 'r':
                    self.start_pub.publish(Empty())
                    msg = "send motor-arming command"
                if key == 'h':
                    self.halt_pub.publish(Empty())
                    msg = "send motor-disarming (halt) command"
                if key == 'f':
                    self.force_landing_pub.publish(Empty())
                    msg = "send force landing command"
                if key == 't':
                    self.takeoff_pub.publish(Empty())
                    msg = "send takeoff command"

                if key == "u":
                    pub_msg = Int16()
                    pub_msg.data = 2
                    self.ground_navigation_command_pub.publish(pub_msg)
                    msg = "send to switch STANDING STATE"
                if key == "j":
                    pub_msg = Int16()
                    pub_msg.data = 3
                    self.ground_navigation_command_pub.publish(pub_msg)
                    msg = "send to switch ROLLING STATE"
                if key == "k":
                    pub_msg = Int16()
                    pub_msg.data = 4
                    self.ground_navigation_command_pub.publish(pub_msg)
                    msg = "send to switch DOWN STATE"

                if key == 'p':
                    pub_msg = Int16()
                    pub_msg.data = 0
                    self.ground_motion_command_pub.publish(pub_msg)
                    msg = "send to switch LOCOMOTION MODE"

                if key == ';':
                    pub_msg = Int16()
                    pub_msg.data = 1
                    self.ground_motion_command_pub.publish(pub_msg)
                    msg = "send to switch MANIPULATION MODE"

                if key == 'w':
                    nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_x = self.xy_vel
                    self.nav_pub.publish(nav_msg)
                    msg = "send +x vel command"
                if key == 's':
                    nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_x = -self.xy_vel
                    self.nav_pub.publish(nav_msg)
                    msg = "send -x vel command"

                if key == '[':
                    nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_z = self.z_vel
                    self.nav_pub.publish(nav_msg)
                    msg = "send +z vel command"
                if key == ']':
                    nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                    nav_msg.target_vel_z = -self.z_vel
                    self.nav_pub.publish(nav_msg)
                    msg = "send -z vel command"

                if key == '\x03':
                    break

                msg = msg + "\nground navigation mode: " + str(self.ground_navigation_mode)
                msg = msg + "\nground motion mode: " + str(self.ground_motion_mode)
                self.printMsg(msg)
                rospy.sleep(0.001)

        except Exception as e:
            print(repr(e))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def printMsg(self, msg, msg_len = 50):
        n = 2
        print(f"\033[{n}F" + (" " * 50 + "\n") * n + f"\033[{n}F" + msg.ljust(msg_len), end="")


    def groundNavigationAckCallback(self, msg):
        self.ground_navigation_mode = msg.data

    def groundMotionAckCallback(self, msg):
        self.ground_motion_mode = msg.data

if __name__=="__main__":
    rospy.init_node("keyboard_command")
    node = keyboardCommand()
    rospy.spin()
