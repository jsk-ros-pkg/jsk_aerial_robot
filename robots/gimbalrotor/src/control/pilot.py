#!/usr/bin/env python3

from __future__ import print_function # for print function in python2

import rospy
from std_msgs.msg import Float64, Bool

import sys, select, termios, tty
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph

msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)
o:  open gripper hand
+:  slowly open gripper
-:  slowly close gripper
c:  close gripper hand
m:  specify goal

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)


Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('essam_quad_pilot', anonymous=True)
    robot_ns = rospy.get_param("~robot_ns", "");
    # rate = rospy.Rate(10)  # 10 Hz
    right_position = 0  # Example position for right gripper
    left_position = 0   # Example position for left gripper
    print(msg)
    
    if not robot_ns:
                master = rosgraph.Master('/rostopic')
                try:
                        _, subs, _ = master.getSystemState()

                except socket.error:
                        raise ROSTopicIOException("Unable to communicate with master!")

                teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
                if len(teleop_topics) == 1:
                        robot_ns = teleop_topics[0].split('/teleop')[0]
    
    ns = robot_ns + "/teleop_command"
    land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
    halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
    start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
    takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
    force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
    nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=1)
    right_pub = rospy.Publisher('quadrotor/right_hand_position_controller/command', Float64, queue_size=10)
    left_pub = rospy.Publisher('quadrotor/left_hand_position_controller/command', Float64, queue_size=10)
    gripper = rospy.Publisher('quadrotor/gripper', Bool, queue_size=10)
    
    xy_vel   = rospy.get_param("xy_vel", 0.2)
    z_vel    = rospy.get_param("z_vel", 0.2)
    yaw_vel  = rospy.get_param("yaw_vel", 0.2)

    motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)
    try:
        while(True):
                nav_msg = FlightNav()
                nav_msg.control_frame = FlightNav.WORLD_FRAME
                nav_msg.target = FlightNav.COG
                key = getKey()

                msg = ""
                if key == 'l':
                        land_pub.publish(Empty())
                        msg = "send land command"
                if key == 'r':
                        start_pub.publish(Empty())
                        msg = "send motor-arming command"
                if key == 'h':
                        halt_pub.publish(Empty())
                        msg = "send motor-disarming (halt) command"
                if key == 'f':
                        force_landing_pub.publish(Empty())
                        msg = "send force landing command"
                if key == 't':
                        takeoff_pub.publish(Empty())
                        msg = "send takeoff command"
                if key == 'x':
                        motion_start_pub.publish()
                        msg = "send task-start command"
                if key == 'w':
                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_vel_x = xy_vel
                        nav_pub.publish(nav_msg)
                        msg = "send +x vel command"
                if key == 's':
                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_vel_x = -xy_vel
                        nav_pub.publish(nav_msg)
                        msg = "send -x vel command"
                if key == 'a':
                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_vel_y = xy_vel
                        nav_pub.publish(nav_msg)
                        msg = "send +y vel command"
                if key == 'd':
                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_vel_y = -xy_vel
                        nav_pub.publish(nav_msg)
                        msg = "send -y vel command"
                if key == 'q':
                        nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_omega_z = yaw_vel
                        nav_pub.publish(nav_msg)
                        msg = "send +yaw vel command"
                if key == 'e':
                        nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_omega_z = -yaw_vel
                        msg = "send -yaw vel command"
                        nav_pub.publish(nav_msg)
                if key == '[':
                        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_vel_z = z_vel
                        nav_pub.publish(nav_msg)
                        msg = "send +z vel command"
                if key == ']':
                        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                        nav_msg.target_vel_z = -z_vel
                        nav_pub.publish(nav_msg)
                        msg = "send -z vel command"
                if key == 'o':
                        gripper.publish(True)
                        right_pub.publish(Float64(0.05))
                        left_pub.publish(Float64(0.05))
                        right_position = .05
                        left_position = .05
                        msg = "send open gripper hand command"
                if key == 'c':
                        gripper.publish(False)
                        right_pub.publish(Float64(0.0))
                        left_pub.publish(Float64(0.0))
                        right_position = 0
                        left_position = 0
                        msg = "send close gripper hand command"
                if key == 'm':
                        msg = "pose/velocity?P/v"
                        printMsg(msg)
                        cmd = input()
                        if cmd == 'p' or input == 'P' or input == '\n':
                                msg = "x, y and z?x,y,z"
                                printMsg(msg)
                                targ = input()
                                tx,ty,tz = targ.split(',')
                                nav_msg.pos_xy_nav_mode = 2
                                nav_msg.pos_z_nav_mode = 2
                                nav_msg.target_pos_x = float(tx)
                                nav_msg.target_pos_y = float(ty)
                                nav_msg.target_pos_z = float(tz)
                                nav_pub.publish(nav_msg)
                                msg = "send traget x command"
                        elif cmd == 'v' or cmd == 'V':
                                msg = "not implemented yet"
                if key == '+':
                        right_position += 0.001
                        left_position += 0.001
                        right_pub.publish(Float64(right_position))
                        left_pub.publish(Float64(left_position))
                        msg = "send slowly open gripper hand command"
                if key == '-':
                        right_position -= 0.001
                        left_position -= 0.001
                        right_pub.publish(Float64(right_position))
                        left_pub.publish(Float64(left_position))
                        msg = "send slowly close gripper hand command" + str(right_position)
                if key == '\x03':
                        break
                
                printMsg(msg)
                rospy.sleep(0.001)
    except Exception as e:
            print(repr(e))
    finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)