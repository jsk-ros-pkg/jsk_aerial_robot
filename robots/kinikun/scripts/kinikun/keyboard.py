#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty, Float32
from aerial_robot_msgs.msg import FlightNav
import rosgraph
from geometry_msgs.msg import Quaternion

# --- グローバル変数 ---
v1_mpa = 0.2
v2_mpa = 0.2
v3_mpa = 0.2
v4_mpa = 0.2
step = 0.02   # 増減ステップ（MPa単位）
v1_min, v1_max = 0, 0.7 
v2_min, v2_max = 0, 0.7
v3_min, v3_max = 0, 0.7
v4_min, v4_max = 0, 0.7
step_target_deg = 0.5  # 目標角度増減ステップ（deg単位）

def mpa_to_dac(p_mpa):
    """
    MPa -> DACカウント(0〜4095)
    0〜0.9MPa を 0〜4095 にマップ
    """
    if p_mpa < 0.0:
        p_mpa = 0.0
    if p_mpa > 0.9:
        p_mpa = 0.9
    return int(p_mpa * 4096.0 / 0.9)

def publish_mpa_cmd(pub):
    q = Quaternion()
    q.x = mpa_to_dac(v1_mpa)
    q.y = mpa_to_dac(v2_mpa)
    q.z = mpa_to_dac(v3_mpa)
    q.w = mpa_to_dac(v4_mpa)
    pub.publish(q)

msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)

        v               b               n                  m
(air1 increase)   (air1 decrease) (air2 increase)   (air2 decrease)
        j               k               u                  i
(air3 increase)   (air3 decrease) (air4 increase)   (air4 decrease)
        1               2               3                  4
(target1 +step)   (target1 -step) (target2 +step)   (target2 -step)
        c
(reset all MPA to 0.20 MPa)

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

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        robot_ns = rospy.get_param("~robot_ns", "");
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

        xy_vel   = rospy.get_param("xy_vel", 0.2)
        z_vel    = rospy.get_param("z_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.2)

        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)
        mpa_pub = rospy.Publisher('/mpa_cmd', Quaternion, queue_size=1)
        target_deg_pub = rospy.Publisher('/mppi/theta_target_deg', Float32, queue_size=1)
        target_deg2_pub= rospy.Publisher('/mppi/theta_target_deg_2', Float32, queue_size=1)
        target1_deg = 0.0
        target2_deg = 0.0
        limit_deg = 35.0

        rospy.sleep(0.1)
        publish_mpa_cmd(mpa_pub)

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
                        # ---- MPA1 (Quaternion.x) ----
                        if key == 'v':
                                v1_mpa = min(v1_max, v1_mpa + step)
                                publish_mpa_cmd(mpa_pub)
                                v1 = mpa_to_dac(v1_mpa)
                                msg = f"MPA1 up   {v1_mpa:.2f} MPa (x = {v1})"
                        if key == 'b':
                                v1_mpa = max(v1_min, v1_mpa - step)
                                publish_mpa_cmd(mpa_pub)
                                v1 = mpa_to_dac(v1_mpa)
                                msg = f"MPA1 down {v1_mpa:.2f} MPa (x = {v1})"

                        # ---- MPA2 (Quaternion.y) ----
                        if key == 'n':
                                v2_mpa = min(v2_max, v2_mpa + step)
                                publish_mpa_cmd(mpa_pub)
                                v2 = mpa_to_dac(v2_mpa)
                                msg = f"MPA2 up   {v2_mpa:.2f} MPa (y = {v2})"
                        if key == 'm':
                                v2_mpa = max(v2_min, v2_mpa - step)
                                publish_mpa_cmd(mpa_pub)
                                v2 = mpa_to_dac(v2_mpa)
                                msg = f"MPA2 down {v2_mpa:.2f} MPa (y = {v2})"
                        # ---- MPA3 (Quaternion.z) ----
                        if key == 'j':
                                v3_mpa = min(v3_max, v3_mpa + step)
                                publish_mpa_cmd(mpa_pub)
                                v3 = mpa_to_dac(v3_mpa)
                                msg = f"MPA3 up   {v3_mpa:.2f} MPa (z = {v3})"
                        if key == 'k':
                                v3_mpa = max(v3_min, v3_mpa - step)
                                publish_mpa_cmd(mpa_pub)
                                v3 = mpa_to_dac(v3_mpa)
                                msg = f"MPA3 down {v3_mpa:.2f} MPa (z = {v3})"

                        # ---- MPA4 (Quaternion.w) ----
                        if key == 'u':
                                v4_mpa = min(v4_max, v4_mpa + step)
                                publish_mpa_cmd(mpa_pub)
                                v4 = mpa_to_dac(v4_mpa)
                                msg = f"MPA4 up   {v4_mpa:.2f} MPa (w = {v4})"
                        if key == 'i':
                                v4_mpa = max(v4_min, v4_mpa - step)
                                publish_mpa_cmd(mpa_pub)
                                v4 = mpa_to_dac(v4_mpa)
                                msg = f"MPA4 down {v4_mpa:.2f} MPa (w = {v4})"
                        
                        # ---- ターゲット角度送信（ステップ増減）----
                        if key == '1':
                                # System1 目標：+step
                                target1_deg = max(-limit_deg, min(limit_deg, target1_deg))
                                target1_deg += step_target_deg
                                target_deg_pub.publish(Float32(target1_deg))
                                msg = f"theta_target_deg += {step_target_deg:.2f} -> {target1_deg:.2f} deg"

                        if key == '2':
                                # System1 目標：-step
                                target1_deg = max(-limit_deg, min(limit_deg, target1_deg))
                                target1_deg -= step_target_deg
                                target_deg_pub.publish(Float32(target1_deg))
                                msg = f"theta_target_deg -= {step_target_deg:.2f} -> {target1_deg:.2f} deg"

                        if key == '3':
                                # System2 目標：+step
                                target2_deg = max(-limit_deg, min(limit_deg, target2_deg))
                                target2_deg += step_target_deg
                                target_deg2_pub.publish(Float32(target2_deg))
                                msg = f"theta_target_deg_2 += {step_target_deg:.2f} -> {target2_deg:.2f} deg"

                        if key == '4':
                                # System2 目標：-step
                                target2_deg = max(-limit_deg, min(limit_deg, target2_deg))
                                target2_deg -= step_target_deg
                                target_deg2_pub.publish(Float32(target2_deg))
                                msg = f"theta_target_deg_2 -= {step_target_deg:.2f} -> {target2_deg:.2f} deg"

                        # ---- リセット ----
                        if key == 'c':
                                v1_mpa = 0.2
                                v2_mpa = 0.2
                                v3_mpa = 0.2
                                v4_mpa = 0.2
                                publish_mpa_cmd(mpa_pub)
                                msg = "reset all MPA to 0.20 MPa"

                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

