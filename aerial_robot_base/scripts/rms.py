#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from aerial_robot_msgs.msg import PoseControlPid

import sys, select, termios, tty, math

msg = """

s: start to subscribe the topic regarding to control errors, and start to calculate the RMS
h: stop the calculate the output the RMS results.
"""

def cb(data):
        global start_flag
        if start_flag == True:
                global pose_cnt
                global pose_squared_errors_sum
                pose_cnt = pose_cnt + 1

                pose_squared_errors_sum[0] = pose_squared_errors_sum[0] + data.x.err_p * data.x.err_p
                pose_squared_errors_sum[1] = pose_squared_errors_sum[1] + data.y.err_p * data.y.err_p
                pose_squared_errors_sum[2] = pose_squared_errors_sum[2] + data.z.err_p * data.z.err_p

                pose_squared_errors_sum[3] = pose_squared_errors_sum[3] + data.roll.err_p * data.roll.err_p
                pose_squared_errors_sum[4] = pose_squared_errors_sum[4] + data.pitch.err_p * data.pitch.err_p
                pose_squared_errors_sum[5] = pose_squared_errors_sum[5] + data.yaw.err_p * data.yaw.err_p

                rms = [math.sqrt(i / pose_cnt) for i in pose_squared_errors_sum]
                #rospy.loginfo("RMS errors of pos: [%f, %f, %f]; rot: [%f, %f, %f]", rms[0], rms[1], rms[2], rms[3], rms[4], rms[5])

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)

        start_flag = False
        msg_cnt = 0

        pose_squared_errors_sum = [0] * 6
        pose_cnt = 0

        rospy.Subscriber("debug/pose/pid", PoseControlPid, cb)

        rospy.init_node('rms_error')

        print(msg)
        try:
                while(True):
                        key = getKey()

                        if key == 's':
                                rospy.loginfo("start to calculate RMS errors")
                                start_flag = True
                        if key == 'h':
                                rospy.loginfo("stop calculation")

                                rms = [0] * 6
                                if pose_cnt > 0:
                                        rms = [math.sqrt(i / pose_cnt) for i in pose_squared_errors_sum]

                                rospy.loginfo("RMS of pos errors: [%f, %f, %f], att errors: [%f, %f, %f]", rms[0], rms[1], rms[2], rms[3], rms[4], rms[5])

                                start_flag = False

                                pose_cnt = 0
                                pose_squared_errors_sum = [0] * 6

                        else:
                                if (key == '\x03'):
                                        break
                        rospy.sleep(0.001)

        except Exception as e:
                print(e)
                print(repr(e))

        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


