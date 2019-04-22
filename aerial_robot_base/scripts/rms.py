#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from aerial_robot_msgs.msg import FlatnessPid, States

import sys, select, termios, tty, math

msg = """

s: start to subscribe the topic regarding to control errors, and start to calculate the RMS
h: stop the calculate the output the RMS results.
"""

def pos_err_callback(data):
        global start_flag
        if start_flag == True:
                global four_axis_cnt
                global four_axis_squared_errors_sum
                four_axis_cnt = four_axis_cnt + 1

                four_axis_squared_errors_sum[0] = four_axis_squared_errors_sum[0] + data.pitch.pos_err * data.pitch.pos_err
                four_axis_squared_errors_sum[1] = four_axis_squared_errors_sum[1] + data.roll.pos_err * data.roll.pos_err
                four_axis_squared_errors_sum[2] = four_axis_squared_errors_sum[2] + data.throttle.pos_err * data.throttle.pos_err
                four_axis_squared_errors_sum[3] = four_axis_squared_errors_sum[3] + data.yaw.pos_err * data.yaw.pos_err

                temp_rms = [math.sqrt(i / four_axis_cnt) for i in four_axis_squared_errors_sum]
                rospy.logdebug("RMS errors of position: [%f, %f, %f], RMS error of yaw: %f", temp_rms[0], temp_rms[1], temp_rms[2], temp_rms[3])

def att_err_callback(data):
        global start_flag
        if start_flag == True:
                global att_cnt
                global roll_pitch_squared_errors_sum

                att_cnt = att_cnt + 1

                roll_pitch_squared_errors_sum = [prev_sum + data.states[6 + i].state[2].x**2 for i, prev_sum in enumerate(roll_pitch_squared_errors_sum)]
                #att_pitch_sum = att_pitch_sum + data.states[7].state[2].x * data.states[7].state[2].x

                temp_rms = [math.sqrt(i / att_cnt) for i in roll_pitch_squared_errors_sum ]
                #pitch_rms = math.sqrt(att_pitch_sum / att_cnt)
                rospy.logdebug("RMS errors of roll and pitch: [%f, %f] \n", temp_rms[0], temp_rms[1])

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

        four_axis_squared_errors_sum = [0] * 4
        roll_pitch_squared_errors_sum = [0] * 2

        att_cnt = 0
        four_axis_cnt = 0

        rospy.Subscriber("/controller/debug", FlatnessPid, pos_err_callback)
        rospy.Subscriber("/uav/full_state", States, att_err_callback)

        rospy.init_node('rms_error')

        print msg
        try:
                while(True):
                        key = getKey()

                        if key == 's':
                                rospy.loginfo("start to calculate RMS errors")
                                start_flag = True
                        if key == 'h':
                                rospy.loginfo("stop calculation")

                                temp_rms = [0] * 2
                                temp2_rms = [0] * 4
                                if att_cnt > 0:
                                        temp_rms = [math.sqrt(i / att_cnt) for i in roll_pitch_squared_errors_sum ]
                                if four_axis_cnt > 0:
                                        temp2_rms = [math.sqrt(i / four_axis_cnt) for i in four_axis_squared_errors_sum]


                                rospy.loginfo("RMS of att errors: [%f, %f, %f], pos errors: [%f, %f, %f]", temp_rms[0], temp_rms[1], temp2_rms[3], temp2_rms[0], temp2_rms[1], temp2_rms[2])

                                start_flag = False
                                att_cnt = 0
                                four_axis_cnt = 0

                                four_axis_squared_errors_sum = [0] * 4
                                roll_pitch_squared_errors_sum = [0] * 2

                        else:
                                if (key == '\x03'):
                                        break
                        rospy.sleep(0.001)

        except Exception as e:
                print e
                print repr(e)

        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


