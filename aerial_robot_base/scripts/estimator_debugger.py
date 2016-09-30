#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from aerial_robot_msgs.srv import BoolFlag

import sys, select, termios, tty

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)

	estimate_mode_pub = rospy.Publisher('/estimate_height_mode', UInt8)

	rospy.init_node('keyboard_command')

	estimate_mode = UInt8()

	try:
		while(True):
			key = getKey()
			print "the key value is %d" % ord(key)
			if key == 'b':
                                print "set the barometer estimate to TRUE"
                                rospy.wait_for_service('/estimator/barometer/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/barometer/estimate_flag', BoolFlag)
                                        resp = bool_flag(True)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'B':
                                print "set the barometer estimate to FALSE"
                                rospy.wait_for_service('/estimator/barometer/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/barometer/estimate_flag', BoolFlag)
                                        resp = bool_flag(False)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'h':
                                print "set the range_sensor estimate to TRUE"
                                rospy.wait_for_service('/estimator/range_sensor/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/range_sensor/estimate_flag', BoolFlag)
                                        resp = bool_flag(True)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'H':
                                print "set the range_sensor estimate to FALSE"
                                rospy.wait_for_service('/estimator/range_sensor/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/range_sensor/estimate_flag', BoolFlag)
                                        resp = bool_flag(False)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'o':
                                print "set the optical_flow estimate to TRUE"
                                rospy.wait_for_service('/estimator/optical_flow/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/optical_flow/estimate_flag', BoolFlag)
                                        resp = bool_flag(True)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'O':
                                print "set the optical_flow estimate to FALSE"
                                rospy.wait_for_service('/estimator/optical_flow/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/optical_flow/estimate_flag', BoolFlag)
                                        resp = bool_flag(False)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'm':
                                print "set the mocap estimate to TRUE"
                                rospy.wait_for_service('/estimator/mocap/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/mocap/estimate_flag', BoolFlag)
                                        resp = bool_flag(True)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'M':
                                print "set the mocap estimate to FALSE"
                                rospy.wait_for_service('/estimator/mocap/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/mocap/estimate_flag', BoolFlag)
                                        resp = bool_flag(False)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'i':
                                print "set the imu estimate to TRUE"
                                rospy.wait_for_service('/estimator/imu/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/imu/estimate_flag', BoolFlag)
                                        resp = bool_flag(True)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'I':
                                print "set the imu estimate to FALSE"
                                rospy.wait_for_service('/estimator/imu/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/imu/estimate_flag', BoolFlag)
                                        resp = bool_flag(False)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'g':
                                print "set the rtk_gps estimate to TRUE"
                                rospy.wait_for_service('/estimator/rtk_gps/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/rtk_gps/estimate_flag', BoolFlag)
                                        resp = bool_flag(True)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == 'G':
                                print "set the rtk_gps estimate to FALSE"
                                rospy.wait_for_service('/estimator/rtk_gps/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/rtk_gps/estimate_flag', BoolFlag)
                                        resp = bool_flag(False)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == '0':
                                print "only baro mode"
                                estimate_mode.data = 0
                                estimate_mode_pub.publish(estimate_mode)
			if key == '1':
                                print "with baro mode"
                                estimate_mode.data = 1
                                estimate_mode_pub.publish(estimate_mode)
			if key == '2':
                                print "without baro mode"
                                estimate_mode.data = 2
                                estimate_mode_pub.publish(estimate_mode)
			if key == '3':
                                print "only baro mode"
                                estimate_mode.data = 0
                                estimate_mode_pub.publish(estimate_mode)

                                print "set the range_sensor estimate to FALSE"
                                rospy.wait_for_service('/estimator/range_sensor/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/range_sensor/estimate_flag', BoolFlag)
                                        resp = bool_flag(False)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
                                print "set the mocap estimate to FALSE"
                                rospy.wait_for_service('/estimator/mocap/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/mocap/estimate_flag', BoolFlag)
                                        resp = bool_flag(False)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
			if key == '4':
                                print "without baro mode"
                                estimate_mode.data = 2
                                estimate_mode_pub.publish(estimate_mode)

                                print "set the range_sensor estimate to TRUE"
                                rospy.wait_for_service('/estimator/range_sensor/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/range_sensor/estimate_flag', BoolFlag)
                                        resp = bool_flag(True)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
                                print "set the mocap estimate to TRUE"
                                rospy.wait_for_service('/estimator/mocap/estimate_flag')
                                try:
                                        bool_flag = rospy.ServiceProxy('/estimator/mocap/estimate_flag', BoolFlag)
                                        resp = bool_flag(True)
                                except rospy.ServiceException, e:
                                        print "Service call failed: %s" %e
                        if ord(key) == 27:
                                key = getKey()
                                print "the key value is %d" % ord(key)
                                key = getKey()
                                print "the key value is %d" % ord(key)

                                if ord(key) == 68:
                                        print "68"
                                elif ord(key) == 67:
                                        print "67"
                                elif ord(key) == 65:
                                        print "65"
                                elif ord(key) == 66:
                                        print "66"
                                else:
                                        print "the key value is %d" % ord(key)
                                        print "no valid command"
			else:
				if (key == '\x03'):
					break
			rospy.sleep(0.001)

	except Exception as e:
		print e
		print repr(e)

	finally:
                
    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


