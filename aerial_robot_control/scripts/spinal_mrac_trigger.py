#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool
from spinal.srv import SetMRACParams

ARM_OFF_STATE = 0
START_STATE = 1
ARM_ON_STATE = 2
TAKEOFF_STATE = 3
LAND_STATE = 4
HOVER_STATE = 5
STOP_STATE = 6

class SpinalMRACTrigger:
    def __init__(self):
        rospy.init_node('spinal_mrac_trigger', anonymous=True)

        self.flight_state_ = ARM_OFF_STATE
        self.prev_flight_state_ = ARM_OFF_STATE

        # initialize MRAC Parameters on spinal
        rospy.wait_for_service("set_mrac_params")
        spinal_mrac_trigger_name = "spinal_mrac_trigger"
        rospy.wait_for_service(spinal_mrac_trigger_name)
        try:
            self.spinal_mrac_trigger_ = rospy.ServiceProxy(spinal_mrac_trigger_name, SetBool)
            set_mrac_params = rospy.ServiceProxy("set_mrac_params", SetMRACParams)
            log_rate = rospy.get_param("~log_rate")
            mrac_ratio = rospy.get_param("~mrac_ratio")
            k1m = rospy.get_param("~k1m")
            k2m = rospy.get_param("~k2m")
            K1 = rospy.get_param("~K1")
            K2 = rospy.get_param("~K2")
            kappa = rospy.get_param("~kappa")
            sigma = rospy.get_param("~sigma")
            attitude_p = rospy.get_param("~attitude_p")
            resp = set_mrac_params(log_rate, mrac_ratio, k1m, k2m, K1, K2, kappa, sigma, attitude_p)
            if not resp.success:
                rospy.logerr("spinal_mrac_trigger SetMRACParams service call failed")
                sys.exit()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.Subscriber("flight_state", UInt8, self.flightStateCallback, queue_size=10)

    def flightStateCallback(self, msg):
        self.prev_flight_state_ = self.flight_state_
        self.flight_state_ = msg.data

        if self.prev_flight_state_ == TAKEOFF_STATE and self.flight_state_ == HOVER_STATE:
            # enable state change when start hovering
            try:
                resp = self.spinal_mrac_trigger_(True)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        elif self.prev_flight_state_ != self.flight_state_:
            # disable MRAC on state change
            try:
                resp = self.spinal_mrac_trigger_(False)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

if __name__ == '__main__':
    try:
        spinal_mrac_trigger= SpinalMRACTrigger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

