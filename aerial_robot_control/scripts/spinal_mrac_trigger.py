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

        # disable MRAC in the beginning 
        self.spinalMRACTrigger(False)
        self.takeoff_mrac_wait_time_ = rospy.get_param("~takeoff_wait_time", 20)
        self.takeoff_time_ = rospy.get_time()
        rospy.Subscriber("flight_state", UInt8, self.flightStateCallback, queue_size=10)

    def flightStateCallback(self, msg):
        self.prev_flight_state_ = self.flight_state_
        self.flight_state_ = msg.data

        if self.prev_flight_state_ == ARM_ON_STATE and self.flight_state_ == TAKEOFF_STATE:
            self.takeoff_time_ = rospy.get_time()
            self.spinalMRACTrigger(False)
        elif self.prev_flight_state_ == TAKEOFF_STATE and self.flight_state_ == TAKEOFF_STATE:
            if not self.is_using_mrac_ and rospy.get_time() - self.takeoff_time_ > self.takeoff_mrac_wait_time_:
                self.spinalMRACTrigger(True)
        elif self.prev_flight_state_ == TAKEOFF_STATE and self.flight_state_ == HOVER_STATE:
            # enable state change when start hovering
            if not self.is_using_mrac_:
                self.spinalMRACTrigger(True)
        elif self.flight_state_ == LAND_STATE:
            pass
        elif self.prev_flight_state_ != self.flight_state_:
            # disable MRAC on state change
            self.spinalMRACTrigger(False)

    def spinalMRACTrigger(self, trigger):
        try:
            resp = self.spinal_mrac_trigger_(trigger)
            self.is_using_mrac_ = trigger 
            if trigger:
                rospy.loginfo("MRAC: Triggering %r to spinal", trigger)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    rospy.init_node('spinal_mrac_trigger', anonymous=True)
    enable_MRAC = rospy.get_param("~enable_MRAC")

    try:
        if enable_MRAC:
            spinal_mrac_trigger= SpinalMRACTrigger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
