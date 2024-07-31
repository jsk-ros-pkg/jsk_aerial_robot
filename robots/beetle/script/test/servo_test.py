#!/usr/bin/env python

import rospy
import numpy as np
import math
import time
from modules.kondo_control import KondoControl

class GimbalTest():
    def __init__(self):

        self.kondo = KondoControl('beetle2',2, 1, False)

        self.test_angle_max = 1.0
        
        self.nav_rate = rospy.Rate(40)

        self.t = 0
        
        time.sleep(0.5)

    def main(self):
        while not rospy.is_shutdown():
            self.t = self.t + 0.04
            target_angle = self.test_angle_max * math.sin(self.t)
            self.kondo.sendTargetAngle(target_angle)
            self.nav_rate.sleep()

if __name__ == "__main__":

  rospy.init_node("gimbale_test")

  gimbal_test = GimbalTest()
  gimbal_test.main()



