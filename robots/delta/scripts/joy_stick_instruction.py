#!/usr/bin/env python

# $ rosrun rolling transformation_demo.py _mode:=0

import rospy

rospy.init_node("delta_joy_stick_instruction")

msg = """\

Standing mode: L1 + up
Rolling  mode: L1 + down
Flying   mode: R1 + left
Down     mode: R1 + right

Rolling pitch: L1 + Lstick vertical
Rolling yaw  : L1 + Rstick holizontal

"""

print(msg)
