#!/usr/bin/env python

# $ rosrun rolling transformation_demo.py _mode:=0

import rospy

rospy.init_node("delta_joy_stick_instruction")

msg = """\

Switch to Locomotion mode: L2
  Standing mode: L1 + up
  Rolling  mode: L1 + down
  Flying   mode: L1 + left
  Down     mode: L1 + right
  Rolling pitch: L1 + Lstick vertical
  Rolling yaw  : L1 + Rstick holizontal

Switch to Manipulation mode: R2
  end effector x: R1 + Lstick vertical
  end effector z: R1 + Rstick vertical
"""

print(msg)
