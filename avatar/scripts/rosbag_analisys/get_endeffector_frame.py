#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage

class getEndeffectorFrame():

  def __init__(self):
    self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_cb)
    self.world_root_translation = np.zeros(3)
    self.world_root_rotation = np.zeros(4)

  def get_translation(self,m):
    trans_x = m.transform.translation.x
    trans_y = m.transform.translation.y
    trans_z = m.transform.translation.z
    return [trans_x, trans_y, trans_z]
  
  def get_rotation(self,m):
    rot_x = m.transform.rotation.x
    rot_y = m.transform.rotation.y
    rot_z = m.transform.rotation.z
    rot_w = m.transform.rotation.w
    return [rot_x, rot_y, rot_z, rot_w]
  
  def tf_cb(self,msg):
    for transform in msg.transforms:
      frame_id = transform.header.frame_id
      child_frame_id = transform.child_frame_id
      print("parent is ", frame_id, ", child is", child_frame_id)
      if frame_id == 'world' and child_frame_id == 'root':
        self.world_root_translation = self.get_translation(transform)
        self.world_root_rotation = self.get_rotation(transform)
    print("------------------------")

  def main(self):

    r = rospy.Rate(200)
    while not rospy.is_shutdown():
      r.sleep()


if __name__ == "__main__":
  rospy.init_node("get_endeffector_frame")
  Tracker = getEndeffectorFrame()
  Tracker.main()
