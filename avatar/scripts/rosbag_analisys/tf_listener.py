#!/usr/bin/env python  
import rospy
import math
import numpy as np
import tf
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_from_euler
from aerial_robot_msgs.msg import PoseControlPid


class tfListener():
  
  def __init__(self):
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)
    self.debug_errp_sub = rospy.Subscriber('/hydrus_xi/debug/pose/pid', PoseControlPid, self.debug_cb)

  def transform_position(position, translation, rotation_quaternion):
    rotation_matrix = quaternion_matrix(rotation_quaternion)[:3, :3]
    transformed_position = np.dot(rotation_matrix, position) + translation
    return transformed_position

  def main(self):
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
      try:
        transform = self.tfBuffer.lookup_transform('hydrus_xi/cog', 'hydrus_xi/link1_end', rospy.Time(0))
        trans_x = transform.transform.translation.x
        trans_y = transform.transform.translation.y
        trans_z = transform.transform.translation.z
        trans = np.array([trans_x, trans_y, trans_z])
        rot_x = transform.transform.rotation.x
        rot_y = transform.transform.rotation.y
        rot_z = transform.transform.rotation.z
        rot_w = transform.transform.rotation.w
        rot_q = (rot_x, rot_y, rot_z, rot_w)

        self.transform_position(pos, trans, rot_q)

      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue
    rate.sleep()

if __name__ == '__main__':
  rospy.init_node('tf_listener')
  Tracker = tfListener()
  Tracker.main()