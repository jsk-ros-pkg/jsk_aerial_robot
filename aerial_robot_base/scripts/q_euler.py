#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import tf

class Q2Eulter(object):
  def __init__(self):
      rospy.init_node("q_to_euler")
      rospy.loginfo("quaternion to euler node")
      self.q_sub = rospy.Subscriber("/dji_sdk/odometry", Odometry, self.callback)
      self.euler_pub = rospy.Publisher('euler', Vector3, queue_size=10)

  def callback(self, data):


      quaternion = (
          data.pose.pose.orientation.x,
          data.pose.pose.orientation.y,
          data.pose.pose.orientation.z,
          data.pose.pose.orientation.w)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      pub_msg = Vector3()
      pub_msg.x = euler[0]
      pub_msg.y = euler[1]
      pub_msg.z = euler[2]
      self.euler_pub.publish(pub_msg)

      rospy.loginfo("x:%f, y:%f, z:%f, roll:%f, pitch:%f, yaw:%f", data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, euler[0], euler[1],euler[2])

  def spin(self):
      rospy.spin()

if __name__ == "__main__":
    Q2Eulter().spin()


