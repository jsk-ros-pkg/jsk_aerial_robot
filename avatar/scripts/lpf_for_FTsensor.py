#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header
from std_srvs.srv import Empty

class lpf_for_FTsensor():

  def __init__(self):

    self.robot_name = rospy.get_param("~robot_ns", "hydrus_xi")
    self.delay_param = rospy.get_param("~delay_param", 0.5)
    
    self.ft_sensor_sub = rospy.Subscriber('/cfs/data', WrenchStamped, self.ft_sensor_Cb)
    self.delay_param_sub = rospy.Subscriber('/delay_param_topic', Float32, self.delay_param_Cb)
    self.filterd_val_pub = rospy.Publisher('/filterd_ftsensor', WrenchStamped, queue_size=10)

    self.raw_val = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.filterd_val = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.filterd_val_msg = WrenchStamped()

  def ft_sensor_Cb(self,msg):
    force_x = msg.wrench.force.x
    force_y = msg.wrench.force.y
    force_z = msg.wrench.force.z
    torque_x = msg.wrench.torque.x
    torque_y = msg.wrench.torque.y
    torque_z = msg.wrench.torque.z
    self.raw_val = [force_x, force_y, force_z, torque_x, torque_y, torque_z]

  def delay_param_Cb(self,msg):
    self.delay_param = msg.data

  def call_calib(self):
    rospy.wait_for_service('/cfs_sensor_calib')
    try:
      service = rospy.ServiceProxy('/cfs_sensor_calib', Empty)
      response = service()
    except rospy.ServiceException:
      print('cfs_calib service calling failed')


  def main(self):
    r = rospy.Rate(40)

    # calibration
    self.call_calib()
    
    while not rospy.is_shutdown():

      self.filterd_val_msg.header = Header()
      self.filterd_val_msg.header.stamp = rospy.Time.now()
      self.filterd_val_msg.header.frame_id = 'cfs_frame' 

      for i in range (len(self.raw_val)):
        self.filterd_val[i] = (1-self.delay_param) * self.filterd_val[i] + self.delay_param * self.raw_val[i]

      self.filterd_val_msg.wrench.force.x = self.filterd_val[2]
      self.filterd_val_msg.wrench.force.y = -self.filterd_val[0]
      self.filterd_val_msg.wrench.force.z = -self.filterd_val[1]
      self.filterd_val_msg.wrench.torque.x = self.filterd_val[5]
      self.filterd_val_msg.wrench.torque.y = -self.filterd_val[3]
      self.filterd_val_msg.wrench.torque.z = -self.filterd_val[4]

      self.filterd_val_pub.publish(self.filterd_val_msg)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("lpf_for_FTsensor")
  Tracker = lpf_for_FTsensor()
  Tracker.main()
