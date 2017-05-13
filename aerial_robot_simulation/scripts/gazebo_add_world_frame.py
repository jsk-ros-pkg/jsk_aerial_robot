#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

class WorldFrame:
    def init(self):
        rospy.init_node('add_world_frame')
        self.br = tf.TransformBroadcaster()
        self.sub_hydrus_odom = rospy.Subscriber("/uav/state", Odometry, self.hydrus_odom_callback)
    def hydrus_odom_callback(self, msg):
        self.br.sendTransform((msg.pose.pose.position.x,
                               msg.pose.pose.position.y,
                               msg.pose.pose.position.z),
                              (msg.pose.pose.orientation.x,
                               msg.pose.pose.orientation.y,
                               msg.pose.pose.orientation.z,
                               msg.pose.pose.orientation.w),
                              #rospy.Time.now(),
                              msg.header.stamp,
                              "link1",
                              "world")

if __name__ == '__main__':
    try:
        world_frame = WorldFrame()
        world_frame.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

