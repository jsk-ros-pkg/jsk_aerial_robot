#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from tf import transformations

class WorldFrame:
    def init(self):
        rospy.init_node('add_world_frame')
        self.br = tf.TransformBroadcaster()
        self.sub_hydrus_odom = rospy.Subscriber("/uav/state", Odometry, self.hydrus_odom_callback)
    def hydrus_odom_callback(self, msg):
        origin_t = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        origin_q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        origin_matrix = transformations.concatenate_matrices(transformations.translation_matrix(origin_t),
                                                             transformations.quaternion_matrix(origin_q))
        inv_matrix = transformations.inverse_matrix(origin_matrix)
        inv_t = transformations.translation_from_matrix(inv_matrix)
        inv_q = transformations.quaternion_from_matrix(inv_matrix)
        self.br.sendTransform(inv_t,
                              inv_q,
                              msg.header.stamp,
                              "world",
                              "link2")

if __name__ == '__main__':
    try:
        world_frame = WorldFrame()
        world_frame.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

