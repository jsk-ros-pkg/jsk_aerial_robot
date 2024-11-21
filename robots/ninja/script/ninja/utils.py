#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf

class coordTransformer():
    def __init__(self):
        self.listener = tf.TransformListener()

    def getTransform(self, curr_coord, dist_coord):
        try:
            self.listener.waitForTransform(curr_coord, dist_coord, rospy.Time(), rospy.Duration(5.0))
            transformation = self.listener.lookupTransform(curr_coord, dist_coord, rospy.Time(0))
            return transformation
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
            return None

    def getHomoMatFromCoordName(self, from_coord, to_coord):
        try:
            self.listener.waitForTransform(from_coord, to_coord, rospy.Time(), rospy.Duration(5.0))
            (translation, quaternion) = self.listener.lookupTransform(from_coord, to_coord, rospy.Time(0))
            #translation
            H = np.eye(4)
            H[:3, 3] = translation
            #rotation
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
            H[:3, :3] = rotation_matrix[:3, :3]
            return H
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
            return None

    def getHomoMatFromVector(self, translation, euler_angles):
        rotation_matrix = tf.transformations.euler_matrix(
            euler_angles[0],  # Roll
            euler_angles[1],  # Pitch
            euler_angles[2]   # Yaw
        )
        H = np.eye(4)
        H[:3, :3] = rotation_matrix[:3, :3]
        H[:3, 3] = translation

        return H
        

    def posTransform(self, curr_coord, dist_coord, target_pos_curr):
        try:
            self.listener.waitForTransform(curr_coord, dist_coord, rospy.Time(), rospy.Duration(5.0))

            (trans, rot) = self.listener.lookupTransform(curr_coord, dist_coord, rospy.Time(0))

            target_pos_dist = np.array(target_pos_curr) - np.array(trans)

            return target_pos_curr
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
            return None

if __name__ == '__main__':
    try:
        rospy.init_node("coordTransformer")
        trans = coordTransformer();
        target_base = [1,0,0]
        rospy.loginfo(trans.posTransform('fc', 'cog', target_base))
        rospy.spin()
    except rospy.ROSInterruptException: pass
