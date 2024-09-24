#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf

class coordTransformer():
    def __init__(self,
                 robot_name = 'ninja1'):
        self.robot_name = robot_name
        self.listener = tf.TransformListener()

    def posTransform(self, curr_coord, dist_coord, target_pos_curr):
        try:
            self.listener.waitForTransform(self.robot_name+'/'+curr_coord, self.robot_name+'/'+dist_coord, rospy.Time(), rospy.Duration(5.0))

            (trans, rot) = self.listener.lookupTransform(self.robot_name+'/'+curr_coord, self.robot_name+'/'+dist_coord, rospy.Time(0))

            transform_matrix = tf.transformations.quaternion_matrix(rot)
            transform_matrix[:3, 3] = trans

            target_pos_curr_4d = np.array(list(target_pos_curr) + [1.0])

            target_pos_dist_4d = np.dot(transform_matrix, target_pos_curr_4d)

            target_pos_dist = target_pos_dist_4d[:3]

            return target_pos_dist
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", e)
            return None

if __name__ == '__main__':
    try:
        rospy.init_node("coordTransformer")
        trans = coordTransformer(robot_name = 'ninja1');
        target_base = [1,0,0]
        rospy.loginfo(trans.posTransform('fc', 'cog', target_base))
        rospy.spin()
    except rospy.ROSInterruptException: pass
