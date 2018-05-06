#!/usr/bin/env python

import sys
import time
import rospy
import math
from visualization_msgs.msg import MarkerArray, Marker

if __name__ == "__main__":

    rospy.init_node("simple_collision")

    obj_type = rospy.get_param("~type", 3) # 3: Sphere
    position_x = rospy.get_param("~position_x", 0)
    position_y = rospy.get_param("~position_y", 0)
    scale_x = rospy.get_param("~scale_x", 0.1)
    scale_y = rospy.get_param("~scale_y", 0.1)
    scale_z = rospy.get_param("~scale_z", 0.1)

    pub = rospy.Publisher("/env_collision", MarkerArray, queue_size=10)

    time.sleep(1)
    env_obj = MarkerArray()
    env_obj.markers.append(Marker())
    env_obj.markers[0].header.frame_id = "/world"
    env_obj.markers[0].type = obj_type
    env_obj.markers[0].action = 0
    env_obj.markers[0].pose.position.x = position_x
    env_obj.markers[0].pose.position.y = position_y
    env_obj.markers[0].pose.position.z = 0
    env_obj.markers[0].pose.orientation.w = 1
    env_obj.markers[0].scale.x = scale_x
    env_obj.markers[0].scale.y = scale_y
    env_obj.markers[0].scale.z = scale_z
    env_obj.markers[0].color.r = 1
    env_obj.markers[0].color.a = 1

    while not rospy.is_shutdown():

        env_obj.markers[0].header.stamp = rospy.get_rostime()

        pub.publish(env_obj)
        time.sleep(0.1)
