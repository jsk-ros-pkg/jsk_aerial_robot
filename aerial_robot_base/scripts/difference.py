#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Vector3

uav_state = PoseStamped()
different_pub = rospy.Publisher('different', Vector3, queue_size=10)

def uav_callback(msg):
    global uav_state
    uav_state = msg

def object_callback(msg):
    diff = Vector3()
    diff.x = uav_state.pose.position.x - msg.pose.position.x
    diff.y = uav_state.pose.position.y - msg.pose.position.y
    diff.z = uav_state.pose.position.z - msg.pose.position.z
    global different_pub
    different_pub.publish(diff)

if __name__ == '__main__':
    rospy.init_node('different', anonymous=True)

    rospy.Subscriber("/aerial_robot/pose", PoseStamped, uav_callback)
    rospy.Subscriber("/object/pose", PoseStamped, object_callback)


    rospy.spin()
