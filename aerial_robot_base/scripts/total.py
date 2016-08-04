#!/usr/bin/env python
import rospy
from aerial_robot_base.msg import FourAxisPid
from std_msgs.msg import Float32

total_pub = rospy.Publisher('total', Float32, queue_size=10)

def control_callback(msg):
    total_lift = Float32()
    for i in msg.throttle.total:
        total_lift.data += i
    global total_pub
    total_pub.publish(total_lift)


if __name__ == '__main__':
    rospy.init_node('total', anonymous=True)
    rospy.Subscriber("/controller/debug", FourAxisPid, control_callback)
    rospy.spin()
