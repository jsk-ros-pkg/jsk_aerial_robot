#!/usr/bin/env python
import rospy,math
import numpy as np
from geometry_msgs.msg import WrenchStamped

force_1 = np.array([0,0,0])
torque_1 = np.array([0,0,0])
force_2 = np.array([0,0,0])
torque_2 = np.array([0,0,0])
filtered_force_1 = np.array([0,0,0])
filtered_torque_1 = np.array([0,0,0])
filtered_force_2 = np.array([0,0,0])
filtered_torque_2 = np.array([0,0,0])

def main():
    global force_1, torque_1, force_2, torque_2
    est_wrench_sub_1 = rospy.Subscriber("/beetle1/estimated_external_wrench",WrenchStamped,estWrenchCb1)
    est_wrench_sub_2 = rospy.Subscriber("/beetle2/estimated_external_wrench",WrenchStamped,estWrenchCb2)
    filtered_wrench_pub_1 = rospy.Publisher("/beetle1/filtered_est_wrench", WrenchStamped, queue_size = 1)
    filtered_wrench_pub_2 = rospy.Publisher("/beetle2/filtered_est_wrench", WrenchStamped, queue_size = 1)
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
        filtered_force_1 = (force_1 - force_2)/2.0
        filtered_force_2 = -(force_1 - force_2)/2.0
        filtered_torque_1 = (torque_1 - torque_2)/2.0
        filtered_torque_2 = -(torque_1 - torque_2)/2.0
        wrench_msg_1 = WrenchStamped()
        wrench_msg_1.wrench.force.x = filtered_force_1[0]
        wrench_msg_1.wrench.force.y = filtered_force_1[1]
        wrench_msg_1.wrench.force.z = filtered_force_1[2]
        wrench_msg_1.wrench.torque.x = filtered_torque_1[0]
        wrench_msg_1.wrench.torque.y = filtered_torque_1[1]
        wrench_msg_1.wrench.torque.z = filtered_torque_1[2]
        filtered_wrench_pub_1.publish(wrench_msg_1)
        wrench_msg_2 = WrenchStamped()
        wrench_msg_2.wrench.force.x = filtered_force_2[0]
        wrench_msg_2.wrench.force.y = filtered_force_2[1]
        wrench_msg_2.wrench.force.z = filtered_force_2[2]
        wrench_msg_2.wrench.torque.x = filtered_torque_2[0]
        wrench_msg_2.wrench.torque.y = filtered_torque_2[1]
        wrench_msg_2.wrench.torque.z = filtered_torque_2[2]
        filtered_wrench_pub_2.publish(wrench_msg_2)
        r.sleep()
        
def estWrenchCb1(msg):
    global force_1, torque_1
    force_1 = np.array([msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z])
    torque_1 = np.array([msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z])

def estWrenchCb2(msg):
    global force_2, torque_2
    force_2 = np.array([msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z])
    torque_2 = np.array([msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z])    
    

if __name__ == '__main__':
    rospy.init_node("filtering_wrench")
    main()

