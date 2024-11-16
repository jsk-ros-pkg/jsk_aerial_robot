#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def main():
    rospy.init_node('dynamic_frame_broadcaster')

    # tf2 Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # Broadcaster for the new frame
    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        try:
            # Get the transformation from world to laser_frame
            trans = tf_buffer.lookup_transform('world', 'multirotor/laser_frame', rospy.Time(0))
            
            # Extract translation
            translation = trans.transform.translation

            # Extract rotation and convert to Euler angles
            rotation = trans.transform.rotation
            quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            
            # Reset roll and pitch to zero
            roll = 0.0
            pitch = 0.0
            new_quaternion = quaternion_from_euler(roll, pitch, yaw)

            # Define the new transform
            new_trans = geometry_msgs.msg.TransformStamped()
            new_trans.header.stamp = rospy.Time.now()
            new_trans.header.frame_id = 'world'
            new_trans.child_frame_id = 'multirotor/laser_frame_level'
            new_trans.transform.translation.x = translation.x
            new_trans.transform.translation.y = translation.y
            new_trans.transform.translation.z = translation.z
            new_trans.transform.rotation.x = new_quaternion[0]
            new_trans.transform.rotation.y = new_quaternion[1]
            new_trans.transform.rotation.z = new_quaternion[2]
            new_trans.transform.rotation.w = new_quaternion[3]

            # Broadcast the new transform
            br.sendTransform(new_trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed. Retrying...")
            continue

        rate.sleep()

if __name__ == '__main__':
    main()
