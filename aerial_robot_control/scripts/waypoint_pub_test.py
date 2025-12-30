#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf.transformations

def create_pose(x, y, z, yaw_deg, time_offset_sec):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now() + rospy.Duration(time_offset_sec)
    pose.header.frame_id = "world"

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    # yaw (degree) to quaternion
    yaw_rad = yaw_deg * 3.14159265 / 180.0
    q = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    return pose

def publish_path():
    rospy.init_node('path_publisher')
    pub = rospy.Publisher('/quadrotor/target_path', Path, queue_size=10)
    rospy.sleep(1.0)  # wait for publisher to register

    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "world"

    # Create waypoints
    wp1 = create_pose(0.5, 0.5, 1.0, 0, 2)
    wp2 = create_pose(1.5, -0.5, 2.0, 0, 5)
    wp3 = create_pose(5.0, 0.0, 1.0, 0, 10)

    path.poses.append(wp1)
    path.poses.append(wp2)
    path.poses.append(wp3)

    # Publish the path
    pub.publish(path)
    rospy.loginfo("Published path with 3 waypoints.")

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
