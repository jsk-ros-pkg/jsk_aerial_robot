#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3Stamped, Quaternion, PoseStamped
from std_msgs.msg import Bool
from apriltag_ros.msg import AprilTagDetectionArray
import sys
from module import operation_quaternion as oq

class TagAlignmentController():
    def __init__(self):
        # ros node
        rospy.init_node("tag_alignment_controller")
        self.rpy_publisher = rospy.Publisher("/gimbalrotor/final_target_baselink_rpy", Vector3Stamped, queue_size=10)
        self.tag_detection_subscriber = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback_tag_detection)
        self.body_pose_subscriber = rospy.Subscriber("/gimbalrotor/mocap/pose", PoseStamped, self.callback_body_pose)
        # controller switching
        self.controller_switch = rospy.Subscriber("/gimbalrotor/tag_alignment_controller_switch", Bool, self.callback_controller_switch)
        self.is_controller_enabled = False
        # desired pose of detected tag
        self.desired_tag_orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        self.q_camera_body = Quaternion(x=0.5, y=-0.5, z=0.5, w=0.5)
        self.q_world_body = None
        self.gain_rotation = 0.5
        self.last_publish_time = rospy.Time.now().to_sec()

    def callback_tag_detection(self, msg):
        if len(msg.detections) == 0:
            return
        tag_orientation = msg.detections[0].pose.pose.pose.orientation
        if self.q_world_body is None:
            rospy.logwarn_throttle(1.0, "body pose has not been received yet")
            return
        self.rotation_camera = oq.synthesize_quaternion(tag_orientation, oq.conjugate(self.desired_tag_orientation))
        axis, angle = oq.quaternion_to_axis_angle(self.rotation_camera)
        rospy.loginfo_throttle(1.0, "Rotation axis: x=%.3f, y=%.3f, z=%.3f, angle=%.3f", axis[0], axis[1], axis[2], angle)
        self.rotation_camera = oq.axis_angle_to_quaternion(axis, self.gain_rotation*angle)
        if self.is_controller_enabled and angle > 0.05:
            self.calcluate_final_target_rpy()

    def callback_body_pose(self, msg):
        self.q_world_body = msg.pose.orientation

    def callback_controller_switch(self, msg):
        if msg.data:
            rospy.loginfo("Tag alignment controller is enabled")
            self.is_controller_enabled = True
        else:
            rospy.loginfo("Tag alignment controller is disabled")
            self.is_controller_enabled = False

    def convert_frame(self, q_A_B, q_A):
        q_A_B_conjugate = oq.conjugate(q_A_B)
        q_B = oq.synthesize_quaternion(q_A_B_conjugate, oq.synthesize_quaternion(q_A, q_A_B))
        return q_B

    def calcluate_final_target_rpy(self):
        rotation_body = self.convert_frame(self.q_camera_body, self.rotation_camera)
        rospy.loginfo("Rotation quaternion in camera frame: x=%.3f, y=%.3f, z=%.3f, w=%.3f", self.rotation_camera.x, self.rotation_camera.y, self.rotation_camera.z, self.rotation_camera.w)
        rospy.loginfo("Rotation quaternion in body frame: x=%.3f, y=%.3f, z=%.3f, w=%.3f", rotation_body.x, rotation_body.y, rotation_body.z, rotation_body.w)
        rospy.loginfo("Current body orientation quaternion: x=%.3f, y=%.3f, z=%.3f, w=%.3f", self.q_world_body.x, self.q_world_body.y, self.q_world_body.z, self.q_world_body.w)
        self.final_target_orientation = oq.synthesize_quaternion(self.q_world_body, rotation_body)
        final_target_rpy = oq.quaternion_to_euler(self.final_target_orientation)
        rospy.loginfo("")
        rospy.loginfo("Final target RPY: roll=%.3f, pitch=%.3f, yaw=%.3f", final_target_rpy[0], final_target_rpy[1], final_target_rpy[2])
        current_time = rospy.Time.now().to_sec()
        if current_time - self.last_publish_time > 1:
            self.publish_final_target_rpy(final_target_rpy)
            self.last_publish_time = current_time

    def publish_final_target_rpy(self, rpy):
        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = rospy.Time.now()
        rpy_msg.vector.x = rpy[0]
        rpy_msg.vector.y = rpy[1]
        rpy_msg.vector.z = rpy[2]
        self.rpy_publisher.publish(rpy_msg)

if __name__ == '__main__':
    controller = TagAlignmentController()
    rospy.spin()

