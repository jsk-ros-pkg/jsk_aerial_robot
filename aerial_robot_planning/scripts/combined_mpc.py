#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pure servo control for beetle1 servo id:4
Based on gripper_move.py pattern


Servo specifications:
  - Robot: beetle1
  - Servo ID: 4  
  - Zero position: 2048
  - Upper bound: 9048
  - Lower bound: -4950
"""

import sys
import rospy
import rosparam
import time
from spinal.msg import ServoStates, ServoControlCmd
from sensor_msgs.msg import JointState


class ServoMoveNode:

    def __init__(self):
        print(f'amoeba_deform_initiate')
        rospy.init_node('servo_move', anonymous=True)
        self.servo_index = 0
        self.servo_angle = 0.0
        self.servo_temp = 0
        self.servo_load = 0.0
        self.servo_error = 0
        self.servo_target_index = 4  # beetle1 servo id
        self.servo_target_angles = 0
        self.robot_ns = rospy.get_param("~robot_ns", "beetle1")
        
        # Servo limits for beetle1 servo id:4
        self.servo_zero_position = 2048
        self.servo_max_angles = 9048
        self.servo_min_angles = -4950
        self.servo_max_load = rospy.get_param(self.robot_ns + '/servo_info/max_load', 200)
        
        # Subscribe and publish
        rospy.Subscriber(self.robot_ns + '/servo/states', ServoStates, self._callback_servo_states)
        self.pub_servo_target = rospy.Publisher(self.robot_ns + '/servo/target_states', ServoControlCmd, queue_size=10)
        # Add joint control publisher for extendable joints
        self.central_Servo_pub = rospy.Publisher(f'/{self.robot_ns}/extendable_joints_ctrl', 
                                       JointState, queue_size=10)
        time.sleep(1.0)

    def _callback_servo_states(self, msg):
        for servo in msg.servos:
            if servo.index == self.servo_target_index:
                self.servo_index = servo.index
                self.servo_angle = servo.angle
                self.servo_temp = servo.temp
                self.servo_load = servo.load
                self.servo_error = servo.error
                break

    def servo_target_cmd(self, target_index, target_angle):
        servo_target_cmd = ServoControlCmd()
        servo_target_cmd.index = [target_index]
        servo_target_cmd.angles = [target_angle]
        time.sleep(0.1)
        self.pub_servo_target.publish(servo_target_cmd)
        print(f'servo_target_cmd: {servo_target_cmd}')

    def return_zero(self):
        time.sleep(0.1)
        self.servo_target_cmd(self.servo_target_index, self.servo_zero_position)
        time.sleep(0.5)

    def joint_control(self, servo_angle):
        """
        Convert servo angle to extendable joint positions and publish
        
        Movement equation:
        joint = -4950 to 9048 linear maps to:
        - extendable_joint1,3: -0.1 to 0.1 (linear mapping)
        - extendable_joint2,4: 0.1 to -0.1 (negative linear mapping)
        """
        # Linear mapping from servo range [-4950, 9048] to joint range [-0.1, 0.1]
        servo_min, servo_max = -4950, 9048
        joint_min, joint_max = -0.1, 0.1
        
        
        # Joint names for extendable joints
        joint_names = ['extendable_joint1', 'extendable_joint2', 
                           'extendable_joint3', 'extendable_joint4']
        
        
        # Normalize servo angle to [0, 1] range
        servo_normalized = (servo_angle - servo_min) / (servo_max - servo_min)
        
        # Map to joint positions
        # joints 1,3: linear mapping (0.1 to -0.1)
        joint_13_position = joint_max - servo_normalized * (joint_max - joint_min)
        
        # joints 2,4: negative linear mapping (-0.1 to 0.1)
        joint_24_position = joint_min + servo_normalized * (joint_max - joint_min)
        
        # Create joint positions array
        joint_positions = [joint_13_position, joint_24_position, joint_13_position, joint_24_position]
        
        # Create and publish JointState message
        joint_cmd = JointState()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = joint_names
        joint_cmd.position = joint_positions
        joint_cmd.velocity = [0.01] * len(joint_positions)  # Set reasonable velocities
        joint_cmd.effort = []

        self.central_Servo_pub.publish(joint_cmd)
        # central_Servo_pub = rospy.Publisher(f'/{self.robot_ns}/extendable_joints_ctrl', 
        #                         JointState, queue_size=10)
        # central_Servo_pub.publish(joint_cmd)

        rospy.loginfo(f'Joint positions: j1,j3={joint_13_position:.4f}m, j2,j4={joint_24_position:.4f}m')
        rospy.loginfo(f'Published to /{self.robot_ns}/extendable_joints_ctrl')

    def move_to_position(self, target_angle):
        # Clamp to servo limits
        target_angle = max(self.servo_min_angles, min(self.servo_max_angles, target_angle))
        
        # 1. joint control, 2. servo control
        self.joint_control(target_angle)
        rospy.loginfo(f'Moving servo {self.servo_target_index} to position {target_angle}')
        rospy.sleep(1.0)
         # Send servo command
        self.servo_target_cmd(self.servo_target_index, target_angle)
        rospy.loginfo(f'servo:{self.servo_target_index} command sent to target angle {target_angle}!')

def parse_joint_argument(arg_string):
    """Parse joint argument - direct integer value"""
    try:
        return int(arg_string)
    except ValueError:
        try:
            return int(float(arg_string))
        except ValueError:
            rospy.logerr(f"Failed to parse argument as integer: {arg_string}")
            return None

def main():
    # Parse arguments to handle joint= syntax
    target_angle = None
    
    for arg in sys.argv[1:]:
        if arg.startswith("joint="):
            target_angle = parse_joint_argument(arg.split("=", 1)[1])
    
    # Default to zero position if none provided
    if target_angle is None:
        target_angle = 2048  # Zero position
        rospy.loginfo("No joint target specified, using default: joint=2048 (zero position)")
    
    # Validate range
    if target_angle < -4950 or target_angle > 9048:
        rospy.logerr(f"Servo angle {target_angle} out of range [-4950, 9048]")
        return
    
    rospy.loginfo(f"Servo control: target_angle={target_angle} (range: -4950 to 9048, zero: 2048)")
    
    node = ServoMoveNode()
    rospy.loginfo(f"Initialized ServoMoveNode")
    time.sleep(2)
    node.move_to_position(target_angle)
    
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()
