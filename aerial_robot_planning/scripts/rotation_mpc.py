#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rotation MPC script for aerial robot with continuous arm length control
Usage: rosrun aerial_robot_planning rotation_mpc.py robot_name loop_num=2

Example: rosrun aerial_robot_planning rotation_mpc.py beetle1 loop_num=2
"""

import sys
import rospy
import numpy as np
import time
import os
import threading
import argparse

# Add the current script directory to Python path for local imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import local modules
import amb_trans
from trajs import YawRotationRoll0dTraj
from pub_mpc_joint_traj import MPCTrajPtPub
from spinal.msg import ServoStates, ServoControlCmd
from sensor_msgs.msg import JointState


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Rotation MPC with arm length control")
    parser.add_argument("robot_name", type=str, help="Robot name (e.g., beetle1)")
    
    loop_num = 1  # default value
    
    # Parse loop argument from remaining sys.argv
    for arg in sys.argv[1:]:
        if arg.startswith("loop=") or arg.startswith("loop_num="):
            try:
                loop_num = int(arg.split("=", 1)[1])
            except ValueError:
                rospy.logerr(f"Failed to parse loop argument: {arg}")
                loop_num = 1
                
    args = parser.parse_args([sys.argv[1]])  # Only parse robot_name
    return args.robot_name, loop_num


def rotation_to_length(yaw_angle):
    """
    Convert rotation angle to arm length using sin-like function
    
    Mapping:
    rot/deg  len
    0        50
    90       100
    180      50
    270      0
    360      50
    
    This follows: length = 50 + 50 * sin(yaw + π/2)
    """
    # Convert angle to degrees for easier understanding
    yaw_deg = np.degrees(yaw_angle) % 360
    
    # Sin-like function: 50 + 50 * sin(yaw_rad + π/2)
    # sin(x + π/2) = cos(x), so this becomes: 50 + 50 * cos(yaw_rad)
    length = 50.0 + 50.0 * np.cos(yaw_angle)
    
    # Clamp to valid range [0, 100]
    length = max(0.0, min(100.0, length))
    
    rospy.loginfo(f"Rotation: {yaw_deg:.1f}° -> Length: {length:.1f}mm")
    return length


class ArmController:
    """Separate class to handle arm length control based on trajectory"""
    
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.servo_node = None
        self.running = False
        self.traj = None
        self.start_time = None
        self.last_length = None
        
        # Servo control attributes (copied from amb_trans.ServoMoveNode)
        self.servo_index = 0
        self.servo_angle = 0.0
        self.servo_temp = 0
        self.servo_load = 0.0
        self.servo_error = 0
        self.servo_target_index = 4  # beetle1 servo id
        self.servo_cmd_lengths = 0
        
        # Servo limits for beetle1 servo id:4
        self.servo_zero_position = 2048
        self.servo_max_angles = 9048
        self.servo_min_angles = -4950
        
    def initialize(self):
        """Initialize servo publishers and subscribers"""
        try:
            # Subscribe and publish (don't initialize ROS node)
            rospy.Subscriber(f'{self.robot_name}/servo/states', 
                           ServoStates, self._callback_servo_states)
            self.pub_servo_target = rospy.Publisher(f'{self.robot_name}/servo/target_states', 
                                                  ServoControlCmd, queue_size=10)
            self.rviz_urdf_pub = rospy.Publisher(f'/{self.robot_name}/extendable_joints_ctrl', 
                                               JointState, queue_size=10)
            
            rospy.sleep(1.0)  # Allow connections to establish
            rospy.loginfo("ArmController: Initialized servo publishers/subscribers")
            return True
        except Exception as e:
            rospy.logerr(f"ArmController: Failed to initialize servo control: {e}")
            return False
    
    def _callback_servo_states(self, msg):
        """Callback for servo states"""
        for servo in msg.servos:
            if servo.index == self.servo_target_index:
                self.servo_index = servo.index
                self.servo_angle = servo.angle
                self.servo_temp = servo.temp
                self.servo_load = servo.load
                self.servo_error = servo.error
                break
    
    def servo_target_cmd(self, target_index, cmd_length):
        """Send servo target command"""
        servo_target_cmd = ServoControlCmd()
        servo_target_cmd.index = [target_index]
        servo_target_cmd.angles = [cmd_length]
        time.sleep(0.1)
        self.pub_servo_target.publish(servo_target_cmd)
        rospy.loginfo(f'servo_target_cmd: {servo_target_cmd}')
    
    def rviz_urdf_update(self, servo_angle):
        """Update RVIZ URDF display"""
        # Linear mapping from servo range [-4950, 9048] to joint range [-0.1, 0.1]
        servo_min, servo_max = -4950, 9048
        joint_min, joint_max = -0.1, 0.1
        
        joint_names = ['extendable_joint1', 'extendable_joint2', 
                      'extendable_joint3', 'extendable_joint4']
        servo_normalized = (servo_angle - servo_min) / (servo_max - servo_min)
        joint_13_position = joint_max - servo_normalized * (joint_max - joint_min)
        joint_24_position = joint_min + servo_normalized * (joint_max - joint_min)
        joint_positions = [joint_13_position, joint_24_position, joint_13_position, joint_24_position]
        
        joint_cmd = JointState()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = joint_names
        joint_cmd.position = joint_positions
        joint_cmd.velocity = [0.01] * len(joint_positions)
        joint_cmd.effort = []
        
        self.rviz_urdf_pub.publish(joint_cmd)
    
    def move_to_position(self, cmd_length):
        """Move servo to target position"""
        cmd_length = max(self.servo_min_angles, min(self.servo_max_angles, cmd_length))
        self.rviz_urdf_update(cmd_length)
        rospy.loginfo(f'Moving servo {self.servo_target_index} to position {cmd_length}')
        self.servo_target_cmd(self.servo_target_index, cmd_length)
        
    def start_control(self, traj):
        """Start arm length control thread"""
        self.traj = traj
        self.start_time = rospy.Time.now()
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        rospy.loginfo("ArmController: Started arm control thread")
    
    def stop_control(self):
        """Stop arm length control"""
        self.running = False
        if hasattr(self, 'control_thread'):
            self.control_thread.join(timeout=2.0)
        
        # Return to neutral position
        rospy.loginfo("ArmController: Returning to neutral position...")
        neutral_angle = amb_trans.length2angle(50.0)  # 50mm = neutral
        self.move_to_position(neutral_angle)
        rospy.loginfo("ArmController: Returned to neutral position")
    
    def _control_loop(self):
        """Main control loop running in separate thread"""
        rate = rospy.Rate(10.0)  # 10 Hz
        
        while self.running and not rospy.is_shutdown():
            try:
                # Calculate elapsed time
                current_time = rospy.Time.now()
                elapsed_time = (current_time - self.start_time).to_sec()
                
                # Get current orientation from trajectory
                qw, qx, qy, qz, _, _, _, _, _, _ = self.traj.get_3d_orientation(elapsed_time)
                
                # Extract yaw angle from quaternion
                yaw_angle = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
                
                # Calculate desired arm length based on rotation
                cmd_length = rotation_to_length(yaw_angle)
                
                # Only send command if length changed significantly
                if self.last_length is None or abs(cmd_length - self.last_length) > 1.0:
                    extend_joint_angle = amb_trans.length2angle(cmd_length)
                    self.move_to_position(extend_joint_angle)
                    self.last_length = cmd_length
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"ArmController: Error in control loop: {e}")
                break


def main():
    # Parse command line arguments
    robot_name, loop_num = parse_arguments()
    
    # Initialize ROS node
    rospy.init_node('rotation_mpc', anonymous=True)
    
    # Check if robot exists in ROS parameters
    if not rospy.has_param(robot_name):
        rospy.logerr(f"Robot name '{robot_name}' not found in ROS parameters! Make sure the robot is running.")
        return
    
    rospy.loginfo(f"Starting rotation MPC for robot: {robot_name} with {loop_num} loops")
    
    # Create trajectory
    traj = YawRotationRoll0dTraj(loop_num)
    rospy.loginfo("Created YawRotationRoll0dTraj trajectory")
    
    # Initialize arm controller
    arm_controller = ArmController(robot_name)
    if not arm_controller.initialize():
        rospy.logerr("Failed to initialize arm controller")
        return
    
    # Create MPC trajectory publisher for robot motion
    mpc_node = MPCTrajPtPub(robot_name=robot_name, traj=traj)
    rospy.loginfo("Created MPCTrajPtPub for robot rotation control")
    
    # Start arm length control in parallel
    arm_controller.start_control(traj)
    
    # Wait for MPC trajectory to complete
    rate = rospy.Rate(20)  # 20 Hz check rate
    try:
        while not rospy.is_shutdown():
            if mpc_node.is_finished:
                rospy.loginfo("Rotation trajectory completed!")
                break
            rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("Rotation MPC interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error in rotation MPC: {e}")
    finally:
        # Stop arm control
        arm_controller.stop_control()
        rospy.loginfo("Rotation MPC completed")


if __name__ == '__main__':
    main()
