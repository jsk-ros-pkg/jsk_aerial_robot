#!/usr/bin/env python3
"""
Simple joint control for beetle1 robot.
Controls extendable joints and servo simultaneously with realistic movement speed.

The URDF model moves at realistic servo speed (~0.05 m/s) to match physical 
servo movement (~2π/s) for accurate inertia calculations and crash prevention.

Usage: 
  rosrun aerial_robot_planning combined_mpc.py beetle1 joint=-0.05
  
Joint control examples:
  joint=-0.1     → servo units -3000 (full retraction)
  joint=0.05     → servo units 1500 (half extension) 
  joint=0.0      → servo units 0 (center position)
  
Alternative formats also supported:
  joint=[-0.1]   (list format)
  joint=-0.1     (direct format)
"""

import sys
import os
import numpy as np
import rospy
import time
import argparse
import ast

# Add current path for imports
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

from sensor_msgs.msg import JointState
from spinal.msg import ServoStates, ServoControlCmd


class JointController:
   
    def __init__(self, robot_name: str = "beetle1", joint_target=None):
        rospy.init_node('joint_controller_node', anonymous=True)
        self.robot_name = robot_name
        self.joint_target = joint_target
        
        # Joint control setup
        self.joint_pub = rospy.Publisher(f'/{robot_name}/extendable_joints_ctrl', JointState, queue_size=10)
        self.joint_names = ['extendable_joint1', 'extendable_joint2', 'extendable_joint3', 'extendable_joint4']
        self.joint_ratios = [1.0, -1.0, 1.0, -1.0]  # j1,j3: forward; j2,j4: reverse
        
        # Servo control setup for beetle1 (RS485 protocol)
        self.servo_id = 4  # beetle1 servo
        self.servo_angle = 0
        self.servo_temp = 0
        self.servo_load = 0.0
        self.servo_error = 0
        self.servo_connected = False
        
        # Servo limits for beetle1 servo id:4
        self.servo_zero_position = 0
        self.servo_min_angle = -3000
        self.servo_max_angle = 3000
        
        # Joint velocity for simulation (to match real servo speed ~2π/s ≈ 6.28 rad/s)
        # For 0.1m joint range equivalent to 2π servo rotation: 0.1m at 2π/s = 0.1*2π/(2π) = 0.1 m/s
        # But we want slower movement for safety: use 0.05 m/s (half speed)
        self.joint_velocity = 0.05  # m/s - matches realistic servo speed
        
        # Publishers and subscribers
        self.servo_pub = rospy.Publisher(f'{robot_name}/servo/target_states', ServoControlCmd, queue_size=10)
        rospy.Subscriber(f'{robot_name}/servo/states', ServoStates, self._servo_callback)
        
        # Control timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self._direct_control)
        rospy.loginfo(f"Joint control mode: Target joint={joint_target}")
            
        time.sleep(1.0)  # Wait for connections
    
    def _servo_callback(self, msg):
        """Servo state callback - following gripper_move.py pattern"""
        for servo in msg.servos:
            if servo.index == self.servo_id:
                self.servo_angle = servo.angle
                self.servo_temp = servo.temp
                self.servo_load = servo.load
                self.servo_error = servo.error
                self.servo_connected = True
                break
    
    def joint_to_servo_angle(self, joint_position: float) -> int:
        """
        Convert joint position to servo angle units for beetle1
        Joint range: -0.1m to +0.1m
        Servo range: -3000 to +3000 units (zero at 0)
        Relationship: 0.1m = 3000 units
        """
        # Clamp joint position to valid range
        joint_position = max(-0.1, min(0.1, joint_position))
        
        # Convert: joint_position (m) → servo_angle (units)
        # 0.1m = 3000 units, so: servo_angle = joint_position * (3000 / 0.1) = joint_position * 30000
        servo_angle_units = int(joint_position * 30000)
        
        # Clamp to servo limits
        servo_angle_units = max(self.servo_min_angle, min(self.servo_max_angle, servo_angle_units))
        
        return servo_angle_units
    
    def servo_target_cmd(self, target_index: int, target_angle: int):
        """Send servo command - following gripper_move.py pattern"""
        if self.servo_error == 1:
            rospy.logwarn(f'Servo {target_index} error detected!')
            return
            
        servo_cmd = ServoControlCmd()
        servo_cmd.index = [target_index]
        servo_cmd.angles = [target_angle]
        
        time.sleep(0.1)  # Following gripper_move.py timing
        self.servo_pub.publish(servo_cmd)
        rospy.loginfo(f'Servo command sent: index={target_index}, angle={target_angle}')

    def send_unified_command(self, joint_position: float):
        """Send the same logical command to both servo and joints"""
        # Calculate servo angle from joint position
        servo_angle_units = self.joint_to_servo_angle(joint_position)
        
        # 1. Send servo command (hardware) - using gripper_move.py pattern
        if self.servo_connected:
            self.servo_target_cmd(self.servo_id, servo_angle_units)
        else:
            rospy.logwarn("Servo not connected, skipping servo command")
        
        # 2. Send joint command (simulation) - apply coupling ratios with realistic velocity
        joint_positions = [joint_position * ratio for ratio in self.joint_ratios]
        joint_velocities = [self.joint_velocity * ratio for ratio in self.joint_ratios]
        
        joint_cmd = JointState()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = self.joint_names
        joint_cmd.position = joint_positions
        joint_cmd.velocity = joint_velocities  # Add realistic velocity for simulation
        self.joint_pub.publish(joint_cmd)
        
        # Log the unified command
        rospy.loginfo(f"Unified cmd: joint={joint_position:.3f}m → servo_units={servo_angle_units} "
                     f"joints={[f'{p:.3f}' for p in joint_positions]} vel={self.joint_velocity:.3f}m/s")
    
    def _direct_control(self, event):
        """Direct control mode - maintains target position"""
        if self.joint_target and len(self.joint_target) > 0:
            target_position = self.joint_target[0]  # Use first joint position
            self.send_unified_command(target_position)
    
    def run(self):
        """Keep running until interrupted"""
        try:
            rospy.loginfo("Joint control mode - press Ctrl+C to stop")
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Interrupted by user")
        finally:
            self.timer.shutdown()


def parse_numeric_argument(arg_string):
    """Parse numeric arguments - handles both list format [value] and direct value"""
    try:
        # First try to parse as literal (handles lists like [-0.1])
        result = ast.literal_eval(arg_string)
        if isinstance(result, list):
            return result
        else:
            # Single value, wrap in list
            return [result]
    except:
        try:
            # Try to parse as a simple float
            return [float(arg_string)]
        except:
            rospy.logerr(f"Failed to parse argument: {arg_string}")
            return None


def main():
    # Parse arguments manually to handle joint= syntax
    robot_name = "beetle1"  # default
    joint_target = None
    
    for arg in sys.argv[1:]:
        if arg.startswith("joint="):
            joint_target = parse_numeric_argument(arg.split("=", 1)[1])
        elif not "=" in arg:
            robot_name = arg  # Robot name without =
    
    # Default joint target if none provided
    if joint_target is None:
        joint_target = [0.0]  # Default to center position
        rospy.loginfo("No joint target specified, using default: joint=0.0")
    
    # Validate joint target
    if not isinstance(joint_target, list) or len(joint_target) == 0:
        rospy.logerr("Invalid joint target format. Use: joint=[-0.1]")
        return
    
    # Validate range
    target = joint_target[0]
    if target < -0.1 or target > 0.1:
        rospy.logerr(f"Joint position {target} out of range [-0.1, 0.1]")
        return
    
    rospy.loginfo(f"Joint control: joint={target}m → servo_units={int(target*30000)} "
                 f"(range: {int(-0.1*30000)} to {int(0.1*30000)})")
    
    controller = JointController(robot_name, joint_target)
    controller.run()


if __name__ == '__main__':
    main()
