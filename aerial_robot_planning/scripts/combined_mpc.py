#!/usr/bin/env python3
"""
Simple combined MPC + Servo control for beetle1 robot.
Combines trans_mpc.py and gripper_move.py functionality.

Usage: 
  Trajectory mode: rosrun aerial_robot_planning combined_mpc.py beetle1
  Direct control:  rosrun aerial_robot_planning combined_mpc.py beetle1 joint=-0.05 v=0.01
  
Joint control examples:
  joint=-0.1     → servo rotates -2π (full reverse)
  joint=0.05     → servo rotates π (half forward) 
  joint=0.0      → servo at 0° (center position)
  
Alternative formats also supported:
  joint=[-0.1] velocity=[0.01]  (list format)
  joint=-0.1 velocity=0.01      (direct format)
  joint=-0.1 v=0.01             (shortened velocity)
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

from pub_mpc_joint_traj import MPCTrajPtPub
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates, ServoControlCmd
import trajs


class CombinedMPCController:
   
    def __init__(self, robot_name: str = "beetle1", joint_target=None, joint_velocity=None):
        rospy.init_node('combined_mpc_node', anonymous=True)
        self.robot_name = robot_name
        self.joint_target = joint_target
        self.joint_velocity = joint_velocity if joint_velocity else [0.01]
        
        # Control mode
        self.trajectory_mode = joint_target is None
        
        # Trajectory setup (only if in trajectory mode)
        if self.trajectory_mode:
            self.circle_traj = trajs.CircleTraj(loop_num=1)
            self.mpc_traj_pub = MPCTrajPtPub(robot_name=robot_name, traj=self.circle_traj)
        
        # Joint control setup
        self.joint_pub = rospy.Publisher(f'/{robot_name}/extendable_joints_ctrl', JointState, queue_size=10)
        self.joint_names = ['extendable_joint1', 'extendable_joint2', 'extendable_joint3', 'extendable_joint4']
        self.joint_ratios = [1.0, -1.0, 1.0, -1.0]  # j1,j3: forward; j2,j4: reverse
        
        # Servo control setup
        self.servo_id = 4  # beetle1 servo
        self.servo_pub = rospy.Publisher(f'{robot_name}/servo/target_states', ServoControlCmd, queue_size=10)
        rospy.Subscriber(f'{robot_name}/servo/states', ServoStates, self._servo_callback)
        
        # Servo state
        self.servo_angle = 0.0
        self.servo_connected = False
        
        # Control timer
        self.start_time = rospy.Time.now().to_sec()
        
        if self.trajectory_mode:
            self.timer = rospy.Timer(rospy.Duration(0.05), self._trajectory_control)
            rospy.loginfo(f"Trajectory mode: Circle trajectory started for {robot_name}")
        else:
            self.timer = rospy.Timer(rospy.Duration(0.1), self._direct_control)
            rospy.loginfo(f"Direct control mode: Target joint={joint_target} velocity={self.joint_velocity}")
            
        time.sleep(1.0)  # Wait for connections
    
    def _servo_callback(self, msg):
        """Servo state callback (from gripper_move.py)"""
        for servo in msg.servos:
            if servo.index == self.servo_id:
                self.servo_angle = servo.angle
                self.servo_connected = True
                break
    
    def joint_to_servo_angle(self, joint_position: float) -> float:
        """
        Convert joint position to servo angle
        Joint range: -0.1m to +0.1m
        Servo range: -2π to +2π radians
        Relationship: 0.1m = 2π radians
        """
        # Clamp joint position to valid range
        joint_position = max(-0.1, min(0.1, joint_position))
        
        # Convert: joint_position (m) → servo_angle (rad)
        # 0.1m = 2π rad, so: servo_angle = joint_position * (2π / 0.1) = joint_position * 20π
        servo_angle_rad = joint_position * (2 * np.pi / 0.1)
        
        return servo_angle_rad
    
    def send_unified_command(self, joint_position: float, velocity: float = 0.01):
        """Send the same logical command to both servo and joints"""
        # Calculate servo angle from joint position
        servo_angle_rad = self.joint_to_servo_angle(joint_position)
        servo_angle_units = int(servo_angle_rad * 651.74)  # Convert to servo units
        
        # 1. Send servo command (hardware)
        if self.servo_connected:
            servo_cmd = ServoControlCmd()
            servo_cmd.index = [self.servo_id]
            servo_cmd.angles = [servo_angle_units]
            self.servo_pub.publish(servo_cmd)
        
        # 2. Send joint command (simulation) - apply coupling ratios
        joint_positions = [joint_position * ratio for ratio in self.joint_ratios]
        
        joint_cmd = JointState()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = self.joint_names
        joint_cmd.position = joint_positions
        joint_cmd.velocity = [velocity] * 4
        self.joint_pub.publish(joint_cmd)
        
        # Log the unified command
        rospy.loginfo(f"Unified cmd: joint={joint_position:.3f}m → servo={servo_angle_rad:.3f}rad ({np.degrees(servo_angle_rad):.1f}°) "
                     f"→ servo_units={servo_angle_units} joints={[f'{p:.3f}' for p in joint_positions]}")
    
    def _trajectory_control(self, event):
        """Trajectory control mode - follows circle trajectory"""
        t_elapsed = rospy.Time.now().to_sec() - self.start_time
        
        # Check if trajectory finished
        if self.circle_traj.check_finished(t_elapsed):
            rospy.loginfo("Trajectory completed!")
            return
        
        # Calculate trajectory progress (0.0 to 1.0)
        progress = min(1.0, t_elapsed / self.circle_traj.T)
        
        # Convert progress to joint position (-0.1m to +0.1m)
        # Full trajectory = full joint range
        joint_position = (progress - 0.5) * 0.2  # Maps 0→1 to -0.1→+0.1
        
        self.send_unified_command(joint_position, 0.01)
    
    def _direct_control(self, event):
        """Direct control mode - maintains target position"""
        if self.joint_target and len(self.joint_target) > 0:
            target_position = self.joint_target[0]  # Use first joint position
            velocity = self.joint_velocity[0] if len(self.joint_velocity) > 0 else 0.01
            
            self.send_unified_command(target_position, velocity)
    
    def run(self):
        """Keep running until finished or interrupted"""
        rate = rospy.Rate(10)
        try:
            if self.trajectory_mode:
                while not rospy.is_shutdown() and not self.mpc_traj_pub.is_finished:
                    rate.sleep()
                rospy.loginfo("Trajectory mode finished!")
            else:
                rospy.loginfo("Direct control mode - press Ctrl+C to stop")
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
    # Parse arguments manually to handle joint= and velocity= syntax
    robot_name = "beetle1"  # default
    joint_target = None
    joint_velocity = None
    
    for arg in sys.argv[1:]:
        if arg.startswith("joint="):
            joint_target = parse_numeric_argument(arg.split("=", 1)[1])
        elif arg.startswith("velocity=") or arg.startswith("v="):
            joint_velocity = parse_numeric_argument(arg.split("=", 1)[1])
        elif not "=" in arg:
            robot_name = arg  # Robot name without =
    
    # Validate joint target
    if joint_target:
        if not isinstance(joint_target, list) or len(joint_target) == 0:
            rospy.logerr("Invalid joint target format. Use: joint=[-0.1]")
            return
        
        # Validate range
        target = joint_target[0]
        if target < -0.1 or target > 0.1:
            rospy.logerr(f"Joint position {target} out of range [-0.1, 0.1]")
            return
        
        rospy.loginfo(f"Direct control: joint={target}m → servo={target*20*np.pi:.2f}rad ({np.degrees(target*20*np.pi):.1f}°)")
    else:
        rospy.loginfo("Trajectory mode: Running circle trajectory")
    
    controller = CombinedMPCController(robot_name, joint_target, joint_velocity)
    controller.run()


if __name__ == '__main__':
    main()
