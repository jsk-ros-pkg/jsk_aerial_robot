#!/usr/bin/env python3
"""
 Created by zicen xiong on 25-1-31.
 Enhanced servo-based MPC trajectory tracking script for Amoeba robot.

 This script provides a hybrid control approach:
 1. Simulation Mode: Direct distance-based joint control (like original trans_mpc.py)
 2. Real Hardware Mode: Servo angle-based control that follows distance commands
 
 Real Hardware Mechanism:
   - Servo id:4 mechanically drives all four extendable joints simultaneously
   - 1 full rotation (2π) causes: joint1,3 → +0.1m, joint2,4 → -0.1m
   - The script automatically detects simulation vs real hardware and adapts
 
 Usage:
   rosrun aerial_robot_planning trans_mpc_servo.py [robot_name] [--mode simulation|hardware]
   
 Example:
   rosrun aerial_robot_planning trans_mpc_servo.py amoeba --mode hardware
   rosrun aerial_robot_planning trans_mpc_servo.py amoeba --mode simulation
   
 Requirements:
   - Robot (e.g., amoeba) should be running with MPC controller
   - For hardware mode: Servo system should be active
   - For simulation mode: Joint controllers should be active
"""

import sys
import os
import numpy as np
import rospy
import argparse

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

from pub_mpc_joint_traj import MPCTrajPtPub
from sensor_msgs.msg import JointState
from spinal.msg import ServoStates, ServoControlCmd
import trajs


class CircleTrajWithServo:
    """
    A CircleTraj wrapper that adds synchronized extendable joint control
    
    Real Hardware Mechanism:
    - Servo id:4 mechanically couples all four extendable joints
    - 1 full rotation (2π) → joint1,3: +0.1m, joint2,4: -0.1m
    - This synchronized motion reflects the actual hardware coupling
    """
    def __init__(self, loop_num: int = 1):
        self.circle_traj = trajs.CircleTraj(loop_num)
        
        # Hardware mechanism parameters
        self.max_displacement = 0.1  # Maximum joint displacement (meters)
        
        # ⭐ CRITICAL: This line controls the ACTUAL joint motion directions!
        # j1,j3: forward (+1.0), j2,j4: reverse (-1.0)
        self.joint_coupling_ratios = [1.0, -1.0, 1.0, -1.0]  # j1,j3: forward; j2,j4: reverse
        
        self.joint_names = ['extendable_joint1', 'extendable_joint2', 
                           'extendable_joint3', 'extendable_joint4']
        
    def get_3d_pt(self, t: float):
        """Get 3D position, velocity, and acceleration from CircleTraj"""
        return self.circle_traj.get_3d_pt(t)
    
    def get_3d_orientation(self, t: float):
        """Get 3D orientation from CircleTraj (returns constant orientation)"""
        try:
            return self.circle_traj.get_3d_orientation(t)
        except AttributeError:
            # CircleTraj doesn't have orientation, return defaults
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
            r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
            r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
            return qw, qx, qy, qz, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc
    
    def check_finished(self, t: float) -> bool:
        """Check if trajectory is finished"""
        return self.circle_traj.check_finished(t)
    
    def get_servo_positions(self, t: float) -> list:
        """
        Calculate synchronized servo positions for all four extendable joints
        
        Args:
            t (float): Current time
            
        Returns:
            list: Joint positions [j1, j2, j3, j4] reflecting hardware mechanism
        """
        # Get trajectory progress (0.0 to 1.0)
        if self.circle_traj.check_finished(t):
            progress = 1.0
        else:
            progress = min(1.0, t / self.circle_traj.T)
        
        # Convert progress to servo rotation equivalent (0 to 2π over trajectory)
        servo_rotation = progress * 2 * np.pi
        
        # Calculate base displacement from servo rotation
        # Real mechanism: 2π rotation → ±max_displacement
        base_displacement = (servo_rotation / (2 * np.pi)) * self.max_displacement
        
        # Apply coupling ratios to get individual joint positions
        joint_positions = [base_displacement * ratio for ratio in self.joint_coupling_ratios]
        
        return joint_positions
    
    def get_servo_angle(self, t: float) -> float:
        """
        Calculate the equivalent servo angle for hardware control
        
        Args:
            t (float): Current time
            
        Returns:
            float: Servo angle in radians
        """
        # Get trajectory progress (0.0 to 1.0)
        if self.circle_traj.check_finished(t):
            progress = 1.0
        else:
            progress = min(1.0, t / self.circle_traj.T)
        
        # Convert progress to servo rotation (0 to 2π over trajectory)
        servo_angle = progress * 2 * np.pi
        
        return servo_angle


class ServoController:
    """
    Hardware servo controller (similar to gripper_move.py)
    Handles real servo id:4 communication
    """
    def __init__(self, robot_name: str):
        self.robot_name = robot_name
        self.servo_id = 4  # Amoeba extendable joints servo
        
        # Servo state tracking
        self.servo_angle = 0.0
        self.servo_temp = 0
        self.servo_load = 0.0
        self.servo_error = 0
        self.servo_connected = False
        
        # Servo parameters (you may need to adjust these for amoeba servo id:4)
        self.servo_max_angle = rospy.get_param(f'{robot_name}/servo_info/max_angles', 4096)
        self.servo_min_angle = rospy.get_param(f'{robot_name}/servo_info/min_angles', 0)
        self.servo_max_load = rospy.get_param(f'{robot_name}/servo_info/max_load', 200)
        
        # Conversion parameters
        # Assuming servo range 0-4096 corresponds to 0-2π radians
        self.servo_angle_scale = 2 * np.pi / (self.servo_max_angle - self.servo_min_angle)
        self.servo_zero_offset = self.servo_min_angle
        
        # Subscribe to servo states
        rospy.Subscriber(f'{robot_name}/servo/states', ServoStates, self._callback_servo_states)
        
        # Publish servo commands
        self.pub_servo_target = rospy.Publisher(f'{robot_name}/servo/target_states', ServoControlCmd, queue_size=10)
        
        rospy.loginfo(f"ServoController initialized for servo id:{self.servo_id}")
        rospy.loginfo(f"Servo range: {self.servo_min_angle}-{self.servo_max_angle}")
        
        # Wait for servo connection
        rospy.sleep(1.0)
    
    def _callback_servo_states(self, msg):
        """Callback for servo state updates"""
        # Find our servo in the message
        for servo_state in msg.servos:
            if servo_state.index == self.servo_id:
                self.servo_angle = servo_state.angle
                self.servo_temp = servo_state.temp
                self.servo_load = servo_state.load
                self.servo_error = servo_state.error
                self.servo_connected = True
                break
    
    def servo_command(self, target_angle_rad: float):
        """
        Send servo command in radians
        
        Args:
            target_angle_rad (float): Target angle in radians (0 to 2π)
        """
        # Convert radians to servo units
        servo_units = int(target_angle_rad / self.servo_angle_scale + self.servo_zero_offset)
        
        # Clamp to servo limits
        servo_units = max(self.servo_min_angle, min(self.servo_max_angle, servo_units))
        
        # Create and send command
        servo_cmd = ServoControlCmd()
        servo_cmd.index = [self.servo_id]
        servo_cmd.angles = [servo_units]
        
        self.pub_servo_target.publish(servo_cmd)
        
        rospy.logdebug(f"Servo cmd: {target_angle_rad:.3f} rad → {servo_units} units")
    
    def is_connected(self) -> bool:
        """Check if servo is connected and responding"""
        return self.servo_connected and self.servo_error == 0


class HybridMPCController:
    """
    Main controller that handles both simulation and hardware modes
    """
    def __init__(self, robot_name: str, mode: str = "auto"):
        self.robot_name = robot_name
        self.mode = mode
        
        # Create trajectory with servo control
        self.traj_with_servo = CircleTrajWithServo(loop_num=1)
        
        # Create MPC trajectory publisher (always needed for robot motion)
        self.mpc_traj_pub = MPCTrajPtPub(robot_name=robot_name, traj=self.traj_with_servo)
        
        # Mode detection and initialization
        self.servo_controller = None
        self.joint_pub = None
        
        if mode == "auto":
            self.mode = self._detect_mode()
        
        if self.mode == "hardware":
            self._init_hardware_mode()
        
        # Always initialize simulation control for unified behavior
        self._init_simulation_mode()
        
        # Control timer (20 Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self._control_callback)
        
        # Track start time
        self.start_time = rospy.Time.now().to_sec()
        
        rospy.loginfo(f"HybridMPCController initialized for robot: {robot_name}")
        rospy.loginfo(f"Operating mode: {self.mode}")
        rospy.loginfo("Circle trajectory with synchronized extendable joint control started")
    
    def _detect_mode(self) -> str:
        """Auto-detect if we're running in simulation or with real hardware"""
        # Check if servo topics exist (indicating real hardware)
        topics = rospy.get_published_topics()
        servo_topics = [topic for topic, _ in topics if 'servo' in topic and self.robot_name in topic]
        
        if servo_topics:
            rospy.loginfo("Detected servo topics - using hardware mode")
            return "hardware"
        else:
            rospy.loginfo("No servo topics detected - using simulation mode")
            return "simulation"
    
    def _init_hardware_mode(self):
        """Initialize hardware servo control"""
        try:
            self.servo_controller = ServoController(self.robot_name)
            rospy.loginfo("Hardware mode: Servo control active")
        except Exception as e:
            rospy.logerr(f"Failed to initialize servo controller: {e}")
            rospy.logwarn("Falling back to simulation mode")
            self.mode = "simulation"
            self._init_simulation_mode()
    
    def _init_simulation_mode(self):
        """Initialize simulation joint control"""
        self.joint_pub = rospy.Publisher(
            f'/{self.robot_name}/extendable_joints_ctrl', 
            JointState, 
            queue_size=10
        )
        rospy.loginfo("Simulation mode: Direct joint control active (unified with servo commands)")
    
    def _control_callback(self, event):
        """Main control loop callback"""
        # Calculate elapsed time
        t_elapsed = rospy.Time.now().to_sec() - self.start_time
        
        # Get desired servo angle (this is the UNIFIED command)
        servo_angle = self.traj_with_servo.get_servo_angle(t_elapsed)
        
        # Apply the SAME servo angle to both hardware and simulation
        if self.mode == "hardware" and self.servo_controller:
            self._hardware_control(servo_angle)
        
        # ALWAYS do simulation control (for unified behavior)
        if self.joint_pub:
            self._simulation_control(servo_angle)
    
    def _hardware_control(self, servo_angle: float):
        """Hardware control using servo commands"""
        # Send servo command directly
        if self.servo_controller.is_connected():
            self.servo_controller.servo_command(servo_angle)
            
            # Log occasionally
            rospy.loginfo_throttle(2.0, f"Hardware mode - Servo angle: {servo_angle:.3f} rad ({np.degrees(servo_angle):.1f}°)")
            rospy.loginfo_throttle(2.0, f"Servo status: temp={self.servo_controller.servo_temp}, "
                                        f"load={self.servo_controller.servo_load}")
        else:
            rospy.logwarn_throttle(5.0, "Servo not connected or in error state")
    
    def _simulation_control(self, servo_angle: float):
        """Simulation control - convert servo angle to joint distances"""
        # Convert servo angle to joint positions using the mechanical relationship
        # servo_angle (radians) → joint displacement (meters)
        # 2π radians → 0.1m displacement, so: displacement = (angle/2π) * 0.1
        base_displacement = (servo_angle / (2 * np.pi)) * self.traj_with_servo.max_displacement
        
        # Apply coupling ratios to get individual joint positions
        joint_positions = [base_displacement * ratio for ratio in self.traj_with_servo.joint_coupling_ratios]
        
        # Create JointState message
        joint_cmd = JointState()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = self.traj_with_servo.joint_names
        joint_cmd.position = joint_positions
        joint_cmd.velocity = [0.01] * len(joint_positions)
        joint_cmd.effort = []
        
        self.joint_pub.publish(joint_cmd)
        
        # Log occasionally with both servo angle and resulting distances
        rospy.loginfo_throttle(2.0, f"Unified Control - Servo: {servo_angle:.3f} rad ({np.degrees(servo_angle):.1f}°)")
        rospy.loginfo_throttle(2.0, f"Joint distances: {[f'{pos:.4f}' for pos in joint_positions]} (m)")
        rospy.loginfo_throttle(2.0, f"Motion pattern: j1,j3→{joint_positions[0]:.3f}m, j2,j4→{joint_positions[1]:.3f}m")
    
    def is_finished(self) -> bool:
        """Check if trajectory is finished"""
        return self.mpc_traj_pub.is_finished
    
    def shutdown(self):
        """Clean shutdown"""
        self.control_timer.shutdown()
        
        # Send safe stop commands
        if self.mode == "hardware" and self.servo_controller:
            # Return servo to safe position (0 radians)
            self.servo_controller.servo_command(0.0)
            rospy.loginfo("Hardware mode: Sent servo to safe position")
        elif self.joint_pub:
            # Send zero joint positions
            zero_cmd = JointState()
            zero_cmd.header.stamp = rospy.Time.now()
            zero_cmd.name = self.traj_with_servo.joint_names
            zero_cmd.position = [0.0] * 4
            zero_cmd.velocity = [0.0] * 4
            self.joint_pub.publish(zero_cmd)
            rospy.loginfo("Simulation mode: Sent zero joint positions")
        
        rospy.loginfo("HybridMPCController shutdown complete")

    def test_unified_control(self, servo_angle_degrees: float):
        """
        Test function to demonstrate unified control
        
        Args:
            servo_angle_degrees (float): Servo angle in degrees (e.g., 180)
        """
        servo_angle_rad = np.radians(servo_angle_degrees)
        
        rospy.loginfo(f"Testing unified control with {servo_angle_degrees}° ({servo_angle_rad:.3f} rad)")
        
        # Hardware control
        if self.mode == "hardware" and self.servo_controller:
            self._hardware_control(servo_angle_rad)
        
        # Simulation control (always active for unified behavior)
        if self.joint_pub:
            self._simulation_control(servo_angle_rad)
        
        # Calculate expected distance for verification
        expected_distance = (servo_angle_rad / (2 * np.pi)) * self.traj_with_servo.max_displacement
        rospy.loginfo(f"Expected joint distances: j1,j3→{expected_distance:.4f}m, j2,j4→{-expected_distance:.4f}m")


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Hybrid MPC Controller for Amoeba Robot')
    parser.add_argument('robot_name', nargs='?', default='amoeba',
                       help='Robot name (default: amoeba)')
    parser.add_argument('--mode', choices=['auto', 'simulation', 'hardware'], 
                       default='auto',
                       help='Control mode (default: auto-detect)')
    
    # Parse known args to handle rosrun arguments
    args, unknown = parser.parse_known_args()
    return args


def main():
    """Main function"""
    rospy.init_node('trans_mpc_servo_node', anonymous=True)
    
    # Parse arguments
    args = parse_arguments()
    robot_name = args.robot_name
    mode = args.mode
    
    rospy.loginfo(f"Starting trans_mpc_servo with robot: {robot_name}, mode: {mode}")
    
    # Check if robot parameters exist
    if not rospy.has_param(robot_name):
        rospy.logerr(f"Robot name '{robot_name}' not found in ROS parameters! "
                    f"Make sure the robot is running.")
        return
    
    # Create and run the controller
    try:
        controller = HybridMPCController(robot_name, mode)
        
        # Keep node running until trajectory is finished or ROS shuts down
        rate = rospy.Rate(10)  # 10 Hz monitoring rate
        
        while not rospy.is_shutdown() and not controller.is_finished():
            rate.sleep()
        
        rospy.loginfo("Circle trajectory with hybrid servo control completed!")
        
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
    except Exception as e:
        rospy.logerr(f"Controller failed: {e}")
    finally:
        if 'controller' in locals():
            controller.shutdown()


if __name__ == '__main__':
    main()
