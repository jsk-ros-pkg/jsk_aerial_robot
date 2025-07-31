#!/usr/bin/env python3
"""
 Created by zicen xiong on 25-1-31.

 A standalone MPC trajectory tracking script that:
 1. Uses CircleTraj for robot motion (1 loop only)
 2. Controls servo ID 4 to move all four extendable joints according to real hardware mechanism
 3. Can be run with rosrun
 
 Real Hardware Mechanism:
   - Servo id:4 mechanically drives all four extendable joints simultaneously
   - 1 full rotation (2π) causes: joint1,3 → +0.1m, joint2,4 → -0.1m
   - This reflects the actual mechanical coupling in the amoeba robot
 
 Usage:
   rosrun aerial_robot_planning trans_mpc.py [robot_name]
   
 Example:
   rosrun aerial_robot_planning trans_mpc.py amoeba
   
 Requirements:
   - Robot (e.g., amoeba) should be running with MPC controller
   - Servo system should be active and responsive to /{robot_name}/extendable_joints_ctrl
   
 Trajectory Details:
   - Circle trajectory: radius=1m, center=(-1,0,0.5), period=10s, 1 loop
   - Extendable joint motion: Synchronized motion reflecting real hardware
   - Controls all extendable joints via JointState messages
"""

import sys
import os
import numpy as np
import rospy

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

from pub_mpc_joint_traj import MPCTrajPtPub
from sensor_msgs.msg import JointState
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


class TransMPCNode:
    """
    Main node class that combines trajectory tracking with servo control
    """
    def __init__(self, robot_name: str):
        self.robot_name = robot_name
        
        # Create trajectory with servo control
        self.traj_with_servo = CircleTrajWithServo(loop_num=1)
        
        # Create MPC trajectory publisher
        self.mpc_traj_pub = MPCTrajPtPub(robot_name=robot_name, traj=self.traj_with_servo)
        
        # Servo command publisher (using JointState for extendable joints)
        self.servo_pub = rospy.Publisher(f'/{robot_name}/extendable_joints_ctrl', 
                                       JointState, queue_size=10)
        
        # Servo control timer (20 Hz)
        self.servo_timer = rospy.Timer(rospy.Duration(0.05), self.servo_timer_callback)
        
        # Track start time for servo control
        self.start_time = rospy.Time.now().to_sec()
        
        rospy.loginfo(f"TransMPC initialized for robot: {robot_name}")
        rospy.loginfo("Circle trajectory with synchronized extendable joint control started")
        rospy.loginfo("Real hardware mechanism: servo id:4 drives all four joints")
        rospy.loginfo("Expected motion: joint1/2 (0→+0.1m), joint3/4 (0→-0.1m)")
        rospy.loginfo("Using JointState messages to /{robot_name}/extendable_joints_ctrl")
    
    def servo_timer_callback(self, event):
        """Timer callback to publish synchronized servo commands"""
        # Calculate elapsed time
        t_elapsed = rospy.Time.now().to_sec() - self.start_time
        
        # Get desired servo positions for all four joints (synchronized)
        joint_positions = self.traj_with_servo.get_servo_positions(t_elapsed)
        
        # Create JointState message for extendable joints control
        joint_cmd = JointState()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = self.traj_with_servo.joint_names
        joint_cmd.position = joint_positions
        joint_cmd.velocity = [0.01] * len(joint_positions)  # Set reasonable velocities
        joint_cmd.effort = []
        
        self.servo_pub.publish(joint_cmd)
        
        # Log joint positions occasionally
        rospy.loginfo_throttle(2.0, f"Joint positions: {[f'{pos:.4f}' for pos in joint_positions]} (m)")
        rospy.loginfo_throttle(2.0, f"Motion pattern: j1,j3→{joint_positions[0]:.3f}m, j2,j4→{joint_positions[1]:.3f}m")
    
    def is_finished(self) -> bool:
        """Check if both trajectory and servo control are finished"""
        return self.mpc_traj_pub.is_finished
    
    def shutdown(self):
        """Clean shutdown"""
        self.servo_timer.shutdown()
        rospy.loginfo("TransMPC node shutdown complete")


def main():
    """Main function"""
    rospy.init_node('trans_mpc_node', anonymous=True)
    
    # Get robot name from command line argument or use default
    import sys
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        robot_name = "amoeba"
        rospy.logwarn(f"No robot name provided, using default: {robot_name}")
    
    # Check if robot parameters exist
    if not rospy.has_param(robot_name):
        rospy.logerr(f"Robot name '{robot_name}' not found in ROS parameters! "
                    f"Make sure the robot is running.")
        return
    
    # Create and run the node
    trans_mpc_node = TransMPCNode(robot_name)
    
    # Keep node running until trajectory is finished or ROS shuts down
    rate = rospy.Rate(10)  # 10 Hz monitoring rate
    
    try:
        while not rospy.is_shutdown() and not trans_mpc_node.is_finished():
            rate.sleep()
        
        rospy.loginfo("Circle trajectory with servo control completed!")
        
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
    
    finally:
        trans_mpc_node.shutdown()


if __name__ == '__main__':
    main()
