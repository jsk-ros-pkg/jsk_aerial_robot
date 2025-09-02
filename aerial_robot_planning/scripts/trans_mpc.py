#!/usr/bin/env python3
"""
 Usage:
   rosrun aerial_robot_planning trans_mpc.py [robot_name]
   
 Trajectory Details:
   - Circle trajectory: radius=1m, center=(-1,0,0.5), period=10s, 1 loop
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
import trajs


class TransMPCNode:
    """
    Main node class for centroidal trajectory tracking only
    """
    def __init__(self, robot_name: str):
        self.robot_name = robot_name
        
        # Create simple circle trajectory
        circle_traj = trajs.CircleTraj(loop_num=1)
        
        # Create MPC trajectory publisher
        self.mpc_traj_pub = MPCTrajPtPub(robot_name=robot_name, traj=circle_traj)
        
        rospy.loginfo(f"TransMPC initialized for robot: {robot_name}")
        rospy.loginfo("Circle trajectory tracking started (centroidal movement only)")
    
    def is_finished(self) -> bool:
        """Check if trajectory tracking is finished"""
        return self.mpc_traj_pub.is_finished


def main():
    """Main function"""
    rospy.init_node('trans_mpc_node', anonymous=True)
    
    # Get robot name from command line argument or use default
    import sys
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        robot_name = "beetle1"
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
        
        rospy.loginfo("Circle trajectory completed!")
        
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")


if __name__ == '__main__':
    main()
