# This message is used to set the desired full body pose of the robot,
# including the first link's state, as well as each joint's state.

# Header with timestamp
Header header

# Desired state for the first link
nav_msgs/Odometry root_state

# Desired joint states
sensor_msgs/JointState joint_state
