import numpy as np

def sample_random_target(current_pos, world_radius, aggressive=True):
    """
    Generates a target pose for the robot to reach.

    :param current_pos: Current 3D position of the robot. Only used if 'aggressive' is set to True.
    :param world_radius: Radius in meters of the area in which target points are sampled from.
    :param aggressive: Flag to enable aggressive target points, which are sampled far from the current robot position.
                       If set to False, target points are sampled uniformly in the world.
    :return: List of a single randomly sampled target point as a 3-dimensional numpy array.
    """
    if aggressive:
        max_attempts = 100
        min_distance = 1 * world_radius
        z_offset = 1.1 * world_radius  # Offset to ensure z is above the ground
        
        for _ in range(max_attempts):
            # Generate random point uniformly in sphere
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)
            u = np.random.uniform(0, 1)
            
            # Uniform distribution in sphere
            r = world_radius * (u ** (1/3))  # Cube root for uniform volume distribution
            
            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)  
            z = r * np.cos(phi) + z_offset

            target_pos = np.array([x, y, z])
            
            # Check if far enough from current position
            if np.linalg.norm(target_pos - current_pos) >= min_distance:
                break
        
        # Fallback: If final target_pos is outside sphere, project it back
        if np.linalg.norm(target_pos - np.array([0, 0, z_offset])) > world_radius:
            target_pos = target_pos / np.linalg.norm(target_pos) * world_radius * 0.95
    
        # # Polar coordinates
        # theta = np.random.uniform(0, 2 * np.pi, 1)
        # phi = np.random.uniform(0, 2 * np.pi, 1)
        # r = 1 * world_radius + np.random.uniform(-0.5, 0.5, 1) * world_radius

        # # Transform to Cartesian
        # x = r * np.sin(theta) * np.cos(phi)
        # y = r * np.sin(theta) * np.sin(phi)
        # z = r * np.cos(theta)
        # # TODO: currently world radius is used as sample area ontop of current position so could sample outside of the "world area"
        
        # Add offset to current position
        # target_pos = current_pos + np.array([x, y, z]).reshape((3))

    else:
        # Sample random target position
        target_pos = np.random.uniform(-world_radius, world_radius, (1, 3))

    # Set fixed velocity, rotation and angular vel
    target_vel = np.array([0, 0, 0])
    target_rot = np.array([0, 0, 0])
    target_omega = np.array([0, 0, 0])

    # Append quaternions to target position
    return np.concatenate((target_pos, target_vel, target_rot, target_omega))[np.newaxis, :]
