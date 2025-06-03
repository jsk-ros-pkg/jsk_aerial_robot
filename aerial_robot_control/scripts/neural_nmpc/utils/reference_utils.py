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
        # TODO 'aggressive' is not correct terminology

        # Polar coordinates
        theta = np.random.uniform(0, 2 * np.pi, 1)
        psi = np.random.uniform(0, 2 * np.pi, 1)
        r = 1 * world_radius + np.random.uniform(-0.5, 0.5, 1) * world_radius

        # Transform to Cartesian
        x = r * np.sin(theta) * np.cos(psi)
        y = r * np.sin(theta) * np.sin(psi)
        z = r * np.cos(theta)
        # TODO: currently world radius is used as sample area ontop of current position so could sample outside of the "world area"
        
        # Add offset to current position
        target_pos = current_pos + np.array([x, y, z]).reshape((1, 3))
        target_vel = np.array([0, 0, 0])    # Fixed linear velocity

        # Set fixed quaternions
        target_quat = np.array([1, 0, 0, 0])
        target_omega = np.array([0, 0, 0])  # Fixed angular velocity

        # Append quaternions to target position
        return [np.concatenate((target_pos, target_vel, target_quat, target_omega))]

    else:
        # Sample random target position
        target_pos = np.random.uniform(-world_radius, world_radius, (1, 3))
        target_vel = np.array([0, 0, 0])    # Fixed linear velocity

        # Set fixed quaternions
        target_quat = np.array([[1, 0, 0, 0], [0, 0, 0], [0, 0, 0]])
        target_omega = np.array([0, 0, 0])  # Fixed angular velocity

        # Append quaternions to target position
        return [np.concatenate((target_pos, target_vel, target_quat, target_omega))]
