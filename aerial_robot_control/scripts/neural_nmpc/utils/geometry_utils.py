import numpy as np
import casadi as cs

def euclidean_dist(x, y, thresh=None):
    """
    Measures the euclidean distance between points x and y. If a threshold value is provided, this function returns True
    if the calculated distance is smaller than the threshold, or False otherwise. If no threshold is provided, this
    function returns the computed distance.

    :param x: n-dimension point
    :param y: n-dimension point
    :param thresh: threshold
    :type thresh: float
    :return: If thresh is not None: whether x and y are closer to each other than the threshold.
    If thresh is None: the distance between x and y
    """

    dist = np.sqrt(np.sum((x - y) ** 2))

    if thresh is None:
        return dist

    return dist < thresh

def v_dot_q(v, q):
    rot_mat = q_to_rot_mat(q)
    if isinstance(q, np.ndarray) and isinstance(rot_mat, np.ndarray):
        return rot_mat.dot(v)
    elif isinstance(q, cs.SX) or isinstance(rot_mat, cs.MX):
        return cs.mtimes(rot_mat, v)
    else:
        raise TypeError("Unsupported type for rotation matrix or vector.")

def q_to_rot_mat(q):
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]

    if isinstance(q, np.ndarray):
        rot_mat = np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)]])

    else:
        rot_mat = cs.vertcat(
            cs.horzcat(1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
            cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)),
            cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)))

    return rot_mat