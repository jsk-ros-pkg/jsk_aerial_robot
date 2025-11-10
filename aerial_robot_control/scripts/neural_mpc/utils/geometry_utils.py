import numpy as np
import casadi as cs
import pyquaternion


def quaternion_to_euler(q):
    q = pyquaternion.Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
    yaw, pitch, roll = q.yaw_pitch_roll
    return [roll, pitch, yaw]


def quaternion_inverse(q):
    w, x, y, z = q[0], q[1], q[2], q[3]

    if isinstance(q, np.ndarray):
        return np.array([w, -x, -y, -z])
    else:
        return cs.vertcat(w, -x, -y, -z)


def unit_quaternion(q):
    """
    Normalizes a quaternion to be unit modulus.
    :param q: 4-dimensional numpy array or CasADi object
    :return: the unit quaternion in the same data format as the original one
    """

    if isinstance(q, np.ndarray):
        q_norm = np.linalg.norm(q)
    else:
        q_norm = cs.sqrt(cs.sumsqr(q))
    return q / q_norm


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
    """
    Applies the rotation of quaternion q to vector v. In order words, rotates vector v by q.
    Quaternion format: wxyz.
    """
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
        rot_mat = np.array(
            [
                [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
                [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
            ]
        )

    else:
        rot_mat = cs.vertcat(
            cs.horzcat(1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)),
            cs.horzcat(2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)),
            cs.horzcat(2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)),
        )

    return rot_mat


def q_dot_q(q, r):
    """
    Applies the rotation of quaternion r to quaternion q. In order words, rotates quaternion q by r. Quaternion format:
    wxyz.

    :param q: 4-length numpy array or CasADi MX. Initial rotation
    :param r: 4-length numpy array or CasADi MX. Applied rotation
    :return: The quaternion q rotated by r, with the same format as in the input.
    """

    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    rw, rx, ry, rz = r[0], r[1], r[2], r[3]

    t0 = rw * qw - rx * qx - ry * qy - rz * qz
    t1 = rw * qx + rx * qw - ry * qz + rz * qy
    t2 = rw * qy + rx * qz + ry * qw - rz * qx
    t3 = rw * qz - rx * qy + ry * qx + rz * qw

    if isinstance(q, np.ndarray):
        return np.array([t0, t1, t2, t3])
    else:
        return cs.vertcat(t0, t1, t2, t3)


# fmt: off
def skew_symmetric(v):
    """
    Computes the skew-symmetric matrix of a 3D vector (PAMPC version)

    :param v: 3D numpy vector or CasADi MX
    :return: the corresponding skew-symmetric matrix of v with the same data type as v
    """

    if isinstance(v, np.ndarray):
        return np.array([[0,    -v[0], -v[1], -v[2]],
                         [v[0],     0,  v[2], -v[1]],
                         [v[1], -v[2],     0,  v[0]],
                         [v[2],  v[1], -v[0],     0]])

    return cs.vertcat(
        cs.horzcat(0,    -v[0], -v[1], -v[2]),
        cs.horzcat(v[0],     0,  v[2], -v[1]),
        cs.horzcat(v[1], -v[2],     0,  v[0]),
        cs.horzcat(v[2],  v[1], -v[0],     0))
