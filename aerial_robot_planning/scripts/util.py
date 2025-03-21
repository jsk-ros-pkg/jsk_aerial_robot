'''
 Created by jiaxuan and jinjie on 25/01/22.
'''

from functools import wraps
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R

import tf_conversions as tf
from nav_msgs.msg import Odometry

def check_first_data_received(obj: object, attr: str, object_name: str):
    """
    Waits until the position is initialized. Logs a message repeatedly
    :param obj:  The object to check the position of
    :param attr:  The attribute of the object to check
    :param object_name:  The name of the object being tracked
    :return:
    """
    while not rospy.is_shutdown() and getattr(obj, attr) is None:
        rospy.loginfo(f"Waiting for {object_name} '{attr}' msg...")
        rospy.sleep(0.2)

    if getattr(obj, attr) is not None:
        rospy.loginfo(f"{object_name} '{attr}' msg is received for the first time")


class TopicNotAvailableError(Exception):
    pass


def check_topic_subscription(func):
    @wraps(func)
    def wrapper(self, topic_name, msg_type, *args, **kwargs):
        topics = rospy.get_published_topics()
        topic_names = [t[0] for t in topics]
        if topic_name not in topic_names:
            raise TopicNotAvailableError(f"check_topic_subscription: Topic '{topic_name}' is not currently available.")
        subscriber = func(self, topic_name, msg_type, *args, **kwargs)
        return subscriber

    return wrapper


def check_traj_info(x: np.ndarray):
    """
    Check trajectory information and print out:
      - Overall time (number of time steps, assuming dt = 1 per step).
      - For position (px, py, pz): min and max values.
      - For velocity (vx, vy, vz): maximum values and overall maximum speed.
      - For orientation: convert quaternion (qw, qx, qy, qz) to Euler angles (roll, pitch, yaw) in degrees,
        then report min and max for each angle.
      - For angular velocity (wx, wy, wz): maximum values and overall maximum angular speed.

    The state vector for each time step is assumed to be in the order:
      [px, py, pz, vx, vz, vy, qw, qx, qy, qz, wx, wy, wz].
    """
    print("\n===== Checking Trajectory Information =====")
    # --- Overall Time ---
    total_time_steps = x.shape[0]
    # Assuming each row represents 1 time unit.
    print("Overall time: {} time steps".format(total_time_steps))

    # --- Position ---
    # Positions are columns 0, 1, 2.
    px = x[:, 0]
    py = x[:, 1]
    pz = x[:, 2]

    print("Position (px): min = {:.3f}, max = {:.3f}".format(np.min(px), np.max(px)))
    print("Position (py): min = {:.3f}, max = {:.3f}".format(np.min(py), np.max(py)))
    print("Position (pz): min = {:.3f}, max = {:.3f}".format(np.min(pz), np.max(pz)))

    # --- Linear Velocity ---
    # Note: according to the provided order, velocity components are:
    # vx in column 3, vz in column 4, vy in column 5.
    vx = x[:, 3]
    vz = x[:, 4]
    vy = x[:, 5]

    print("Velocity (vx): max = {:.3f}".format(np.max(vx)))
    print("Velocity (vy): max = {:.3f}".format(np.max(vy)))
    print("Velocity (vz): max = {:.3f}".format(np.max(vz)))

    # Compute overall speed v = sqrt(vx^2 + vy^2 + vz^2)
    v = np.sqrt(vx ** 2 + vy ** 2 + vz ** 2)
    print("Speed (v): max = {:.3f}".format(np.max(v)))

    # --- Orientation ---
    # Extract quaternions: columns 6-9 in the order [qw, qx, qy, qz]
    quats = x[:, 6:10]
    # scipy's Rotation.from_quat expects quaternions in [qx, qy, qz, qw] (scalar-last)
    quats_xyzw = np.hstack((quats[:, 1:], quats[:, 0:1]))

    # Convert quaternion to Euler angles in degrees using the 'xyz' (roll, pitch, yaw) convention
    eulers = R.from_quat(quats_xyzw).as_euler('xyz', degrees=True)
    roll = eulers[:, 0]
    pitch = eulers[:, 1]
    yaw = eulers[:, 2]

    print("Roll:  min = {:.3f}°, max = {:.3f}°".format(np.min(roll), np.max(roll)))
    print("Pitch: min = {:.3f}°, max = {:.3f}°".format(np.min(pitch), np.max(pitch)))
    print("Yaw:   min = {:.3f}°, max = {:.3f}°".format(np.min(yaw), np.max(yaw)))

    # --- Angular Velocity ---
    # Angular velocity components are columns 10, 11, 12.
    wx = x[:, 10]
    wy = x[:, 11]
    wz = x[:, 12]

    print("Angular Velocity (wx): max = {:.3f}".format(np.max(wx)))
    print("Angular Velocity (wy): max = {:.3f}".format(np.max(wy)))
    print("Angular Velocity (wz): max = {:.3f}".format(np.max(wz)))

    # Compute overall angular speed w = sqrt(wx^2 + wy^2 + wz^2)
    w = np.sqrt(wx ** 2 + wy ** 2 + wz ** 2)
    print("Angular speed (w): max = {:.3f}".format(np.max(w)))
    print("===========================================\n")


class TrackingErrorCalculator:
    def __init__(self):
        self.all_pos_err = [[], [], []]  # x, y, z
        self.all_ang_err = [[], [], []]  # roll, pitch, yaw

    def reset(self):
        self.__init__()

    def update(self, odom, traj_msg):
        err_px, err_py, err_pz, err_roll, err_pitch, err_yaw = self._cal_tracking_error(odom, traj_msg)
        self.all_pos_err[0].append(err_px)
        self.all_pos_err[1].append(err_py)
        self.all_pos_err[2].append(err_pz)
        self.all_ang_err[0].append(err_roll)
        self.all_ang_err[1].append(err_pitch)
        self.all_ang_err[2].append(err_yaw)

        return err_px, err_py, err_pz, err_roll, err_pitch, err_yaw

    def get_rmse_error(self):
        all_pos_err_np = np.array(self.all_pos_err)
        all_ang_err_np = np.array(self.all_ang_err)

        pos_rmse = np.sqrt(np.mean(all_pos_err_np ** 2, axis=1))
        pos_rmse_norm = np.linalg.norm(pos_rmse)
        ang_rmse = np.sqrt(np.mean(all_ang_err_np ** 2, axis=1))
        ang_rmse_norm = np.linalg.norm(ang_rmse)

        return pos_rmse_norm, pos_rmse, ang_rmse_norm, ang_rmse

    @staticmethod
    def _cal_tracking_error(uav_odom: Odometry, ref_traj):
        """
        Calculate position and orientation error between current odom and ref_traj,
        which can be a MultiDOFJointTrajectory or PredXU.
        """
        # Current pose
        cur_pos = uav_odom.pose.pose.position
        q_cur = [uav_odom.pose.pose.orientation.x,
                 uav_odom.pose.pose.orientation.y,
                 uav_odom.pose.pose.orientation.z,
                 uav_odom.pose.pose.orientation.w]

        # Extract reference pose
        try:
            if hasattr(ref_traj, "points"):  # MultiDOFJointTrajectory
                ref_px = ref_traj.points[0].transforms[0].translation.x
                ref_py = ref_traj.points[0].transforms[0].translation.y
                ref_pz = ref_traj.points[0].transforms[0].translation.z
                ref_qx = ref_traj.points[0].transforms[0].rotation.x
                ref_qy = ref_traj.points[0].transforms[0].rotation.y
                ref_qz = ref_traj.points[0].transforms[0].rotation.z
                ref_qw = ref_traj.points[0].transforms[0].rotation.w
            else:  # PredXU
                ref_px = ref_traj.x.data[0]
                ref_py = ref_traj.x.data[1]
                ref_pz = ref_traj.x.data[2]
                ref_qx = ref_traj.x.data[6]
                ref_qy = ref_traj.x.data[7]
                ref_qz = ref_traj.x.data[8]
                ref_qw = ref_traj.x.data[9]
        except AttributeError:
            raise AttributeError("Reference trajectory must be either MultiDOFJointTrajectory or PredXU!")

        # Position error
        dx = cur_pos.x - ref_px
        dy = cur_pos.y - ref_py
        dz = cur_pos.z - ref_pz

        # Orientation error
        m_cur = tf.transformations.quaternion_matrix(q_cur)
        m_ref = tf.transformations.quaternion_matrix([ref_qx, ref_qy, ref_qz, ref_qw])
        m_err = m_cur.dot(m_ref.T)
        euler_err = tf.transformations.euler_from_matrix(m_err, axes="sxyz")

        return dx, dy, dz, euler_err[0], euler_err[1], euler_err[2]
