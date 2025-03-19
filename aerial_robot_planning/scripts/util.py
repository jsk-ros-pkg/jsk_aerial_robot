'''
 Created by jiaxuan and jinjie on 25/01/22.
'''

from functools import wraps
import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R


def check_position_initialized(obj: object, position_attr: str, object_name: str):
    """
    Waits until the position is initialized. Logs a message repeatedly
    :param obj:  The object to check the position of
    :param position_attr:  The attribute of the object to check
    :param object_name:  The name of the object being tracked
    :return:
    """
    while not rospy.is_shutdown() and getattr(obj, position_attr) is None:
        rospy.loginfo(f"Waiting for {object_name} position...")
        rospy.sleep(0.2)

    if getattr(obj, position_attr) is not None:
        rospy.loginfo(f"{object_name} Position received for the first time")


def check_topic_subscription(func):
    @wraps(func)
    def wrapper(self, topic_name, msg_type, *args, **kwargs):
        try:
            topics = rospy.get_published_topics()
            topic_names = [t[0] for t in topics]
            if topic_name not in topic_names:
                raise ValueError(f"Topic '{topic_name}' is not currently available.")
            subscriber = func(self, topic_name, msg_type, *args, **kwargs)
            return subscriber
        except Exception as e:
            rospy.logerr(f"Error: {str(e)}")
            rospy.signal_shutdown(f"Error: {str(e)}")
            raise e

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
    v = np.sqrt(vx**2 + vy**2 + vz**2)
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
    w = np.sqrt(wx**2 + wy**2 + wz**2)
    print("Angular speed (w): max = {:.3f}".format(np.max(w)))
    print("===========================================\n")