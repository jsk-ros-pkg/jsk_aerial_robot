'''
 Created by jiaxuan and jinjie on 25/01/22.
'''

from functools import wraps
from typing import Optional

import numpy as np
import rospy
import math
from scipy.spatial.transform import Rotation as R

import tf_conversions as tf
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, PoseStamped


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


def check_traj_info(x: np.ndarray, if_return_path=False) -> Optional[Path]:
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

    if if_return_path:
        # ---------- Build nav_msgs/Path ----------
        frame_id = "world"
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = rospy.Time.now()

        for k in range(total_time_steps):
            pose = PoseStamped()
            pose.header.frame_id = frame_id

            # Position
            pose.pose.position.x = float(px[k])
            pose.pose.position.y = float(py[k])
            pose.pose.position.z = float(pz[k])

            # Orientation (keep original quaternion)
            qw, qx, qy, qz = quats[k]
            pose.pose.orientation.w = float(qw)
            pose.pose.orientation.x = float(qx)
            pose.pose.orientation.y = float(qy)
            pose.pose.orientation.z = float(qz)

            path_msg.poses.append(pose)

        return path_msg


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


def create_wall_markers(points, thickness=0.1, height=2.0,
                        frame_id="world", ns="walls", color=None):
    """
    Build a MarkerArray in which each polygon edge is rendered as a thin box.

    Args
    ----
    points     : list of (x, y) tuples.  Edges are formed in order.
    thickness  : wall thickness in metres
    height     : wall height in metres
    frame_id   : target TF frame for RViz
    ns         : marker namespace

    Returns
    -------
    visualization_msgs/MarkerArray
    """
    if color is None:
        color = [0.6, 0.6, 0.6, 1.0]
    markers = MarkerArray()

    for i in range(len(points) - 1):
        # End-points of the current edge
        (x0, y0), (x1, y1) = points[i], points[i + 1]

        # Length and yaw of the edge
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)

        # Mid-point of the edge (box centre)
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        cz = height / 2.0

        # Quaternion for rotation about Z
        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(0, 0, yaw)

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = i  # Unique ID inside the namespace
        m.type = Marker.CUBE
        m.action = Marker.ADD

        # Pose
        m.pose.position = Point(cx, cy, cz)
        m.pose.orientation = Quaternion(qx, qy, qz, qw)

        # Dimensions: length × thickness × height
        m.scale.x = length
        m.scale.y = thickness
        m.scale.z = height

        # Colour (light grey, fully opaque)
        if color is not None:
            if len(color) != 4:
                raise ValueError("Color must be a list of 4 floats [r, g, b, a]")
            m.color.r, m.color.g, m.color.b, m.color.a = color

        markers.markers.append(m)

    return markers


def pub_0066_wall_rviz(cleanup=False):
    pub = rospy.Publisher("walls", MarkerArray, queue_size=1, latch=True)  # latch is important

    if cleanup:
        m = Marker()
        m.action = Marker.DELETEALL

        markers = MarkerArray()
        markers.markers.append(m)

        pub.publish(markers)
        rospy.loginfo("Wall markers deleted on topic 'walls'.")
        return

    # ---- Corner points of the walls (m) ----
    pts = [(-2.2, -2.9),
           (3.9, -2.9),
           (3.9, 3.4),
           (-3.4, 3.4),
           (-3.4, -0.4),
           (-2.2, -0.4), ]
    # Close the polygon by repeating the first point
    pts.append(pts[0])
    # ----------------------------------------

    pub.publish(create_wall_markers(pts, thickness=0.01, height=2.0, color=[22 / 255, 97 / 255, 171 / 255, 0.2]))
    # color: DIAN QING
    rospy.loginfo("Wall markers published on topic 'walls'. Open RViz and add a 'Marker' display.")


def create_hand_markers(poses,
                        mesh_resource="package://aerial_robot_planning/meshes/plastic_hand_9cm_wide.dae",
                        frame_id="world",
                        ns="hand_mesh",
                        scale=(1.0, 1.0, 1.0)):
    """
    Build a MarkerArray that places one mesh for every pose in *poses*.

    poses         : iterable of (x, y, z, roll_deg, pitch_deg, yaw_deg)
    mesh_resource : URI to the DAE mesh (package:// or file://)
    frame_id      : TF frame for RViz
    ns            : marker namespace
    scale         : (sx, sy, sz) scale factors for the mesh
    """
    markers = MarkerArray()

    for idx, (x, y, z, r_deg, p_deg, y_deg) in enumerate(poses):
        # Convert Euler angles (degrees) to quaternion (xyzw scalar-last)
        quat_xyzw = tf.transformations.quaternion_from_euler(
            math.radians(r_deg),
            math.radians(p_deg),
            math.radians(y_deg)
        )

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = idx  # unique inside this namespace
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD

        # Pose
        m.pose.position = Point(x, y, z)
        m.pose.orientation = Quaternion(*quat_xyzw)

        # Scale (keep original units if scale = (1,1,1))
        m.scale.x, m.scale.y, m.scale.z = scale

        # Use embedded material/texture if present
        m.mesh_resource = mesh_resource
        m.mesh_use_embedded_materials = True

        # If the mesh has no embedded colour, uncomment the next line:
        # m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.9, 0.9, 1.0

        markers.markers.append(m)

    return markers


def pub_hand_markers_rviz(viz_type):
    pub = rospy.Publisher("hand_markers", MarkerArray, queue_size=1, latch=True)

    if viz_type == 0:  # Cleanup
        m = Marker()
        m.action = Marker.DELETEALL

        markers = MarkerArray()
        markers.markers.append(m)

        pub.publish(markers)
        rospy.loginfo("Wall markers deleted on topic 'walls'.")
        return

    # ---- Hand poses (m, deg) ----
    hand_poses = []
    mesh_path = ""
    if viz_type == 1:
        hand_poses = [
            (1.0, 1.0, 1.0, 0.0, 30.0, 0.0),
            (-1.0, 1.0, 1.5, 0.0, 0.0, 30.0),
            (0.0, 2.0, 1.0, 30.0, 0.0, 0.0),
        ]  # meters, degrees
        mesh_path = "package://aerial_robot_planning/meshes/plastic_hand_9cm_wide.dae"

    if viz_type == 2:
        hand_poses = [
            (1.0, 1.0, 1.0, 0.0, 30.0, 0.0),
            (-1.0, 1.0, 1.5, 0.0, 0.0, 30.0),
            (0.0, 2.0, 1.0, 30.0, 0.0, 0.0),
        ]  # meters, degrees
        mesh_path = "package://aerial_robot_planning/meshes/plastic_hand_16cm_wide.dae"

    marker_array = create_hand_markers(hand_poses, mesh_resource=mesh_path)
    pub.publish(marker_array)
    rospy.loginfo("Hand mesh markers published on /hand_meshes.")
