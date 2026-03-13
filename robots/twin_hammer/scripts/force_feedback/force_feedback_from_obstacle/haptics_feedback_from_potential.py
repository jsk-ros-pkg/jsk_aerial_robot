#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import numpy as np
import rospy
import tf.transformations as tf
import math
from geometry_msgs.msg import PoseStamped, Vector3, WrenchStamped

# Path to the potential map (.npz) relative to this script
script_dir = os.path.dirname(os.path.realpath(__file__))
NPZ_PATH = os.path.join(script_dir, "./sdf_potential.npz")

# Publish frequency [Hz]
PUBLISH_RATE = 100.0

# Wrench scale factor
WRENCH_SCALE_FACTOR = 5.5

# Small safety eps (to avoid division by zero)
_EPS = 1e-12


def find_surrounding_indices(grid, val):
    """
    grid : 1D increasing array (xs, ys, or zs)
    val : query coordinate
    returns: i0, i1, t
      where i0 and i1 are indices with grid[i0] <= val <= grid[i1] (i1 = i0+1 normally),
      and t is interpolation factor in [0,1] (if i0 == i1, t=0).
    If val <= grid[0], returns (0,0,0.0); if val >= grid[-1], returns (n-1,n-1,0.0).
    """
    n = len(grid)
    if n == 0:
        raise ValueError("Empty grid")
    if n == 1:
        return 0, 0, 0.0
    # below lower bound
    if val <= grid[0]:
        return 0, 0, 0.0
    # above upper bound
    if val >= grid[-1]:
        return n - 1, n - 1, 0.0
    # find insertion point
    idx = np.searchsorted(grid, val)
    # idx is index of first grid[idx] >= val, so lower is idx-1
    i0 = idx - 1
    i1 = idx
    x0 = grid[i0]
    x1 = grid[i1]
    # avoid division by zero
    denom = (x1 - x0) if abs(x1 - x0) > _EPS else 1.0
    t = (val - x0) / denom
    return i0, i1, float(t)


def trilinear_interp(xs, ys, zs, potential, point):
    """
    Trilinear interpolation of the scalar 'potential' defined on grids xs, ys, zs.
    point: (x,y,z)
    Returns float potential value at point.
    """
    x, y, z = point
    nx, ny, nz = potential.shape
    # find indices and interpolation factors for each axis
    ix0, ix1, tx = find_surrounding_indices(xs, x)
    iy0, iy1, ty = find_surrounding_indices(ys, y)
    iz0, iz1, tz = find_surrounding_indices(zs, z)

    # If any axis is clamped to a single index (at boundaries), handle gracefully.
    # Get the 8 corner values (some may collapse to same index at boundaries).
    # Indices should be integers and within array bounds.
    ix0 = int(np.clip(ix0, 0, nx - 1))
    ix1 = int(np.clip(ix1, 0, nx - 1))
    iy0 = int(np.clip(iy0, 0, ny - 1))
    iy1 = int(np.clip(iy1, 0, ny - 1))
    iz0 = int(np.clip(iz0, 0, nz - 1))
    iz1 = int(np.clip(iz1, 0, nz - 1))

    # corner values
    c000 = float(potential[ix0, iy0, iz0])
    c100 = float(potential[ix1, iy0, iz0])
    c010 = float(potential[ix0, iy1, iz0])
    c110 = float(potential[ix1, iy1, iz0])
    c001 = float(potential[ix0, iy0, iz1])
    c101 = float(potential[ix1, iy0, iz1])
    c011 = float(potential[ix0, iy1, iz1])
    c111 = float(potential[ix1, iy1, iz1])

    # linear interpolate along x
    c00 = c000 * (1 - tx) + c100 * tx
    c10 = c010 * (1 - tx) + c110 * tx
    c01 = c001 * (1 - tx) + c101 * tx
    c11 = c011 * (1 - tx) + c111 * tx

    # along y
    c0 = c00 * (1 - ty) + c10 * ty
    c1 = c01 * (1 - ty) + c11 * ty

    # along z
    c = c0 * (1 - tz) + c1 * tz
    return float(c)


def potential_at_point(xs, ys, zs, potential, point):
    """
    Wrapper that returns interpolated potential at point.
    If the point is outside the grid, it will be clamped to boundary values
    (i.e., nearest boundary value). This preserves a defined behavior.
    """
    return trilinear_interp(xs, ys, zs, potential, point)


def gradient_at_point(xs, ys, zs, potential, point, delta=None):
    """
    Compute gradient by central difference using interpolated potential:
      dV/dx = (V(x+δx) - V(x-δx)) / (2δx)
    delta: optional tuple/list (dx, dy, dz) for finite difference offsets (in meters / units of xs/ys/zs).
           If None, automatically choose small deltas based on grid spacing:
             δ = min(dx_grid, dy_grid, dz_grid) * 0.25
    Returns numpy array [dV/dx, dV/dy, dV/dz]
    """
    # base grid steps (use first adjacent spacing; assumes regular-ish grid)
    dx_grid = xs[1] - xs[0] if len(xs) > 1 else 1.0
    dy_grid = ys[1] - ys[0] if len(ys) > 1 else 1.0
    dz_grid = zs[1] - zs[0] if len(zs) > 1 else 1.0

    if delta is None:
        # choose delta small relative to grid spacing but not too small to cause numeric noise
        base = min(abs(dx_grid), abs(dy_grid), abs(dz_grid))
        # 0.25 * grid spacing is a sensible compromise; you can tune to 0.1 etc.
        delta_val = max(base * 0.25, base * 0.01, 1e-6)
        dx = delta_val
        dy = delta_val
        dz = delta_val
    else:
        dx, dy, dz = delta

    x, y, z = point

    # compute centered differences using interpolated potentials
    v_x_plus = potential_at_point(xs, ys, zs, potential, (x + dx, y, z))
    v_x_minus = potential_at_point(xs, ys, zs, potential, (x - dx, y, z))
    dv_dx = (v_x_plus - v_x_minus) / (2.0 * dx) if abs(dx) > _EPS else 0.0

    v_y_plus = potential_at_point(xs, ys, zs, potential, (x, y + dy, z))
    v_y_minus = potential_at_point(xs, ys, zs, potential, (x, y - dy, z))
    dv_dy = (v_y_plus - v_y_minus) / (2.0 * dy) if abs(dy) > _EPS else 0.0

    v_z_plus = potential_at_point(xs, ys, zs, potential, (x, y, z + dz))
    v_z_minus = potential_at_point(xs, ys, zs, potential, (x, y, z - dz))
    dv_dz = (v_z_plus - v_z_minus) / (2.0 * dz) if abs(dz) > _EPS else 0.0

    return np.array([dv_dx, dv_dy, dv_dz], dtype=float)

def unwrap_angle(prev, current):
  """
  prev, current: rad
  return: unwrapped current
  """
  if prev is None:
    return current
  diff = current - prev
  while diff > math.pi:
    diff -= 2.0 * math.pi
  while diff < -math.pi:
    diff += 2.0 * math.pi
  return prev + diff

def rotate_xy_world_to_device(dx, dy, yaw_rel):
    """
    World XY -> device-local XY
    yaw_rel : device yaw relative to initial yaw [rad]
    """
    c = math.cos(-yaw_rel)
    s = math.sin(-yaw_rel)
    x_local = c * dx - s * dy
    y_local = s * dx + c * dy
    return x_local, y_local



class PotentialGradientNode:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "gimbalrotor")
        rospy.loginfo("Loading potential map from: %s", NPZ_PATH)
        if not os.path.exists(NPZ_PATH):
            rospy.logerr("NPZ file not found. Check relative path.")
            raise FileNotFoundError(NPZ_PATH)

        data = np.load(NPZ_PATH)
        # expected arrays: potential (nx,ny,nz), xs (nx,), ys (ny,), zs (nz,)
        self.potential = data["potential"]
        self.xs = data["xs"]
        self.ys = data["ys"]
        self.zs = data["zs"]
        rospy.loginfo("Potential map loaded successfully: potential.shape=%s, xs=%d, ys=%d, zs=%d",
                      str(self.potential.shape), len(self.xs), len(self.ys), len(self.zs))

        # Optionally we could pre-smooth the potential here if desired (commented out).
        # from scipy.ndimage import gaussian_filter
        # self.potential = gaussian_filter(self.potential, sigma=0.5)

        self.latest_point = None
        self.device_init_yaw = None
        self.device_yaw_unwrapped = None
        mocap_topic = "/" + self.robot_name + "/mocap/pose"
        rospy.loginfo("Subscribing to mocap pose: %s", mocap_topic)
        self.robot_pos_sub = rospy.Subscriber(mocap_topic, PoseStamped, self.robot_pos_cb)
        self.device_pos_sub = rospy.Subscriber("/twin_hammer/mocap/pose", PoseStamped, self.device_pos_cb)
        self.grad_pub = rospy.Publisher("/potential_gradient", Vector3, queue_size=1)
        # keep original wrench topic name (note original had a typo 'haprtics' — preserve or change as needed)
        self.wrench_pub = rospy.Publisher("/twin_hammer/feedback_from_obstacle", WrenchStamped, queue_size=1)

    def robot_pos_cb(self, msg):
        """Store the latest mocap position for gradient computation."""
        self.latest_point = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    def device_pos_cb(self, msg):
        device_orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        roll, pitch, yaw = tf.euler_from_quaternion(device_orientation_q)
        self.device_yaw_unwrapped = unwrap_angle(self.device_yaw_unwrapped, yaw)
        if self.device_init_yaw == None:
            self.device_init_yaw = self.device_yaw_unwrapped

    def run(self):
        """Main loop: compute and publish gradient."""
        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            if self.latest_point is not None:
                try:
                    grad = gradient_at_point(self.xs, self.ys, self.zs, self.potential, self.latest_point)
                except Exception as e:
                    rospy.logerr("Gradient computation failed: %s", str(e))
                    grad = np.array([0.0, 0.0, 0.0])

                # publish Vector3
                grad_msg = Vector3()
                grad_msg.x, grad_msg.y, grad_msg.z = float(grad[0]), float(grad[1]), float(grad[2])
                self.grad_pub.publish(grad_msg)

                # publish wrench (negative gradient as force)
                # wrench_msg = WrenchStamped()
                # wrench_msg.header.stamp = rospy.Time.now()
                # wrench_msg.wrench.force.x = -WRENCH_SCALE_FACTOR * float(grad[0])
                # wrench_msg.wrench.force.y = -WRENCH_SCALE_FACTOR * float(grad[1])
                # wrench_msg.wrench.force.z = -WRENCH_SCALE_FACTOR * float(grad[2])
                # self.wrench_pub.publish(wrench_msg)
                # world force (negative gradient)
                fx_w = float(grad[0])
                fy_w = float(grad[1])
                fz_w = float(grad[2])

                # yaw relative to device initial yaw
                if self.device_yaw_unwrapped is not None and self.device_init_yaw is not None:
                    yaw_rel = self.device_yaw_unwrapped - self.device_init_yaw
                else:
                    yaw_rel = 0.0

                # rotate force into device-local frame (yaw only)
                fx_d, fy_d = rotate_xy_world_to_device(fx_w, fy_w, yaw_rel)

                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = rospy.Time.now()
                wrench_msg.wrench.force.x = -WRENCH_SCALE_FACTOR * fx_d
                wrench_msg.wrench.force.y = -WRENCH_SCALE_FACTOR * fy_d
                wrench_msg.wrench.force.z = -WRENCH_SCALE_FACTOR * fz_w  # z is unchanged
                self.wrench_pub.publish(wrench_msg)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("potential_gradient_node")
    node = PotentialGradientNode()
    node.run()
