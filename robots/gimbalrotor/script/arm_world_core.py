"""ROS-independent URDF kinematics and position-priority arm stabilization."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def transform(xyz=(0, 0, 0), rotation=None):
    out = np.eye(4)
    out[:3, 3] = xyz
    if rotation is not None:
        out[:3, :3] = rotation
    return out


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return transform([p.x, p.y, p.z], Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix())


class ArmKinematics:
    """Chain is extracted from the expanded robot_description, including fixed joints.

    The observation reference need not be an ancestor (fc is a sibling of the arm).
    All joints outside the controlled chain must be fixed; gripper joint is held.
    """
    def __init__(self, xml, base='fc', tip='arm_gripper_base_link', tcp=(0, 0, 0.047), velocity_cap=None):
        root = ET.fromstring(xml)
        self.by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.base_chain = self.chain(base)
        self.tip_chain = self.chain(tip)
        self.names = ['arm_joint%d_joint1' % i for i in range(1, 5)]
        joints = {j.get('name'): j for j in root.findall('joint')}
        self.lower = np.array([float(joints[n].find('limit').get('lower')) for n in self.names]) + 0.05
        self.upper = np.array([float(joints[n].find('limit').get('upper')) for n in self.names]) - 0.05
        self.urdf_velocity = np.array([float(joints[n].find('limit').get('velocity')) for n in self.names])
        self.velocity = self.urdf_velocity.copy() if velocity_cap is None else np.minimum(float(velocity_cap), self.urdf_velocity)
        self.effort = np.array([float(joints[n].find('limit').get('effort')) for n in self.names])
        self.tcp = transform(tcp)

    def chain(self, link):
        chain = []
        while link in self.by_child:
            j = self.by_child[link]
            origin = j.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            axis = j.find('axis')
            a = np.fromstring(axis.get('xyz'), sep=' ') if axis is not None else np.array([1., 0, 0])
            chain.append((j.get('name'), j.get('type'), transform(xyz, Rotation.from_euler('xyz', rpy).as_matrix()), a))
            link = j.find('parent').get('link')
        return chain[::-1]

    @staticmethod
    def evaluate(chain, values):
        out = np.eye(4)
        for name, kind, origin, axis in chain:
            out = out @ origin
            if kind in ('revolute', 'continuous'):
                out = out @ transform(rotation=Rotation.from_rotvec(axis * values[name]).as_matrix())
            elif kind == 'prismatic':
                out = out @ transform(axis * values[name])
        return out

    def fk(self, q):
        values = dict(zip(self.names, q))
        values['arm_joint5_joint1'] = 0.0  # opening is independent of positioning task
        return np.linalg.inv(self.evaluate(self.base_chain, values)) @ self.evaluate(self.tip_chain, values) @ self.tcp

    def jacobian(self, q):
        current = self.fk(q)
        jac = np.zeros((6, len(q)))
        for i in range(len(q)):
            dq = np.zeros(len(q)); dq[i] = 1e-6
            other = self.fk(q + dq)
            jac[:3, i] = (other[:3, 3] - current[:3, 3]) / 1e-6
            jac[3:, i] = Rotation.from_matrix(other[:3, :3] @ current[:3, :3].T).as_rotvec() / 1e-6
        return jac

    def step(self, q, base_world, target_world, dt, posture):
        """3-D position first, orientation only in the position Jacobian nullspace.

        Near singularity a truncated SVD leaves infeasible directions uncommanded.
        Rate and joint limits are enforced after both tasks; residuals are measured.
        """
        desired = np.linalg.inv(base_world) @ target_world
        actual = self.fk(q)
        ep = desired[:3, 3] - actual[:3, 3]
        er = Rotation.from_matrix(desired[:3, :3] @ actual[:3, :3].T).as_rotvec()
        j = self.jacobian(q)
        pinv = np.linalg.pinv(j[:3], rcond=0.03)
        null = np.eye(4) - pinv @ j[:3]
        v = pinv @ (4.0 * ep)
        jr = j[3:] @ null
        # Damped secondary task: cannot sacrifice primary position locally.
        v += null @ jr.T @ np.linalg.solve(jr @ jr.T + 0.04**2 * np.eye(3), 2.0 * er - j[3:] @ v)
        v += 0.05 * null @ (posture - q)
        scale = max(1., float(np.max(np.abs(v) / self.velocity)))
        unconstrained = q + v / scale * dt
        commanded = np.clip(unconstrained, self.lower, self.upper)
        singular = np.linalg.svd(j[:3], compute_uv=False)
        self.last_diagnostics = dict(velocity_limited=scale > 1., velocity_scale=scale,
                                     joint_limit_clipped=bool(np.any(np.abs(commanded-unconstrained)>1e-12)),
                                     minimum_position_singular_value=float(singular[-1]))
        return commanded, singular, self.last_diagnostics['velocity_limited'] or self.last_diagnostics['joint_limit_clipped']
