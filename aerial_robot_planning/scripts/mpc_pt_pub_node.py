"""
 Created by li-jinjie on 24-1-3.
"""

import sys
import os
import argparse

current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)

import rospy
import rospkg
import yaml
import tf_conversions as tf
from trajs import *

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCFull.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)
nmpc_params = param_dict["controller"]["nmpc"]


class MPCPtPubNode:
    def __init__(self, robot_name: str, traj_type: int, loop_num: int) -> None:
        self.robot_name = robot_name
        self.node_name = "mpc_pt_pub_node"
        rospy.init_node(self.node_name, anonymous=False)
        self.namespace = rospy.get_namespace().rstrip("/")

        # Sub --> feedback
        self.uav_odom = Odometry()
        rospy.Subscriber(f"/{robot_name}/uav/cog/odom", Odometry, self.sub_odom_callback)

        # Pub
        self.pub_ref_traj = rospy.Publisher(f"/{robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=5)

        # nmpc and robot related
        self.N_nmpc = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

        # traj
        if traj_type == 0:
            self.traj = SetPointTraj(loop_num)
        elif traj_type == 1:
            self.traj = CircleTraj(loop_num)
        elif traj_type == 2:
            self.traj = LemniscateTraj(loop_num)
        elif traj_type == 3:
            self.traj = LemniscateTrajOmni(loop_num)
        elif traj_type == 4:
            self.traj = PitchRotationTraj(loop_num)
        elif traj_type == 5:
            self.traj = RollRotationTraj(loop_num)
        elif traj_type == 6:
            self.traj = PitchSetPtTraj(loop_num)
        else:
            raise ValueError("Invalid trajectory type!")

        rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory type: {self.traj.__str__()}")

        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

        # Timer
        self.ts_pt_pub = 0.02  # [s]  ~50Hz
        self.tmr_pt_pub = rospy.Timer(rospy.Duration.from_sec(self.ts_pt_pub), self.callback_tmr_pt_pub)
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

    def callback_tmr_pt_pub(self, timer: rospy.timer.TimerEvent):
        """publish the reference points to the controller"""
        # 1. check if the frequency is too slow
        if timer.last_duration is not None and self.ts_pt_pub < timer.last_duration:
            rospy.logwarn(
                f"{self.namespace}: Control is too slow!"
                f"ts_pt_pub: {self.ts_pt_pub * 1000:.3f} ms < ts_one_round: {timer.last_duration * 1000:.3f} ms"
            )

        multi_dof_joint_traj = MultiDOFJointTrajectory()

        # 2. calculate the reference points
        is_ref_different = True

        if isinstance(self.traj, PitchRotationTraj) or isinstance(self.traj, RollRotationTraj):
            is_ref_different = False

        t_has_started = rospy.Time.now().to_sec() - self.start_time
        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()

            if is_ref_different:
                t_pred = i * nmpc_params["T_integ"]  # all future reference points are different
            else:
                t_pred = 0.0

            t_cal = t_pred + t_has_started

            # position
            x, y, z, vx, vy, vz, ax, ay, az = self.traj.get_3d_pt(t_cal)

            # orientation
            # check if there is get_3d_orientation method inside self.traj
            try:
                roll, pitch, yaw, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc = self.traj.get_3d_orientation(t_cal)
            except AttributeError:
                roll, pitch, yaw = 0.0, 0.0, 0.0
                r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
                r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            qx, qy, qz, qw = q

            traj_pt.transforms.append(Transform(translation=Vector3(x, y, z), rotation=Quaternion(qx, qy, qz, qw)))
            traj_pt.velocities.append(Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate)))
            traj_pt.accelerations.append(Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc)))
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        # 3. send the reference points to the controller
        multi_dof_joint_traj.header.stamp = rospy.Time.now()
        multi_dof_joint_traj.header.frame_id = "map"
        self.pub_ref_traj.publish(multi_dof_joint_traj)

        # 4. check if the trajectory is finished
        if self.traj.check_finished(rospy.Time.now().to_sec() - self.start_time):
            rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory finished!")
            rospy.signal_shutdown("Trajectory finished!")
            return

    def sub_odom_callback(self, msg: Odometry):
        self.uav_odom = msg


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MPC Point Trajectory Publisher Node")
    parser.add_argument("robot_name", type=str, help="Robot name, e.g., beetle1, gimbalrotors")
    parser.add_argument(
        "traj_type",
        type=int,
        help="Trajectory type: 0 for set-point, 1 for Circular, 2 for Lemniscate, 3 for Lemniscate omni",
    )
    parser.add_argument("-num", "--loop_num", type=int, default=np.inf, help="Loop number for the trajectory")
    args = parser.parse_args()

    try:
        node = MPCPtPubNode(args.robot_name, args.traj_type, args.loop_num)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
