#!/usr/bin/env python3
"""World-fixed TCP controller and Gazebo takeoff-to-landing evaluation.

No automatic flight or disturbance is allowed unless /use_sim_time is true and
Gazebo services are present. --control-only exposes explicit enable/disable and
can consume mocap PoseStamped or FAST-LIO Odometry with a calibrated extrinsic.
"""
import argparse
from collections import deque
import csv
import json
import threading
import time
import sys
from pathlib import Path
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Empty, UInt8
from std_srvs.srv import SetBool, SetBoolResponse
from aerial_robot_msgs.msg import FlightNav
sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_world_core import ArmKinematics, pose_matrix, transform


class WorldHold:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.base = None; self.q = None; self.odom = None
        self.pose_stamp = 0.; self.joint_stamp = 0.
        self.pose_wall = 0.; self.joint_wall = 0.
        self.state = None; self.target = None; self.enabled = False
        self.last_command = None; self.saturated = False
        self.pending_poses = deque(maxlen=500)
        self.rng = np.random.default_rng(args.seed)
        if (args.position_noise or args.angle_noise_deg or args.latency) and not rospy.get_param('/use_sim_time', False):
            raise ValueError('Synthetic noise/latency is allowed only in simulation')
        self.robot = args.robot_ns.rstrip('/')
        xml = rospy.get_param(self.robot+'/robot_description')
        self.kin = ArmKinematics(xml, args.base_link, args.tip_link, args.tcp,
                                 args.joint_speed_cap if args.joint_speed_cap > 0 else None)
        self.home = np.array(args.home)
        if np.any(self.home < self.kin.lower) or np.any(self.home > self.kin.upper):
            raise ValueError('Home violates URDF limits')
        self.extrinsic = transform(args.sensor_to_base[:3], Rotation.from_euler('xyz', args.sensor_to_base[3:]).as_matrix())
        self.pub = rospy.Publisher(self.robot+'/joints_ctrl', JointState, queue_size=1)
        self.target_pub = rospy.Publisher('~target', PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber(args.pose_topic or self.robot+'/ground_truth', PoseStamped if args.pose_type == 'pose' else Odometry, self.pose_cb, queue_size=1)
        rospy.Subscriber(self.robot+'/joint_states', JointState, self.joint_cb, queue_size=1)
        rospy.Subscriber(self.robot+'/uav/cog/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(self.robot+'/flight_state', UInt8, self.state_cb, queue_size=1)
        rospy.Service('~set_enabled', SetBool, self.enable_cb)

    def pose_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.args.world_frame:
            rospy.logerr_throttle(5., 'Pose frame %s differs from configured world frame %s', msg.header.frame_id, self.args.world_frame)
            return
        p = msg.pose if self.args.pose_type == 'pose' else msg.pose.pose
        try:
            matrix = pose_matrix(p) @ self.extrinsic
        except ValueError:
            return
        if not np.isfinite(matrix).all():
            return
        matrix[:3,3] += self.rng.normal(0,self.args.position_noise,3)
        matrix[:3,:3] = Rotation.from_rotvec(self.rng.normal(0,np.deg2rad(self.args.angle_noise_deg),3)).as_matrix() @ matrix[:3,:3]
        with self.lock:
            self.pending_poses.append((msg.header.stamp.to_sec(),matrix))
            cutoff=rospy.Time.now().to_sec()-self.args.latency
            while self.pending_poses and self.pending_poses[0][0] <= cutoff:
                self.pose_stamp,self.base=self.pending_poses.popleft()
                self.pose_wall=time.monotonic()

    def joint_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(n in values for n in self.kin.names):
            return
        with self.lock:
            self.q = np.array([values[n] for n in self.kin.names])
            self.joint_stamp = msg.header.stamp.to_sec()
            self.joint_wall = time.monotonic()

    def odom_cb(self, msg):
        self.odom = msg

    def state_cb(self, msg):
        self.state = msg.data

    def fresh(self):
        now = time.monotonic(); stamp = rospy.Time.now().to_sec()
        return (self.base is not None and self.q is not None and np.isfinite(self.q).all()
                and max(now-self.pose_wall, now-self.joint_wall) < 0.3
                and self.pose_stamp > 0 and self.joint_stamp > 0
                and -0.05 < stamp-self.pose_stamp < 0.3
                and -0.05 < stamp-self.joint_stamp < 0.3
                and abs(self.pose_stamp-self.joint_stamp) < 0.08)

    def enable_cb(self, req):
        with self.lock:
            if req.data and (not self.fresh() or self.state != 5):
                return SetBoolResponse(False, 'Fresh pose/joints and HOVER_STATE required')
            if req.data:
                self.latch()
            self.enabled = req.data
        return SetBoolResponse(True, 'latched current world TCP' if req.data else 'disabled')

    def latch(self, target=None):
        self.target = self.base @ self.kin.fk(self.q) if target is None else target.copy()
        self.last_command = self.q.copy()
        msg = PoseStamped(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = self.args.world_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.target[:3, 3]
        q = Rotation.from_matrix(self.target[:3, :3]).as_quat()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.target_pub.publish(msg)

    def command(self, q):
        if not np.isfinite(q).all():
            raise RuntimeError('Nonfinite arm command')
        msg = JointState(); msg.header.stamp = rospy.Time.now()
        msg.name = self.kin.names; msg.position = q.tolist()
        self.pub.publish(msg)

    def tick(self, dt):
        with self.lock:
            if not self.enabled:
                return
            if not self.fresh() or self.state != 5:
                self.enabled = False
                rospy.logerr('World hold disabled: stale inputs or non-hover state; re-enable explicitly')
                return
            commanded, self.singular_values, self.saturated = self.kin.step(self.last_command, self.base, self.target, dt, self.home)
            # Bound command increments too (tracking error must not accumulate jumps).
            commanded = self.last_command + np.clip(commanded-self.last_command, -self.kin.velocity*dt, self.kin.velocity*dt)
            self.command(commanded); self.last_command = commanded


class GazeboEvaluation:
    def __init__(self, hold, args):
        if not rospy.get_param('/use_sim_time', False):
            raise RuntimeError('Automatic evaluation is Gazebo-only; use --control-only on hardware')
        from gazebo_msgs.msg import LinkStates
        from gazebo_msgs.srv import GetLinkState
        self.hold = hold; self.args = args; self.truth = None; self.truth_stamp = 0.
        self.rows = []; self.events = []; self.phase = 'initialize'
        self.output = Path(args.output); self.output.mkdir(parents=True, exist_ok=True)
        self.started = False; self.landed = False
        self.nav = rospy.Publisher(hold.robot+'/uav/nav', FlightNav, queue_size=1)
        self.rpy = rospy.Publisher(hold.robot+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)
        self.teleop = {s: rospy.Publisher(hold.robot+'/teleop_command/'+s, Empty, queue_size=1) for s in ['start','takeoff','land']}
        rospy.wait_for_service('/gazebo/get_link_state', timeout=15)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.truth_cb, queue_size=1)
        # Fixed link-to-tool transform; independent measured Gazebo arm link pose.
        chain = hold.kin.chain('arm_joint4_link1')
        values = dict(zip(hold.kin.names, np.zeros(4))); values['arm_joint5_joint1'] = 0.
        self.truth_offset = np.linalg.inv(hold.kin.evaluate(chain, values)) @ hold.kin.evaluate(hold.kin.tip_chain, values) @ hold.kin.tcp
        self.sway = np.load(args.sway) if args.sway else None

    def truth_cb(self, msg):
        name = self.hold.robot.strip('/')+'::arm_joint4_link1'
        if name in msg.name and hasattr(self, 'truth_offset'):
            self.truth = pose_matrix(msg.pose[msg.name.index(name)]) @ self.truth_offset
            self.truth_stamp = time.monotonic()

    def event(self, phase):
        self.phase = phase
        self.events.append(dict(phase=phase, sim_time=rospy.Time.now().to_sec(), flight_state=self.hold.state))
        rospy.loginfo('Evaluation phase: %s', phase)

    def wait(self, predicate, timeout, reason):
        deadline = time.monotonic()+timeout
        while not rospy.is_shutdown():
            if predicate():
                return
            if time.monotonic() > deadline:
                raise RuntimeError('Timeout: '+reason)
            time.sleep(.02)
        raise RuntimeError('ROS shutdown during '+reason)

    def navigate(self, xyz, yaw=0.):
        msg = FlightNav(); msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME; msg.target = FlightNav.COG
        msg.pos_xy_nav_mode = FlightNav.POS_MODE; msg.pos_z_nav_mode = FlightNav.POS_MODE
        msg.yaw_nav_mode = FlightNav.POS_MODE
        msg.target_pos_x, msg.target_pos_y, msg.target_pos_z = xyz
        msg.target_yaw = yaw; self.nav.publish(msg)

    def rotate(self, roll, pitch):
        msg = Vector3Stamped(); msg.header.stamp = rospy.Time.now()
        msg.vector.x = roll; msg.vector.y = pitch; self.rpy.publish(msg)

    def duration(self, seconds, fn=None):
        start = rospy.Time.now().to_sec(); last = start; wall = time.monotonic()
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec(); t = now-start
            if t >= seconds:
                return
            if time.monotonic()-wall > max(30., seconds*5):
                raise RuntimeError('Simulation stopped or running too slowly')
            if now-last >= 1./self.args.rate:
                if fn:
                    fn(t, min(now-last, .05))
                last = now
            time.sleep(.002)

    def ramp(self, target, seconds=3.):
        initial = self.hold.q.copy()
        self.duration(seconds, lambda t, dt: self.hold.command(initial+(target-initial)*min(1.,t/seconds)))
        self.hold.command(target)

    def disturbance(self, t):
        if self.sway is not None:
            tt = min(t, self.sway['time'][-1])
            # World-frame position residuals and local rotation vector residuals.
            d = np.array([np.interp(tt, self.sway['time'], self.sway['residual'][:, i]) for i in range(6)])
        else:
            d = np.array([.012*np.sin(2*np.pi*.2*t), .012*np.sin(2*np.pi*.17*t), .008*np.sin(2*np.pi*.13*t),
                          .008*np.sin(2*np.pi*.19*t), .008*np.sin(2*np.pi*.14*t), .01*np.sin(2*np.pi*.11*t)])
        # Smooth start/end to avoid pose setpoint steps. These are flight setpoints,
        # NOT guaranteed physical motions: actual base motion is logged separately.
        envelope = min(1., t/2., (self.args.duration-t)/2.)
        return d * max(0., envelope) * self.args.scale

    def sample(self, trial, t, d):
        h = self.hold
        if self.truth is None or time.monotonic()-self.truth_stamp > .3:
            raise RuntimeError('No fresh independent Gazebo TCP truth')
        estimated = h.base @ h.kin.fk(h.q)
        truth = self.truth.copy()
        ep = truth[:3, 3]-h.target[:3, 3]
        er = Rotation.from_matrix(truth[:3, :3] @ h.target[:3, :3].T).as_rotvec()
        eproxy = estimated[:3, 3]-h.target[:3, 3]
        diagnostic = getattr(h.kin, 'last_diagnostics', {}) if h.enabled else {}
        command = h.last_command if h.enabled else h.home
        self.rows.append([trial, t, *ep, np.linalg.norm(ep), *er, np.linalg.norm(er),
                          *h.base[:3, 3], *Rotation.from_matrix(h.base[:3, :3]).as_rotvec(),
                          *h.q, *d, int(h.saturated), *eproxy,
                          int(diagnostic.get('velocity_limited', False)),
                          int(diagnostic.get('joint_limit_clipped', False)),
                          diagnostic.get('minimum_position_singular_value', float('nan')), *command])

    def run(self):
        h = self.hold
        try:
            self.wait(lambda: h.fresh() and h.odom is not None and h.state is not None and self.truth is not None and h.pub.get_num_connections()>0,
                      30, 'pose, joints, Gazebo TCP and publishers')
            if h.state != 0:
                raise RuntimeError('Expected disarmed simulator at start')
            self.event('preflight_sensor_warmup')
            # Gazebo's spinal interface intentionally starts IMU output after 5s.
            # Arming earlier prevents a clean estimator/controller initialization.
            rospy.wait_for_message(h.robot+'/sensor_plugin/imu1/ros_converted', Imu, timeout=25)
            self.duration(3.)  # allow the configured 2s IMU bias calibration to finish
            self.event('preflight_arm_home'); self.ramp(h.home)
            self.event('arm'); self.teleop['start'].publish(Empty()); self.started = True
            self.wait(lambda: h.state == 2, 15, 'ARM_ON')
            self.event('takeoff'); self.teleop['takeoff'].publish(Empty())
            self.wait(lambda: h.state == 5, 90, 'HOVER_STATE')
            p = h.odom.pose.pose.position
            self.center = np.array([p.x,p.y,self.args.height])
            self.yaw = Rotation.from_matrix(h.base[:3,:3]).as_euler('xyz')[2]
            self.navigate(self.center, self.yaw)
            self.event('settle'); self.duration(8.)
            common_target = None
            for trial in ['fixed_joints', 'world_hold']:
                h.enabled = False
                self.event(trial+'_reset'); self.navigate(self.center,self.yaw); self.rotate(0,0)
                self.ramp(h.home); self.duration(5.)
                self.wait(lambda: h.fresh(), 5, 'fresh controller inputs')
                h.latch(common_target)
                if common_target is None:
                    common_target = h.target.copy()
                h.enabled = trial == 'world_hold'
                self.event(trial)
                def tick(t, dt):
                    if h.state != 5 or not h.fresh():
                        raise RuntimeError('Flight left hover or observations stale')
                    if h.base[2,3] < .4 or np.linalg.norm(h.base[:2,3]-self.center[:2]) > .7:
                        raise RuntimeError('Base exceeded evaluation envelope')
                    d = self.disturbance(t)
                    rpy=(Rotation.from_euler('z',self.yaw)*Rotation.from_rotvec(d[3:])).as_euler('xyz')
                    self.navigate(self.center+d[:3], rpy[2]); self.rotate(rpy[0],rpy[1])
                    if h.enabled:
                        h.tick(dt)
                    else:
                        h.command(h.home)
                    self.sample(trial,t,d)
                self.duration(self.args.duration,tick)
            h.enabled = False
            self.event('recover'); self.navigate(self.center,self.yaw); self.rotate(0,0)
            self.ramp(h.home); self.duration(3.)
            self.event('land'); self.teleop['land'].publish(Empty())
            self.wait(lambda: h.state == 0, 60, 'landed and disarmed')
            self.landed = True; self.event('complete')
        finally:
            h.enabled = False
            if self.started and not self.landed:
                self.rotate(0,0); self.teleop['land'].publish(Empty())
                self.event('abort_land_requested')
                try:
                    self.wait(lambda: h.state == 0, 45, 'abort landing')
                    self.landed = True
                except RuntimeError as error:
                    rospy.logerr(str(error))
            self.save()

    def save(self):
        names = ['trial','time_s','ex_m','ey_m','ez_m','position_error_m','erx_rad','ery_rad','erz_rad','orientation_error_rad',
                 'base_x','base_y','base_z','base_rx','base_ry','base_rz','q1','q2','q3','q4',
                 'command_dx','command_dy','command_dz','command_rx','command_ry','command_rz','saturated',
                 'proxy_ex','proxy_ey','proxy_ez','velocity_limited','joint_limit_clipped','minimum_position_singular_value',
                 'command_q1','command_q2','command_q3','command_q4']
        with (self.output/'samples.csv').open('w') as f:
            writer = csv.writer(f); writer.writerow(names); writer.writerows(self.rows)
        result = dict(completed=self.phase=='complete', landed=self.landed, events=self.events, configuration=vars(self.args),
                      target_world=self.hold.target.tolist() if self.hold.target is not None else None,
                      truth='Gazebo arm_joint4_link1 pose plus fixed URDF transform to configured TCP',
                      disturbance='take9 waveform used as flight setpoint offsets, not exact physical replay',
                      arm_limits=dict(names=self.hold.kin.names,velocity_rad_s=self.hold.kin.velocity.tolist(),
                                      effort_Nm=self.hold.kin.effort.tolist(),source='URDF XL430-W250 at 12 V'), trials={})
        for trial in ['fixed_joints','world_hold']:
            rows = np.array([row[1:] for row in self.rows if row[0]==trial],dtype=float)
            if not len(rows):
                continue
            result['trials'][trial] = dict(samples=len(rows), position_rms_m=float(np.sqrt(np.mean(rows[:,4]**2))),
                position_p95_m=float(np.percentile(rows[:,4],95)), position_max_m=float(rows[:,4].max()),
                orientation_rms_deg=float(np.rad2deg(np.sqrt(np.mean(rows[:,8]**2)))),
                base_position_std_m=np.std(rows[:,9:12],axis=0).tolist(),
                base_rotation_std_deg=np.rad2deg(np.std(rows[:,12:15],axis=0)).tolist(),
                saturation_fraction=float(np.mean(rows[:,25])))
            result['trials'][trial]['fk_truth_position_disagreement_rms_m'] = float(np.sqrt(np.mean(np.sum((rows[:,26:29]-rows[:,1:4])**2,axis=1))))
            result['trials'][trial]['velocity_limited_fraction'] = float(np.mean(rows[:,29]))
            result['trials'][trial]['joint_limit_clipped_fraction'] = float(np.mean(rows[:,30]))
            result['trials'][trial]['joint_tracking_rms_rad'] = np.sqrt(np.mean((rows[:,15:19]-rows[:,32:36])**2,axis=0)).tolist()
        (self.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        if self.rows:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(2,1,figsize=(10,6),sharex=True)
            for trial in result['trials']:
                rows=np.array([r[1:] for r in self.rows if r[0]==trial],float)
                axes[0].plot(rows[:,0],rows[:,4]*1000,label=trial)
                axes[1].plot(rows[:,0],np.rad2deg(rows[:,8]),label=trial)
            axes[0].set_ylabel('TCP position error [mm]'); axes[1].set_ylabel('TCP orientation error [deg]')
            axes[1].set_xlabel('trial time [s]')
            for ax in axes:
                ax.grid(True); ax.legend()
            fig.tight_layout(); fig.savefig(self.output/'comparison.png'); plt.close(fig)
        print(json.dumps(result,indent=2),flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--control-only',action='store_true')
    p.add_argument('--robot-ns',default='/gimbalrotor')
    p.add_argument('--pose-topic',default='')
    p.add_argument('--pose-type',choices=['odom','pose'],default='odom')
    p.add_argument('--base-link',default='fc'); p.add_argument('--tip-link',default='arm_gripper_base_link')
    p.add_argument('--world-frame',default='world')
    p.add_argument('--tcp',nargs=3,type=float,default=[0,0,.047])
    p.add_argument('--sensor-to-base',nargs=6,type=float,default=[0]*6,metavar='XYZ_RPY')
    p.add_argument('--home',nargs=4,type=float,default=[0.,-.6,.9,-.3])
    p.add_argument('--height',type=float,default=1.2); p.add_argument('--duration',type=float,default=10.)
    p.add_argument('--rate',type=float,default=30.); p.add_argument('--sway',default='')
    p.add_argument('--scale',type=float,default=1.); p.add_argument('--output',default='/tmp/arm_world_eval')
    p.add_argument('--position-noise',type=float,default=0.,help='Simulation observation noise standard deviation [m]')
    p.add_argument('--angle-noise-deg',type=float,default=0.,help='Simulation observation rotation-vector noise std [deg]')
    p.add_argument('--latency',type=float,default=0.,help='Simulation observation delay [s], must be below freshness/skew limits')
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--joint-speed-cap',type=float,default=0.,help='Additional joint speed cap [rad/s]; 0 uses URDF limits')
    args=p.parse_args(rospy.myargv()[1:])
    if args.rate <= 0 or args.duration <= 0 or args.scale < 0 or min(args.position_noise,args.angle_noise_deg,args.latency,args.joint_speed_cap) < 0:
        p.error('rate/duration must be positive; scale/noise/latency must be nonnegative')
    rospy.init_node('arm_world_hold')
    deadline=time.monotonic()+30.
    while not rospy.has_param(args.robot_ns.rstrip('/')+'/robot_description'):
        if rospy.is_shutdown() or time.monotonic()>deadline:
            raise RuntimeError('Timed out waiting for robot_description')
        time.sleep(.1)
    h=WorldHold(args)
    if args.control_only:
        last=rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            now=rospy.Time.now().to_sec()
            if now-last >= 1./args.rate:
                h.tick(min(now-last,.05)); last=now
            time.sleep(.002)
    else:
        GazeboEvaluation(h,args).run()


if __name__ == '__main__':
    main()
