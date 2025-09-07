'''
 Created by jinjie on 25/03/21.
'''
from abc import ABC, abstractmethod
import rospy

from nav_msgs.msg import Odometry
from util import check_first_data_received, TrackingErrorCalculator


##########################################
# Base Class
##########################################
class MPCPubBase(ABC):
    """
    A base class that handles:
      - Robot name / node namespace
      - Parameter loading (T_pred, T_integ, T_samp)
      - Odom subscription & storing latest state
      - set_ref_traj publisher
      - Timer callback scaffolding (frequency check, set 'finished' flag)
      - Abstract methods for building the MultiDOFJointTrajectory and checking finish conditions
    """

    def __init__(self, robot_name: str, node_name: str, is_calc_rmse=True):
        # Basic config
        self.robot_name = robot_name
        self.node_name = node_name
        self.namespace = rospy.get_namespace().rstrip("/")
        self.is_finished = False  # Flag to indicate trajectory is complete

        # Load NMPC parameters
        try:
            self.T_horizon = rospy.get_param(f"{robot_name}/controller/nmpc/T_horizon")
            self.T_step = rospy.get_param(f"{robot_name}/controller/nmpc/T_step")
            self.T_samp = rospy.get_param(f"{robot_name}/controller/nmpc/T_samp")
            self.N_nmpc = rospy.get_param(f"{robot_name}/controller/nmpc/NN") # in tilt_mt_servo_nmpc_controller.cpp and acados_solver_tilt_qd_servo_dist_mdl.h
            self.nx = rospy.get_param(f"{robot_name}/controller/nmpc/NX")
            self.nu = rospy.get_param(f"{robot_name}/controller/nmpc/NU")
        except KeyError:
            raise KeyError("Parameters for NMPC not found! Ensure the NMPC controller is running!")

        # Store latest odometry here
        self.uav_odom = None
        self.odom_sub = rospy.Subscriber(f"/{robot_name}/uav/cog/odom", Odometry, self._sub_odom_callback)
        check_first_data_received(self, "uav_odom", robot_name)

        # Calculate tracking error
        if is_calc_rmse:
            self.track_err_calc = TrackingErrorCalculator()

        # data type for timer
        self.start_time = float()
        self.ts_pt_pub = float()
        self.tmr_pt_pub = None

    def start_timer(self):
        """
        Note: the timer should be manually after everything is set up.
        :return:
        """
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

        # Start time
        self.start_time = rospy.Time.now().to_sec()

        # Timer for publishing
        self.ts_pt_pub = 0.02  # ~50Hz
        self.tmr_pt_pub = rospy.Timer(
            rospy.Duration.from_sec(self.ts_pt_pub),
            self._timer_callback
        )
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

    def _sub_odom_callback(self, msg: Odometry):
        """Store the latest odometry data."""
        self.uav_odom = msg

    def _timer_callback(self, timer_event: rospy.timer.TimerEvent):
        """Common timer callback that handles frequency checking and calls user-defined steps."""
        # 1) Check frequency
        if (timer_event.last_duration is not None) and (self.ts_pt_pub < timer_event.last_duration):
            rospy.logwarn(
                f"{self.namespace}: Control loop too slow! "
                f"ts_pt_pub: {self.ts_pt_pub * 1000:.3f} ms < "
                f"timer event duration: {timer_event.last_duration * 1000:.3f} ms"
            )

        # 2) Fill the trajectory from a child-class method
        t_has_started = rospy.Time.now().to_sec() - self.start_time
        traj_msg = self.fill_trajectory_points(t_has_started)

        # 2.1) Calculate tracking error
        if hasattr(self, "track_err_calc"):
            err_px, err_py, err_pz, err_roll, err_pitch, err_yaw = self.track_err_calc.update(self.uav_odom, traj_msg)

            # rospy.loginfo_throttle(1, f"{self.namespace}/{self.node_name}: Tracking error: "
            #                           f"pos_err = {err_px:.3f} m, {err_py:.3f} m, {err_pz:.3f} m, "
            #                           f"ang_err = {err_roll:.3f} deg, {err_pitch:.3f} deg, {err_yaw:.3f} deg")

        # 3) Publish
        self.pub_trajectory_points(traj_msg)

        # 4) Check if done from a child-class method
        # is_finished can be also set by other function to quit, so we need to check it first
        if self.is_finished:
            rospy.loginfo(f"{self.namespace}/{self.node_name}: is_finished is set to True!")

            if hasattr(self, "track_err_calc"):
                # Calculate RMSE of tracking error
                pos_rmse_norm, pos_rmse, ang_rmse_norm, ang_rmse = self.track_err_calc.get_rmse_error()

                # rospy.loginfo(f"\033[1;36m{self.namespace}/{self.node_name}: RMSE of tracking error: \n"
                #               f"pos_err_norm = {pos_rmse_norm:.3f} m, \n"
                #               f"pos_err = {pos_rmse[0]:.3f} m, {pos_rmse[1]:.3f} m, {pos_rmse[2]:.3f} m, \n"
                #               f"ang_err_norm = {ang_rmse_norm:.3f} deg, \n"
                #               f"ang_err = {ang_rmse[0]:.3f} deg, {ang_rmse[1]:.3f} deg, {ang_rmse[2]:.3f} deg\033[0m")  # cyan highlight

                self.track_err_calc.reset()

            # Shutdown the timer
            self.tmr_pt_pub.shutdown()

        self.is_finished = self.check_finished(t_has_started)

    @abstractmethod
    def fill_trajectory_points(self, t_elapsed: float):
        """
        Construct and return a MultiDOFJointTrajectory or PredXU for the current time.
        Must be implemented by the child class.
        """
        pass

    @abstractmethod
    def pub_trajectory_points(self, traj_msg):
        """
        Publish the MultiDOFJointTrajectory or PredXU message.
        Must be implemented by the child class.
        """
        pass

    @abstractmethod
    def check_finished(self, t_elapsed: float) -> bool:
        """
        Return True if we are done publishing / have reached the target.
        Must be implemented by the child class.
        """
        pass
