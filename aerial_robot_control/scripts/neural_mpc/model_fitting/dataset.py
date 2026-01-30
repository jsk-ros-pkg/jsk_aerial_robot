import os
import yaml
import rospkg
import numpy as np
from torch.utils.data import Dataset
from config.configurations import ModelFitConfig, MLPConfig
from sim_environment.forward_prop import init_forward_prop, forward_prop
from utils.statistics_utils import prune_dataset
from utils.geometry_utils import v_dot_q, quaternion_inverse
from utils.data_utils import undo_jsonify
from utils.visualization_utils import plot_dataset
from utils.filter_utils import moving_average_filter, low_pass_filter


class TrajectoryDataset(Dataset):
    """
    Dataset for training neural networks on trajectory data.
    """

    def __init__(
        self,
        dataframe,
        state_feats,
        u_feats,
        y_reg_dims,
        save_file_path=None,
        save_file_name=None,
        mode=None,
    ):
        self.df = dataframe

        self.prepare_data(state_feats, u_feats, y_reg_dims)
        if ModelFitConfig.prune and MLPConfig.delay_horizon == 0:
            # Don't prune when using temporal networks with history since pruning causes incontinuities
            self.prune(
                state_feats,
                y_reg_dims,
                ModelFitConfig.histogram_n_bins,
                ModelFitConfig.histogram_thresh,
                ModelFitConfig.vel_cap,
                ModelFitConfig.plot_dataset,
            )
        if MLPConfig.delay_horizon > 0:
            self.append_history(MLPConfig.delay_horizon, state_feats, u_feats)
        self.calculate_statistics()
        if ModelFitConfig.plot_dataset:
            if not ModelFitConfig.save_plots:
                save_file_path = None
                save_file_name = None
            plot_dataset(
                self.x,
                self.y,
                self.dt,
                self.state_in,
                self.state_out,
                self.state_prop,
                self.control,
                ModelFitConfig.use_moving_average_filter,
                self.state_in_filtered if ModelFitConfig.use_moving_average_filter else None,
                self.control_filtered if ModelFitConfig.use_moving_average_filter else None,
                self.y_raw if ModelFitConfig.use_moving_average_filter else None,
                self.y_filtered if ModelFitConfig.use_moving_average_filter else None,
                save_file_path,
                save_file_name,
                mode,
            )

    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        return self.x[idx], self.y[idx]

    def prepare_data(self, state_feats, u_feats, y_reg_dims):
        state_in = undo_jsonify(self.df["state_in"].to_numpy())
        state_raw = state_in.copy()
        state_out = undo_jsonify(self.df["state_out"].to_numpy())
        state_prop = undo_jsonify(self.df["state_prop"].to_numpy())
        control = undo_jsonify(self.df["control"].to_numpy())
        dt = self.df["dt"].to_numpy()

        # Remove invalid entries (dt = 0)
        invalid = np.where(dt == 0)
        state_in = np.delete(state_in, invalid, axis=0)
        state_out = np.delete(state_out, invalid, axis=0)
        state_prop = np.delete(state_prop, invalid, axis=0)
        control = np.delete(control, invalid, axis=0)
        dt = np.delete(dt, invalid, axis=0)

        # Sanity check
        if (
            state_in.shape != state_out.shape
            or state_in.shape != state_prop.shape
            or state_in.shape[0] != control.shape[0]
        ):
            raise ValueError("Inconsistent shapes in the dataset.")
        
        ########## DEBUG ##########
        #### Filtering
        # NOTE: Filter before generating state_prop
        if ModelFitConfig.use_moving_average_filter:
            print(f"[DATASET] Applying moving average filter with window size {ModelFitConfig.window_size} to network input and labels.")
            state_in = moving_average_filter(state_in, window_size=ModelFitConfig.window_size)
            if ModelFitConfig.control_filtering:
                control = moving_average_filter(control, window_size=ModelFitConfig.window_size)
            if ModelFitConfig.plot_dataset:
                # For plotting only
                self.state_in_filtered = state_in.copy()
                self.control_filtered = control.copy()

        #### Run forward propagation again        
        # Prepare mathematical model for forward propagation
        import sys
        sys.path.append(os.path.dirname(os.path.dirname(__file__)))
        from nmpc.nmpc_tilt_mt.tilt_qd import phys_param_beetle_omni as phys_omni
        from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_servo import NMPCTiltQdServo
        mpc = NMPCTiltQdServo(phys=phys_omni, build=True)  # JUST FOR PARAMS TO WRITE IN METADATA! AND TO GET T_SAMP
        
        # Define nominal model
        dynamics_forward_prop, state_forward_prop, u_forward_prop = init_forward_prop(mpc)

        # - Simulation parameters
        # Technically can be chosen arbitrary but it makes a lot of sense to choose it as the frequency of the controller (i.e., T_step)
        # since we want to replicate how the model performs inside the MPC prediction scheme. In acados the solver is set up with total horion time
        # solver_options.tf and the number of steps solver_options.N, leading to a step size of T_step = tf / N. But we define the step size and
        # the prediction horizon in the robots ROS package under config and compute the corresponding number of steps. 
        # With T_step = 0.1s and T_horizon = 2.0s, we have N = 20 steps.
        # The propagation time step (meaning how fine we discretize the continuous dynamics) can be chosen independently but it also makes 
        # sense to choose T_samp ≈ dt (= 0.01s)
        T_step = mpc.params["T_step"]  # 0.1s
        T_prop = mpc.params["T_samp"]  # 0.01s
        print("Forward propagation step size: ", T_prop)
        print("Average time step dt in dataset: ", np.mean(dt))
        print("Running forward prop...")
        # - Run forward propagation with nominal model based with state_in and control
        # takeoff_steps = 200  # 2s = 200 * 0.01s (T_samp)
        # skip = False
        for t in range(state_in.shape[0]):
            # for t_rec_start in data["recording_start_idx"]:
            #     if t_rec_start <= t and t <= t_rec_start + takeoff_steps:
            #         # Set state_prop to state_in at start of each recording to avoid learning from takeoff dynamics
            #         state_prop = np.append(state_prop, state_in[t, :][np.newaxis, :], axis=0)
            #         skip = True
            #         break
            # if skip:
            #     skip = False
            #     continue
            # Get current state and control
            state_curr = state_in[t, :]
            u_cmd = control[t, :]

            # Propagate forward
            state_prop_curr = forward_prop(
                dynamics_forward_prop,
                state_forward_prop,
                u_forward_prop,
                state_curr[np.newaxis, :],
                u_cmd[np.newaxis, :],
                T_horizon=T_step,
                T_step=T_prop,
                num_stages=4,
            )
            state_prop_curr = state_prop_curr[-1, :]  # Get last predicted state
            state_prop = np.append(state_prop, state_prop_curr[np.newaxis, :], axis=0)

        # Shift state_prop by one timestep to match timestamps of state_out
        state_prop = state_prop[1:, :]
        # NOTE: state_prop is now the propagated state after dt seconds, meaning the state of the system
        # after T_samp (= dt) seconds. By this time the last input command has been applied for dt seconds
        # and the state has changed according to the dynamics. This means state_prop now has the same
        # timestamps as state_out.


        #### Choose which state to use to compare with propagated state
        # This should be the state at time t+T_step, meaning the actual state after applying the last input command for T_step seconds.
        # Find time index corresponding to t+T_step for each sample
        timestamps = self.df["timestamp"].to_numpy()
        next_timestamps = timestamps + T_step
        next_idx = []
        for t_next in next_timestamps:
            idx_closest = (np.abs(timestamps - t_next)).argmin()
            next_idx.append(idx_closest)
        next_idx = np.array(next_idx)
        state_out_new = state_out[next_idx, :]
        state_out = state_out_new



        # 1. Compensate for delay in measurements by
        # cutoff_dt = 0.014
        # invalid_idx = np.where(dt>cutoff_dt)
        # timestamps_comp = timestamps.copy()
        # timestamps_comp[invalid_idx[0] + 1] = timestamps_comp[invalid_idx] + 0.01  (desired dt)

        # recompute dt and iterate?! -> not really necessary since the corrected timestamps will be finely spaced anyway

        # plot:
            # state_idx = 4
            # plt.figure()
            # plt.plot(timestamps_comp, state_in[:,state_idx], marker="x")
            # plt.plot(timestamps[..., np.where(dt>cutoff_dt)], state_in[np.where(dt>cutoff_dt),state_idx], marker="x", color="red")


        # 2. Get correct index for state_out! -> should all be 1 apart from each other! Check with assert
        # next_idx[1:] - next_idx[:-1]

        # plot:
            # plt.figure()
            # plt.scatter(next_idx[1:] - next_idx[:-1])
            # plt.hist(next_idx[1:] - next_idx[:-1])


        # 3. warum hat label am anfang so einen offset in ax ay
        # 4. Make sure that the restructuring of forward prop method is used correctly everywhere
        # 4. Sichergehen dass state_prop und state_out richtig sind -> es kann ja nicht sein dass das label basically only noise ist!!
        ##########################


        # Transform velocity to body frame
        def velocity_mapping(state_sequence):
            p_traj = state_sequence[:, :3]
            v_w_traj = state_sequence[:, 3:6]
            q_traj = state_sequence[:, 6:10]
            other_traj = state_sequence[:, 10:]  # w, a_s, f_s, etc.

            v_b_traj = np.empty_like(v_w_traj)
            for t in range(len(v_w_traj)):
                v_b_traj[t, :] = v_dot_q(v_w_traj[t, :], quaternion_inverse(q_traj[t, :]))
            return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)

        if ModelFitConfig.input_transform:
            state_in = velocity_mapping(state_in)
        else:
            # Don't transform input but let network learn in world frame directly
            pass
        if ModelFitConfig.label_transform:
            state_prop = velocity_mapping(state_prop)
            state_out = velocity_mapping(state_out)
        else:
            # Don't transform labels but let network predict in world frame directly
            pass

        # =============================================================
        # Compute residual dynamics of actual state and predicted (or "propagated") state
        y = (state_out - state_prop) / np.expand_dims(dt, 1)

        # Nonlinear quaternion error computation
        # NOTE: The difference between two quaternions can be computed by q_diff = q2 quaternion-multiply inverse(q1)
        # where: inverse(q1) = conjugate(q1) / abs(q1)
        # and:   conjugate( quaternion(re, i, j, k) ) = quaternion(re, -i, -j, -k)
        # and:   abs(q1) = 1 for unit quaternions.
        # therefore: q_diff =   w1*w2 - x1*x2 - y1*y2 - z1*z2
        #                     i(w1*x2 + x1*w2 + y1*z2 - z1*y2)
        #                     j(w1*y2 - x1*z2 + y1*w2 + z1*x2)
        #                     k(w1*z2 + x1*y2 - y1*x2 + z1*w2)
        # NOTE: Here q1 is state_prop and q2 is state_out because we want to compute the error from propagated to actual
        qw_prop = state_prop[:, 6]
        qx_prop = state_prop[:, 7]
        qy_prop = state_prop[:, 8]
        qz_prop = state_prop[:, 9]
        qw_out = state_out[:, 6]
        qx_out = state_out[:, 7]
        qy_out = state_out[:, 8]
        qz_out = state_out[:, 9]

        qe_w = qw_out * qw_prop - qx_out * qx_prop - qy_out * qy_prop - qz_out * qz_prop
        qe_x = qw_out * qx_prop + qx_out * qw_prop + qy_out * qz_prop - qz_out * qy_prop
        qe_y = qw_out * qy_prop - qx_out * qz_prop + qy_out * qw_prop + qz_out * qx_prop
        qe_z = qw_out * qz_prop + qx_out * qy_prop - qy_out * qx_prop + qz_out * qw_prop
        q_e = np.stack((qe_w, qe_x, qe_y, qe_z), axis=1)
        y[:, 6:10] = q_e / np.expand_dims(dt, 1)
        self.y_raw = y[:, y_reg_dims].copy()
        # =============================================================

        # Store data
        self.state_raw = state_raw
        self.state_in = state_in
        self.state_out = state_out
        self.state_prop = state_prop
        self.control = control
        self.dt = dt

        # Moving average filter
        # NOTE: Apply after computing residual dynamics to have more significant smoothing effect
        # If applied before, the effectiveness of the smoothing is drastically reduced
        if ModelFitConfig.use_moving_average_filter:
            y = moving_average_filter(y, window_size=ModelFitConfig.window_size)
            if ModelFitConfig.plot_dataset:
                self.y_filtered = y[:, y_reg_dims].copy()
        if ModelFitConfig.use_low_pass_filter:
            print(f"[DATASET] Applying low-pass filter with cutoff frequency {ModelFitConfig.low_pass_filter_cutoff_input} to network input and labels.")

            # Sampling frequency
            fs = 1.0 / np.mean(dt)

            # Cutoff frequencies for different features
            cutoff_pos = ModelFitConfig.low_pass_filter_cutoff_input  # 0.8
            cutoff_vel = ModelFitConfig.low_pass_filter_cutoff_input  #  0.8
            cutoff_quat = ModelFitConfig.low_pass_filter_cutoff_input  #  0.8
            cutoff_angular_vel = ModelFitConfig.low_pass_filter_cutoff_input  # 0.8
            cutoff_thrust = ModelFitConfig.low_pass_filter_cutoff_input  # 1.0
            cutoff_servo = ModelFitConfig.low_pass_filter_cutoff_input  # 0.3

            cutoff_acc = ModelFitConfig.low_pass_filter_cutoff_label

            # State features
            if {2}.issubset(set(state_feats)):
                # Position
                state_in[:, 2] = low_pass_filter(state_in[:, 2], cutoff=cutoff_pos, fs=fs)
            if {3, 4, 5}.issubset(set(state_feats)):
                # Velocity
                for dim in [3, 4, 5]:
                    state_in[:, dim] = low_pass_filter(state_in[:, dim], cutoff=cutoff_vel, fs=fs)
            if {6, 7, 8, 9}.issubset(set(state_feats)):
                # Quaternion
                for dim in [6, 7, 8, 9]:
                    state_in[:, dim] = low_pass_filter(state_in[:, dim], cutoff=cutoff_quat, fs=fs)
            if {10, 11, 12}.issubset(set(state_feats)):
                # Angular velocity
                for dim in [10, 11, 12]:
                    state_in[:, dim] = low_pass_filter(state_in[:, dim], cutoff=cutoff_angular_vel, fs=fs)

            # Input features
            if ModelFitConfig.control_averaging:
                # 4 rotor/servo signals are combined into one
                if {0, 1, 2, 3}.issubset(set(u_feats)):
                    # Thrust
                    control[:, 0] = low_pass_filter(control[:, 0], cutoff=cutoff_thrust, fs=fs)
                if {4, 5, 6, 7}.issubset(set(u_feats)):
                    # Servo angle
                    control[:, 1] = low_pass_filter(control[:, 1], cutoff=cutoff_servo, fs=fs)
            else:
                if {0, 1, 2, 3}.issubset(set(u_feats)):
                    # Thrust
                    for dim in [0, 1, 2, 3]:
                        control[:, dim] = low_pass_filter(control[:, dim], cutoff=cutoff_thrust, fs=fs)
                if {4, 5, 6, 7}.issubset(set(u_feats)):
                    # Servo angle
                    for dim in [4, 5, 6, 7]:
                        control[:, dim] = low_pass_filter(control[:, dim], cutoff=cutoff_servo, fs=fs)

            # Labels
            if {3}.issubset(set(y_reg_dims)):
                # vx
                y[:, 3] = low_pass_filter(y[:, 3], cutoff=cutoff_acc, fs=fs)
            if {4}.issubset(set(y_reg_dims)):
                # vy
                y[:, 4] = low_pass_filter(y[:, 4], cutoff=cutoff_acc, fs=fs)
            if {5}.issubset(set(y_reg_dims)):
                # vz (Note: Sometimes only vz is used instead of full velocity)
                y[:, 5] = low_pass_filter(y[:, 5], cutoff=cutoff_acc, fs=fs)

        # Store network input
        self.x = np.concatenate((state_in[:, state_feats], control[:, u_feats]), axis=1, dtype=np.float32)
        # Store labels
        self.y = y[:, y_reg_dims].astype(np.float32)

    def prune(self, state_feats, y_reg_dims, histogram_n_bins, histogram_thresh, vel_cap=None, plot=False):
        """
        Prune the dataset to remove samples with outliers in velocity.
        """
        vel_idx = np.array([3, 4, 5])
        v_z_idx = np.array([5])
        if set(vel_idx).issubset(set(state_feats)) and set(vel_idx).issubset(set(y_reg_dims)):
            x_vel_idx_real = np.where(np.in1d(state_feats, vel_idx))[0]
            y_vel_idx_real = np.where(np.in1d(y_reg_dims, vel_idx))[0]
        elif set(vel_idx).issubset(set(state_feats)) and set(v_z_idx).issubset(set(y_reg_dims)):
            # Only vz in labels
            x_vel_idx_real = np.where(np.in1d(state_feats, vel_idx))[0]
            y_vel_idx_real = np.where(np.in1d(y_reg_dims, v_z_idx))[0]
        else:
            print("[PRUNING] Velocity features not part of input AND output, skipping pruning.")
            # Pruning only works right now if the velocity features are part of the input and output
            return

        # Prune noisy data
        if histogram_n_bins is not None and histogram_thresh is not None:
            labels = ["vx", "vy", "vz"]

            self.pruned_idx = prune_dataset(
                self.x[:, x_vel_idx_real],
                self.y[:, y_vel_idx_real],
                vel_cap,
                histogram_n_bins,
                histogram_thresh,
                plot=plot,
                labels=labels,
            )
            self.x = self.x[self.pruned_idx]
            self.y = self.y[self.pruned_idx]

    def append_history(self, delay, state_feats, u_feats):
        """
        Append previous states and controls to the input features for temporal networks.
        Creates sliding windows of historical data with zero-padding for initial samples.

        :param delay: Number of historical time steps to include
        :param state_feats: List of state feature indices
        """
        n_samples = self.x.shape[0]
        n_state_feats = len(state_feats)
        n_control_feats = len(u_feats)

        # Create new input array with history: current + delay previous steps
        state_history = np.zeros((n_samples, n_state_feats * (delay + 1)), dtype=np.float32)
        control_history = np.zeros((n_samples, n_control_feats * (delay + 1)), dtype=np.float32)

        for i in range(n_samples):
            # Current time step (most recent)
            state_history[i, :n_state_feats] = self.x[i, :n_state_feats]
            control_history[i, :n_control_feats] = self.x[i, n_state_feats:]

            # Historical time steps (delay previous steps)
            for j in range(1, delay + 1):
                state_start_idx = j * n_state_feats
                state_end_idx = (j + 1) * n_state_feats
                control_start_idx = j * n_control_feats
                control_end_idx = (j + 1) * n_control_feats

                if i - j >= 0:
                    # Use actual historical data
                    state_history[i, state_start_idx:state_end_idx] = self.x[i - j, :n_state_feats]
                    control_history[i, control_start_idx:control_end_idx] = self.x[i - j, n_state_feats:]
                else:
                    # Pad with initial value where history doesn't exist
                    state_history[i, state_start_idx:state_end_idx] = self.x[0, :n_state_feats]
                    control_history[i, control_start_idx:control_end_idx] = self.x[0, n_state_feats:]

        # Update the dataset with the new input features
        self.x = np.append(state_history, control_history, axis=1)

    def calculate_statistics(self):
        # Calculate mean and std for normalization
        self.x_mean = np.mean(self.x, axis=0)
        self.x_std = np.std(self.x, axis=0)
        self.y_mean = np.mean(self.y, axis=0)
        self.y_std = np.std(self.y, axis=0)
        # Sanity check since we divide by x_std in normalization
        if np.any(self.x_std == 0):
            raise ValueError("Input features have zero standard deviation, cannot normalize.")


def read_params(mode, method, robot_package, file_name):
    # Read parameters from configuration file in the robot's package
    # 'mode' is either "controller" or "estimation"
    # 'method' is either "nmpc" or "mhe"

    rospack = rospkg.RosPack()
    param_path = os.path.join(rospack.get_path(robot_package), "config", file_name)

    try:
        with open(param_path, "r") as f:
            param_dict = yaml.load(f, Loader=yaml.FullLoader)
        params = param_dict[mode][method]
    except FileNotFoundError:
        raise FileNotFoundError(f"Configuration file {param_path} not found.")
    except KeyError:
        raise KeyError(f"Mode or method not found in configuration file {param_path}.")

    # Compute number of shooting intervals or "steps" (or "nodes") along the horizon
    params["N_steps"] = int(params["T_horizon"] / params["T_step"])

    return params
