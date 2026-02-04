import os
import yaml
import rospkg
import numpy as np
from torch.utils.data import Dataset
from config.configurations import ModelFitConfig, MLPConfig, EnvConfig
from utils.statistics_utils import prune_dataset
from utils.geometry_utils import v_dot_q, quaternion_inverse
from utils.data_utils import undo_jsonify
from utils.visualization_utils import plot_dataset
from utils.filter_utils import moving_average_filter, low_pass_filter

import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from neural_controller import NeuralMPC
model_options = EnvConfig.model_options
model_options["only_use_nominal"] = True
neural_mpc = NeuralMPC(
    model_options,
    EnvConfig.solver_options,
    EnvConfig.sim_options,
    EnvConfig.run_options,
)
T_samp = neural_mpc.params["T_samp"]
T_step = neural_mpc.params["T_step"]


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
        if ModelFitConfig.prop_long_horizon:
            state_prop = undo_jsonify(self.df["state_prop_long"].to_numpy())
        else:
            state_prop = undo_jsonify(self.df["state_prop_short"].to_numpy())
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
        
        if ModelFitConfig.prop_long_horizon:
            # Shift output to correspond to state_prop after T_step seconds
            timestamp = self.df["timestamp"].to_numpy()
            next_timestamp = timestamp + T_step
            next_idx = []
            for t_next in next_timestamp:
                idx_closest = (np.abs(timestamp - t_next)).argmin()
                next_idx.append(idx_closest)
            next_idx = np.array(next_idx)
            state_out_revised = state_out[next_idx, :]

            state_out_revised = state_out_revised[:-10, :]

        # =============================================================
        # Compute residual dynamics of actual state and predicted (or "propagated") state
        if ModelFitConfig.prop_long_horizon:
            y = (state_out - state_prop) / T_step
            # y = np.zeros_like(state_in)
            # for t in range(state_in.shape[0]):
            #     int_prop = state_prop_trajs.shape[1]  # Number of steps for each propagation
            #     y_t = np.zeros((int_prop, state_in.shape[1]))
            #     for n in range(int_prop):
            #         next_timestamp = timestamp[t] + (n+1) * T_samp
            #         next_idx = (np.abs(timestamp - next_timestamp)).argmin()
            #         state_out_n = state_out[next_idx, :]
            #         state_prop_n = state_prop_trajs[t, n, :]
            #         y_t[n, :] = (state_out_n - state_prop_n) / ((n+1) * T_samp)
                
                # Compute weighted average with linear weights (first entry weighted more than last)
                # weights = np.linspace(int_prop, 1, int_prop)  # Linearly weigh closer steps more
                # y[t, :] = np.average(y_t, axis=0, weights=weights)
                # y[t, :] = np.average(y_t, axis=0)
        else:
            y = (state_out - state_prop) / np.expand_dims(dt, 1)
            # y = (state_out - state_prop) / T_samp

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

        # Data filtering
        # NOTE: Apply after computing residual dynamics to have more significant smoothing effect
        # If applied before, the effectiveness of the smoothing is drastically reduced
        if ModelFitConfig.use_moving_average_filter:
            print(f"[DATASET] Applying moving average filter with window size {ModelFitConfig.window_size} to network input and labels.")
            state_in = moving_average_filter(state_in, window_size=ModelFitConfig.window_size)
            y = moving_average_filter(y, window_size=ModelFitConfig.window_size)
            if ModelFitConfig.control_filtering:
                control = moving_average_filter(control, window_size=ModelFitConfig.window_size)
            if ModelFitConfig.plot_dataset:
                self.state_in_filtered = state_in.copy()
                self.control_filtered = control.copy()
                self.y_filtered = y[:, y_reg_dims].copy()
        if ModelFitConfig.use_low_pass_filter:
            # Sampling frequency
            fs = 1.0 / np.mean(dt)
            # Cutoff frequencies
            # cutoff_pos = 0.8
            # cutoff_vel = 0.8
            # cutoff_quat =  0.8
            # cutoff_angular_vel = 0.8
            # cutoff_thrust = 1.0
            # cutoff_servo = 0.3
            cutoff_input = ModelFitConfig.low_pass_filter_cutoff_input
            cutoff_acc = ModelFitConfig.low_pass_filter_cutoff_label
            print(f"[DATASET] Applying low-pass filter with cutoff frequency {cutoff_input} to network input and "
                  f"labels with cutoff frequency {cutoff_acc}.")

            for dim in range(state_in.shape[1]):
                state_in[:, dim] = low_pass_filter(state_in[:, dim], cutoff=cutoff_input, fs=fs)
            for dim in range(control.shape[1]):
                control[:, dim] = low_pass_filter(control[:, dim], cutoff=cutoff_input, fs=fs)
            for dim in y_reg_dims:
                y[:, dim] = low_pass_filter(y[:, dim], cutoff=cutoff_acc, fs=fs)

        # Store data
        self.state_raw = state_raw
        self.state_in = state_in
        self.state_out = state_out
        self.state_prop = state_prop
        self.control = control
        self.dt = dt

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
