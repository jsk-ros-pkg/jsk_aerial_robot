import os
import yaml
import rospkg
import numpy as np
from torch.utils.data import Dataset
from config.configurations import ModelFitConfig, MLPConfig
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
                save_file_path,
                save_file_name,
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
        # TODO CAREFUL: This error is not always linear -> q_err = q_1 * q_2
        y = (state_out - state_prop) / np.expand_dims(dt, 1)
        # =============================================================

        # Store data
        self.state_raw = state_raw
        self.state_in = state_in
        self.state_out = state_out
        self.state_prop = state_prop
        self.control = control
        self.dt = dt

        # Apply control averaging if specified
        if ModelFitConfig.control_averaging:
            print("[DATASET] Averaging input control inputs to a single dimension.")
            if [0, 1, 2, 3] == u_feats:
                thrust_cmd_avg = np.mean(control[:, u_feats], axis=1, keepdims=True)
                control_in = thrust_cmd_avg
            elif [0, 1, 2, 3, 4, 5, 6, 7] == u_feats:
                thrust_cmd_avg = np.mean(control[:, [0, 1, 2, 3]], axis=1, keepdims=True)
                alpha_cmd_avg = np.mean(control[:, [4, 5, 6, 7]], axis=1, keepdims=True)
                control_in = np.concatenate((thrust_cmd_avg, alpha_cmd_avg), axis=1)
            else:
                raise ValueError("Control averaging only implemented for full 4 or 8 motor commands.")
        else:
            control_in = control[:, u_feats]

        # Moving average filter
        # Note: Apply after computing residual dynamics to have more significant smoothing effect
        # If applied before, the effectiveness of the smoothing is drastically reduced
        if ModelFitConfig.use_moving_average_filter:
            print("[DATASET] Applying moving average filter to network input and labels.")
            state_in = moving_average_filter(state_in, window_size=ModelFitConfig.window_size)
            control_in = moving_average_filter(control_in, window_size=ModelFitConfig.window_size)
            y = moving_average_filter(y, window_size=ModelFitConfig.window_size)
        if ModelFitConfig.use_low_pass_filter:
            print("[DATASET] Applying low-pass filter to network input and labels.")

            # Sampling frequency
            fs = 1.0 / np.mean(dt)

            # Cutoff frequencies for different features
            cutoff_pos = 1.0  # 0.8
            cutoff_vel = 1.0  # 0.8
            cutoff_quat = 1.0  # 0.8
            cutoff_angular_vel = 1.0  # 0.8
            cutoff_thrust = 1.0
            cutoff_servo = 1.0  # 0.3
            cutoff_acc = 0.1  # 0.8  # Labels

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
                    control_in[:, 0] = low_pass_filter(control_in[:, 0], cutoff=cutoff_thrust, fs=fs)
                if {4, 5, 6, 7}.issubset(set(u_feats)):
                    # Servo angle
                    control_in[:, 1] = low_pass_filter(control_in[:, 1], cutoff=cutoff_servo, fs=fs)
            else:
                if {0, 1, 2, 3}.issubset(set(u_feats)):
                    # Thrust
                    for dim in [0, 1, 2, 3]:
                        control_in[:, dim] = low_pass_filter(control_in[:, dim], cutoff=cutoff_thrust, fs=fs)
                if {4, 5, 6, 7}.issubset(set(u_feats)):
                    # Servo angle
                    for dim in [4, 5, 6, 7]:
                        control_in[:, dim] = low_pass_filter(control_in[:, dim], cutoff=cutoff_servo, fs=fs)

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
        self.x = np.concatenate((state_in[:, state_feats], control_in), axis=1, dtype=np.float32)
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
