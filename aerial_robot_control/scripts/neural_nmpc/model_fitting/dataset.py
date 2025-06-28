import numpy as np
from torch.utils.data import Dataset

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.statistic_utils import prune_dataset
from utils.geometry_utils import v_dot_q, quaternion_inverse
from utils.data_utils import undo_jsonify


class TrajectoryDataset(Dataset):
    # TODO dataset pruning based on samples with velocity outliers (idea from RTNMPC)
    # TODO Normalization
    def __init__(self, dataframe, histogram_pruning_n_bins=None, histogram_pruning_thresh=None, vel_cap=None, plot=False):
        self.df = dataframe
        self.prepare_data()
        self.prune(histogram_pruning_n_bins, histogram_pruning_thresh, vel_cap, plot)
        self.calculate_statistics()

    def __len__(self):
        return len(self.x)

    def __getitem__(self, idx):
        # TODO support only a selection of features as input (e.g., only everything except position)
        # TODO support not using control as input for network
        return self.x[idx], self.y[idx]

    def prepare_data(self):
        state_raw = undo_jsonify(self.df['state_in'].to_numpy())
        state_out = undo_jsonify(self.df['state_out'].to_numpy())
        state_pred = undo_jsonify(self.df['state_pred'].to_numpy())
        control = undo_jsonify(self.df['control'].to_numpy())
        # control = undo_jsonify(self.df['input_in'].to_numpy())
        dt = self.df["dt"].to_numpy()

        # Remove invalid entries (dt = 0)
        invalid = np.where(dt == 0)
        state_raw = np.delete(state_raw, invalid, axis=0)
        state_out = np.delete(state_out, invalid, axis=0)
        state_pred = np.delete(state_pred, invalid, axis=0)
        control = np.delete(control, invalid, axis=0)
        dt = np.delete(dt, invalid, axis=0)

        # Sanity check
        if state_raw.shape != state_out.shape or \
           state_raw.shape != state_pred.shape or \
           state_raw.shape[0] != control.shape[0]:
            raise ValueError("Inconsistent shapes in the dataset.")

        # Transform velocity to body frame
        def velocity_mapping(state_sequence):
            p_traj = state_sequence[:, :3]
            v_w_traj = state_sequence[:, 3:6]
            q_traj = state_sequence[:, 6:10]
            other_traj = state_sequence[:, 10:]     # w, a_s, f_s, etc.
            # p_traj = state_sequence[:, :3]
            # q_traj = state_sequence[:, 3:7]
            # v_w_traj = state_sequence[:, 7:10]
            # other_traj = state_sequence[:, 10:]     # w, a_s, f_s, etc.

            v_b_traj = np.empty_like(v_w_traj)
            for t in range(len(v_w_traj)):
                v_b_traj[t] = v_dot_q(v_w_traj[t], quaternion_inverse(q_traj[t]))
            return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)
            # return np.concatenate((p_traj, q_traj, v_b_traj, other_traj), axis=1)

        state_raw = velocity_mapping(state_raw)
        state_pred = velocity_mapping(state_pred)
        state_out = velocity_mapping(state_out)

        # Compute error between predicted and actual state
        y_err = state_out - state_pred
        # Normalize error by window time, i.e., predict error dynamics instead of error itself
        y_err /= np.expand_dims(dt, 1)  # TODO maybe reduce number of predicted states

        # Store features
        self.state_raw = state_raw
        self.state_out = state_out
        self.state_pred = state_pred
        self.control = control
        self.y = y_err.astype(np.float32)
        self.dt = dt

        # Store network input
        # TODO select subset of features to use as input
        self.x = np.concatenate((state_raw, control), axis=1, dtype=np.float32)

    def prune(self, histogram_n_bins, histogram_thresh, vel_cap=None, plot=False):
        """
        Prune the dataset to remove samples with outliers in velocity.
        """
        # Prune noisy data
        if histogram_n_bins is not None and histogram_thresh is not None:
            x_vel_idx = np.array([3, 4, 5])
            y_vel_idx = np.array([3, 4, 5])
            # x_vel_idx = np.array([7, 8, 9])
            # y_vel_idx = np.array([7, 8, 9])

            # labels = [self.data_labels[dim] for dim in np.atleast_1d(y_vel_idx)]
            labels = ['vx', 'vy', 'vz']

            self.pruned_idx = prune_dataset(self.x[:, x_vel_idx], self.y[:, y_vel_idx],
                                            vel_cap, histogram_n_bins, histogram_thresh,
                                            plot=plot, labels=labels)
            self.x = self.x[self.pruned_idx]
            self.y = self.y[self.pruned_idx]

    def calculate_statistics(self):
        # Calculate mean and std for normalization
        self.x_mean = np.mean(self.x, axis=0)
        self.x_std = np.std(self.x, axis=0)
        self.y_mean = np.mean(self.y, axis=0)
        self.y_std = np.std(self.y, axis=0)