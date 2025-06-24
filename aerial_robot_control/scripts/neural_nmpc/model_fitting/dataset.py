import numpy as np
from torch.utils.data import Dataset

import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.geometry_utils import v_dot_q, quaternion_inverse
from utils.data_utils import undo_jsonify


class TrajectoryDataset(Dataset):
    # TODO dataset pruning based on samples with velocity outliers (idea from RTNMPC)
    # TODO Normalization
    def __init__(self, dataframe):
        self.df = dataframe
        self.prepare_data()

    def __len__(self):
        return len(self.inputs)

    def __getitem__(self, idx):
        # TODO support only a selection of features as input (e.g., only everything except position)
        # TODO support not using control as input for network
        return self.inputs[idx], self.y[idx]

    def prepare_data(self):
        state_raw = undo_jsonify(self.df['state_in'].to_numpy())
        state_out = undo_jsonify(self.df['state_out'].to_numpy())
        state_pred = undo_jsonify(self.df['state_pred'].to_numpy())
        control = undo_jsonify(self.df['control'].to_numpy())
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

            v_b_traj = np.empty_like(v_w_traj)
            for t in range(len(v_w_traj)):
                v_b_traj[t] = v_dot_q(v_w_traj[t], quaternion_inverse(q_traj[t]))
            return np.concatenate((p_traj, v_b_traj, q_traj, other_traj), axis=1)

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
        self.inputs = np.concatenate((state_raw, control), axis=1, dtype=np.float32)