import os
import json

import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.configurations import DirectoryConfig

import torch
import numpy as np
from ml_casadi import torch as mc
from network_architecture.naive_mlp import NaiveMLP
from network_architecture.normalized_mlp import NormalizedMLP

def get_output_mapping(state_dim, y_reg_dims):
    M = np.zeros((state_dim, len(y_reg_dims)))
    for i in range(len(y_reg_dims)):
        M[y_reg_dims[i], i] = 1
    return M

def load_model(model_options, sim_options):
    """
    Load a pre-trained neural network for the NMPC controller.
    """
    # Load metadata from trained model
    neural_model_name = model_options['neural_model_name']
    neural_model_instance = model_options['neural_model_instance']
    json_file_name = os.path.join(DirectoryConfig.SAVE_DIR, "metadata.json")
    with open(json_file_name, 'r') as json_file:
        metadata = json.load(json_file)[neural_model_name][neural_model_instance]

    # Cross-check simulation environment options with metadata
    metadata = cross_check_options(model_options, sim_options, metadata)

    # Load trained MLP model
    file_name = os.path.join(DirectoryConfig.SAVE_DIR, neural_model_name, f"{neural_model_instance}.pt")
    saved_dict = torch.load(file_name)

    device = get_device()
    if metadata["MLPConfig"]["approximated_mlp"]:
        # Use RTNMPC library for approximated MLP
        base_mlp = mc.nn.MultiLayerPerceptron(
            saved_dict['input_size'], saved_dict['hidden_sizes'][0],
            saved_dict['output_size'], len(saved_dict['hidden_sizes']),
            activation='Tanh')

        neural_model = NormalizedMLP(base_mlp,
            torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
            torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
            torch.tensor(np.zeros((saved_dict['output_size'],))).float(),
            torch.tensor(np.zeros((saved_dict['output_size'],))).float()).to(device)
        
    else:
        neural_model = NaiveMLP(
            saved_dict['input_size'], saved_dict['hidden_sizes'], saved_dict['output_size'],
            activation=saved_dict['activation'],
            use_batch_norm=saved_dict['use_batch_norm'],
            dropout_p=saved_dict['dropout_p'],
            x_mean=torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
            x_std=torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
            y_mean=torch.tensor(np.zeros((saved_dict['output_size'],))).float(),
            y_std=torch.tensor(np.zeros((saved_dict['output_size'],))).float()).to(device)

    # Load weights and biases from saved model
    neural_model.load_state_dict(saved_dict['state_dict'])
    neural_model.eval()

    return neural_model, metadata
    
def cross_check_options(model_options, sim_options, metadata):
    """
    Cross-check the model options and simulation options to ensure they match the configuration
    that the neural model was trained with.
    """
    if model_options["nmpc_type"] != metadata["ds_nmpc_type"]:
        raise ValueError("NMPC type used in dataset for training the neural model doesn't match the simulation environment.")

    if sim_options["disturbances"] != metadata["ds_disturbances"]:
        raise ValueError("Disturbances used in dataset for training the neural model don't match the simulation environment.")

    return metadata

def sanity_check_features_and_reg_dims(x_feats, u_feats, y_reg_dims, in_dim, out_dim):
    """
    Simple check to ensure that the features and regression dimensions are valid.
    """
    if not isinstance(x_feats, list) or not isinstance(u_feats, list) or not isinstance(y_reg_dims, list):
        raise ValueError("x_feats, u_feats, and y_reg_dims must be lists.")

    if len(x_feats) == 0 or len(y_reg_dims) == 0:
        raise ValueError("x_feats and y_reg_dims cannot be empty lists.")

    for y_reg in y_reg_dims:
        if y_reg not in x_feats:
            raise ValueError(f"Feature {y_reg} in y_reg_dims is not present in x_feats.")

    for x_feat in x_feats:
        if x_feat < 0 or x_feat >= in_dim:
            raise ValueError(f"Feature index {x_feat} in x_feats is out of bounds for input dimension {in_dim}.")

    if len(x_feats) + len(u_feats) != in_dim:
        raise ValueError(f"Total number of features {len(x_feats) + len(u_feats)} does not match input dimension {in_dim}.")

    if len(y_reg_dims) != out_dim:
        raise ValueError(f"Total number of regression dimensions {len(y_reg_dims)} does not match output dimension {out_dim}.")

def get_device():
    device = torch.accelerator.current_accelerator().type if torch.accelerator.is_available() else "cpu"
    print(f"Using {device} device")
    if torch.cuda.is_available():
        return torch.device("cuda")
    elif torch.backends.mps.is_available():
        return torch.device("mps")
    else:
        return torch.device("cpu")
