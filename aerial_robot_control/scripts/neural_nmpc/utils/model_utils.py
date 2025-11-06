import os, sys
import json
import numpy as np
import torch

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.configurations import DirectoryConfig
from network_architecture.mlp import MLP


def set_approximation_params(rtnmpc, ocp_solver):
    # a = 1
    # mlp_params = rtnmpc.neural_model.approx_params(a, order=rtnmpc.mlp_conf['approx_order'], flat=True)
    # mlp_params = np.vstack([mlp_params, mlp_params[[-1]]])
    # for j in range(ocp_solver.N + 1):
    #     rtnmpc.acados_parameters[j, rtnmpc.approx_start_idx : rtnmpc.approx_end_idx] = 1
    pass


def set_temporal_states_as_params(rtnmpc, ocp_solver, history_y, u_cmd):
    # Set previous state and control as parameters
    if u_cmd is None:
        # Take init values for all nodes at t=0
        rtnmpc.acados_parameters[:, rtnmpc.delay_start_idx : rtnmpc.delay_end_idx] = history_y.copy().flatten()
    else:
        for j in range(ocp_solver.N + 1):
            if j == 0:
                # Use all available history steps from current node with size of the delay horizon
                # Take all available history steps in order of actuality
                running_y = history_y.copy()
            else:
                # Shift history steps from current node with size of the delay horizon
                # Note, delay horizon is sorted from newest to oldest, so discard last indices first
                running_y = running_y[:-1, :]

                # Use predicted states for all previous nodes from current node to fill up delay window
                # Note, this is an approximation since we are using the state prediction from the previous optimization scheme and
                # not the best possible estimate for the previous states
                predicted_x = ocp_solver.get(j, "x")
                if j < ocp_solver.N - 1:
                    predicted_u = ocp_solver.get(j, "u")
                else:
                    # Terminal node
                    predicted_u = [None] * rtnmpc.nu
                running_y = np.append(np.append(predicted_x, predicted_u)[np.newaxis, :], running_y, axis=0)

            # Set acados parameters in OCP solver
            rtnmpc.acados_parameters[j, rtnmpc.delay_start_idx : rtnmpc.delay_end_idx] = running_y.flatten()


def get_output_mapping(state_dim, y_reg_dims, label_transform=False, only_vz=False):
    M = np.zeros((state_dim, len(y_reg_dims)))
    for i in range(len(y_reg_dims)):
        M[y_reg_dims[i], i] = 1

    if label_transform and only_vz:
        # Special case: MLP only predicts v_z
        # Set mapping for v_x and v_y to 1 to account for their influence from v_z
        M_v_xy = np.zeros((state_dim, 2))
        M_v_xy[3, 0] = 1
        M_v_xy[4, 1] = 1
        M = np.append(M_v_xy, M, axis=1)
    return M


def get_inverse_output_mapping(state_dim, y_reg_dims):
    O = np.eye(state_dim)
    for i in y_reg_dims:
        O[i, i] = 0
    return O


def get_device():
    if torch.cuda.is_available():
        device = "cuda"
    elif torch.backends.mps.is_available():
        device = "mps"
    else:
        device = "cpu"
    print(f"Using {device} device")
    return torch.device(device)


def load_model(model_options, sim_options, run_options):
    """
    Load a pre-trained neural network for the MPC controller.
    """
    # Load metadata from trained model
    neural_model_name = model_options["neural_model_name"]
    neural_model_instance = model_options["neural_model_instance"]
    json_file_name = os.path.join(DirectoryConfig.SAVE_DIR, "metadata.json")
    with open(json_file_name, "r") as json_file:
        metadata = json.load(json_file)[neural_model_name][neural_model_instance]

    # Cross-check simulation environment options with metadata
    cross_check_options(model_options, sim_options, run_options, metadata)

    # Define trained MLP model
    device = "cpu"  # get_device()  # TODO Make use of GPU later
    file_name = os.path.join(DirectoryConfig.SAVE_DIR, neural_model_name, f"{neural_model_instance}.pt")
    saved_dict = torch.load(file_name, map_location=device)

    neural_model = MLP(
        saved_dict["input_size"],
        saved_dict["hidden_sizes"],
        saved_dict["output_size"],
        activation=saved_dict["activation"],
        use_batch_norm=saved_dict["use_batch_norm"],
        dropout_p=saved_dict["dropout_p"],
        x_mean=torch.tensor(np.zeros((saved_dict["input_size"],))).float(),
        x_std=torch.tensor(np.zeros((saved_dict["input_size"],))).float(),
        y_mean=torch.tensor(np.zeros((saved_dict["output_size"],))).float(),
        y_std=torch.tensor(np.zeros((saved_dict["output_size"],))).float(),
    ).to(device)

    # Load weights and biases from saved model
    neural_model.load_state_dict(saved_dict["state_dict"])
    neural_model.eval()

    return neural_model, metadata


def cross_check_options(model_options, sim_options, run_options, metadata):
    """
    Cross-check the model options and simulation options to ensure they match the configuration
    that the neural model was trained with.
    """
    if model_options["nmpc_type"] != metadata["ds_nmpc_type"]:
        raise ValueError(
            "NMPC type used in dataset for training the neural model doesn't match the simulation environment."
        )

    if not run_options["real_machine"] and not (
        sim_options["use_nominal_simulator"] or sim_options["use_real_world_simulator"]
    ):
        for dist, value in sim_options["disturbances"].items():
            if dist not in metadata["ds_disturbances"]:
                raise ValueError(
                    f"Disturbance '{dist}' used in simulation environment is not present in the dataset for training the neural model."
                )
            if value != metadata["ds_disturbances"][dist]:
                if dist == "cog_dist_factor":
                    pass
                else:
                    raise ValueError(
                        f"Disturbance '{dist}' used in dataset for training the neural model doesn't match the simulation environment."
                    )


def cross_check_params(nmpc_params, mlp_metadata):
    """
    Cross-check the NMPC parameters with the metadata of the MLP model to ensure they match.
    """
    if nmpc_params["T_samp"] != mlp_metadata["ds_nmpc_params"]["T_samp"]:
        raise ValueError(
            "Sampling time used in dataset for training the neural model doesn't match the NMPC parameters."
        )

    if nmpc_params["T_horizon"] != mlp_metadata["ds_nmpc_params"]["T_horizon"]:
        raise ValueError(
            "Prediction horizon used in dataset for training the neural model doesn't match the NMPC parameters."
        )

    if nmpc_params["T_step"] != mlp_metadata["ds_nmpc_params"]["T_step"]:
        raise ValueError("Step time used in dataset for training the neural model doesn't match the NMPC parameters.")


def sanity_check_features_and_reg_dims(model_name, state_feats, u_feats, y_reg_dims, in_dim, out_dim, delay):
    """
    Simple check to ensure that the features and regression dimensions are valid.
    """
    if not isinstance(state_feats, list) or not isinstance(u_feats, list) or not isinstance(y_reg_dims, list):
        raise ValueError("state_feats, u_feats, and y_reg_dims must be lists.")

    if len(state_feats) == 0 or len(y_reg_dims) == 0:
        raise ValueError("state_feats and y_reg_dims cannot be empty lists.")

    for y_reg in y_reg_dims:
        if y_reg not in state_feats:
            raise ValueError(f"Feature {y_reg} in y_reg_dims is not present in state_feats.")

    if (len(state_feats) + len(u_feats)) * (delay + 1) != in_dim:
        raise ValueError(
            f"Total number of features {len(state_feats) + len(u_feats)} does not match input dimension {in_dim}."
        )

    if len(y_reg_dims) != out_dim:
        raise ValueError(
            f"Total number of regression dimensions {len(y_reg_dims)} does not match output dimension {out_dim}."
        )

    if delay > 0:
        if "temporal" not in model_name:
            raise ValueError(
                "Delay horizon is set but model name does not contain 'temporal'. Please use a temporal model."
            )
    else:
        if "temporal" in model_name:
            raise ValueError(
                "Delay horizon is not set but model name contains 'temporal'. Please use a non-temporal model."
            )
