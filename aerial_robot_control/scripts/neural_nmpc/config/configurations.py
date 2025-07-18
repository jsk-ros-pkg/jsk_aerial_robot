""" Set of tunable parameters for the Simplified Simulator and model fitting.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

import os


class DirectoryConfig:
    """
    Class for storing directories within the package
    """

    _dir_path = os.path.dirname(os.path.realpath(__file__))
    SAVE_DIR = _dir_path + '/../results/model_fitting'
    RESULTS_DIR = _dir_path + '/../results'
    CONFIG_DIR = _dir_path + ''
    DATA_DIR = _dir_path + '/../data'


class EnvConfig:
    """
    Class for storing the Simulator configurations.
    """

    model_options = {
            "model_name": "standard_nmpc",
            "arch_type": "qd", # or "bi" or "tri"
            "nmpc_type": "NMPCTiltQdServo",
            #    NMPCFixQdAngvelOut
            #    NMPCFixQdThrustOut
            #    NMPCTiltQdNoServo
            #    NMPCTiltQdServo
            #    NMPCTiltQdServoDist
            #    NMPCTiltQdServoImpedance
            #    NMPCTiltQdServoThrustDist
            #    NMPCTiltQdServoThrustImpedance
            #    NMPCTiltTriServo
            #    NMPCTiltBiServo
            #    NMPCTiltBi2OrdServo
            #    MHEWrenchEstAccMom
            "only_use_nominal": True,
            "end_to_end_mlp": False,
            "neural_model_name": "naive_residual_mlp", # "naive_e2e_mlp" or "naive_residual_mlp" or "approximated_mlp"
            "neural_model_instance": "neuralmodel_016",
    }
    if model_options["only_use_nominal"] and model_options["end_to_end_mlp"]:
        raise ValueError("Conflict in options.")

    solver_options = {
        "solver_type": "PARTIAL_CONDENSING_HPIPM",  # TODO actually implement this
        "terminal_cost": True,
    }

    dataset_options = {
            "ds_name_suffix": "residual_dataset"
    }
    sim_options = {
            # Choice of disturbances modeled in our Simplified Simulator. For more details about the parameters used refer to
            # the script: src/quad_mpc/quad_3d.py.
            # TODO actually implement the disturbances in NMPC and network
            "disturbances": {
                "cog_dist": True,                    # Disturbance forces and torques on CoG
                "motor_noise": True,                 # Asymmetric voltage noise in the motors
                "drag": False,                       # 2nd order polynomial aerodynamic drag effect
                "payload": False,                    # Payload force in the Z axis
            },
            "max_sim_time": 30,
            "world_radius": 3,
            "seed": 678,
    }
    run_options = {
            "preset_targets": None,
            "initial_state": None,
            "initial_guess": None,
            "aggressive": True,  # TODO for now always use aggressive targets
            "recording": False,
            "plot_traj": True,
            "real_time_plot": False,
            "save_animation": False,
    }

    ################################################################
    # Set to True to show a real-time Matplotlib animation of the experiments for the Simulator. Execution
    # will be slower if the GUI is turned on. Note: setting to True may require some further library installation work.
    custom_sim_gui = False

    # Set to True to display a plot describing the trajectory tracking results after the execution.
    result_plots = True

    # Set to True to show the trajectory that will be executed before the execution time
    pre_run_debug_plots = True


class MLPConfig:
    # Use naive implementation of MLP using torch
    model_name = "naive_residual_mlp"
    # model_name = "naive_e2e_mlp"

    # Use propietary RTNMPC library for approximated MLP
    # model_name = "approximated_mlp"

    # Number of neurons in each hidden layer
    hidden_sizes = [64, 64, 64, 64] # In_features of each hidden layer

    # Activation function
    activation = "ReLU"  # Options: "ReLU", "LeakyReLU", "Tanh", "Sigmoid"

    # Use batch normalization after each layer
    use_batch_norm = True

    # Use dropout after each layer
    dropout_p = 0.0     # To disable dropout, set to 0.0

    # -----------------------------------------------------------------------------------------

    # Number of epochs
    num_epochs = 250

    # Batch size
    batch_size = 64

    # Optimizer
    optimizer = "Adam"  # Options: "Adam", "SGD", "RMSprop", "Adagrad", "AdamW"

    # Learning rate
    learning_rate = 1e-2
    lr_scheduler = "ReduceLROnPlateau" # "ReduceLROnPlateau", "LRScheduler", None

    # Number of workers, i.e., number of threads for loading data
    num_workers = 0

    # ------------------------------------------------------------------------------------------

    # Histogram pruning parameters
    histogram_n_bins = 40
    histogram_thresh = 0.005    # Remove bins where the total ratio of data is lower than threshold
    vel_cap = 16                # Remove datapoints where abs(velocity) > vel_cap

class ModelFitConfig:
    # ------- Dataset loading -------
    ds_name = "NMPCTiltQdServo" + "_" + "residual" + "_dataset"
            #    NMPCFixQdAngvelOut
            #    NMPCFixQdThrustOut
            #    NMPCTiltQdNoServo
            #    NMPCTiltQdServo
            #    NMPCTiltQdServoDist
            #    NMPCTiltQdServoImpedance
            #    NMPCTiltQdServoThrustDist
            #    NMPCTiltQdServoThrustImpedance
            #    NMPCTiltTriServo
            #    NMPCTiltBiServo
            #    NMPCTiltBi2OrdServo
            #    MHEWrenchEstAccMom
    ds_instance = "dataset_001"

    # ------- Features used for the model -------
    # State features
    # state_feats = [3, 4, 5]
    state_feats = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    # state_feats = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]  # [x, y, z, vx, vy, vz, qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate]
    # state_feats.extend([13, 14, 15, 16])  # [servo_angle_1, servo_angle_2, servo_angle_3, servo_angle_4]
    # state_feats.extend([17, 18, 19, 20, 21, 22])  # [fds_1, fds_2, fds_3, tau_ds_1, tau_ds_2, tau_ds_3]
    # state_feats.extend([17, 18, 19, 20])  # [thrust_1, thrust_2, thrust_3, thrust_4]

    # Control input features
    # u_feats = []
    u_feats = [0, 1, 2, 3]  # [thrust_cmd_1, thrust_cmd_2, thrust_cmd_3, thrust_cmd_4]
    u_feats.extend([4, 5, 6, 7])  # [servo_angle_cmd_1, servo_angle_cmd_2, servo_angle_cmd_3, servo_angle_cmd_4]

    # Variables to be regressed
    y_reg_dims = [3, 4, 5]  # [vx, vy, vz]
    # y_reg_dims = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]  # [x, y, z, vx, vy, vz, qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate]
    # y_reg_dims.extend([13, 14, 15, 16])  # [servo_angle_1, servo_angle_2, servo_angle_3, servo_angle_4]
    # y_reg_dims.extend([17, 18, 19, 20, 21, 22])  # [fds_1, fds_2, fds_3, tau_ds_1, tau_ds_2, tau_ds_3]
    # y_reg_dims.extend([17, 18, 19, 20])  # [thrust_1, thrust_2, thrust_3, thrust_4]

    # ds_disturbances = {
    #     "noisy": True,
    #     "drag": True,
    #     "payload": False,
    #     "motor_noise": True
    # }

    # ds_name = "agisim_dataset"
    # ds_disturbances = {
    #     "agisim": "default",
    # }

    # ds_name = "arena_dataset"
    # ds_disturbances = {
    #     "arena": "default",
    # }

    # ds_name = "neurobem_dataset"
    # ds_disturbances = {
    #     "arena": "default",
    # }

    # ## Visualization ## #
    # Training mode
    visualize_training_result = True
    visualize_data = False

    # Visualization mode
    grid_sampling_viz = True
    x_viz = [7, 8, 9]
    u_viz = []
    y_viz = [7, 8, 9]

    # ############# Experimental ############# #

    # ## Use fit model to generate synthetic data ## #
    use_dense_model = False
    dense_model_version = ""
    dense_model_name = ""
    dense_training_points = 200

    # ## Clustering for multidimensional models ## #
    clusters = 1
    load_clusters = False


class GroundEffectMapConfig:
    """
    Class for storing parameters for the ground effect map.
    """
    resolution = 0.1
    origin = (-4, 9)
    horizon = ((-7, 7), (-7, 7))
    box_min = (-4.25, 9.37)
    box_max = (-2.76, 10.13)
    box_height = 0.7
