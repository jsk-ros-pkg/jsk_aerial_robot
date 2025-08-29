import os


class DirectoryConfig:
    """
    Class for storing directories within the package
    """

    _dir_path = os.path.dirname(os.path.realpath(__file__))
    SAVE_DIR = _dir_path + "/../results/model_fitting"
    RESULTS_DIR = _dir_path + "/../results"
    CONFIG_DIR = _dir_path + ""
    DATA_DIR = _dir_path + "/../data"


class EnvConfig:
    """
    Class for storing the Simulator configurations.
    """

    model_options = {
        "model_name": "standard_nmpc",
        "arch_type": "qd",  # or "bi" or "tri"
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
    }

    # MLP options
    model_options.update(
        {
            "only_use_nominal": True,
            "end_to_end_mlp": False,
            "neural_model_name": "residual_mlp",  # "e2e_mlp" or "residual_mlp" or "residual_temporal_mlp" or "approximated_mlp"
            "neural_model_instance": "neuralmodel_032",  # 29, 31
            "approximated_mlp": False,
            "approx_order": 0,
        }
    )

    if model_options["only_use_nominal"] and model_options["end_to_end_mlp"]:
        raise ValueError("Conflict in options.")
    if "approximated" in model_options["neural_model_name"] and not model_options["approximated_mlp"]:
        raise ValueError("Conflict in options.")

    solver_options = {
        "solver_type": "PARTIAL_CONDENSING_HPIPM",  # TODO actually implement this
        "terminal_cost": True,
    }

    dataset_options = {"ds_name_suffix": "residual_dataset_03"}
    sim_options = {
        # Choice of disturbances modeled in our Simplified Simulator
        # TODO actually implement the disturbances in NMPC and network
        "disturbances": {
            "cog_dist": True,  # Disturbance forces and torques on CoG
            "cog_dist_model": "mu = 1 / (z+1)**2 * cog_dist_factor * max_thrust * 4 / std = 0",
            "cog_dist_factor": 0.2,
            "motor_noise": False,  # Asymmetric noise in the rotor thrust and servo angles
            "drag": False,  # 2nd order polynomial aerodynamic drag effect
            "payload": False,  # Payload force in the Z axis
        },
        "max_sim_time": 30,
        "world_radius": 3,
        "seed": 567,
    }

    # Trajectory options
    run_options = {
        "preset_targets": None,
        "low_flight_targets": True,
        "initial_state": None,
        "initial_guess": None,
        "aggressive": True,
    }

    # Recording options
    run_options.update(
        {
            "recording": False,
        }
    )

    # Visualization options
    run_options.update(
        {
            "plot_trajectory": True,
            "real_time_plot": False,
            "save_animation": False,
        }
    )


class MLPConfig:
    # Define characteristics of the MLP model with its name
    # model_name = "residual_temporal_mlp"
    model_name = "residual_mlp"
    # model_name = "e2e_mlp"

    # Delay horizon for temporal networks
    delay_horizon = 0  # Number of time steps into the past to consider (set to 0 to only use current state)

    # Number of neurons in each hidden layer
    hidden_sizes = [64, 64, 64, 64]  # In_features of each hidden layer

    # Activation function
    activation = "GELU"  # Options: "ReLU", "LeakyReLU", "GELU", "Tanh", "Sigmoid"

    # Use batch normalization after each layer
    use_batch_norm = False

    # Use dropout after each layer
    dropout_p = 0.0  # To disable dropout, set to 0.0

    # -----------------------------------------------------------------------------------------

    # Number of epochs
    num_epochs = 250

    # Batch size
    batch_size = 64

    # Optimizer
    optimizer = "Adam"  # Options: "Adam", "SGD", "RMSprop", "Adagrad", "AdamW"

    # Learning rate
    learning_rate = 1e-3  # for residual
    # learning_rate = 1e-5 # for temporal
    # learning_rate = 1e-3 # for LR scheduling
    lr_scheduler = "LambdaLR"  # "ReduceLROnPlateau", "LambdaLR", "LRScheduler", None

    # Number of workers, i.e., number of threads for loading data
    num_workers = 0

    # ------------------------------------------------------------------------------------------

    # Histogram pruning parameters
    histogram_n_bins = 40
    histogram_thresh = 0.005  # Remove bins where the total ratio of data is lower than threshold
    vel_cap = 16  # Remove datapoints where abs(velocity) > vel_cap


class ModelFitConfig:
    # ------- Dataset loading -------
    ds_name = "NMPCTiltQdServo" + "_" + "residual" + "_dataset" + "_03"
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
    state_feats = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    # state_feats = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]  # [x, y, z, vx, vy, vz, qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate]
    # state_feats.extend([13, 14, 15, 16])  # [servo_angle_1, servo_angle_2, servo_angle_3, servo_angle_4]
    # state_feats.extend([17, 18, 19, 20, 21, 22])  # [fds_1, fds_2, fds_3, tau_ds_1, tau_ds_2, tau_ds_3]
    # state_feats.extend([17, 18, 19, 20])  # [thrust_1, thrust_2, thrust_3, thrust_4]

    # Control input features
    # u_feats = []
    u_feats = [0, 1, 2, 3]  # [thrust_cmd_1, thrust_cmd_2, thrust_cmd_3, thrust_cmd_4]
    u_feats.extend([4, 5, 6, 7])  # [servo_angle_cmd_1, servo_angle_cmd_2, servo_angle_cmd_3, servo_angle_cmd_4]

    # Variables to be regressed
    # y_reg_dims = [5]  # [vz]
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
