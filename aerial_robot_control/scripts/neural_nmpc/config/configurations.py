import os
from datetime import datetime


class DirectoryConfig:
    """
    Class for storing directories within the package.
    """

    _dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    SAVE_DIR = _dir_path + "/results/model_fitting"
    RESULTS_DIR = _dir_path + "/results"
    SIMULATION_DIR = _dir_path + "/sim_plots/" + datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    CONFIG_DIR = _dir_path + "/config"
    DATA_DIR = _dir_path + "/data"


class EnvConfig:
    """
    Class for storing the model, solver, simulator and environment configurations.
    """

    # MPC options
    model_options = {
        "model_name": "",
        "arch_type": "tilt_qd",  # or "fix_qd" or "tilt_bi" or "tilt_tri"
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
    }

    # MLP options
    model_options.update(
        {
            "only_use_nominal": True,
            "plus_neural": True,
            "minus_neural": False,
            "neural_model_name": "residual_mlp",  # "residual_mlp" or "temporal_mlp"
            "neural_model_instance": "neuralmodel_087",  # 63, 58, 60, 29, 31, 35
            # 32: label transform, no output denormalization, no dt normalization
            # 33: 0.4 dist, no label transform, output denormalization, dt normalization (VERY SUCCESSFUL) but large network and thus slow
            # 34: same as 33 but minimal network size (with 4 times the val loss)
            # 35: same as 34 but on dataset 04 (dist_factor=0.1)
            # 39 & 40: on real data (but small set)
            # 41: real data, large dataset (200k points)
            # 42: on dataset 008 with mainly ground effect data
            # 43: two hidden layers
            # 45: real data with rotation (115k datapoints)
            # 46: same as 45 but no pruning (better results)
            # 47: same as 46 but on dataset 013 (fixed prop and dt)
            # 48: only vz
            # 49: only vz with larger network (not much better than 48)
            # 50: only vz with larger network and LambdaLR (slightly worse than 49)
            # 51: only vz with small network and constant LR (slightly better than all)
            # 54: vz, prune, no input transform and no takeoff
            # 55: same as 54 but longer and LambdaLR
            # 57: With vx, vy, vz, With input and label transform, and weight 1,1,10 -> used for training network for paper
            # 58: same as 57 but no input and label transform -> NEURAL SIMULATOR FOR PAPER TRAINED ON REAL WORLD DATA
            # 59: only vz, no input/label transform, on sim with neural and nominal control dataset
            # 60: same as 59 but with vx, vy, vz -> NEURAL CONTROLLER FOR PAPER TRAINED ON SIMULATED LABELS
            # (61): same as 60 but longer training (250 epochs) -> somehow worse
            # ---- all before dont have standalone solver ----
            # 62: trained on residual_06 (first on standalone controller) (with 0.1 dist) (vx,vy,vz, no transform) -> good results
            # 63: trained on residual_neural_sim_nominal_control_03 -> WITH standalone SOLVER BUILDING
            # 64: small network trained on real hovering & ground effect data -> for CONTROLLER
            # 65: large network trained on real hovering & ground effect data -> for SIMULATOR
            # 66: really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 67: A LOT OF OVERFITTING really, really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 68: BN & L1 REGULARIZATION really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 71: (good) L1 REGULARIZATION really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 72: L2 REGULARIZATION really large network trained on real hovering & ground effect data -> for SIMULATOR
            # 73: (VERY GOOD) L1 REGULARIZATION small network trained on simulated labels with 72 -> for CONTROLLER
            # 74: same as 73 but only 50 epochs to avoid oscillations
            # 75: same as 74 but without regularization
            # ---- all before have high oscillations ----
            # 76: (only 50 epochs) small network trained on real hovering & ground effect data -> for CONTROLLER
            # 79: (only 50 epochs) very large network trained on real hovering & ground effect data -> for SIMULATOR
            # 80: (only 25 epochs) large network trained on real hovering & ground effect data -> for SIMULATOR
            # 81: (only 75 epochs, lower lr) large network trained on real hovering & ground effect data -> for SIMULATOR
            # 82: (same as 81 but full state input and input & label transform) large network trained on real hovering & ground effect data -> for SIMULATOR
            # 83: small network trained on simulated labels with 82 -> for CONTROLLER
            # 84: same as 83 but one more layer
            # 85: same as 84
            # 86: same as 85 but without L1 regularization
            # 87: same as 86 but with angular velocities in input and 100 epochs (instead of 50) -> GOOD!
            "approximate_mlp": False,  # TODO implement!; Approximation using first or second order Taylor Expansion
            "approx_order": 1,  # Order of Taylor Expansion (first or second)
            "scale_by_weight": False,  # Scale MLP output by robot's weight
        }
    )

    solver_options = {
        "solver_type": "PARTIAL_CONDENSING_HPIPM",  # TODO actually implement this
        "terminal_cost": True,  # TODO actually implement this
        "include_floor_bounds": False,
        "include_soft_constraints": False,
        "include_quaternion_constraint": False,
    }

    dataset_options = {"ds_name_suffix": "dataset_neural_sim_nominal_control"}  # "compare_nominal_neural_sim"}
    sim_options = {
        # Choice of disturbances modeled in our Simplified Simulator
        "disturbances": {
            "cog_dist": False,  # Disturbance forces and torques on CoG
            "cog_dist_model": "mu = 1 / (z+1)**2 * cog_dist_factor * max_thrust * 4 / std = 0",
            "cog_dist_factor": 0.1,
            "motor_noise": False,  # Asymmetric noise in the rotor thrust and servo angles
            "drag": False,  # 2nd order polynomial aerodynamic drag effect
            "payload": False,  # Payload force in the Z axis
        },
        "use_nominal_simulator": False,  # Use nominal model as simulator
        "use_real_world_simulator": True,  # Use neural model trained on real world data as simulator
        "sim_neural_model_instance": "neuralmodel_087",  # 58  # Used when use_real_world_simulator = True
        "max_sim_time": 10,
        "world_radius": 3,
        "seed": 897,
    }

    # Run options
    run_options = {
        "real_machine": False,
    }

    # Trajectory tracking options
    run_options.update(
        {
            "trajectories": [
                "line",
                "circle",
                "helix",
                "lemniscate_I",
                "lemniscate_II",
            ],  # "step", "hover", "takeoff", "smooth_takeoff", "circle"
            "trajectory_length": 20.0,
        }
    )

    # Point tracking options
    run_options.update(
        {
            "preset_targets": None,
            "low_flight_targets": True,
            "initial_state": None,
            "initial_guess": None,
            "aggressive": True,
        }
    )

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
            "save_figures": False,
            "real_time_plot": False,
            "save_animation": False,
        }
    )

    if model_options["minus_neural"] and model_options["plus_neural"]:
        raise ValueError("Conflict in options.")
    if model_options["approximate_mlp"] and model_options["approx_order"] == 0:
        raise ValueError("Conflict in options.")
    if sim_options["use_real_world_simulator"] and sim_options["use_nominal_simulator"]:
        raise ValueError("Conflict in options.")
    if sim_options["use_real_world_simulator"]:
        for value in sim_options["disturbances"].values():
            if value == True:
                raise ValueError("Simulated disturbances not meaningful when using real world simulator.")
    if run_options["real_machine"]:
        for value in sim_options["disturbances"].values():
            if value == True:
                raise ValueError("No simulated disturbances allowed on real machine.")


class MLPConfig:
    # Define characteristics of the MLP model with its name
    model_name = "residual_mlp"
    # model_name = "temporal_mlp"

    # Delay horizon for temporal networks
    delay_horizon = 0  # Number of time steps into the past to consider (set to 0 to only use current state)

    # Number of neurons in each hidden layer
    hidden_sizes = [64, 64]  # , 64, 64, 64]

    # Activation function
    activation = "ReLU"  # Options: "ReLU", "LeakyReLU", "GELU", "Tanh", "Sigmoid"

    # Use batch normalization after each layer
    use_batch_norm = False

    # Use dropout after each layer
    dropout_p = 0.0  # To disable dropout, set to 0.0

    # -----------------------------------------------------------------------------------------

    # Number of epochs
    num_epochs = 50

    # Batch size
    batch_size = 64

    # Loss weighting of different predicted dimensions (default ones-vector)
    # loss_weight = [1.0]
    loss_weight = [1.0, 1.0, 10.0]
    # Optimizer
    optimizer = "Adam"  # Options: "Adam", "SGD", "RMSprop", "Adagrad", "AdamW"
    # Weight decay (L2 regularization)
    weight_decay = 0.0  # 1e-3  # Set to 0.0 to disable
    # L1 regularization
    l1_lambda = 0.0  # 1e-4  # Set to 0.0 to disable

    # Learning rate
    learning_rate = 1e-3  # for residual
    # learning_rate = 1e-2  # for residual
    # learning_rate = 1e-5  # for temporal
    # learning_rate = 1e-3  # for LR scheduling
    lr_scheduler = "LambdaLR"  # "ReduceLROnPlateau", "LambdaLR", "LRScheduler", None

    # Number of workers, i.e., number of threads for loading data
    num_workers = 0

    if weight_decay != 0.0 and l1_lambda != 0.0:
        raise ValueError("Don't use both L1 and L2 regularization at the same time.")


class ModelFitConfig:
    # ------- Coordinate Transform -------
    label_transform = True
    input_transform = True

    # ------- Time Normalization -------
    normalize_by_T_step = False

    # ------- Dataset loading -------
    # ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_FOR_PAPER"# + "_01"
    # ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_GROUND_EFFECT_ONLY"
    ds_name = "NMPCTiltQdServo" + "_" + "residual_dataset_neural_sim_nominal_control_07"
    # ds_name = "NMPCTiltQdServo" + "_" + "residual_dataset_06"
    ds_instance = "dataset_001"  # "dataset_020"
    # real machine 01, dataset 007: Large dataset from many old flights with mode 0 and other discrepancies (200k datapoints)
    # real machine 01, dataset 007: Large dataset from mode 10 with focus on ground effect (66k datapoints)
    # real machine 01, dataset 013: same as 007 but with fixed prop and dt
    # residual 06, dataset 001 on neural standalone
    # residual neural sim nominal control 03, dataset 001: first with correct solver building
    # residual neural sim nominal control 05, dataset 001: on simulator as large network trained with L1 regularization
    # residual neural sim nominal control 07, dataset 001: on simulator as large network trained with full state and transforms and L1 regularization
    # real machine GROUND_EFFECT_ONLY: hovering and ground effect data only (48k datapoints)

    # ------- Features used for the model -------
    # State features
    state_feats = [2, 3, 4, 5]  # [z, vx, vy, vz]
    state_feats.extend([6, 7, 8, 9])  # [qw, qx, qy, qz]
    state_feats.extend([10, 11, 12])  # [roll_rate, pitch_rate, yaw_rate]
    # state_feats.extend([13, 14, 15, 16])  # [servo_angle_1, servo_angle_2, servo_angle_3, servo_angle_4]
    # state_feats.extend([17, 18, 19, 20, 21, 22])  # [fds_1, fds_2, fds_3, tau_ds_1, tau_ds_2, tau_ds_3]
    # state_feats.extend([17, 18, 19, 20])  # [thrust_1, thrust_2, thrust_3, thrust_4]

    # Control input features
    u_feats = [0, 1, 2, 3]  # [thrust_cmd_1, thrust_cmd_2, thrust_cmd_3, thrust_cmd_4]
    u_feats.extend([4, 5, 6, 7])  # [servo_angle_cmd_1, servo_angle_cmd_2, servo_angle_cmd_3, servo_angle_cmd_4]

    # Variables to be regressed
    # y_reg_dims = [5]  # [vz]
    y_reg_dims = [3, 4, 5]  # [vx, vy, vz]
    # y_reg_dims = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]  # [x, y, z, vx, vy, vz, qw, qx, qy, qz, roll_rate, pitch_rate, yaw_rate]
    # y_reg_dims.extend([13, 14, 15, 16])  # [servo_angle_1, servo_angle_2, servo_angle_3, servo_angle_4]
    # y_reg_dims.extend([17, 18, 19, 20, 21, 22])  # [fds_1, fds_2, fds_3, tau_ds_1, tau_ds_2, tau_ds_3]
    # y_reg_dims.extend([17, 18, 19, 20])  # [thrust_1, thrust_2, thrust_3, thrust_4]

    # ------------------------------- PRUNING -------------------------------
    prune = True

    # Histogram pruning parameters
    histogram_n_bins = 10
    histogram_thresh = 0.001  # Remove bins where the total ratio of data is lower than threshold
    vel_cap = 16  # Remove datapoints where abs(velocity) > vel_cap
