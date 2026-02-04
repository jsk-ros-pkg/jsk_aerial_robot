import os
from datetime import datetime
import numpy as np


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
        "mpc_type": "NMPCTiltQdServo",
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
            "only_use_nominal": False,
            "plus_neural": True,
            "minus_neural": False,
            "neural_model_name": "residual_mlp",  # "residual_mlp" or "temporal_mlp"
            "neural_model_instance": "neuralmodel_159",  # 129, 120, 113, 90, 88, 87, 63, 58, 60, 29, 31, 35
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
            # ---- all before have no moving average filter ----
            # 88: [GOOD!] WITH LABELS & INPUT DATA FILTERED, trained on GROUND_EFFECT_ONLY, middle size, normal settings -> for CONTROLLER
            # 89: (NO DIFFERENCE to 88) same as 88 but large size -> for SIMULATOR
            # 90: [GOOD!] same as 88 but large size and on TRAIN_FOR_PAPER -> for SIMULATOR
            # 92: Only z and cmd inputs (w/o transforms), on GROUND_EFFECT_ONLY -> for CONTROLLER
            # 93: Only z and cmd inputs (w/o transforms), on TRAIN_FOR_PAPER -> for CONTROLLER
            # ---- all before have no low pass filter ----
            # 96: [very good] Only z and avg cmd inputs (w/o transforms) & az as label, with LPF (1.0 ctf) on TRAIN_FOR_PAPER -> for CONTROLLER
            # 97: [a bit too good] Regular state in, avg input and large network (w/o transforms), with LPF (1.0 ctf) on TRAIN_FOR_PAPER -> for SIMULATOR
            # 98: Only z and avg cmd inputs (w/o transforms) but full labels, with LPF (0.1 ctf) on TRAIN_FOR_PAPER -> for CONTROLLER
            # [BAD LEARNING] 99: Same as 98 but without control averaging and with homogenous weight and weight decay (L2) -> for CONTROLLER
            # 100: Same as 99 but without weight decay -> for CONTROLLER
            # 101: Same as 97 but with grad penalty and consistency regularization -> for SIMULATOR
            # 102: Same as 101 but without LPF but with moving average
            # 103: Same as 96 but without extra losses and WITH L2 regularization
            # 104, 105, 106, 107: Same as 103 but slightly larger L2 factor
            # 108 (good), 109, 110 (VERY GOOD): Same as 107 but with consistency loss
            # 113 (BEST SO FAR), 114, 115 (Cant find solution when too high lambda): Same as 110 but with gradient penalty
            # 116 (bad): No mov avg (raw data), no grad pen, low consistency with L2
            # 117 (not as good as 113): Same as 116 but with mov avg
            # 118 (not as good as 113): Same as 113 but regular state
            # ---- all before not on FULL dataset ----
            # 119 (not good): Same as 113 but trained on FULL dataset
            # 120 (BEST SO FAR [similar to 113]): Same as 113 on fixed FULL dataset
            # 121: Same as 113 but on 13_RECORDINGS dataset
            # 124 (BETTER THAN 125): Same as 120 but with permutation symmetry loss (symmetry t1&t2, t3&t4, a1&a3, a2&a4)
            # 125 (worse than 124): Same as 120 but with permutation symmetry loss (symmetry t1&t3, t2&t4, a1&a3, a2&a4)
            # 127 (worse than 124): Same as 120 but with permutation symmetry loss (symmetry t1&t2, t3&t4, a1&a2, a3&a4)
            # 128 (worse than 124): Same as 120 but with permutation symmetry loss (symmetry t1&t3, t2&t4, a1&a2, a3&a4)
            # 129 (very simple functions due not much learning): Same as 120 but with permutation symmetry loss (and on TRAIN ds) (symmetry thrust change 50% which two to swap, a1&a3, a2&a4)
            # 133 (no learning): Same as 120 but with label transform ON TRAIN
            # ---- No symmetry loss and grad loss from here ----
            # 134 (symmetry doesnt make sense! UNSTABLE EVEN IN SIM FOR CIRC TRAJ!): Same as 120 / with consistency loss and weight decay L2 on TRAIN (no symmetry loss, no grad loss) ON TRAIN
            # 135 (better learning behavior for normal L2 loss): Same as 134 but with lower consistency weight and way lower epsilon ON TRAIN
            # 136 (good learning but unstable flight EVEN IN SIM): Same as 135 but with FULL state input and no servo cmd (no transform) && only normal loss + zero-out regularization loss ON TRAIN
            # 137 (decent learning UNSTABLE IN SIM): Same as 136 (full state in) but with input and label transform & consistency + reg loss (& no moving average filter) ON FULL DATASET
            # 138 (decent learning): Same as 137 but with higher consist and reg losses
            # 140 (decent learning): Same as 138 but with higher consist epsilon
            # 141: Same as 140 but with grad loss and higher epsilon & zero-out lambda
            # 142: Same as 141 but without ang vel in input and servo only in cmd (not as state)
            # 143: Same as 142 but no reg loss and higher grad loss & consist
            # 144: Same as 143 but without vel in input (only label transform) and only with reg (no grad, no consist) and with seperate val dataset
            # 145: Same as 144 but with mov avg filter (33 window size) and on vx, vy, vz as labels(!!)
            # 146: Same as 145 but with revised propagation
            # 147: Same as 146 but average over propagation steps
            # 148: Same as 147 but with grad & consist loss
            # 149: Labels with MPC & T_step=0.01 (no revised out) (ReLU, 32 nodes)
            # 150 (val loss high): Labels with MPC & T_step=0.01
            # 151 (no learning): Same as 150 but with zero-out & consist & grad loss (bigger network)
            # 152 (decent learning): Same as 151 but only consist & zero-out loss & lower lambas (bigger network)
            # 153 (good learning): Same as 152 but with tiny network (16 nodes) and lower lr
            # 155 (best learning): Same as 153 but with MultiStepLR (experiment)
            # 159: On long prediction horizon propagation dataset (with mov avg on label)
            "approximate_mlp": False,  # TODO implement!; Approximation using first or second order Taylor Expansion
            "approx_order": 1,  # Order of Taylor Expansion (first or second)
        }
    )

    solver_options = {
        "solver_type": "PARTIAL_CONDENSING_HPIPM",  # TODO actually implement this
        "terminal_cost": True,  # TODO actually implement this
        "include_floor_bounds": False,
        "include_soft_constraints": False,
        "include_quaternion_constraint": False,
        "include_delta_u": True,
    }

    dataset_options = {"ds_name_suffix": "dataset_neural_sim_nominal_control"}  # "compare_nominal_neural_sim"}
    sim_options = {
        # Choice of disturbances modeled in our Simplified Simulator
        "disturbances": {
            "cog_dist": False,  # Disturbance forces and torques on CoG
            "cog_dist_model": "mu = 1 / (abs(z)+1)**2 * cog_dist_factor * max_thrust * 4 | std = 0",
            "cog_dist_factor": 0.2,  # 0.1
            "motor_noise": False,  # Asymmetric noise in the rotor thrust and servo angles
            "drag": False,  # 2nd order polynomial aerodynamic drag effect
            "payload": False,  # Payload force in the Z axis
        },
        "use_nominal_simulator": False,  # Use nominal model as simulator
        "use_real_world_simulator": False,  # Use neural model trained on real world data as simulator
        "sim_neural_model_instance": "neuralmodel_113",  # 90, 87, 58  # Used when use_real_world_simulator = True
        "max_sim_time": 25,
        "world_radius": 3,
        "seed": 897,
    }

    # Run options
    run_options = {
        "real_machine": True,
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
    # if run_options["real_machine"]:
    #     for value in sim_options["disturbances"].values():
    #         if value == True:
    #             raise ValueError("No simulated disturbances allowed on real machine.")


class MLPConfig:
    # Define characteristics of the MLP model with its name
    model_name = "residual_mlp"
    # model_name = "temporal_mlp"

    # Delay horizon for temporal networks
    delay_horizon = 0  # Number of time steps into the past to consider (set to 0 to only use current state)

    # Number of neurons in each hidden layer
    hidden_sizes = [32, 32]
    # hidden_sizes = [64, 64]
    # hidden_sizes = [128, 256, 128, 64]

    # Activation function
    activation = "ReLU"  # Options: "ReLU", "LeakyReLU", "GELU", "Tanh", "Sigmoid"

    # Use batch normalization after each layer
    use_batch_norm = False

    # Use dropout after each layer
    dropout_p = 0.0  # To disable dropout, set to 0.0

    # -----------------------------------------------------------------------------------------

    # Number of epochs
    num_epochs = 200

    # Batch size
    batch_size = 64

    # Loss weighting of different predicted dimensions (default ones-vector)
    # loss_weight = [1.0]
    loss_weight = [1.0, 1.0, 1.0]
    # loss_weight = [1.0, 1.0, 10.0]
    # Optimizer
    optimizer = "AdamW"  # Options: "Adam", "SGD", "RMSprop", "Adagrad", "AdamW"
    # Weight decay (L2 regularization)
    weight_decay = 1e-1  # Set to 0.0 to disable [1e-2 for AdamW, 1e-4 for Adam]
    # Zero-output regularization
    zero_out_lambda = 1e-1  # Set to 0.0 to disable
    # L1 regularization
    l1_lambda = 0.0  #1e-4  # Set to 0.0 to disable
    # Penalize gradients
    gradient_lambda = 0.0  # 1e3  # Set to 0.0 to disable
    # Output consistency regularization epsilon
    consistency_lambda = 0.0 #1e1 # 10.0  #1e4 # 5.0  # Set to 0.0 to disable
    consistency_epsilon = 0.3  # Relative noise to input; Set to 0.0 to disable
    # Output symmetry regularization
    symmetry_lambda = 0.0  # Set to 0.0 to disable

    # Learning rate
    learning_rate = 1e-3  # for residual
    # learning_rate = 1e-2  # for residual
    # learning_rate = 1e-5  # for temporal
    # learning_rate = 1e-3  # for LR scheduling
    lr_milestones = [100, 150]
    lr_scheduler = "MultiStepLR"  # "ReduceLROnPlateau", "LambdaLR", "MultiStepLR", "LRScheduler", None

    # Number of workers, i.e., number of threads for loading data
    num_workers = 0

    if weight_decay != 0.0 and l1_lambda != 0.0:
        raise ValueError("Don't use both L1 and L2 regularization at the same time.")


class ModelFitConfig:
    # ------- Propagation -------
    prop_long_horizon = True

    # ------- Low-Pass Filter -------
    use_low_pass_filter = False
    low_pass_filter_cutoff_input = 1.0
    low_pass_filter_cutoff_label = 0.1

    # ------- Moving Average Filter -------
    use_moving_average_filter = False
    use_moving_average_filter_only_label = True
    control_filtering = False  # USE WAY SMALLER WINDOW SIZE IF TRUE!
    window_size = 5 #33  # Must be odd

    # ------- Coordinate Transform -------
    input_transform = False
    label_transform = False

    # ------- Pruning -------
    prune = False

    # Histogram pruning parameters
    histogram_n_bins = 10
    histogram_thresh = 0.001  # Remove bins where the total ratio of data is lower than threshold
    vel_cap = 16  # Remove datapoints where abs(velocity) > vel_cap

    # ------- Plotting -------
    plot_dataset = False
    save_plots = False

    # ------- Dataset loading -------
    train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_WITH_REF_ALL_PROP"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_FULL"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_TRAIN_FOR_PAPER"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_GROUND_EFFECT_ONLY"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "residual_dataset_neural_sim_nominal_control_07"
    # train_ds_name = "NMPCTiltQdServo" + "_" + "residual_dataset_06"
    train_ds_instance = "dataset_001"  # "dataset_020"
    # real machine 01, dataset 007: Large dataset from many old flights with mode 0 and other discrepancies (200k datapoints)
    # real machine 01, dataset 007: Large dataset from mode 10 with focus on ground effect (66k datapoints)
    # real machine 01, dataset 013: same as 007 but with fixed prop and dt
    # residual 06, dataset 001 on neural standalone
    # residual neural sim nominal control 03, dataset 001: first with correct solver building
    # residual neural sim nominal control 05, dataset 001: on simulator as large network trained with L1 regularization
    # residual neural sim nominal control 07, dataset 001: on simulator as large network trained with full state and transforms and L1 regularization
    # real machine GROUND_EFFECT_ONLY: hovering and ground effect data only (48k datapoints)
    # val_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_VAL_FOR_PAPER"
    val_ds_name = "NMPCTiltQdServo" + "_" + "real_machine" + "_dataset_VAL_WITH_REF_ALL_PROP"
    val_ds_instance = "dataset_001"
    # === FROM HERE WITH MOVING AVERAGE FILTER APPLIED ===
    # NMPCTiltQdServo_real_machine_dataset_GROUND_EFFECT_ONLY,  dataset_002
    # ------- Features used for the model -------
    # State features
    state_feats = [2]  # [z]
    state_feats.extend([3, 4, 5])  # [vx, vy, vz]
    state_feats.extend([6, 7, 8, 9])  # [qw, qx, qy, qz]
    # state_feats.extend([10, 11, 12])  # [roll_rate, pitch_rate, yaw_rate]
    # state_feats.extend([13, 14, 15, 16])  # [servo_angle_1, servo_angle_2, servo_angle_3, servo_angle_4]
    # state_feats.extend([17, 18, 19, 20, 21, 22])  # [fds_1, fds_2, fds_3, tau_ds_1, tau_ds_2, tau_ds_3]
    # state_feats.extend([17, 18, 19, 20])  # [thrust_1, thrust_2, thrust_3, thrust_4]

    # Control input features
    u_feats = [0, 1, 2, 3]  # [thrust_cmd_1, thrust_cmd_2, thrust_cmd_3, thrust_cmd_4]
    u_feats.extend([4, 5, 6, 7])  # [servo_angle_cmd_1, servo_angle_cmd_2, servo_angle_cmd_3, servo_angle_cmd_4]

    # Variables to be regressed
    y_reg_dims = []
    # y_reg_dims = [5]  # [az]
    # y_reg_dims.extend([0, 1, 2])  # [vx, vy, vz]
    y_reg_dims.extend([3, 4, 5])  # [ax, ay, az]
    # y_reg_dims.extend([6, 7, 8, 9])  # [qw_dot, qx_dot, qy_dot, qz_dot]
    # y_reg_dims.extend([10, 11, 12])  # [roll_acc, pitch_acc, yaw_acc]
