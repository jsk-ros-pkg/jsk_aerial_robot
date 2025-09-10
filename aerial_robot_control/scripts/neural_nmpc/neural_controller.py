import os, sys
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver

from utils.data_utils import delete_previous_solver_files
from utils.geometry_utils import quaternion_inverse, v_dot_q
from utils.model_utils import load_model, get_output_mapping, get_inverse_output_mapping
from utils.model_utils import cross_check_params

# Quadrotor
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import nmpc.nmpc_tilt_mt.tilt_qd.phys_param_beetle_omni as phys_omni

# - Naive models
from nmpc.nmpc_tilt_mt.archive.tilt_qd_no_servo_ac_cost import NMPCTiltQdNoServoAcCost
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_no_servo import NMPCTiltQdNoServo

# - Consider the servo delay with its model
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_servo import NMPCTiltQdServo
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_servo_dist import NMPCTiltQdServoDist
from nmpc.nmpc_tilt_mt.archive.tilt_qd_servo_drag_w_dist import NMPCTiltQdServoDragDist
from nmpc.nmpc_tilt_mt.archive.tilt_qd_servo_w_cog_end_dist import NMPCTiltQdServoWCogEndDist

# - Conside servo angle derivative as state
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_servo_diff import NMPCTiltQdServoDiff

# - Consider the absolute servo angle command in cost
from nmpc.nmpc_tilt_mt.archive.tilt_qd_servo_old_cost import NMPCTiltQdServoOldCost

# - Consider the thrust delay with its model
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_thrust import NMPCTiltQdThrust

# - Consider the servo & thrust delay with its models
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust import NMPCTiltQdServoThrust
from nmpc.nmpc_tilt_mt.tilt_qd.tilt_qd_servo_thrust_dist import NMPCTiltQdServoThrustDist
from nmpc.nmpc_tilt_mt.archive.tilt_qd_servo_thrust_drag import NMPCTiltQdServoThrustDrag

# Fixed Quadrotor
# from nmpc.nmpc_tilt_mt.fix_qd.fix_qd_angvel_out import NMPCFixQdAngvelOut
# from nmpc.nmpc_tilt_mt.fix_qd.fix_qd_thrust_out import NMPCFixQdThrustOut

# Birotor
# from nmpc.nmpc_tilt_mt.tilt_bi.tilt_bi_servo import NMPCTiltBiServo
# from nmpc.nmpc_tilt_mt.tilt_bi.tilt_bi_2ord_servo import NMPCTiltBi2OrdServo

# Trirotor
# from nmpc.nmpc_tilt_mt.tilt_tri.tilt_tri_servo import NMPCTiltTriServo
# from nmpc.nmpc_tilt_mt.tilt_tri.tilt_tri_servo_dist import NMPCTiltTriServoDist


class NeuralNMPC:
    def __init__(self, model_options, solver_options, sim_options, run_options, rdrv_d_mat=None):
        # TODO implement solver options flexibly
        # TODO run options should be removed, if possible
        """
        :param model_options: Dictionary containing model options, such as architecture type
        :param use_mlp: Flag to toggle use of a pre-trained MLP in the NMPC controller
        :param solver_options: Set of additional options dictionary for acados solver
        :param rdrv_d_mat: 3x3 matrix that corrects the drag with a linear model
                           according to Faessler et al. 2018
        """
        self.model_options = model_options
        # Nominal model and solver
        # TODO pass down solver options
        self.arch_type = model_options["arch_type"]
        self.nmpc_type = model_options["nmpc_type"]
        if model_options["only_use_nominal"]:
            identifier = "nominal"
            identifier2 = ""
        else:
            identifier = "neural"
            identifier2 = ""
            if "minus_neural" in model_options:
                if model_options["minus_neural"]:
                    identifier2 = "_minus"

            if "plus_neural" in model_options:
                if model_options["plus_neural"]:
                    identifier2 = "_plus"

        delete_previous_solver_files(model_options, identifier)

        # Include disturbance parameters in model
        if sim_options["disturbances"]["cog_dist"]:
            include_cog_dist_parameter = True
        else:
            include_cog_dist_parameter = False
        if sim_options["disturbances"]["motor_noise"]:
            include_motor_noise_parameter = True
        else:
            include_motor_noise_parameter = False
        if solver_options["include_floor_bounds"]:
            include_floor_bounds = True
        else:
            include_floor_bounds = False
        if solver_options["include_soft_constraints"]:
            include_soft_constraints = True
        else:
            include_soft_constraints = False
        if solver_options["include_quaternion_constraint"]:
            include_quaternion_constraint = True
        else:
            include_quaternion_constraint = False

        if self.arch_type == "qd":
            if self.nmpc_type == "NMPCTiltQdNoServo":
                raise NotImplementedError(
                    "When using this controller the simulation is unstable! \
                    The control input in init step is weirdly only 15N when \
                    given a 30N maximum... Need to investigate error, \
                    Check with using Servo yaml"
                )
                self.nmpc = NMPCTiltQdNoServo(
                    model_name=f"tilt_qd_{identifier}_no_servo{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltQdServo":
                self.nmpc = NMPCTiltQdServo(
                    model_name=f"tilt_qd_{identifier}_servo{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltQdThrust":
                self.nmpc = NMPCTiltQdThrust(
                    model_name=f"tilt_qd_{identifier}_thrust{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltQdServoThrust":
                self.nmpc = NMPCTiltQdServoThrust(
                    model_name=f"tilt_qd_{identifier}_servo_thrust{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )

            elif self.nmpc_type == "NMPCTiltQdServoDist":
                self.nmpc = NMPCTiltQdServoDist(
                    model_name=f"tilt_qd_{identifier}_servo_dist{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltQdServoThrustDist":
                self.nmpc = NMPCTiltQdServoThrustDist(
                    model_name=f"tilt_qd_{identifier}_servo_thrust_dist{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )

            # Fixed rotor models
            elif self.nmpc_type == "NMPCFixQdAngvelOut":
                raise NotImplementedError("Not implemented yet.")
                self.nmpc = NMPCFixQdAngvelOut(
                    model_name="fix_qd_neural_angvel_out_mdl",
                    method="neural_nmpc",
                    build=False,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCFixQdThrustOut":
                raise NotImplementedError("Not implemented yet.")
                self.nmpc = NMPCFixQdThrustOut(
                    model_name="fix_qd_neural_thrust_out_mdl",
                    method="neural_nmpc",
                    build=False,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )

            # Archived methods
            elif self.nmpc_type == "NMPCTiltQdNoServoAcCost":
                self.nmpc = NMPCTiltQdNoServoAcCost(
                    model_name=f"tilt_qd_{identifier}_no_servo_ac_cost{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltQdServoOldCost":
                self.nmpc = NMPCTiltQdServoOldCost(
                    model_name=f"tilt_qd_{identifier}_servo_old_cost{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltQdServoDiff":
                self.nmpc = NMPCTiltQdServoDiff(
                    model_name=f"tilt_qd_{identifier}_servo_diff{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
                alpha_integ = np.zeros(4)  # TODO not implemented yet
            elif self.nmpc_type == "NMPCTiltQdServoDragDist":
                self.nmpc = NMPCTiltQdServoDragDist(
                    model_name=f"tilt_qd_{identifier}_servo_drag_dist{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltQdServoThrustDrag":
                self.nmpc = NMPCTiltQdServoThrustDrag(
                    model_name=f"tilt_qd_{identifier}_servo_thrust_drag{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltQdServoWCogEndDist":
                self.nmpc = NMPCTiltQdServoWCogEndDist(
                    model_name=f"tilt_qd_{identifier}_servo_thrust_drag{identifier2}_mdl",
                    method="neural_nmpc",
                    build=False,
                    phys=phys_omni,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            else:
                raise ValueError(f"Invalid control NMPC model {self.nmpc_type}.")
        elif self.arch_type == "bi":
            raise NotImplementedError("Not implemented yet.")
            if self.nmpc_type == "NMPCTiltBiServo":
                self.nmpc = NMPCTiltBiServo(
                    model_name="tilt_bi_neural_servo_mdl",
                    method="neural_nmpc",
                    build=False,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltBi2OrdServo":
                self.nmpc = NMPCTiltBi2OrdServo(
                    model_name="tilt_bi2_ord_neural_servo_mdl",
                    method="neural_nmpc",
                    build=False,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            else:
                raise ValueError(f"Invalid control NMPC model {self.nmpc_type}.")
        elif self.arch_type == "tri":
            raise NotImplementedError("Not implemented yet.")
            if self.nmpc_type == "NMPCTiltTriServo":
                self.nmpc = NMPCTiltTriServo(
                    model_name="tilt_tri_neural_servo_mdl",
                    method="neural_nmpc",
                    build=False,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            elif self.nmpc_type == "NMPCTiltTriServoDist":
                self.nmpc = NMPCTiltTriServoDist(
                    model_name="tilt_tri_neural_servo_dist_mdl",
                    method="neural_nmpc",
                    build=False,
                    include_cog_dist_parameter=include_cog_dist_parameter,
                    include_motor_noise_parameter=include_motor_noise_parameter,
                    include_soft_constraints=include_soft_constraints,
                    include_quaternion_constraint=include_quaternion_constraint,
                    include_floor_bounds=include_floor_bounds,
                )
            else:
                raise ValueError(f"Invalid control NMPC model {self.nmpc_type}.")
        else:
            raise ValueError("Invalid architecture type specified.")

        # Get OCP object from NMPC
        self.ocp = self.nmpc.get_ocp()
        self.T_samp = self.nmpc.params["T_samp"]  # Sampling time for the NMPC controller, time step between two steps
        self.T_horizon = self.nmpc.params["T_horizon"]  # Time horizon for optimization loop in MPC controller
        self.N = self.nmpc.params["N_steps"]  # Number of MPC nodes
        self.T_step = self.nmpc.params["T_step"]  # Step size used in optimization loop in MPC controller
        # TODO set somewhere else and by default set to T_samp/2

        # Get OCP model from NMPC TODO implement drag correction with RDRv
        self.nominal_model = self.nmpc.get_acados_model()
        self.model_name = self.nominal_model.name  # Already set at initialization of sub-controller
        self.state = self.nominal_model.x
        self.controls = self.nominal_model.u

        # Get OCP solver from NMPC
        self.nominal_solver = self.nmpc.get_ocp_solver()
        self.parameters = self.nominal_model.p
        self.nx = self.nominal_solver.acados_ocp.dims.nx
        self.nu = self.nominal_solver.acados_ocp.dims.nu

        if not model_options["only_use_nominal"]:
            self.use_mlp = True
            # Load pre-trained MLP
            self.neural_model, self.mlp_metadata = load_model(model_options, sim_options, run_options)
            # Cross-check weights and meta parameters used for MPC to train MLP
            cross_check_params(self.nmpc.params, self.mlp_metadata)

            # ========================================================================================================
            # import torch
            # sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))))
            # import ml_casadi_original.torch as mc_original

            # class NormalizedMLP(mc_original.TorchMLCasadiModule):
            #     def __init__(self, model, x_mean, x_std, y_mean, y_std):
            #         super().__init__()
            #         self.model = model
            #         self.input_size = self.model.input_size
            #         self.output_size = self.model.output_size
            #         self.register_buffer('x_mean', x_mean)
            #         self.register_buffer('x_std', x_std)
            #         self.register_buffer('y_mean', y_mean)
            #         self.register_buffer('y_std', y_std)

            #     def forward(self, x):
            #         return (self.model((x - self.x_mean) / self.x_std) * self.y_std) + self.y_mean

            #     def cs_forward(self, x):
            #         return (self.model((x - self.x_mean.cpu().numpy()) / self.x_std.cpu().numpy()) * self.y_std.cpu().numpy()) + self.y_mean.cpu().numpy()

            # directory, file_name = (
            #     "/home/johannes/ros/neural-mpc/ros_dd_mpc/results/model_fitting/5f15661/simple_sim_mlp",
            #     "drag__motor_noise__noisy__no_payload.pt",
            # )
            # saved_dict = torch.load(os.path.join(directory, f"{file_name}"))
            # mlp_model = mc_original.nn.MultiLayerPerceptron(saved_dict['input_size'], saved_dict['hidden_size'],
            #                                        saved_dict['output_size'], saved_dict['hidden_layers'], 'Tanh')
            # model = NormalizedMLP(mlp_model, torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
            #                       torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
            #                       torch.tensor(np.zeros((saved_dict['output_size'],))).float(),
            #                       torch.tensor(np.zeros((saved_dict['output_size'],))).float())
            # model.load_state_dict(saved_dict['state_dict'])
            # model.eval()
            # self.neural_model = model
            # self.mlp_metadata = {
            #     "ModelFitConfig": {
            #         "state_feats": str(list(range(13))),
            #         "u_feats": str(list(range(4))),
            #         "y_reg_dims": str(list(range(13))),
            #         "label_transform": True
            #     },
            #     "MLPConfig": {
            #         "model_name": "test",
            #         "delay_horizon": 0
            #     }
            # }
            # ========================================================================================================
        else:
            self.use_mlp = False

        # Add neural network model to nominal model
        self.extend_acados_model()

        # Extend the acados solver with the extended model
        self.extend_acados_solver()

    def get_reference_generator(self):
        return self.nmpc.get_reference_generator()

    def extend_acados_model(self):
        # -----------------------------------------------------
        # TODO Understand and implement correctly
        # state = self.gp_x * self.trigger_var + self.x * (1 - self.trigger_var)
        # -----------------------------------------------------

        # Here f is already symbolically evaluated to f(x,u)
        nominal_dynamics = self.nominal_model.f_expl_expr
        if not self.use_mlp:
            f_total = nominal_dynamics
        else:
            # === MLP input ===
            self.state_feats = eval(self.mlp_metadata["ModelFitConfig"]["state_feats"])
            self.u_feats = eval(self.mlp_metadata["ModelFitConfig"]["u_feats"])
            self.y_reg_dims = np.array(eval(self.mlp_metadata["ModelFitConfig"]["y_reg_dims"]))
            if np.array_equal(self.y_reg_dims, np.array([5])):
                only_vz = True
            else:
                only_vz = False

            # MLP is trained to receive and predict the velocity in the body frame
            # Transform input velocity to body frame
            v_b = v_dot_q(self.state[3:6], quaternion_inverse(self.state[6:10]))
            state_b = ca.vertcat(self.state[:3], v_b, self.state[6:])

            mlp_in = ca.vertcat(state_b[self.state_feats], self.controls[self.u_feats])

            if "temporal" in self.mlp_metadata["MLPConfig"]["model_name"]:
                # Use previous (i.e. delayed) states and controls time steps as input to the MLP
                delay = self.mlp_metadata["MLPConfig"]["delay_horizon"]  # Delay as number of previous time steps
                state_prev, controls_prev = self.append_delay(delay)

                # Append each previous state and control to MLP input
                for i in range(delay):
                    # States are sorted from newest to oldest in acados parameters
                    # Get state components from previous states that are stored concatenated after each other
                    state_prev_i = state_prev[:, i]
                    controls_prev_i = controls_prev[:, i]
                    # Transform input velocity to body frame
                    v_b = v_dot_q(state_prev_i[3:6], quaternion_inverse(state_prev_i[6:10]))
                    state_prev_i[3:6] = v_b  # Replace velocity in world frame with body frame velocity

                    mlp_in = ca.vertcat(mlp_in, state_prev_i[self.state_feats], controls_prev_i[self.u_feats])

            # === MLP forward pass ===
            if self.model_options["approximate_mlp"]:
                # TODO investigate this function and parallel Flag!
                # Parallel flag only active for order == 2
                mlp_out = self.neural_model.approx(mlp_in, order=self.model_options["approx_order"], parallel=False)
                approx_params = self.neural_model.sym_approx_params(order=self.model_options["approx_order"], flat=True)
                self.approx_start_idx = self.parameters.size()[0]
                self.parameters = ca.vertcat(self.parameters, approx_params)
                self.approx_end_idx = self.parameters.size()[0]
            else:
                mlp_out = self.neural_model(mlp_in)

            if self.mlp_metadata["ModelFitConfig"]["label_transform"]:
                # Transform velocity back to world frame
                if set([3, 4, 5]).issubset(set(self.y_reg_dims)):
                    v_idx = np.where(self.y_reg_dims == 3)[0][0]  # Assumed that v_x, v_y, v_z are consecutive
                    v_b = mlp_out[v_idx : v_idx + 3]
                    v_w = v_dot_q(v_b, self.state[6:10])
                    mlp_out = ca.vertcat(mlp_out[:v_idx, :], v_w, mlp_out[v_idx + 3 :, :])
                elif set([4, 5]).issubset(set(self.y_reg_dims)):
                    raise NotImplementedError("Adjust mapping through M like for only vz")
                    v_idx = np.where(self.y_reg_dims == 4)[0][0]  # Assumed that v_y, v_z are consecutive
                    v_b = ca.vertcat(0, mlp_out[v_idx : v_idx + 2])
                    v_w = v_dot_q(v_b, self.state[6:10])
                    mlp_out = ca.vertcat(mlp_out[:v_idx, :], v_w, mlp_out[v_idx + 2 :, :])
                elif set([5]).issubset(set(self.y_reg_dims)):
                    # Predict only v_z so set v_x and v_y to 0 in Body frame and then transform to World frame
                    # The predicted v_z therefore also has influence on the x and y velocities in World frame
                    # Adjust mapping later on
                    v_idx = np.where(self.y_reg_dims == 5)[0][0]
                    v_b = ca.vertcat(0, 0, mlp_out[v_idx])
                    v_w = v_dot_q(v_b, self.state[6:10])
                    mlp_out = ca.vertcat(mlp_out[:v_idx, :], v_w, mlp_out[v_idx + 1 :, :])

            # === Fuse dynamics ===
            if self.model_options["end_to_end_mlp"]:
                # MLP is trained to predict the state end-to-end without adding to the nominal dynamics
                # If some state dimensions are not predicted by the MLP, use the nominal dynamics
                M = get_output_mapping(self.state.shape[0], self.y_reg_dims)
                O = get_inverse_output_mapping(self.state.shape[0], self.y_reg_dims)

                # Combine nominal dynamics with neural dynamics
                f_total = O @ nominal_dynamics + M @ mlp_out
            else:
                # Map output of MLP to the state space
                M = get_output_mapping(self.state.shape[0], self.y_reg_dims, only_vz=only_vz)

                # Combine nominal dynamics with neural dynamics
                f_total = nominal_dynamics + M @ mlp_out
                # f_total = nominal_dynamics - M @ mlp_out

                if "minus_neural" in self.model_options:
                    if self.model_options["minus_neural"]:
                        f_total = nominal_dynamics - M @ mlp_out
                if "plus_neural" in self.model_options:
                    if self.model_options["plus_neural"]:
                        f_total = nominal_dynamics + M @ mlp_out

        # Implicit dynamics
        x_dot = ca.MX.sym("x_dot", self.state.size())
        f_impl = x_dot - f_total

        # Cost function TODO think this over!
        state_y, state_y_e, control_y = self.nmpc.get_cost_function()

        # Assemble acados model
        self.acados_model = AcadosModel()
        self.acados_model.name = self.model_name
        self.acados_model.f_expl_expr = f_total  # CasADi expression for the explicit dynamics
        self.acados_model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        self.acados_model.x = self.state
        self.acados_model.xdot = x_dot
        self.acados_model.u = self.controls
        self.acados_model.p = self.parameters
        self.acados_model.cost_y_expr_0 = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        self.acados_model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        self.acados_model.cost_y_expr_e = state_y_e

        if self.nmpc.include_quaternion_constraint:
            # TODO is this necessary to include here???
            # Note: This is necessary to ensure that the quaternion stays on the unit sphere
            # and represents a valid rotation.
            self.acados_model.con_h_expr = (
                self.state[6] ** 2 + self.state[7] ** 2 + self.state[8] ** 2 + self.state[9] ** 2 - 1.0
            )  # ||q||^2 - 1 = 0
            self.acados_model.con_h_expr_e = self.acados_model.con_h_expr

    def extend_acados_solver(self):
        """
        Create a new solver with the extended model.
        """
        # Get the same OCP object from NMPC for the OCP cost, constraints and settings
        _ocp = self.nmpc.get_ocp()

        # Overwrite the OCP model with the extended model
        _ocp.model = self.acados_model

        # =====================================================================
        # Initial state and reference: Set all values such that robot is hovering
        # TODO debatable which initial states/inputs make sense -> not necessarily better than just all-zero!
        x_ref = np.zeros(_ocp.dims.nx)
        x_ref[6] = 1.0  # Quaternion qw

        if self.nmpc.tilt:
            # When included servo AND thrust, use further indices
            if self.nmpc.include_servo_model and self.nmpc.include_thrust_model:
                x_ref[17:21] = self.nmpc.phys.mass * self.nmpc.phys.gravity / 4  # ft1s, ft2s, ft3s, ft4s
            # When only included thrust, use the same indices
            elif self.nmpc.include_thrust_model:
                x_ref[13:17] = self.nmpc.phys.mass * self.nmpc.phys.gravity / 4  # ft1s, ft2s, ft3s, ft4s
        else:
            x_ref[13:17] = self.nmpc.phys.mass * self.nmpc.phys.gravity / 4  # ft1s, ft2s, ft3s, ft4s

        u_ref = np.zeros(_ocp.dims.nu)
        # Obeserved to be worse than zero!
        u_ref[0:4] = self.nmpc.phys.mass * self.nmpc.phys.gravity / 4  # ft1c, ft2c, ft3c, ft4c

        _ocp.constraints.x0 = x_ref
        _ocp.cost.yref = np.concatenate((x_ref, u_ref))
        _ocp.cost.yref_e = x_ref

        # TODO OVERTHINK PARAMETERS OF OCP!!!!
        # Adjust parameters
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        self.acados_parameters = np.zeros((_ocp.dims.N + 1, _ocp.dims.np))
        self.acados_parameters[:, 0:4] = x_ref[6:10]  # For nonlinear quaternion error
        if len(self.nmpc.phys.physical_param_list) != 31:
            raise ValueError("Physical parameters are not as expected. Please check the physical model.")
        self.acados_parameters[:, 4 : 4 + len(self.nmpc.phys.physical_param_list)] = np.array(
            self.nmpc.phys.physical_param_list
        )

        _ocp.dims.np = self.acados_model.p.size()[0]  # Number of parameters
        _ocp.parameter_values = self.acados_parameters[0, :]
        # =====================================================================

        # Build acados ocp into current working directory (which was created in super class)
        json_file_path = os.path.join("./" + _ocp.model.name + "_acados_ocp.json")
        self.ocp_solver = AcadosOcpSolver(_ocp, json_file=json_file_path, build=True)
        print("Generated C code for acados solver successfully to " + os.getcwd())

        # TODO Extend constraints ?
        # TODO Extend cost function ?
        # TODO CHECK SOLVER OPTIONS ?

    def create_acados_sim_solver(self, T_sim) -> AcadosSimSolver:
        """
        Create a simulation solver based on the acados model.
        :param T_sim: Discretized time-step for the aerial robot simulation
        """
        self.T_sim = T_sim
        # Create a simulation solver based on the acados model
        acados_sim = AcadosSim()
        acados_sim.model = self.acados_model

        # =====================================================================
        # TODO OVERTHINK PARAMETERS OF SOLVER!!!!
        # Set the initial parameters for the simulation
        n_param = self.acados_model.p.size()[0]
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        # TODO set this elsewhere since its confusing where this gets modified/accessed
        self.sim_acados_parameters = np.zeros(n_param)
        self.sim_acados_parameters[0] = 1.0  # qw
        self.sim_acados_parameters[4 : 4 + len(self.nmpc.phys.physical_param_list)] = np.array(
            self.nmpc.phys.physical_param_list
        )
        acados_sim.parameter_values = self.sim_acados_parameters
        # =====================================================================

        # Set the horizon for the simulation
        acados_sim.solver_options.T = T_sim
        acados_sim.solver_options.num_steps = 1  # Default TODO think about increasing this to increase accuracy

        # Generate solver
        json_file_path = os.path.join("./" + self.acados_model.name + "_acados_sim.json")
        sim_solver = AcadosSimSolver(acados_sim, json_file=json_file_path, build=True)
        print("Generated C code for sim solver successfully to " + os.getcwd())
        return sim_solver

    def track(self, xr, ur):
        """
        Tracks a trajectory defined by xr and ur, where xr is the reference state and ur is the reference control input.
        :param xr: Reference state trajectory (N+1, state_dim)
        :param ur: Reference control input trajectory (N, control_dim)
        """
        # 0 ~ N-1
        for j in range(self.ocp_solver.N):
            yr = np.concatenate((xr[j, :], ur[j, :]))
            self.ocp_solver.set(j, "yref", yr)
            quaternion_ref = xr[j, 6:10]
            self.acados_parameters[j, 0:4] = quaternion_ref  # For nonlinear quaternion error

        # N
        yr = xr[self.ocp_solver.N, :]
        self.ocp_solver.set(self.ocp_solver.N, "yref", yr)
        quaternion_ref = xr[self.ocp_solver.N, 6:10]
        self.acados_parameters[self.ocp_solver.N, 0:4] = quaternion_ref  # For nonlinear quaternion error

    def append_delay(self, delay: int = 0):
        """
        Append delay of the state and control as parameters to the acados model and solver.
        The additional states and controls are used as input to the neural network to account for temporal dependencies.
        Note, the states and input pairs are defined to be sorted from newest to oldest in parameters.
        :param delay: Number of previous time steps to include as the model input.
        """
        state_prev = ca.MX()
        controls_prev = ca.MX()

        # Store the previous state and control as parameters in the model
        self.delay_start_idx = self.parameters.size()[0]
        for i in range(1, delay):
            # Create symbolic variables for the previous state and control
            state_prev_i = ca.MX.sym(f"state_prev_{i}", self.nx)
            controls_prev_i = ca.MX.sym(f"controls_prev_{i}", self.nu)

            # Concatenate state and input together for each time step
            self.parameters = ca.vertcat(self.parameters, state_prev_i, controls_prev_i)
            state_prev = ca.horzcat(state_prev, state_prev_i)
            controls_prev = ca.horzcat(controls_prev, controls_prev_i)
        self.delay_end_idx = self.parameters.size()[0]
        return state_prev, controls_prev

    def simulate_trajectory(self, sim_solver, state_curr):
        """
        Simulate trajectory using optimal control sequence from the OCP solver.
        """
        # Number of steps in the OCP
        N = self.ocp_solver.acados_ocp.dims.N

        # Get all control inputs from OCP solution
        u_sequence = np.array([self.ocp_solver.get(i, "u") for i in range(N)])

        # Simulate forward
        state_traj = np.zeros((N + 1, state_curr.shape[0]))
        state_traj[0, :] = state_curr

        for n in range(N):
            state_curr = sim_solver.simulate(x=state_curr, u=u_sequence[n])
            state_traj[n + 1, :] = state_curr

        return state_traj

    def check_state_constraints(self, state_curr, i):
        # Boundary constraints
        for idx in self.ocp_solver.acados_ocp.constraints.idxbx:
            lbxi = np.where(self.ocp_solver.acados_ocp.constraints.idxbx == idx)[0][0]
            if (
                state_curr[idx] < self.ocp_solver.acados_ocp.constraints.lbx[lbxi] * 1.01
                or state_curr[idx] > self.ocp_solver.acados_ocp.constraints.ubx[lbxi] * 1.01
            ):
                print(
                    f"Warning: Constraint violation at index {idx} in simulation step {i}. "
                    f"Value: {state_curr[idx]:.14f}, "
                    f"Lower bound: {self.ocp_solver.acados_ocp.constraints.lbx[lbxi]}, "
                    f"Upper bound: {self.ocp_solver.acados_ocp.constraints.ubx[lbxi]}"
                )
        # Nonlinear unit quaternion constraint
        quat_norm = np.linalg.norm(state_curr[6:10])
        if quat_norm < 0.999 or quat_norm > 1.001:
            print(
                f"Warning: Constraint violation for unit_q in simulation step {i}. "
                f"Value: {quat_norm:.14f} != 1.0, "
                f"Quaternion: {state_curr[6:10]}"
            )

    def check_input_constraints(self, u_cmd, i):
        """
        Check if the control input command u_cmd respects the constraints of the system.
        """
        if np.any(u_cmd[:4]) < self.nmpc.params["thrust_min"] or np.any(u_cmd[:4]) > self.nmpc.params["thrust_max"]:
            print(f"=== Control thrust input violates constraints: {u_cmd[:4]} at index {i} ===")
            raise ValueError("Control thrust input violates constraints.")
        if u_cmd.shape[0] > 4:
            if np.any(u_cmd[4:]) < self.nmpc.params["a_min"] or np.any(u_cmd[4:]) > self.nmpc.params["a_max"]:
                print(f"=== Control servo angle input violates constraints: {u_cmd[4:]} at index {i} ===")
                raise ValueError("Control servo angle input violates constraints.")

    def get_rotor_positions(self):
        """
        Get the positions of the rotors in the body frame.
        :return: 3x4 matrix containing the positions of the rotors in the body frame.
        """
        if self.arch_type == "bi":
            return np.array([self.nmpc.phys.p1_b, self.nmpc.phys.p2_b])
        elif self.arch_type == "tri":
            return np.array([self.nmpc.phys.p1_b, self.nmpc.phys.p2_b, self.nmpc.phys.p3_b])
        elif self.arch_type == "qd":
            return np.array([self.nmpc.phys.p1_b, self.nmpc.phys.p2_b, self.nmpc.phys.p3_b, self.nmpc.phys.p4_b])
