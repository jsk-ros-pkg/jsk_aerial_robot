import os, sys
from typing import NamedTuple
import numpy as np
import torch
import casadi as ca
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import torch

import ml_casadi.torch as mc
from network_architecture.normalized_mlp import NormalizedMLP

from utils.data_utils import get_model_dir_and_file

# Quadrotor
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
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
from nmpc.nmpc_tilt_mt.fix_qd.fix_qd_angvel_out import NMPCFixQdAngvelOut
from nmpc.nmpc_tilt_mt.fix_qd.fix_qd_thrust_out import NMPCFixQdThrustOut

# Birotor
from nmpc.nmpc_tilt_mt.tilt_bi.tilt_bi_servo import NMPCTiltBiServo
from nmpc.nmpc_tilt_mt.tilt_bi.tilt_bi_2ord_servo import NMPCTiltBi2OrdServo

# Trirotor
from nmpc.nmpc_tilt_mt.tilt_tri.tilt_tri_servo import NMPCTiltTriServo
from nmpc.nmpc_tilt_mt.tilt_tri.tilt_tri_servo_dist import NMPCTiltTriServoDist

class NeuralNMPC():
    def __init__(self, model_options, solver_options, sim_options,
                 T_sim=0.005, rdrv_d_mat=None):
        # TODO: Introduce approximated MLP
        # TODO: Add GP regressor
        # TODO: Load / save trained model -> overwrite model
        # TODO implement solver options flexibly
        """
        :param T_sim: Discretized time-step for the aerial robot simulation
        :param model_options: Dictionary containing model options, such as architecture type
        :param use_mlp: Flag to toggle use of a pre-trained MLP in the NMPC controller
        :param solver_options: Set of additional options dictionary for acados solver
        :param rdrv_d_mat: 3x3 matrix that corrects the drag with a linear model
                           according to Faessler et al. 2018
        """
        # Nominal model and solver
        # TODO pass down solver options
        self.arch_type = model_options["arch_type"]
        self.nmpc_type = model_options["nmpc_type"]
        if self.arch_type == "qd":
            if self.nmpc_type == "NMPCTiltQdNoServo":
                self.nmpc = NMPCTiltQdNoServo(build=False)
            elif self.nmpc_type == "NMPCTiltQdServo":
                self.nmpc = NMPCTiltQdServo(build=False)
            elif self.nmpc_type == "NMPCTiltQdThrust":
                self.nmpc = NMPCTiltQdThrust(build=False)
            elif self.nmpc_type == "NMPCTiltQdServoThrust":
                self.nmpc = NMPCTiltQdServoThrust(build=False)

            elif self.nmpc_type == "NMPCTiltQdServoDist":
                self.nmpc = NMPCTiltQdServoDist(build=False)
            elif self.nmpc_type == "NMPCTiltQdServoThrustDist":
                self.nmpc = NMPCTiltQdServoThrustDist(build=False)

            # Fixed rotor models
            elif self.nmpc_type == "NMPCFixQdAngvelOut":
                self.nmpc = NMPCFixQdAngvelOut(build=False)
            elif self.nmpc_type == "NMPCFixQdThrustOut":
                self.nmpc = NMPCFixQdThrustOut(build=False)

            # Archived methods
            elif self.nmpc_type == "NMPCTiltQdNoServoAcCost":
                self.nmpc = NMPCTiltQdNoServoAcCost(build=False)
            elif self.nmpc_type == "NMPCTiltQdServoOldCost":
                self.nmpc = NMPCTiltQdServoOldCost(build=False)
            elif self.nmpc_type == "NMPCTiltQdServoDiff":
                self.nmpc = NMPCTiltQdServoDiff(build=False)
                alpha_integ = np.zeros(4)   # TODO not implemented yet
            elif self.nmpc_type == "NMPCTiltQdServoDragDist":
                self.nmpc = NMPCTiltQdServoDragDist(build=False)
            elif self.nmpc_type == "NMPCTiltQdServoThrustDrag":
                self.nmpc = NMPCTiltQdServoThrustDrag(build=False)
            elif self.nmpc_type == "NMPCTiltQdServoWCogEndDist":
                self.nmpc = NMPCTiltQdServoWCogEndDist(build=False)
            else:
                raise ValueError(f"Invalid control NMPC model {self.nmpc_type}.")
        elif self.arch_type == 'bi':

            if self.nmpc_type == "NMPCTiltBiServo":
                self.nmpc = NMPCTiltBiServo(build=False)
            elif self.nmpc_type == "NMPCTiltBi2OrdServo":
                self.nmpc = NMPCTiltBi2OrdServo(build=False)
            else:
                raise ValueError(f"Invalid control NMPC model {self.nmpc_type}.")
        elif self.arch_type == 'tri':
            if self.nmpc_type == "NMPCTiltTriServo":
                self.nmpc = NMPCTiltTriServo(build=False)
            elif self.nmpc_type == "NMPCTiltTriServoDist":
                self.nmpc = NMPCTiltTriServoDist(build=False)
            else:
                raise ValueError(f"Invalid control NMPC model {self.nmpc_type}.")
        else:
            raise ValueError("Invalid architecture type specified.")

        # Get OCP object from NMPC
        self.ocp = self.nmpc.get_ocp()
        self.T_samp = self.nmpc.params["T_samp"]        # Sampling time for the NMPC controller, i.e., time step between two successive optimizations
        self.T_horizon = self.nmpc.params["T_horizon"]  # Time horizon for optimization loop in MPC controller
        self.N = self.nmpc.params["N_steps"]            # Number of MPC nodes
        self.T_step = self.nmpc.params["T_step"]        # Step size used in optimization loop in MPC controller
        # TODO set somewhere else and by default set to T_samp/2
        self.T_sim = T_sim                  # TODO set somewhere else! Discretized time-step for the aerial robot simulation
        
        # Get OCP model from NMPC TODO implement drag correction with RDRv
        self.nominal_model = self.nmpc.get_acados_model()
        self.state = self.nominal_model.x
        self.controls = self.nominal_model.u
        self.parameters = self.nominal_model.p

        # Get OCP solver from NMPC
        self.nominal_solver = self.nmpc.get_ocp_solver()

        # Set name of the controller
        self.model_name = "neural_" + self.nominal_model.name

        # Load pre-trained MLP
        # Note: If no pre-trained model is given, the controller will only use the nominal model
        if model_options["use_mlp"]:
            self.neural_model, self.mlp_config = self.load_model(model_options, sim_options)
            self.mlp_approx = None
        else:
            self.neural_model = None
            self.mlp_config = None
            self.mlp_approx = None

        # Load pre-trained RDRv model
        # Not use MLP AND RDRv at the same time!
        # rdrv_d = load_rdrv(model_options=load_ops)

        # Add neural network model to nominal model 
        self.extend_acados_model()
        # Extend the acados solver with the extended model
        self.extend_acados_solver()
        # Create sim solver for the extended model
        self.create_acados_sim_solver()
    
    def get_reference_generator(self):
        return self.nmpc.get_reference_generator()

    def extend_acados_model(self):
        # TODO Important!!! Understand approximation --- Propietary library used here
        # if not self.mlp_conf['approximated']:
        #     params = ca.vertcat(self.gp_x, self.trigger_var)
        # else:
        #     params = ca.vertcat(self.gp_x, self.trigger_var,
        #                         self.mlp_regressor.sym_approx_params(order=self.mlp_conf['approx_order'],
        #                                                                 flat=True))

        # -----------------------------------------------------
        # TODO Understand and implement correctly
        # state = self.gp_x * self.trigger_var + self.x * (1 - self.trigger_var)
        # #  Transform velocity to body frame
        # v_b = v_dot_q(state[7:10], quaternion_inverse(state[3:7]))
        # state = ca.vertcat(state[:7], v_b, state[10:])
        # mlp_in = v_b
        # -----------------------------------------------------

        # Adjust input vector
        # if self.mlp_conf['torque_output']:
        #     mlp_in = ca.vertcat(mlp_in, state[10:])

        # if self.mlp_conf['u_inp']:
        #     mlp_in = ca.vertcat(mlp_in, self.controls)

        # if self.mlp_conf['ground_map_input']:
        #     map_conf = GroundEffectMapConfig
        #     map = GroundMapWithBox(np.array(map_conf.box_min),
        #                             np.array(map_conf.box_max),
        #                             map_conf.box_height,
        #                             horizon=map_conf.horizon,
        #                             resolution=map_conf.resolution)

        #     self._map_res = map_conf.resolution

        #     self._static_ground_map, self._org_to_map_org = map.at(np.array(map_conf.origin))
        #     ground_map_dx = ca.MX(self._static_ground_map)

        #     idx = ca.DM(np.arange(0, 3, 1))

        #     x, y, z = state[0], state[1], state[2]
        #     orientation = state[3:7]

        #     x_idxs = ca.floor((x - self._org_to_map_org[0]) / map_conf.resolution) + idx - 1
        #     y_idxs = ca.floor((y - self._org_to_map_org[1]) / map_conf.resolution) + idx - 1
        #     ground_patch = ground_map_dx[x_idxs, y_idxs]

        #     relative_ground_patch = z - ground_patch
        #     relative_ground_patch = 4 * (ca.fmax(ca.fmin(relative_ground_patch, 0.5), 0.0) - 0.25)

        #     ground_effect_in = ca.vertcat(ca.reshape(relative_ground_patch, 9, 1), orientation*0)

        #     mlp_in = ca.vertcat(mlp_in, ground_effect_in)

        # # TODO Important!!! Understand approximation --- Propietary library used here
        # if not self.mlp_conf['approximated']:
        #     mlp_out = self.mlp_regressor(mlp_in)
        # else:
        #     mlp_out = self.mlp_regressor.approx(mlp_in, order=self.mlp_conf['approx_order'], parallel=False)

        # # Unpack prediction outputs. Transform back to world reference frame
        # if self.mlp_conf['torque_output']:
        #     # Append torque if needed 
        #     mlp_out_force = mlp_out[:3]
        #     mlp_out_torque = mlp_out[3:]
        #     mlp_out_means = ca.vertcat(v_dot_q(mlp_out_force, state[3:7]), mlp_out_torque)
        # else:
        #     mlp_out_means = v_dot_q(mlp_out, state[3:7])


        

        # TEMPORARY
        if self.neural_model:
            self.neural_model(mlp_in=torch.zeros((1, self.neural_model.input_size)))
        mlp_out = np.zeros(self.state.size())

        # Explicit dynamics
        # Here f is already symbolically evaluated to f(x,u)
        nominal_dynamics = self.nominal_model.f_expl_expr
        f_neural = nominal_dynamics + mlp_out
        
        # Implicit dynamics
        x_dot = ca.SX.sym('x_dot', self.state.size())
        f_impl = x_dot - f_neural

        # Cost function TODO think this over!
        state_y, state_y_e, control_y = self.nmpc.get_cost_function()
        
        # Assemble acados model
        self.acados_model = AcadosModel()
        self.acados_model.name = self.model_name
        self.acados_model.f_expl_expr = f_neural  # CasADi expression for the explicit dynamics
        self.acados_model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        self.acados_model.x = self.state  # State vector
        self.acados_model.xdot = x_dot
        self.acados_model.u = self.controls
        self.acados_model.p = self.parameters
        self.acados_model.cost_y_expr_0 = ca.vertcat(state_y, control_y)    # NONLINEAR_LS
        self.acados_model.cost_y_expr = ca.vertcat(state_y, control_y)      # NONLINEAR_LS
        self.acados_model.cost_y_expr_e = state_y_e    

    def extend_acados_solver(self):
        """
        Create a new solver with the extended model.
        """
        # Get the same OCP object from NMPC for the OCP cost, constraints and settings
        _ocp = self.nmpc.get_ocp()

        # Overwrite the OCP model with the extended model
        _ocp.model = self.acados_model

        # =====================================================================
        # TODO OVERTHINK PARAMETERS OF OCP!!!!
        # Adjust parameters
        # TODO Check if this is correct
        _ocp.dims.np = self.acados_model.p.size()[0]     # Number of parameters
        _ocp.parameter_values = np.zeros(_ocp.dims.np)   # Initialize parameters with zeros

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

        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        self.acados_init_p = np.zeros(_ocp.dims.np)
        self.acados_init_p[0] = x_ref[6]  # qw
        if len(self.nmpc.phys.physical_param_list) != 24:
            raise ValueError("Physical parameters are not in the correct order. Please check the physical model.")
        self.acados_init_p[4:28] = np.array(self.nmpc.phys.physical_param_list)

        _ocp.constraints.x0 = x_ref
        _ocp.cost.yref = np.concatenate((x_ref, u_ref))
        _ocp.cost.yref_e = x_ref
        _ocp.parameter_values = self.acados_init_p
        # =====================================================================

        # Build acados ocp into current working directory (which was created in super class)
        json_file_path = os.path.join("./" + _ocp.model.name + "_acados_ocp.json")
        self.ocp_solver = AcadosOcpSolver(_ocp, json_file=json_file_path, build=True)
        print("Generated C code for acados solver successfully to " + os.getcwd())

        # TODO Extend constraints ?
        # TODO Extend cost function ?
        # TODO CHECK SOLVER OPTIONS ?
    
    def create_acados_sim_solver(self) -> AcadosSimSolver:
        # Create a simulation solver based on the acados model
        acados_sim = AcadosSim()
        acados_sim.model = self.acados_model

        # =====================================================================
        # TODO OVERTHINK PARAMETERS OF SOLVER!!!!
        # Set the initial parameters for the simulation
        n_param = self.acados_model.p.size()[0]
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        # TODO set this elsewhere since its confusing where this gets modified/accessed
        self.nmpc.acados_init_p = np.zeros(n_param)
        self.nmpc.acados_init_p[0] = 1.0  # qw
        self.nmpc.acados_init_p[4:28] = np.array(self.nmpc.phys.physical_param_list)
        acados_sim.parameter_values = self.nmpc.acados_init_p
        # =====================================================================

        # Set the horizon for the simulation
        acados_sim.solver_options.T = self.T_sim
        acados_sim.solver_options.num_steps = 1     # Default TODO think about increasing this to increase accuracy

        # Generate solver
        json_file_path = os.path.join("./" + self.acados_model.name + "_acados_sim.json")
        self.sim_solver = AcadosSimSolver(acados_sim, json_file=json_file_path, build=True)
        print("Generated C code for sim solver successfully to " + os.getcwd())

    def track(self, xr, ur):
        """
        Tracks a trajectory defined by xr and ur, where xr is the reference state and ur is the reference control input.
        
        :param xr: Reference state trajectory (N+1, state_dim)
        :param ur: Reference control input trajectory (N, control_dim)
        """
        # TODO what is self.nmpc.acados_init_p and why do we need to set it?

        # 0 ~ N-1
        for j in range(self.ocp_solver.N):
            yr = np.concatenate((xr[j, :], ur[j, :]))
            self.ocp_solver.set(j, "yref", yr)
            quaternion_ref = xr[j, 6:10]
            self.nmpc.acados_init_p[0:4] = quaternion_ref
            self.ocp_solver.set(j, "p", self.nmpc.acados_init_p)                # For nonlinear quaternion error

        # N
        yr = xr[self.ocp_solver.N, :]
        self.ocp_solver.set(self.ocp_solver.N, "yref", yr)
        quaternion_ref = xr[self.ocp_solver.N, 6:10]
        self.nmpc.acados_init_p[0:4] = quaternion_ref
        self.ocp_solver.set(self.ocp_solver.N, "p", self.nmpc.acados_init_p)    # For nonlinear quaternion error

    # def set_optimization_parameters(self, initial_state=None, use_model=0, return_x=False):
    #  ========= SET PARAMETERS FOR NOMINAL =========
    # .....
    # .....
    #  ========= SET PARAMETERS FOR NETWORK =========
    #     if self.with_mlp:
    #         if self.x_opt_acados is None:
    #             self.x_opt_acados = np.hstack(self.target)
    #         if self.w_opt_acados is None:
    #              self.w_opt_acados = self.u_target

    #         if not self.mlp_conf['approximated']:
    #             self.acados_ocp_solver[use_model].set(0, 'p', np.hstack([np.array(gp_state + [1])]))
    #             for j in range(1, self.N):
    #                 self.acados_ocp_solver[use_model].set(j, 'p', np.hstack([np.array([0.0] * (len(gp_state) + 1))]))
    #         else:
    #             state = np.vstack([np.array([initial_state]), self.x_opt_acados[1:]])
    #             a_list = []
    #             for i in range(state.shape[0]):
    #                 a_list.append(v_dot_q(np.array(state[i, 7:10]), quaternion_inverse(np.array(state[i, 3:7]))))
    #             a = np.array(a_list)[:self.N]

    #             if self.mlp_conf['torque_output']:
    #                 a = np.concatenate([a, state[:self.N, 10:]], axis=-1)

    #             if self.mlp_conf['u_inp']:
    #                 a = np.concatenate([a, self.w_opt_acados], axis=-1)

    #             if self.mlp_conf['ground_map_input']:
    #                 ground_maps = []
    #                 for i in range(state.shape[0]):
    #                     pos = state[i][:3]
    #                     x_idxs = np.floor((pos[0] - self._org_to_map_org[0]) / self._map_res).astype(int) - 1
    #                     y_idxs = np.floor((pos[1] - self._org_to_map_org[1]) / self._map_res).astype(int) - 1
    #                     ground_patch = self._static_ground_map[x_idxs:x_idxs + 3, y_idxs:y_idxs + 3]

    #                     relative_ground_patch = 4 * (np.clip(pos[2] - ground_patch, 0, 0.5) - 0.25)

    #                     flatten_relative_ground_patch = relative_ground_patch.flatten(order='F')

    #                     ground_effect_in = np.hstack([flatten_relative_ground_patch,
    #                                                   flatten_relative_ground_patch[..., :4] * 0])

    #                     ground_maps.append(ground_effect_in)

    #                 a = np.concatenate([a, np.array(ground_maps)[:self.N]], axis=-1)

    #             mlp_params = self.mlp_regressor.approx_params(a, order=self.mlp_conf['approx_order'], flat=True)
    #             mlp_params = np.vstack([mlp_params, mlp_params[[-1]]])
    #             self.acados_ocp_solver[use_model].set(0, 'p',
    #                                                   np.hstack([np.array(gp_state + [1]), mlp_params[0]]))
    #             for j in range(1, self.N):
    #                 self.acados_ocp_solver[use_model].set(j, 'p', np.hstack([np.array([0.0] * (len(gp_state) + 1)),
    #                                                                          mlp_params[j]]))

    def simulate_trajectory(self):
        """
        Simulate trajectory using optimal control sequence from the OCP solver.
        """
        # Number of steps in the OCP
        N = self.ocp_solver.acados_ocp.dims.N
        
        # Get all control inputs from OCP solution
        u_sequence = np.array([self.ocp_solver.get(i, "u") for i in range(N)])

        # Get initial state
        state_curr = self.sim_solver.get("x")
        
        # Simulate forward
        x_traj = np.zeros((N + 1, state_curr.shape[0]))
        x_traj[0, :] = state_curr
        
        for n in range(N):
            self.sim_solver.set("x", state_curr)
            self.sim_solver.set("u", u_sequence[n])
            
            status = self.sim_solver.solve()
            if status != 0:
                print(f"Round {n}: acados ocp_solver returned status {self.sim_solver.status}. Exiting.")
                return 
                
            state_curr = self.sim_solver.get("x")
            x_traj[n + 1, :] = state_curr
        
        return x_traj

    def load_model(self, model_options, sim_options):
        """
        Load a pre-trained neural network for the NMPC controller.
        """
        # Get options for model loading
        model_params = {"git": model_options.get("git", False),
                        "model_name": model_options.get("name", None),
                        "disturbances": sim_options.get("disturbances", False)}

        # TODO set config elsewhere
        mlp_config = {'approximated': False, 'v_inp': True, 'u_inp': False, 'T_out': False, 'ground_map_input': False,
                      'torque_output': False, 'two_step_rti': False}
        
        # Load trained MLP model
        directory, file_name = get_model_dir_and_file(model_params)
        saved_network = torch.load(os.path.join(directory, f"{file_name}.pt"))
        # saved_network = {"input_size": 12, "hidden_size": 64, "output_size": 12, "hidden_layers": 3, "state_dict": {}}
        base_mlp = mc.nn.MultiLayerPerceptron(saved_network['input_size'], saved_network['hidden_size'],
                                              saved_network['output_size'], saved_network['hidden_layers'],
                                              'Tanh')
        neural_model = NormalizedMLP(
                            base_mlp,
                            torch.tensor(np.zeros((saved_network['input_size'],))).float(),
                            torch.tensor(np.zeros((saved_network['input_size'],))).float(),
                            torch.tensor(np.zeros((saved_network['output_size'],))).float(),
                            torch.tensor(np.zeros((saved_network['output_size'],))).float())
        # Load weights and biases from saved model
        neural_model.load_state_dict(saved_network['state_dict'])
        neural_model.eval()

        # if reg_type.endswith('approx2'):
        #     mlp_config['approximated'] = True
        #     mlp_config['approx_order'] = 2
        # elif reg_type.endswith('approx') or reg_type.endswith('approx_1'):
        #     mlp_config['approximated'] = True
        #     mlp_config['approx_order'] = 1
        # if '_u' in reg_type:
        #     mlp_config['u_inp'] = True
        # if '_T' in reg_type:
        #     mlp_config['T_out'] = True

        return neural_model, mlp_config

    def check_constraints(self, u_cmd):
        """
        Check if the control input command u_cmd respects the constraints of the system.
        :param u_cmd: Control input command.
        """
        if np.any(u_cmd[:4]) < self.nmpc.params["thrust_min"] or np.any(u_cmd[:4]) > self.nmpc.params["thrust_max"]:
            print(f"=== Control thrust input violates constraints: {u_cmd[:4]} ===")
            raise ValueError("Control thrust input violates constraints.")
        if u_cmd.shape[0] > 4:
            if np.any(u_cmd[4:]) < self.nmpc.params["a_min"] or np.any(u_cmd[4:]) > self.nmpc.params["a_max"]:
                print(f"=== Control servo angle input violates constraints: {u_cmd[4:]} ===")
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
