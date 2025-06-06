import os, sys
import numpy as np
import torch
import casadi as ca
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
from neural_nmpc.mlp import MLP
from neural_nmpc.config.configuration_parameters import ModelConfig
from nmpc.tilt_bi.tilt_bi_servo import NMPCTiltBiServo
from nmpc.tilt_tri.tilt_tri_servo import NMPCTiltTriServo
from nmpc.tilt_qd.tilt_qd_servo_thrust import NMPCTiltQdServoThrust


class NeuralNMPC():
    def __init__(self, model_options, T_sim=0.005,
                 pre_trained_model=None, 
                 solver_options=None, rdrv_d_mat=None):
        # TODO: Introduce approximated MLP
        # TODO: Add GP regressor
        # TODO: Load / save trained model -> overwrite model
        """
        :param T_sim: Discretized time-step for the aerial robot simulation
        :param model_options: Dictionary containing model options, such as architecture type
        :param pre_trained_model: Pre-trained MLP to be combined with nominal model in MPC framework
        :param solver_options: Set of additional options dictionary for acados solver
        :param rdrv_d_mat: 3x3 matrix that corrects the drag with a linear model
                           according to Faessler et al. 2018
        """
        # Nominal model and solver
        # TODO avoid building solver twice with flag
        self.arch_type = model_options["arch_type"]
        if self.arch_type == "tilt_bi":
            self.nmpc = NMPCTiltBiServo()
        elif self.arch_type == "tilt_tri":
            self.nmpc = NMPCTiltTriServo()
        elif self.arch_type == "tilt_qd":
            self.nmpc = NMPCTiltQdServoThrust()
        else:
            raise ValueError("Invalid architecture type specified.")

        # Get OCP object from NMPC
        self.ocp = self.nmpc.get_ocp()
        self.T_samp = self.nmpc.params["T_samp"]            # Sampling time for the NMPC controller, i.e., time step between two successive optimizations
        self.T_horizon = self.nmpc.params["T_horizon"]      # Time horizon for optimization loop in MPC controller
        self.N = self.nmpc.params["N_steps"]                # Number of MPC nodes
        self.T_step = self.nmpc.params["T_step"]   # Step size used in optimization loop in MPC controller
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
        if isinstance(pre_trained_model, torch.nn.Module):
            self.mlp = pre_trained_model
            self.mlp_approx = None
        else:
            self.mlp = None
            self.mlp_approx = None

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
        self.acados_model.cost_y_expr_0 = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        self.acados_model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        self.acados_model.cost_y_expr_e = state_y_e    

    def extend_acados_solver(self):
        ##########
        # CORRECT?!
        # Create a new solver with the extended model
        _ocp = self.nmpc.get_ocp()

        # Extend the OCP with the new model
        _ocp.model = self.acados_model

        # Build acados ocp into current working directory (which was created in super class)
        json_file_path = os.path.join("./" + _ocp.model.name + "_acados_ocp.json")
        self.ocp_solver = AcadosOcpSolver(_ocp, json_file=json_file_path, build=True)
        print("Generated C code for acados solver successfully to " + os.getcwd())

        # TODO Extend constraints ?
        # TODO Extend cost function ?
        # TODO CHECK SOLVER OPTIONS ?

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

    # def run_optimization(self):
    #     """
    #     Optimizes a trajectory to reach the pre-set target state, starting from the input initial state, that minimizes
    #     the quadratic cost function and respects the constraints of the system

    #     # :param initial_state: 13-element list of the initial state. If None, 0 state will be used
    #     # :param use_model: integer, select which model to use from the available options.
    #     # :param return_x: bool, whether to also return the optimized sequence of states alongside with the controls.
    #     # :param gp_regression_state: 13-element list of state for GP prediction. If None, initial_state will be used.
    #     :return: optimized control input sequence (flattened)
    #     """
        

    #     # =================================================
    #     # BASICALLY saying to set rest of solver params (which for clarity should
    #     # be set together with ref in dedicated function (utils?))
    #     # AND then solving the OCP and getting the optimized control input and next state
    #     # which is also just put in simulation sequence by Jinjie
    #     # =================================================


    #     if self.with_mlp:
    #         if self.x_opt_acados is None:
    #             if isinstance(self.target[0], list):
    #                 self.x_opt_acados = np.expand_dims(
    #                     np.concatenate([self.target[i] for i in range(len(self.target))]), 0)
    #                 self.x_opt_acados = self.x_opt_acados.repeat(self.N, 0)
    #             else:
    #                 self.x_opt_acados = np.hstack(self.target)
    #         if self.w_opt_acados is None:
    #             if len(self.u_target.shape) == 1:
    #                 self.w_opt_acados = self.u_target[np.newaxis]
    #                 self.w_opt_acados = self.w_opt_acados.repeat(self.N, 0)
    #             else:
    #                 self.w_opt_acados = np.hstack(self.u_target)

    #         gp_state = gp_regression_state if gp_regression_state is not None else initial_state
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

    #     # Solve OCP
    #     self.acados_ocp_solver[use_model].solve()

    #     # Get u
    #     w_opt_acados = np.ndarray((self.N, 4))
    #     x_opt_acados = np.ndarray((self.N + 1, len(x_init)))
    #     x_opt_acados[0, :] = self.acados_ocp_solver[use_model].get(0, "x")
    #     for i in range(self.N):
    #         w_opt_acados[i, :] = self.acados_ocp_solver[use_model].get(i, "u")
    #         x_opt_acados[i + 1, :] = self.acados_ocp_solver[use_model].get(i + 1, "x")

    #     self.x_opt_acados = x_opt_acados.copy()
    #     self.w_opt_acados = w_opt_acados.copy()

    #     w_opt_acados = np.reshape(w_opt_acados, (-1))
    #     return w_opt_acados if not return_x else (w_opt_acados, x_opt_acados)

    # =========================================================
    
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

    def load_controller(self):
        """
        Load a pretrained model into the NMPC controller.
        """

        load_ops = {"params": simulation_options}
        load_ops.update({"git": version, "model_name": name})

        # Configuration for MLP Model
        mlp_config = None
        
        # Load trained GP model
        if reg_type == "gp":
            pre_trained_models = load_pickled_models(model_options=load_ops)
            rdrv_d = None

        # Load trained MLP model
        elif 'mlp' in reg_type:
            mlp_config = {'approximated': False, 'v_inp': True, 'u_inp': False, 'T_out': False, 'ground_map_input': False,
                        'torque_output': False, 'two_step_rti': False}
            directory, file_name = get_model_dir_and_file(load_ops)
            saved_dict = torch.load(os.path.join(directory, f"{file_name}.pt"))
            mlp_model = mc.nn.MultiLayerPerceptron(saved_dict['input_size'], saved_dict['hidden_size'],
                                                saved_dict['output_size'], saved_dict['hidden_layers'], 'Tanh')
            neural_model = NormalizedMLP(mlp_model, torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
                                torch.tensor(np.zeros((saved_dict['input_size'],))).float(),
                                torch.tensor(np.zeros((saved_dict['output_size'],))).float(),
                                torch.tensor(np.zeros((saved_dict['output_size'],))).float())
            neural_model.load_state_dict(saved_dict['state_dict'])
            neural_model.eval()
            pre_trained_models = neural_model
            rdrv_d = None

            if reg_type.endswith('approx2'):
                mlp_config['approximated'] = True
                mlp_config['approx_order'] = 2
            elif reg_type.endswith('approx') or reg_type.endswith('approx_1'):
                mlp_config['approximated'] = True
                mlp_config['approx_order'] = 1
            if '_u' in reg_type:
                mlp_config['u_inp'] = True
            if '_T' in reg_type:
                mlp_config['T_out'] = True

        # Load pre-trained RDRv model
        else:
            rdrv_d = load_rdrv(model_options=load_ops)
            pre_trained_models = None

        nmpc = RTNMPC(
            model_name=model_name,
            reg_type=reg_type,
            neural_model=neural_model,
            rdrv_d=rdrv_d,
            acados_options=acados_config
        )

        return nmpc
    
    def create_acados_sim_solver(self) -> AcadosSimSolver:
        # Create a simulation solver based on the acados model
        acados_sim = AcadosSim()
        acados_sim.model = self.acados_model

        # Set the initial parameters for the simulation
        n_param = self.acados_model.p.size()[0]
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        # TODO set this elsewhere since its confusing where this gets modified/accessed
        self.nmpc.acados_init_p = np.zeros(n_param)
        self.nmpc.acados_init_p[0] = 1.0  # qw
        self.nmpc.acados_init_p[4:28] = np.array(self.nmpc.phys.physical_param_list)
        acados_sim.parameter_values = self.nmpc.acados_init_p

        # Set the horizon for the simulation
        acados_sim.solver_options.T = self.T_sim
        acados_sim.solver_options.num_steps = 1     # Default TODO think about increasing this to increase accuracy

        # Generate solver
        json_file_path = os.path.join("./" + self.acados_model.name + "_acados_sim.json")
        self.sim_solver = AcadosSimSolver(acados_sim, json_file=json_file_path, build=True)
        print("Generated C code for sim solver successfully to " + os.getcwd())

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
        if self.arch_type == "tilt_bi":
            return np.array([self.nmpc.phys.p1_b, self.nmpc.phys.p2_b])
        elif self.arch_type == "tilt_tri":
            return np.array([self.nmpc.phys.p1_b, self.nmpc.phys.p2_b, self.nmpc.phys.p3_b])
        elif self.arch_type == "tilt_qd":
            return np.array([self.nmpc.phys.p1_b, self.nmpc.phys.p2_b, self.nmpc.phys.p3_b, self.nmpc.phys.p4_b])
