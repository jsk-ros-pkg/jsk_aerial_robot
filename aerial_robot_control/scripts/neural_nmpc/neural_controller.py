import os, sys
import numpy as np
import torch
import casadi as ca
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver

from config.configuration_parameters import ModelConfig
from mlp import MLP

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # Add parent directory to path to allow relative imports
from nmpc.tilt_bi.tilt_bi_servo import NMPCTiltBiServo
from nmpc.tilt_tri.tilt_tri_servo import NMPCTiltTriServo
from nmpc.tilt_qd.tilt_qd_servo import NMPCTiltQdServo


class NeuralNMPC():
    def __init__(self, model_options):        
        # TODO: Simple MLP addon
        # TODO: Adjust solver
        # TODO: Introduce approximated MLP
        # TODO: Add GP regressor
        # TODO: Load / save trained model -> overwrite model

        # Setup Neural NMPC
        self.setup(...)

        # Nominal model and solver
        if model_options["arch_type"] == "tilt_bi":
            self.nmpc = NMPCTiltBiServo()
        elif model_options["arch_type"] == "tilt_tri":
            self.nmpc = NMPCTiltTriServo()
        elif model_options["arch_type"] == "tilt_qd":
            self.nmpc = NMPCTiltQdServo()
        else:
            raise ValueError("Invalid architecture type specified.")
        
        # Adopt all important characteristics from NMPC object
        self.map_properties()
        # Add neural network model to nominal model
        self.extend_acados_model()
    

    def setup(self, model_name="aerial_robot", 
              t_horizon=1.0, N=5, optimization_dt=5e-2, simulation_dt=5e-4,
              pre_trained_model=None, solver_options=None,
              rdrv_d_mat=None, model_conf=None):
        """
        :param t_horizon: Time horizon for optimization loop in MPC controller
        :param N: Number of MPC nodes
        :param optimization_dt: Time step between two successive optimizations
        :param simulation_dt: Discretized time-step for the aerial robot simulation
        :param pre_trained_model: Pre-trained MLP to be combined with nominal model in MPC framework
        :param solver_options: Set of additional options dictionary for acados solver
        :param rdrv_d_mat: 3x3 matrix that corrects the drag with a linear model
                           according to Faessler et al. 2018
        """

        self.state = np.zeros(ModelConfig.STATE_DIM)


        self.t_horizon = t_horizon
        self.N = N
        self.simulation_dt = simulation_dt
        self.optimization_dt = optimization_dt

        # Load pre-trained models that can be used for the dynamics model in the controller
        # Note: If no pre-trained model is given, the controller will only use the nominal model
        if isinstance(pre_trained_model, torch.nn.Module):
            self.mlp = pre_trained_model
            self.mlp_approx = None
            self.B_x = {}
            pred_dims = [7, 8, 9] + ([10, 11, 12] if model_conf['torque_output'] else [])
            for y_dim in pred_dims:
                self.B_x[y_dim] = make_bx_matrix(ModelConfig.STATE_DIM, [y_dim])
        else:
            self.mlp = None
            self.mlp_approx = None
            self.B_x = {}  # Selection matrix of the GP regressor-modified system state

        # TODO implement drag correction with RDRv
        # For MPC optimization use
        self.quad_opt = Quad3DOptimizer(mlp_regressor=self.mlp, mlp_conf=model_conf,
                                        solver_options=solver_options, rdrv_d_mat=rdrv_d_mat)

    def map_properties(self):
        # TODO map all (important) properties of NMPC object
        self.state = self.nmpc.state
        self.controls = self.nmpc.controls
        self.parameters = self.nmpc.parameters
        self.model_name = self.nmpc.model_name
        self._ocp_solver = self.nmpc.get_ocp_solver()

    def set_state(self, state):
        # Set all state TODO
        self.state = state
        # If given state is not full state
        if state.length != self.state.length:
            raise ValueError(f"Dimensions don't match! {state.length} != {self.state.length}")
    
    def get_ocp_solver(self):
        return self._ocp_solver
    
    def get_reference_generator(self):
        return self.nmpc.get_reference_generator()

    def extend_acados_model(self):
        # Get the acados model from nominal OCPm
        nominal_model = self.nmpc.get_acados_model()
        
        # Model state
        self.state = nominal_model.x

        # Control inputs
        self.controls = nominal_model.u

        # Model Parameters
        self.parameters = nominal_model.p
        # TODO Important!!! Understand approximation --- Propietary library used here
        if not self.mlp_conf['approximated']:
            params = ca.vertcat(self.gp_x, self.trigger_var)
        else:
            params = ca.vertcat(self.gp_x, self.trigger_var,
                                self.mlp_regressor.sym_approx_params(order=self.mlp_conf['approx_order'],
                                                                        flat=True))

        #### Construct MLP model
        mlp_model = MLP()

        # -----------------------------------------------------
        # TODO Understand and implement correctly
        state = self.gp_x * self.trigger_var + self.x * (1 - self.trigger_var)
        #  Transform velocity to body frame
        v_b = v_dot_q(state[7:10], quaternion_inverse(state[3:7]))
        state = ca.vertcat(state[:7], v_b, state[10:])
        mlp_in = v_b
        # -----------------------------------------------------

        # Adjust input vector
        if self.mlp_conf['torque_output']:
            mlp_in = ca.vertcat(mlp_in, state[10:])

        if self.mlp_conf['u_inp']:
            mlp_in = ca.vertcat(mlp_in, self.controls)

        if self.mlp_conf['ground_map_input']:
            map_conf = GroundEffectMapConfig
            map = GroundMapWithBox(np.array(map_conf.box_min),
                                    np.array(map_conf.box_max),
                                    map_conf.box_height,
                                    horizon=map_conf.horizon,
                                    resolution=map_conf.resolution)

            self._map_res = map_conf.resolution

            self._static_ground_map, self._org_to_map_org = map.at(np.array(map_conf.origin))
            ground_map_dx = ca.MX(self._static_ground_map)

            idx = ca.DM(np.arange(0, 3, 1))

            x, y, z = state[0], state[1], state[2]
            orientation = state[3:7]

            x_idxs = ca.floor((x - self._org_to_map_org[0]) / map_conf.resolution) + idx - 1
            y_idxs = ca.floor((y - self._org_to_map_org[1]) / map_conf.resolution) + idx - 1
            ground_patch = ground_map_dx[x_idxs, y_idxs]

            relative_ground_patch = z - ground_patch
            relative_ground_patch = 4 * (ca.fmax(ca.fmin(relative_ground_patch, 0.5), 0.0) - 0.25)

            ground_effect_in = ca.vertcat(ca.reshape(relative_ground_patch, 9, 1), orientation*0)

            mlp_in = ca.vertcat(mlp_in, ground_effect_in)

        # TODO Important!!! Understand approximation --- Propietary library used here
        if not self.mlp_conf['approximated']:
            mlp_out = self.mlp_regressor(mlp_in)
        else:
            mlp_out = self.mlp_regressor.approx(mlp_in, order=self.mlp_conf['approx_order'], parallel=False)

        # Unpack prediction outputs. Transform back to world reference frame
        if self.mlp_conf['torque_output']:
            # Append torque if needed 
            mlp_out_force = mlp_out[:3]
            mlp_out_torque = mlp_out[3:]
            mlp_out_means = ca.vertcat(v_dot_q(mlp_out_force, state[3:7]), mlp_out_torque)
        else:
            mlp_out_means = v_dot_q(mlp_out, state[3:7])

        # Explicit dynamics
        # Here f is already symbolically evaluated to f(x,u)
        nominal_dynamics = nominal_model.f_expl_expr
        f_eval = nominal_dynamics + ca.mtimes(self.B_x, mlp_out_means)
        
        # Implicit dynamics
        x_dot = ca.MX.sym('x_dot', state.size())
        f_impl = x_dot - f_eval

        # Cost function
        state_y, state_y_e, control_y = self.get_cost_function()
        
        # Assemble acados model
        acados_model = AcadosModel()
        acados_model.name = self.model_name
        acados_model.f_expl_expr = f_eval  # CasADi expression for the explicit dynamics
        acados_model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        acados_model.x = state
        acados_model.xdot = x_dot
        acados_model.u = nominal_model.u
        acados_model.p = self.parameters
        acados_model.cost_y_expr = ca.vertcat(state_y, control_y)  # NONLINEAR_LS
        acados_model.cost_y_expr_e = state_y_e

        return acados_model
    

    def extend_acados_solver(self):
        # Get the acados solver from nominal OCP
        nominal_solver = self.nmpc.get_ocp_solver()
        
        ##########
        # CORRECT?!
        # Create a new solver with the extended model
        extended_solver = AcadosOcpSolver(self.nmpc.get_ocp(), json_file="acados_ocp.json")
        ###########

        # Extend constraints 
        
        # Extend cost function
        

        # CHECK SOLVER OPTIONS
        self.acados_ocp_solver = extended_solver


    def run_optimization(self, initial_state=None, use_model=0, return_x=False, gp_regression_state=None):
        """
        Optimizes a trajectory to reach the pre-set target state, starting from the input initial state, that minimizes
        the quadratic cost function and respects the constraints of the system

        :param initial_state: 13-element list of the initial state. If None, 0 state will be used
        :param use_model: integer, select which model to use from the available options.
        :param return_x: bool, whether to also return the optimized sequence of states alongside with the controls.
        :param gp_regression_state: 13-element list of state for GP prediction. If None, initial_state will be used.
        :return: optimized control input sequence (flattened)
        """


        # =================================================
        # BASICALLY saying to set rest of solver params (which for clarity should
        # be set together with ref in dedicated function (utils?))
        # AND then solving the OCP and getting the optimized control input and next state
        # which is also just put in simulation sequence by Jinjie
        # =================================================

        if initial_state is None:
            initial_state = [0, 0, 0] + [1, 0, 0, 0] + [0, 0, 0] + [0, 0, 0]

        # Set initial state. Add gp state if needed
        x_init = initial_state
        x_init = np.stack(x_init)

        # Set initial condition, equality constraint
        self.acados_ocp_solver[use_model].set(0, 'lbx', x_init)
        self.acados_ocp_solver[use_model].set(0, 'ubx', x_init)

        # Set parameters
        if self.with_gp:
            gp_state = gp_regression_state if gp_regression_state is not None else initial_state
            self.acados_ocp_solver[use_model].set(0, 'p', np.array(gp_state + [1]))
            for j in range(1, self.N):
                self.acados_ocp_solver[use_model].set(j, 'p', np.array([0.0] * (len(gp_state) + 1)))

        if self.with_mlp:
            if self.x_opt_acados is None:
                if isinstance(self.target[0], list):
                    self.x_opt_acados = np.expand_dims(
                        np.concatenate([self.target[i] for i in range(len(self.target))]), 0)
                    self.x_opt_acados = self.x_opt_acados.repeat(self.N, 0)
                else:
                    self.x_opt_acados = np.hstack(self.target)
            if self.w_opt_acados is None:
                if len(self.u_target.shape) == 1:
                    self.w_opt_acados = self.u_target[np.newaxis]
                    self.w_opt_acados = self.w_opt_acados.repeat(self.N, 0)
                else:
                    self.w_opt_acados = np.hstack(self.u_target)

            gp_state = gp_regression_state if gp_regression_state is not None else initial_state
            if not self.mlp_conf['approximated']:
                self.acados_ocp_solver[use_model].set(0, 'p', np.hstack([np.array(gp_state + [1])]))
                for j in range(1, self.N):
                    self.acados_ocp_solver[use_model].set(j, 'p', np.hstack([np.array([0.0] * (len(gp_state) + 1))]))
            else:
                state = np.vstack([np.array([initial_state]), self.x_opt_acados[1:]])
                a_list = []
                for i in range(state.shape[0]):
                    a_list.append(v_dot_q(np.array(state[i, 7:10]), quaternion_inverse(np.array(state[i, 3:7]))))
                a = np.array(a_list)[:self.N]

                if self.mlp_conf['torque_output']:
                    a = np.concatenate([a, state[:self.N, 10:]], axis=-1)

                if self.mlp_conf['u_inp']:
                    a = np.concatenate([a, self.w_opt_acados], axis=-1)

                if self.mlp_conf['ground_map_input']:
                    ground_maps = []
                    for i in range(state.shape[0]):
                        pos = state[i][:3]
                        x_idxs = np.floor((pos[0] - self._org_to_map_org[0]) / self._map_res).astype(int) - 1
                        y_idxs = np.floor((pos[1] - self._org_to_map_org[1]) / self._map_res).astype(int) - 1
                        ground_patch = self._static_ground_map[x_idxs:x_idxs + 3, y_idxs:y_idxs + 3]

                        relative_ground_patch = 4 * (np.clip(pos[2] - ground_patch, 0, 0.5) - 0.25)

                        flatten_relative_ground_patch = relative_ground_patch.flatten(order='F')

                        ground_effect_in = np.hstack([flatten_relative_ground_patch,
                                                      flatten_relative_ground_patch[..., :4] * 0])

                        ground_maps.append(ground_effect_in)

                    a = np.concatenate([a, np.array(ground_maps)[:self.N]], axis=-1)

                mlp_params = self.mlp_regressor.approx_params(a, order=self.mlp_conf['approx_order'], flat=True)
                mlp_params = np.vstack([mlp_params, mlp_params[[-1]]])
                self.acados_ocp_solver[use_model].set(0, 'p',
                                                      np.hstack([np.array(gp_state + [1]), mlp_params[0]]))
                for j in range(1, self.N):
                    self.acados_ocp_solver[use_model].set(j, 'p', np.hstack([np.array([0.0] * (len(gp_state) + 1)),
                                                                             mlp_params[j]]))

        # Solve OCP
        self.acados_ocp_solver[use_model].solve()

        # Get u
        w_opt_acados = np.ndarray((self.N, 4))
        x_opt_acados = np.ndarray((self.N + 1, len(x_init)))
        x_opt_acados[0, :] = self.acados_ocp_solver[use_model].get(0, "x")
        for i in range(self.N):
            w_opt_acados[i, :] = self.acados_ocp_solver[use_model].get(i, "u")
            x_opt_acados[i + 1, :] = self.acados_ocp_solver[use_model].get(i + 1, "x")

        self.x_opt_acados = x_opt_acados.copy()
        self.w_opt_acados = w_opt_acados.copy()

        w_opt_acados = np.reshape(w_opt_acados, (-1))
        return w_opt_acados if not return_x else (w_opt_acados, x_opt_acados)

    # =========================================================
    def simulate(self, u_cmd):
        """
        Simulate the plant with the optimized control input sequence.
        :param u_cmd: Control input command sequence to be applied to the plant.
        """
        self.sim_solver.set("x", x_now_sim)
        self.sim_solver.set("u", u_cmd)

        status = self.sim_solver.solve()
        if status != 0:
            raise Exception(f"acados integrator returned status {status} in closed loop instance {i}")

        x_now_sim = self.sim_solver.get("x")

    def simulate_plant(self, w_opt, t_horizon=None, dt_vec=None, progress_bar=False):
        # TODO why also plant??
    
    def forward_prop(self):
        # TODO what here???
    # =========================================================

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
    
    def create_acados_sim_solver(self, ts_sim: float, is_build: bool = True) -> AcadosSimSolver:
        acados_sim = AcadosSim()
        acados_sim.model = self.ocp_model

        n_param = self.ocp_model.p.size()[0]
        # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
        self.acados_init_p = np.zeros(n_param)
        self.acados_init_p[0] = 1.0  # qw
        self.acados_init_p[4:28] = np.array(self.phys.physical_param_list)
        acados_sim.parameter_values = self.acados_init_p

        acados_sim.solver_options.T = ts_sim
        return AcadosSimSolver(acados_sim, json_file=self.ocp_model.name + "_acados_sim.json", build=is_build)