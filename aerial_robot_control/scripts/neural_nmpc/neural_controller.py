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
    

    def setup(self, my_quad, t_horizon=1.0, n_nodes=5, q_cost=None, r_cost=None,
                 optimization_dt=5e-2, simulation_dt=5e-4, pre_trained_models=None, model_name="my_quad", q_mask=None,
                 solver_options=None, rdrv_d_mat=None, model_conf=None):
        """
        :param my_quad: Quadrotor3D simulator object
        :type my_quad: Quadrotor3D
        :param t_horizon: time horizon for optimization loop MPC controller
        :param n_nodes: number of MPC nodes
        :param optimization_dt: time step between two successive optimizations intended to be used.
        :param simulation_dt: discretized time-step for the quadrotor simulation
        :param pre_trained_models: additional pre-trained GP regressors to be combined with nominal model in the MPC
        :param q_cost: diagonal of Q matrix for LQR cost of MPC cost function. Must be a numpy array of length 13.
        :param r_cost: diagonal of R matrix for LQR cost of MPC cost function. Must be a numpy array of length 4.
        :param q_mask: Optional boolean mask that determines which variables from the states compute towards the
        cost function. In case no argument is passed, all variables are weighted.
        :param solver_options: Optional set of extra options dictionary for acados solver.
        :param rdrv_d_mat: 3x3 matrix that corrects the drag with a linear model according to Faessler et al. 2018. None
        if not used
        """

        self.states = np.zeros(ModelConfig.STATE_DIM)

        if rdrv_d_mat is not None:
            # rdrv is currently not compatible with covariance mode or with GP-MPC.
            print("RDRv mode")
            self.rdrv = rdrv_d_mat
            assert pre_trained_models is None
        else:
            self.rdrv = None

        self.quad = my_quad
        self.simulation_dt = simulation_dt
        self.optimization_dt = optimization_dt

        # motor commands from last step
        self.motor_u = np.array([0., 0., 0., 0.])

        self.n_nodes = n_nodes
        self.t_horizon = t_horizon

        self.mlp = None
        self.mlp_approx = None

        # Load augmented dynamics model with GP regressor
        if isinstance(pre_trained_models, torch.nn.Module):
            self.gp_ensemble = None
            self.B_x = {}
            x_dims = len(my_quad.get_state(quaternion=True, stacked=True))
            pred_dims = [7, 8, 9] + ([10, 11, 12] if model_conf['torque_output'] else [])
            for y_dim in pred_dims:
                self.B_x[y_dim] = make_bx_matrix(x_dims, [y_dim])
                self.mlp = pre_trained_models

        elif pre_trained_models is not None:
            self.gp_ensemble = restore_gp_regressors(pre_trained_models)
            x_dims = len(my_quad.get_state(quaternion=True, stacked=True))
            self.B_x = {}
            for y_dim in self.gp_ensemble.gp.keys():
                self.B_x[y_dim] = make_bx_matrix(x_dims, [y_dim])

        else:
            self.gp_ensemble = None
            self.B_x = {}  # Selection matrix of the GP regressor-modified system states

        # For MPC optimization use
        self.quad_opt = Quad3DOptimizer(my_quad, t_horizon=t_horizon, n_nodes=n_nodes,
                                        q_cost=q_cost, r_cost=r_cost,
                                        B_x=self.B_x, gp_regressors=self.gp_ensemble,
                                        mlp_regressor=self.mlp, mlp_conf=model_conf,
                                        model_name=model_name, q_mask=q_mask,
                                        solver_options=solver_options, rdrv_d_mat=rdrv_d_mat)

    def map_properties(self):
        # TODO map all (important) properties of NMPC object
        self.x = self.nmpc.x

    def set_state(self, states):
        # Set all states TODO
        self.states = states
        # If given states is not full states
        if states.length != self.states.length:
            raise ValueError(f"Dimensions don't match! {states.length} != {self.states.length}")

    def get_state(self):
        return self.model
    
    def get_ocp_solver(self):
        return self._ocp_solver
    
    def get_reference_generator(self):
        return self.nmpc.get_reference_generator()

    def extend_acados_model(self):
        # Get the acados model from nominal OCPm
        nominal_model = self.nmpc.get_acados_model()
        
        # Model states
        self.states = nominal_model.x

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
        states = self.gp_x * self.trigger_var + self.x * (1 - self.trigger_var)
        #  Transform velocity to body frame
        v_b = v_dot_q(states[7:10], quaternion_inverse(states[3:7]))
        states = ca.vertcat(states[:7], v_b, states[10:])
        mlp_in = v_b
        # -----------------------------------------------------

        # Adjust input vector
        if self.mlp_conf['torque_output']:
            mlp_in = ca.vertcat(mlp_in, states[10:])

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

            x, y, z = states[0], states[1], states[2]
            orientation = states[3:7]

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
            mlp_out_means = ca.vertcat(v_dot_q(mlp_out_force, states[3:7]), mlp_out_torque)
        else:
            mlp_out_means = v_dot_q(mlp_out, states[3:7])

        # Explicit dynamics
        # Here f is already symbolically evaluated to f(x,u)
        nominal_dynamics = nominal_model.f_expl_expr
        f_eval = nominal_dynamics + ca.mtimes(self.B_x, mlp_out_means)
        
        # Implicit dynamics
        x_dot = ca.MX.sym('x_dot', states.size())
        f_impl = x_dot - f_eval

        # Cost function
        state_y, state_y_e, control_y = self.get_cost_function()
        
        # Assemble acados model
        acados_model = AcadosModel()
        acados_model.name = self.model_name
        acados_model.f_expl_expr = f_eval  # CasADi expression for the explicit dynamics
        acados_model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics
        acados_model.x = states
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

    def track(self, target):
        """
        Set target state as 'yref' in OCP solver in preparation of solving step.
        :param target: np array containing the current target state
        """
        self._ocp_solver.set()



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