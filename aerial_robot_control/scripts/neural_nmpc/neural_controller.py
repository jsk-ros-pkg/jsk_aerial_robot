import os, sys
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver

from utils.geometry_utils import quaternion_inverse, v_dot_q
from utils.model_utils import load_model, get_output_mapping, get_inverse_output_mapping
from utils.model_utils import cross_check_params

# Quadrotor
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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
                 run_options, rdrv_d_mat=None):
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
        
        # Get OCP model from NMPC TODO implement drag correction with RDRv
        self.nominal_model = self.nmpc.get_acados_model()
        self.state = self.nominal_model.x
        self.controls = self.nominal_model.u
        self.parameters = self.nominal_model.p

        # Get OCP solver from NMPC
        self.nominal_solver = self.nmpc.get_ocp_solver()

        # Set name of the controller
        self.model_name = "Neural_" + self.nominal_model.name

        # Load pre-trained MLP
        self.neural_model, self.mlp_metadata = load_model(model_options, sim_options, run_options)
        # Cross-check weights and meta parameters used for MPC to train MLP
        if not model_options["only_use_nominal"]:
            cross_check_params(self.nmpc.params, self.mlp_metadata)

        # Add neural network model to nominal model 
        self.extend_acados_model()

        # Extend the acados solver with the extended model
        self.extend_acados_solver()
    
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

        # TODO Important!!! Understand approximation --- Propietary library used here
        # if not self.mlp_conf['approximated']:
        #     mlp_out = self.mlp_regressor(mlp_in)
        # else:
        #     mlp_out = self.mlp_regressor.approx(mlp_in, order=self.mlp_conf['approx_order'], parallel=False)

        # -----------------------------------------------------
        # TODO Understand and implement correctly
        # state = self.gp_x * self.trigger_var + self.x * (1 - self.trigger_var)
        # -----------------------------------------------------

        # === MLP input ===
        # MLP is trained to receive and predict the velocity in the body frame
        # Transform input velocity to body frame
        v_b = v_dot_q(self.state[3:6], quaternion_inverse(self.state[6:10]))
        state_b = ca.vertcat(self.state[:3], v_b, self.state[6:])

        self.state_feats = eval(self.mlp_metadata['state_feats'])
        self.u_feats = eval(self.mlp_metadata['u_feats'])
        mlp_in = ca.vertcat(state_b[self.state_feats], self.controls[self.u_feats])

        # === MLP forward pass ===
        mlp_out = self.neural_model(mlp_in)

        # Transform velocity back to world frame
        self.y_reg_dims = np.array(eval(self.mlp_metadata["y_reg_dims"]))
        if set([3, 4, 5]).issubset(set(self.y_reg_dims)):
            v_idx = np.where(self.y_reg_dims == 3)[0][0]  # Assumed that v_x, v_y, v_z are consecutively in output
            v_w = v_dot_q(mlp_out[v_idx:v_idx+3], self.state[6:10])
            mlp_out = ca.vertcat(mlp_out[:v_idx], v_w, mlp_out[v_idx+3:])

        # Explicit dynamics
        if self.model_options["end_to_end_mlp"]:
            # Map output of MLP to the state space
            M = get_output_mapping(self.state.shape[0],
                                   eval(self.mlp_metadata["y_reg_dims"]))
            O = get_inverse_output_mapping(self.state.shape[0],
                                   eval(self.mlp_metadata["y_reg_dims"]))

            # Combine nominal dynamics with neural dynamics
            f_total = O @ self.nominal_model.f_expl_expr + M @ mlp_out
        elif self.model_options["only_use_nominal"]:
            f_total = self.nominal_model.f_expl_expr
        else:
            # Here f is already symbolically evaluated to f(x,u)
            nominal_dynamics = self.nominal_model.f_expl_expr

            # Map output of MLP to the state space
            M = get_output_mapping(self.state.shape[0],
                                   eval(self.mlp_metadata["y_reg_dims"]))

            # Combine nominal dynamics with neural dynamics
            f_total = nominal_dynamics + M @ mlp_out

        # Implicit dynamics
        x_dot = ca.MX.sym('x_dot', self.state.size())
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
        self.acados_model.cost_y_expr_0 = ca.vertcat(state_y, control_y)    # NONLINEAR_LS
        self.acados_model.cost_y_expr = ca.vertcat(state_y, control_y)      # NONLINEAR_LS
        self.acados_model.cost_y_expr_e = state_y_e

        if self.nmpc.include_quaternion_constraint:
            # Note: This is necessary to ensure that the quaternion stays on the unit sphere
            # and represents a valid rotation.
            self.acados_model.con_h_expr = self.state[7] ** 2 + self.state[8] ** 2 + self.state[9] ** 2 + self.state[10] ** 2 - 1.0   # ||q||^2 - 1 = 0
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
        self.nmpc.acados_init_p = np.zeros(n_param)
        self.nmpc.acados_init_p[0] = 1.0  # qw
        self.nmpc.acados_init_p[4:28] = np.array(self.nmpc.phys.physical_param_list)
        acados_sim.parameter_values = self.nmpc.acados_init_p
        # =====================================================================

        # Set the horizon for the simulation
        acados_sim.solver_options.T = T_sim
        acados_sim.solver_options.num_steps = 1     # Default TODO think about increasing this to increase accuracy

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

    def simulate_trajectory(self, sim_solver):
        """
        Simulate trajectory using optimal control sequence from the OCP solver.
        """
        # Number of steps in the OCP
        N = self.ocp_solver.acados_ocp.dims.N
        
        # Get all control inputs from OCP solution
        u_sequence = np.array([self.ocp_solver.get(i, "u") for i in range(N)])

        # Get initial state
        state_curr = sim_solver.get("x")
        
        # Simulate forward
        x_traj = np.zeros((N + 1, state_curr.shape[0]))
        x_traj[0, :] = state_curr
        
        for n in range(N):
            sim_solver.set("x", state_curr)
            sim_solver.set("u", u_sequence[n])

            status = sim_solver.solve()
            if status != 0:
                print(f"Round {n}: acados ocp_solver returned status {sim_solver.status}. Exiting.")
                return

            state_curr = sim_solver.get("x")
            x_traj[n + 1, :] = state_curr
        
        return x_traj
    
    def check_state_constraints(self, state_curr, i):
        # Boundary constraints
        for idx in self.ocp_solver.acados_ocp.constraints.idxbx:
            lbxi = np.where(self.ocp_solver.acados_ocp.constraints.idxbx==idx)[0][0]
            if state_curr[idx] < self.ocp_solver.acados_ocp.constraints.lbx[lbxi]*1.01 or \
                    state_curr[idx] > self.ocp_solver.acados_ocp.constraints.ubx[lbxi]*1.01:
                print(f"Warning: Constraint violation at index {idx} in simulation step {i}. "
                      f"Value: {state_curr[idx]:.14f}, "
                      f"Lower bound: {self.ocp_solver.acados_ocp.constraints.lbx[lbxi]}, "
                      f"Upper bound: {self.ocp_solver.acados_ocp.constraints.ubx[lbxi]}")
        # Nonlinear unit quaternion constraint
        quat_norm = np.linalg.norm(state_curr[6:10])
        if quat_norm < 0.999 or quat_norm > 1.001:
            print(f"Warning: Constraint violation for unit_q in simulation step {i}. "
                  f"Value: {quat_norm:.14f} != 1.0, "
                  f"Quaternion: {state_curr[6:10]}")

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
