import os
import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver, AcadosSim, AcadosSimSolver
from neural_controller_standalone import NeuralNMPC


def create_acados_sim_solver(mpc: NeuralNMPC, acados_model: AcadosModel, T_sim) -> AcadosSimSolver:
    """
    Create a simulation solver based on the acados model.
    :param T_sim: Discretized time-step for the aerial robot simulation
    """
    # Create a simulation solver based on the acados model
    acados_sim = AcadosSim()
    acados_sim.model = acados_model

    # Set the initial parameters for the simulation
    n_param = acados_model.p.size()[0]
    sim_acados_parameters = np.zeros(n_param)
    sim_acados_parameters[0] = 1.0  # qw
    # same order: phy_params = ca.vertcat(mass, gravity, inertia, kq_d_kt, dr, p1_b, p2_b, p3_b, p4_b, t_rotor, t_servo)
    sim_acados_parameters[4 : 4 + len(mpc.phys.physical_param_list)] = np.array(mpc.phys.physical_param_list)
    acados_sim.parameter_values = sim_acados_parameters

    # Set the horizon for the simulation
    acados_sim.solver_options.T = T_sim
    acados_sim.solver_options.num_steps = 1  # Default TODO think about increasing this to increase accuracy
    acados_sim.solver_options.num_stages = 4  # Default TODO think about increasing this to increase accuracy

    # Generate solver
    json_file_path = os.path.join("./" + acados_model.name + "_acados_sim.json")
    sim_solver = AcadosSimSolver(acados_sim, json_file=json_file_path, build=True)
    print("Generated C code for sim solver successfully to " + os.getcwd())
    return sim_solver


def simulate_trajectory(ocp_solver: AcadosOcpSolver, sim_solver: AcadosSimSolver, state_curr) -> np.ndarray:
    """
    Simulate trajectory using optimal control sequence from the OCP solver.
    """
    # Number of steps in the OCP
    N = ocp_solver.acados_ocp.dims.N

    # Get all control inputs from OCP solution
    u_sequence = np.array([ocp_solver.get(i, "u") for i in range(N)])

    # Simulate forward
    state_traj = np.zeros((N + 1, state_curr.shape[0]))
    state_traj[0, :] = state_curr

    for n in range(N):
        # TODO Parameters should not be constant -> update!
        state_curr = sim_solver.simulate(x=state_curr, u=u_sequence[n], p=sim_solver.acados_sim.parameter_values)
        state_traj[n + 1, :] = state_curr

    return state_traj
