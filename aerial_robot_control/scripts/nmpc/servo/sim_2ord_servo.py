import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
import rospkg
import casadi as ca
from acados_template import AcadosSim, AcadosSimSolver, AcadosModel, latexify_plot


# Read parameters from configuration file in the robot's package
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("gimbalrotor"), "config", "PhysParamBirotor.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)

physical_params = param_dict["physical"]

# Servo parameters
kps = physical_params["kps"]
kds = physical_params["kds"]
i_sxx = physical_params["servo_inertia_x"]


def create_acados_model() -> AcadosModel:
    """
    Create acados model object.
    """
    # Model name
    model_name = 'servo_ode_2ord'

    # Servo time constant
    t_servo = 0.08

    # States
    a_s = ca.SX.sym('a', 2)   # Servo angle alpha between Body frame and Rotor frame as state
    b_s = ca.SX.sym('b', 2)   # Servo angular velocity beta (continuous time-derivative of alpha)
    a_s = ca.SX.sym('a_s')
    states = ca.vertcat(a_s, b_s)

    # Input
    a_c = ca.SX.sym('a_c', 2)
    u = ca.vertcat(a_c)

    # Explicit dynamics
    f = ca.vertcat(
        b_s,
        (kps * (a_c - a_s) + kds * (0 - b_s)) / i_sxx
    )

    # Implicit dynamics
    x_dot = ca.SX.sym('x_dot', states.size())
    f_impl = x_dot - f

    # Assemble acados model
    model = AcadosModel()
    model.name = model_name
    model.f_expl_expr = f
    model.f_impl_expr = f_impl
    model.x = states
    model.xdot = x_dot
    model.u = u

    return model

def plot_servo(shooting_nodes, u_max, U, X_true,
               X_est=None, Y_measured=None, latexify=False,
               plt_show=True, X_true_label=None):
    """
    Plot the simulation of the servo controller.

    :param shooting_nodes: Time values of the discretization
    :param u_max: Maximum absolute value of input u
    :param U: Arrray with shape (N_sim-1, nu) or (N_sim, nu)
    :param X_true: Arrray with shape (N_sim, nx)
    :param X_est: Arrray with shape (N_sim-N_mhe, nx)
    :param Y_measured: Array with shape (N_sim, ny)
    :param latexify: Latex style plots
    """

    if latexify:
        latexify_plot()

    WITH_ESTIMATION = X_est is not None and Y_measured is not None

    N_sim = X_true.shape[0]
    nx = X_true.shape[1]

    Tf = shooting_nodes[N_sim - 1]
    t = shooting_nodes

    Ts = t[1] - t[0]
    if WITH_ESTIMATION:
        N_mhe = N_sim - X_est.shape[0]
        t_mhe = np.linspace(N_mhe * Ts, Tf, N_sim - N_mhe)

    states_lables = ['$a1$', '$a2$', '$b1$', '$b2$']

    for i in range(nx):
        plt.subplot(nx + 1, 1, i + 2)
        line, = plt.plot(t, X_true[:, i], label='true')
        if X_true_label is not None:
            line.set_label(X_true_label)

        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, i], '--', label='estimated')
            plt.plot(t, Y_measured[:, i], 'x', label='measured')

        plt.ylabel(states_lables[i])
        plt.xlabel('$t$')
        plt.grid()
        plt.legend(loc=1)
        plt.xlim(t[0], t[-1])

    plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)

    if plt_show:
        plt.show()


if __name__ == "__main__":
    # Create acados model
    model = create_acados_model()

    # Create acados simulation environment
    acados_sim = AcadosSim()
    acados_sim.model = model

    Tf = 0.001
    nx = model.x.size()[0]; nu = model.u.size()[0]; nn = 1000

    # Set simulation solver options
    acados_sim.solver_options.T = Tf           # Simulation time
    acados_sim.solver_options.integrator_type = 'IRK'
    acados_sim.solver_options.num_stages = 3
    acados_sim.solver_options.num_steps = 3
    acados_sim.solver_options.newton_iter = 3  # For implicit integrator
    acados_sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"

    # # The following setting leads to unconvergence
    # sim.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    # sim.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
    # # # 0: no warm start; 1: warm start; 2: hot start. Default: 0   Seems only works for FULL_CONDENSING_QPOASES
    # # ocp.solver_options.qp_solver_warm_start = 1
    # sim.solver_options.hessian_approx = "GAUSS_NEWTON"
    # sim.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
    # sim.solver_options.print_level = 0
    # sim.solver_options.nlp_solver_type = "SQP_RTI"
    # sim.solver_options.qp_solver_cond_N = nmpc_params["N_steps"]
    # sim.solver_options.tf = nmpc_params["T_horizon"]

    # Create acados simulation solver
    sim_solver = AcadosSimSolver(acados_sim)

    # Initialization
    x_now = np.zeros((nn + 1, nx))

    x0 = np.zeros((nx,))
    u0 = np.array([1.0, -1.0])

    sim_solver.set("u", u0)
    x_now[0, :] = x0

    for i in range(nn):
        # Set initial state
        sim_solver.set("x", x_now[i, :])

        # Initialize IRK
        if acados_sim.solver_options.integrator_type == 'IRK':
            sim_solver.set("xdot", np.zeros((nx,)))

        # Solve optimization problem
        status = sim_solver.solve()
        x_now[i + 1, :] = sim_solver.get("x")

        if status != 0:
            raise Exception(f'acados returned status {status} in closed loop instance {i}.')

    # Retrieve sensitivities
    S_forw = sim_solver.get("S_forw")
    print("S_forw, sensitivities of simulation result wrt x,u:\n", S_forw)

    # plot simulation
    plot_servo(np.linspace(0, nn * Tf, nn + 1), 10, np.repeat(u0, nn), x_now, latexify=False)
