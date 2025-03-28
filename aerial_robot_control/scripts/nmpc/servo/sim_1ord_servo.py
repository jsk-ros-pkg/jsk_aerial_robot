from acados_template import AcadosSim, AcadosSimSolver, AcadosModel, latexify_plot
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt


# Created from the acados python example "pendulum_model".

def create_acados_model() -> AcadosModel:
    """
    Create acados model object.
    """
    # Model name
    model_name = 'servo_ode'

    # Servo time constant
    t_servo = 0.08

    # State
    a_s = ca.SX.sym('a_s')      # Servo angle alpha between Body frame and Rotor frame as state
    states = ca.vertcat(a_s)

    # Input
    a_c = ca.SX.sym('a_c')      # Servo angle alpha between Body frame and Rotor frame as control input
    u = ca.vertcat(a_c)

    # Explicit dynamics
    f = ca.vertcat(
        (a_c - a_s) / t_servo
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

    plt.subplot(nx + 1, 1, 1)
    line, = plt.step(t, np.append([U[0]], U))
    if X_true_label is not None:
        line.set_label(X_true_label)
    else:
        line.set_color('r')

    plt.ylabel('$u$')
    plt.xlabel('$t$')
    plt.hlines(u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
    plt.hlines(-u_max, t[0], t[-1], linestyles='dashed', alpha=0.7)
    plt.ylim([-1.2 * u_max, 1.2 * u_max])
    plt.xlim(t[0], t[-1])
    plt.grid()

    states_lables = ['$x$', r'$\theta$', '$v$', r'$\dot{\theta}$']

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

    # Create acados simulation solver
    sim_solver = AcadosSimSolver(acados_sim)

    # Initialization
    x_now = np.zeros((nn + 1, nx))

    x0 = np.array([0.0])
    u0 = np.array([1.0])

    sim_solver.set("u", u0)
    x_now[0, :] = x0

    for i in range(nn):
        # Set initial solver state
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
    print("S_forw, sensitivities of simulation result w.r.t. x, u:\n", S_forw)

    # Plot simulation
    plot_servo(np.linspace(0, nn * Tf, nn + 1), 10, np.repeat(u0, nn), x_now, latexify=False)
