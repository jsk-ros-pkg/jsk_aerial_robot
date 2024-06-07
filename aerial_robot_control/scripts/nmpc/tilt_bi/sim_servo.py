'''
 Created by jinjie on 24/06/07.
 From acados python example -> pendulum_model
'''

from acados_template import AcadosSim, AcadosSimSolver, AcadosModel, latexify_plot
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt


def export_pendulum_ode_model() -> AcadosModel:
    model_name = 'pendulum_ode'

    # constants
    M = 1.  # mass of the cart [kg] -> now estimated
    m = 0.1  # mass of the ball [kg]
    g = 9.81  # gravity constant [m/s^2]
    l = 0.8  # length of the rod [m]

    # set up states & controls
    x1 = ca.SX.sym('x1')
    theta = ca.SX.sym('theta')
    v1 = ca.SX.sym('v1')
    dtheta = ca.SX.sym('dtheta')

    x = ca.vertcat(x1, theta, v1, dtheta)

    F = ca.SX.sym('F')
    u = ca.vertcat(F)

    # xdot
    x1_dot = ca.SX.sym('x1_dot')
    theta_dot = ca.SX.sym('theta_dot')
    v1_dot = ca.SX.sym('v1_dot')
    dtheta_dot = ca.SX.sym('dtheta_dot')

    xdot = ca.vertcat(x1_dot, theta_dot, v1_dot, dtheta_dot)

    # dynamics
    cos_theta = ca.cos(theta)
    sin_theta = ca.sin(theta)
    denominator = M + m - m * cos_theta * cos_theta
    f_expl = ca.vertcat(v1,
                        dtheta,
                        (-m * l * sin_theta * dtheta * dtheta + m * g * cos_theta * sin_theta + F) / denominator,
                        (-m * l * cos_theta * sin_theta * dtheta * dtheta + F * cos_theta + (M + m) * g * sin_theta) / (
                                    l * denominator)
                        )

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model


def plot_pendulum(shooting_nodes, u_max, U, X_true, X_est=None, Y_measured=None, latexify=False, plt_show=True,
                  X_true_label=None):
    """
    Params:
        shooting_nodes: time values of the discretization
        u_max: maximum absolute value of u
        U: arrray with shape (N_sim-1, nu) or (N_sim, nu)
        X_true: arrray with shape (N_sim, nx)
        X_est: arrray with shape (N_sim-N_mhe, nx)
        Y_measured: array with shape (N_sim, ny)
        latexify: latex style plots
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


def main():
    sim = AcadosSim()

    # export model
    model = export_pendulum_ode_model()

    # set model_name
    sim.model = model

    Tf = 0.1
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    N = 200

    # set simulation time
    sim.solver_options.T = Tf
    # set options
    sim.solver_options.integrator_type = 'IRK'
    sim.solver_options.num_stages = 3
    sim.solver_options.num_steps = 3
    sim.solver_options.newton_iter = 3  # for implicit integrator
    sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"

    # create
    acados_integrator = AcadosSimSolver(sim)

    simX = np.zeros((N + 1, nx))
    x0 = np.array([0.0, np.pi + 1, 0.0, 0.0])
    u0 = np.array([0.0])
    acados_integrator.set("u", u0)

    simX[0, :] = x0

    for i in range(N):
        # set initial state
        acados_integrator.set("x", simX[i, :])
        # initialize IRK
        if sim.solver_options.integrator_type == 'IRK':
            acados_integrator.set("xdot", np.zeros((nx,)))

        # solve
        status = acados_integrator.solve()
        # get solution
        simX[i + 1, :] = acados_integrator.get("x")

        if status != 0:
            raise Exception(f'acados returned status {status}.')

    S_forw = acados_integrator.get("S_forw")
    print("S_forw, sensitivities of simulation result wrt x,u:\n", S_forw)

    # plot results
    plot_pendulum(np.linspace(0, N * Tf, N + 1), 10, np.repeat(u0, N), simX, latexify=False)


if __name__ == "__main__":
    main()
