'''
 Created by jinjie on 24/06/07.
 From acados python example -> pendulum_model
'''

from acados_template import AcadosSim, AcadosSimSolver, AcadosModel, latexify_plot
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import rospkg
import os
import yaml

# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("gimbalrotor"), "config", "TiltBiRotorNMPCFull.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)

nmpc_params = param_dict["controller"]["nmpc"]
nmpc_params["N_node"] = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

physical_params = param_dict["controller"]["physical"]

# servo parameters
kps = physical_params["kps"]
kds = physical_params["kds"]
mus = physical_params["mus"]

i_sxx = physical_params["servo_inertia_x"]


def export_servo_model() -> AcadosModel:
    model_name = 'servo_ode'

    # set up states & controls
    t_servo = 0.08

    a = ca.SX.sym('a', 2)
    b = ca.SX.sym('b', 2)
    x = ca.vertcat(a, b)

    ac = ca.SX.sym('ac', 2)
    u = ca.vertcat(ac)

    # xdot
    xdot = ca.SX.sym('xdot', x.size()[0])

    # dynamics
    f_expl = ca.vertcat(b,
                        (kps * (ac - a) + kds * (0 - b) + mus * b) / i_sxx)

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model


def plot_servo(shooting_nodes, u_max, U, X_true, X_est=None, Y_measured=None, latexify=False, plt_show=True,
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


def main():
    sim = AcadosSim()

    # export model
    model = export_servo_model()

    # set model_name
    sim.model = model

    Tf = 0.001
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    N = 1000

    # set simulation time
    sim.solver_options.T = Tf
    # set options
    sim.solver_options.integrator_type = 'IRK'
    sim.solver_options.num_stages = 3
    sim.solver_options.num_steps = 3
    sim.solver_options.newton_iter = 3  # for implicit integrator
    sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"

    # # The following setting leads to unconvergence
    # sim.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    # sim.solver_options.hpipm_mode = "BALANCE"  # "BALANCE", "SPEED_ABS", "SPEED", "ROBUST". Default: "BALANCE".
    # # # 0: no warm start; 1: warm start; 2: hot start. Default: 0   Seems only works for FULL_CONDENSING_QPOASES
    # # ocp.solver_options.qp_solver_warm_start = 1
    # sim.solver_options.hessian_approx = "GAUSS_NEWTON"
    # sim.solver_options.integrator_type = "ERK"  # explicit Runge-Kutta integrator
    # sim.solver_options.print_level = 0
    # sim.solver_options.nlp_solver_type = "SQP_RTI"
    # sim.solver_options.qp_solver_cond_N = nmpc_params["N_node"]
    # sim.solver_options.tf = nmpc_params["T_pred"]

    # create
    acados_integrator = AcadosSimSolver(sim)

    simX = np.zeros((N + 1, nx))
    x0 = np.zeros((nx,))
    u0 = np.array([1.0, -1.0])
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
    plot_servo(np.linspace(0, N * Tf, N + 1), 10, np.repeat(u0, N), simX, latexify=False)


if __name__ == "__main__":
    main()
