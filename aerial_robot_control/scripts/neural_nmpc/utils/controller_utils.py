import numpy as np
from acados_template import AcadosOcpSolver
from neural_controller_standalone import NeuralNMPC


def check_state_constraints(ocp_solver: AcadosOcpSolver, state_curr, i):
    # Boundary constraints
    relaxation_factor = 1.2
    for idx in ocp_solver.acados_ocp.constraints.idxbx:
        lbxi = np.where(ocp_solver.acados_ocp.constraints.idxbx == idx)[0][0]
        if (
            state_curr[idx] < ocp_solver.acados_ocp.constraints.lbx[lbxi] * relaxation_factor
            or state_curr[idx] > ocp_solver.acados_ocp.constraints.ubx[lbxi] * relaxation_factor
        ):
            print(
                f"Warning: Constraint violation at index {idx} in step {i}. "
                f"Value: {state_curr[idx]:.14f}, "
                f"Lower bound: {ocp_solver.acados_ocp.constraints.lbx[lbxi]}, "
                f"Upper bound: {ocp_solver.acados_ocp.constraints.ubx[lbxi]}"
            )
    # Height constraint
    if state_curr[2] < 0.0:
        print(f"Warning: Constraint violation for height in step {i}. " f"Value: {state_curr[2]:.14f} < 0.0")
    # Nonlinear unit quaternion constraint
    quat_norm = np.linalg.norm(state_curr[6:10])
    if quat_norm < 0.999 or quat_norm > 1.001:
        print(
            f"Warning: Constraint violation for unit_q in step {i}. "
            f"Value: {quat_norm:.14f} != 1.0, "
            f"Quaternion: {state_curr[6:10]}"
        )


def check_input_constraints(mpc: NeuralNMPC, u_cmd, i):
    """
    Check if the control input command u_cmd respects the constraints of the system.
    """
    if np.any(u_cmd[:4]) < mpc.params["thrust_min"] or np.any(u_cmd[:4]) > mpc.params["thrust_max"]:
        print(f"=== Control thrust input violates constraints: {u_cmd[:4]} at index {i} ===")
        raise ValueError("Control thrust input violates constraints.")
    if u_cmd.shape[0] > 4:
        if np.any(u_cmd[4:]) < mpc.params["a_min"] or np.any(u_cmd[4:]) > mpc.params["a_max"]:
            print(f"=== Control servo angle input violates constraints: {u_cmd[4:]} at index {i} ===")
            raise ValueError("Control servo angle input violates constraints.")


def get_rotor_positions(mpc: NeuralNMPC) -> np.ndarray:
    """
    Get the positions of the rotors in the body frame.
    :return: 3x4 matrix containing the positions of the rotors in the body frame.
    """
    if mpc.arch_type == "tilt_bi":
        return np.array([mpc.phys.p1_b, mpc.phys.p2_b])
    elif mpc.arch_type == "tilt_tri":
        return np.array([mpc.phys.p1_b, mpc.phys.p2_b, mpc.phys.p3_b])
    elif mpc.arch_type == "tilt_qd" or mpc.arch_type == "fix_qd":
        return np.array([mpc.phys.p1_b, mpc.phys.p2_b, mpc.phys.p3_b, mpc.phys.p4_b])
    else:
        raise ValueError(f"Architecture type not supported: {mpc.arch_type}")
