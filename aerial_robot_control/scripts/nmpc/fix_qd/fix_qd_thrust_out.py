#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
import casadi as ca

try:
    # For relative import in module
    from ..tilt_qd.qd_nmpc_base import QDNMPCBase
    from . import phys_param_mini_qd as phys_mini_qd
except ImportError:
    # For relative import in script
    import os, sys
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))    # For import from sibling directory
    from tilt_qd.qd_nmpc_base import QDNMPCBase
    import phys_param_mini_qd as phys_mini_qd


class NMPCFixQdThrustOut(QDNMPCBase):
    """
    Controller Name: Fixed Quadrotor NMPC with Thrust Output
    This NMPC controller is used for a fixed-quadrotor (meaning the rotors are fixed to the body frame).
    The output of the controller is the thrust for each rotor.
    
    :param bool overwrite: Flag to overwrite existing c generated code for the OCP solver. Default: False
    """
    def __init__(self, overwrite: bool = False, phys=phys_mini_qd):
        # Model name
        self.model_name = "fix_qd_thrust_out_mdl"
        self.phys = phys

        self.tilt = False
        self.include_servo_model = False
        self.include_servo_derivative = False
        self.include_thrust_model = False   # TODO extend to include_thrust_derivative
        self.include_cog_dist_model = False
        self.include_cog_dist_parameter = False
        self.include_impedance = False
        self.include_a_prev = False

        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "nmpc", "mini_quadrotor", "FlightControlNMPCFullModel.yaml")

        # Create acados model & solver and generate c code
        super().__init__(overwrite)

    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None):
        # Cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp_cost.AcadosOcpCost for details
        # error = y - y_ref
        # NONLINEAR_LS = error^T @ Q @ error
        # qe = qr^* multiply q
        qe_x = self.qwr * self.qx - self.qw * self.qxr - self.qyr * self.qz + self.qy * self.qzr
        qe_y = self.qwr * self.qy - self.qw * self.qyr + self.qxr * self.qz - self.qx * self.qzr
        qe_z = -self.qxr * self.qy + self.qx * self.qyr + self.qwr * self.qz - self.qw * self.qzr

        state_y = ca.vertcat(
            self.p,
            self.v,
            self.qwr,
            qe_x + self.qxr,
            qe_y + self.qyr,
            qe_z + self.qzr,
            self.w,
        )

        state_y_e = state_y

        control_y = self.ft_c

        return state_y, state_y_e, control_y

    def get_weights(self):
        # Define weights
        Q = np.diag(
            [
                self.params["Qp_xy"],
                self.params["Qp_xy"],
                self.params["Qp_z"],
                self.params["Qv_xy"],
                self.params["Qv_xy"],
                self.params["Qv_z"],
                0,
                self.params["Qq_xy"],
                self.params["Qq_xy"],
                self.params["Qq_z"],
                self.params["Qw_xy"],
                self.params["Qw_xy"],
                self.params["Qw_z"],
            ]
        )

        R = np.diag(
            [
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"]
            ]
        )

        return Q,R

    def get_reference(self, target_xyz, target_qwxyz, ft_ref, a_ref):
        """
        Assemble reference trajectory from target pose and reference control values.
        Gets called from reference generator class.
        Note: The definition of the reference is closely linked to the definition of the cost function.
        Therefore, this is explicitly stated in each controller file to increase comprehensiveness.

        :param target_xyz: Target position
        :param target_qwxy: Target quarternions
        :param ft_ref: Target thrust
        :param a_ref: Target servo angles (not needed here)
        :return xr: Reference for the state x
        :return ur: Reference for the input u
        """
        # Get dimensions
        ocp = self.get_ocp(); nn = ocp.dims.N
        nx = ocp.dims.nx; nu = ocp.dims.nu

        # Assemble state reference
        xr = np.zeros([nn + 1, nx])
        xr[:, 0] = target_xyz[0]       # x
        xr[:, 1] = target_xyz[1]       # y
        xr[:, 2] = target_xyz[2]       # z
        # No reference for vx, vy, vz (idx: 3, 4, 5)
        xr[:, 6] = target_qwxyz[0]     # qx
        xr[:, 7] = target_qwxyz[1]     # qx
        xr[:, 8] = target_qwxyz[2]     # qy
        xr[:, 9] = target_qwxyz[3]     # qz
        # No reference for wx, wy, wz (idx: 10, 11, 12)

        # Assemble control reference
        # Note: Reference has to be zero if variable is included as state in cost function!
        ur = np.zeros([nn, nu])
        ur[:, 0] = ft_ref[0]
        ur[:, 1] = ft_ref[1]
        ur[:, 2] = ft_ref[2]
        ur[:, 3] = ft_ref[3]

        return xr, ur


if __name__ == "__main__":
    # Call controller class to generate c code
    overwrite = False
    nmpc = NMPCFixQdThrustOut(overwrite)

    print("Successfully initialized acados OCP solver: ", nmpc.get_ocp_solver())
    print("T_samp: ", nmpc.params["T_samp"])
    print("T_horizon: ", nmpc.params["T_horizon"])
    print("T_step: ", nmpc.params["T_step"])
    print("N_steps: ", nmpc.params["N_steps"])
