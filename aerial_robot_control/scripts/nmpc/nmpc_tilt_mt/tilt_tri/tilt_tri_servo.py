#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
import casadi as ca
from acados_template import AcadosOcpSolver
import os
from ..tilt_qd.qd_nmpc_base import QDNMPCBase
from .tri_reference_generator import TriNMPCReferenceGenerator
from . import phys_param_trirotor as phys_tri


class NMPCTiltTriServo(QDNMPCBase):
    """
    NMPC controller for tiltable trirotor (3 rotors).
    Inherits from QDNMPCBase to share common multirotor implementation.
    """

    def __init__(self, build: bool = True):
        # Model name
        self.model_name = "tilt_tri_servo_mdl"

        # Set number of rotors for trirotor configuration
        self.num_rotors = 3

        # Controller configuration flags
        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = False
        self.include_thrust_model = False
        self.include_cog_dist_model = False
        self.include_cog_dist_parameter = False
        self.include_impedance = False

        # Load robot specific parameters
        self.phys = phys_tri

        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "nmpc", "gimbalrotor", "TiltTriRotorNMPC.yaml")

        # Call parent class constructor (will create acados model & solver and generate c code)
        super().__init__(build)

    def get_cost_function(self, **kwargs):
        """
        Define the cost function for the trirotor NMPC.
        This method is required by the base class.

        Returns:
            state_y: State output for cost function
            state_y_e: Terminal state output for cost function
            control_y: Control output for cost function
        """
        # Cost function
        # error = y - y_ref
        # NONLINEAR_LS = error^T @ Q @ error
        # qe = qr^* multiply q
        qe_x = self.qwr * self.qx - self.qw * self.qxr - self.qyr * self.qz + self.qy * self.qzr
        qe_y = self.qwr * self.qy - self.qw * self.qyr + self.qxr * self.qz - self.qx * self.qzr
        qe_z = -self.qxr * self.qy + self.qx * self.qyr + self.qwr * self.qz - self.qw * self.qzr

        state_y = ca.vertcat(
            self.p, self.v, self.qwr, qe_x + self.qxr, qe_y + self.qyr, qe_z + self.qzr, self.w, self.a_s
        )
        state_y_e = state_y
        control_y = ca.vertcat(self.ft_c, (self.a_c - self.a_s))  # ac_ref must be zero!

        return state_y, state_y_e, control_y

    def get_weights(self):
        # Define weights for trirotor (3 rotors, 3 servo angles)
        # State weights: p(3), v(3), qw_ref(1), qe(3), w(3), a(3)
        Q = np.diag(
            [
                self.params["Qp_xy"],  # px
                self.params["Qp_xy"],  # py
                self.params["Qp_z"],  # pz
                self.params["Qv_xy"],  # vx
                self.params["Qv_xy"],  # vy
                self.params["Qv_z"],  # vz
                0,  # qw_ref
                self.params["Qq_xy"],  # qe_x
                self.params["Qq_xy"],  # qe_y
                self.params["Qq_z"],  # qe_z
                self.params["Qw_xy"],  # wx
                self.params["Qw_xy"],  # wy
                self.params["Qw_z"],  # wz
                self.params["Qa"],  # a1
                self.params["Qa"],  # a2
                self.params["Qa"],  # a3
            ]
        )
        print("Q: \n", Q)

        # Control weights: ft(3), a_c_derivative(3)
        R = np.diag(
            [
                self.params["Rt"],  # ft1
                self.params["Rt"],  # ft2
                self.params["Rt"],  # ft3
                self.params["Rac_d"],  # a1c derivative
                self.params["Rac_d"],  # a2c derivative
                self.params["Rac_d"],  # a3c derivative
            ]
        )
        print("R: \n", R)

        return Q, R

    def get_reference(self, target_xyz, target_qwxyz, ft_ref, a_ref):
        """
        Assemble reference trajectory from target pose and reference control values.
        Gets called from reference generator class.
        Note: The definition of the reference is closely linked to the definition of the cost function.
        Therefore, this is explicitly stated in each controller file to increase comprehensiveness.

        :param target_xyz: Target position
        :param target_qwxy: Target quarternions
        :param ft_ref: Target thrust
        :param a_ref: Target servo angles
        :return xr: Reference for the state x
        :return ur: Reference for the input u
        """
        # Get dimensions
        ocp = self.get_ocp()
        nn = ocp.solver_options.N_horizon
        nx = ocp.dims.nx
        nu = ocp.dims.nu

        # Assemble state reference
        xr = np.zeros([nn + 1, nx])
        xr[:, 0] = target_xyz[0]  # x
        xr[:, 1] = target_xyz[1]  # y
        xr[:, 2] = target_xyz[2]  # z
        # No reference for vx, vy, vz (idx: 3, 4, 5)
        xr[:, 6] = target_qwxyz[0]  # qx
        xr[:, 7] = target_qwxyz[1]  # qx
        xr[:, 8] = target_qwxyz[2]  # qy
        xr[:, 9] = target_qwxyz[3]  # qz
        # No reference for wx, wy, wz (idx: 10, 11, 12)
        xr[:, 13] = a_ref[0]
        xr[:, 14] = a_ref[1]
        xr[:, 15] = a_ref[2]

        # Assemble input reference
        # Note: Reference has to be zero if variable is included as state in cost function!
        ur = np.zeros([nn, nu])
        ur[:, 0] = ft_ref[0]
        ur[:, 1] = ft_ref[1]
        ur[:, 2] = ft_ref[2]

        return xr, ur

    def _create_reference_generator(self) -> TriNMPCReferenceGenerator:
        """
        Create the reference generator for trirotor.
        Overrides the base class method to use TriNMPCReferenceGenerator instead of QDNMPCReferenceGenerator.
        """
        # fmt: off
        # Pass the model's and robot's properties to the reference generator
        return TriNMPCReferenceGenerator(self,
                                         self.phys.p1_b, self.phys.p2_b, self.phys.p3_b,
                                         self.phys.dr1, self.phys.dr2, self.phys.dr3,
                                         self.phys.kq_d_kt, self.phys.mass, self.phys.gravity)
        # fmt: on


if __name__ == "__main__":
    print("Please run the gen_nmpc_code.py in the nmpc folder to generate the code for this controller.")
