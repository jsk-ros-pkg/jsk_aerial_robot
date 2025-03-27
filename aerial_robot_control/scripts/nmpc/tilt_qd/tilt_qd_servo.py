#!/usr/bin/env python
# -*- encoding: ascii -*-
import numpy as np
import casadi as ca
from qd_nmpc_base import QDNMPCBase


class NMPCTiltQdServo(QDNMPCBase):
    """
    Controller Name: Tiltable Quadrotor NMPC including Servo Model
    The controller itself is constructed in base class. This file is used to define the properties
    of the controller, specifically, the weights and cost function for the acados solver.
    The output of the controller is the thrust and servo angle command for each rotor.
    
    :param bool overwrite: Flag to overwrite existing c generated code for the OCP solver. Default: False
    """
    def __init__(self, overwrite: bool = False):
        # Model name
        model_name = "tilt_qd_servo_mdl"

        # ====== Define controller setup through flags ======
        #
        # - tilt: Flag to include tiltable rotors and their effect on the rotation matrix.
        # - include_servo_model: Flag to include the servo model based on the angle alpha (a) between frame E (end of arm) and R (rotor). If not included, angle control is assumed to be equal to angle state.
        # - include_servo_derivative: Flag to include the continuous time-derivative of the servo angle as control input(!) instead of numeric differentation.
        # - include_thrust_model: Flag to include dynamics from rotor and use thrust as state. If not included, thrust control is assumed to be equal to thrust state.
        # - include_cog_dist_model: Flag to include disturbance on the CoG into the acados model states. Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
        # - include_cog_dist_parameter: Flag to include disturbance on the CoG into the acados model parameters. Disturbance on each rotor individually was investigated into but didn't properly work, therefore only disturbance on CoG implemented.
        # - include_impedance: Flag to include virtual mass and inertia to calculate impedance cost. Doesn't add any functionality for the model.
        # - include_a_prev: Flag to include reference value for the servo angle command in NMPCReferenceGenerator() based on command from previous timestep.

        self.tilt = True
        self.include_servo_model = True
        self.include_servo_derivative = False
        self.include_thrust_model = False   # TODO extend to include_thrust_derivative
        self.include_cog_dist_model = False
        self.include_cog_dist_parameter = False  # TODO seperation between model and parameter necessary?
        self.include_impedance = False

        # Read parameters from configuration file in the robot's package
        self.read_params("controller", "nmpc", "beetle", "BeetleNMPCFull.yaml")
        
        # Create acados model & solver and generate c code
        super().__init__(model_name, overwrite)

    def get_cost_function(self, lin_acc_w=None, ang_acc_b=None):
        # Cost function
        # see https://docs.acados.org/python_interface/#acados_template.acados_ocp_cost.AcadosOcpCost for details
        # NONLINEAR_LS = error^T @ Q @ error; error = y - y_ref
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
            self.a_s,
        )
        
        state_y_e = state_y

        control_y = ca.vertcat(
            self.ft_c,
            self.a_c - self.a_s     # a_c_ref must be zero!
        )

        return state_y, state_y_e, control_y

    def get_weights(self):
        # Define Weights
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
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
                self.params["Qa"],
            ]
        )
        print("Q: \n", Q)

        R = np.diag(
            [
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rt"],
                self.params["Rac_d"],
                self.params["Rac_d"],
                self.params["Rac_d"],
                self.params["Rac_d"],
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
        xr[:, 13] = a_ref[0]
        xr[:, 14] = a_ref[1]
        xr[:, 15] = a_ref[2]
        xr[:, 16] = a_ref[3]

        # Assemble input reference
        # Note: Reference has to be zero if variable is included as state in cost function!
        ur = np.zeros([nn, nu])
        ur[:, 0] = ft_ref[0]
        ur[:, 1] = ft_ref[1]
        ur[:, 2] = ft_ref[2]
        ur[:, 3] = ft_ref[3]
        
        return xr, ur


if __name__ == "__main__":
    overwrite = True
    nmpc = NMPCTiltQdServo(overwrite)

    acados_ocp_solver = nmpc.get_ocp_solver()
    print("Successfully initialized acados ocp: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", nmpc.params["T_samp"])
    print("T_horizon: ", nmpc.params["T_horizon"])
    print("T_step: ", nmpc.params["T_step"])
    print("N_steps: ", nmpc.params["N_steps"])
