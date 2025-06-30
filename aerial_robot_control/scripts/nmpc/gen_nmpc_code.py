#!/usr/bin/env python
# -*- encoding: ascii -*-
import argparse

from nmpc_tilt_mt.archive import *
from nmpc_tilt_mt.fix_qd import *
from nmpc_tilt_mt.mhe import *
from nmpc_tilt_mt.tilt_bi import *
from nmpc_tilt_mt.tilt_qd import *
from nmpc_tilt_mt.tilt_tri import *

if __name__ == "__main__":
    # get the name of all classes imported from *
    class_names = [cls for cls in dir() if not cls.startswith('_') and isinstance(globals()[cls], type)]
    class_names_str = ", ".join(class_names)

    debug = 0
    if debug:
        model_name = "NMPCTiltQdServoDist"
        #    NMPCFixQdAngvelOut
        #    NMPCFixQdThrustOut
        #    NMPCTiltQdNoServo
        #    NMPCTiltQdServo
        #    NMPCTiltQdServoDist
        #    NMPCTiltQdServoImpedance
        #    NMPCTiltQdServoThrustDist
        #    NMPCTiltQdServoThrustImpedance
        #    NMPCTiltTriServo
        #    NMPCTiltBiServo
        #    NMPCTiltBi2OrdServo
        #    MHEWrenchEstAccMom

    else:
        parser = argparse.ArgumentParser(description="Run NMPC with different models to generate code.")
        parser.add_argument("-m", "--model", type=str, required=True,
                            help=f"Model name to generate code for. Choose from: {class_names_str}")
        args = parser.parse_args()
        model_name = args.model

    if model_name not in class_names:
        raise ValueError(f"Model '{model_name}' not found. Available models: {class_names_str}")

    print(f"Generating code for model: {model_name}")
    nmpc_class = globals()[model_name]
    nmpc = nmpc_class()

    acados_ocp_solver = nmpc.get_ocp_solver()
    print("Successfully initialized acados OCP solver: ", acados_ocp_solver.acados_ocp)
    print("number of states: ", acados_ocp_solver.acados_ocp.dims.nx)
    print("number of controls: ", acados_ocp_solver.acados_ocp.dims.nu)
    print("number of parameters: ", acados_ocp_solver.acados_ocp.dims.np)
    print("T_samp: ", nmpc.params["T_samp"])
    print("T_horizon: ", nmpc.params["T_horizon"])
    print("T_step: ", nmpc.params["T_step"])
    print("N_steps: ", nmpc.params["N_steps"])
