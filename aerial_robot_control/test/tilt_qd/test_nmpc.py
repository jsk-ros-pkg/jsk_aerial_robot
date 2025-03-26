import unittest
import numpy as np
import sys
import os

sys.path.append('../../scripts/nmpc/tilt_qd/')
from sim_nmpc_tilt_qd import simulate

current_path = os.path.abspath(os.path.dirname(__file__))


def print_error(x, x_true, u, u_true):
    print(f"max error: {np.max(np.abs(x - x_true))}, {np.max(np.abs(u - u_true))}")
    print(f"mean error: {np.mean(np.abs(x - x_true))}, {np.mean(np.abs(u - u_true))}")
    print(f"accumulated error: {np.sum(np.abs(x - x_true))}, {np.sum(np.abs(u - u_true))}")


def run_simulation_test(npz_filename, nmpc_model_id, sim_model_id, atol, plot_type=1, no_viz=True):
    """
    Run simulation and compare against reference data.

    Args:
      npz_filename (str): Path to the npz file containing reference 'x' and 'u' data.
      nmpc_model_id (int): Model id for the NMPC controller.
      sim_model_id (int): Model id for the simulator.
      atol (float): Absolute tolerance for np.allclose.
      plot_type (int): Plot type to use in simulation.
      no_viz (bool): Disable visualization if True.

    Raises:
      AssertionError: If the simulated data does not match the reference data within tolerance.
    """
    os.chdir(current_path)
    npzfile = np.load(npz_filename)
    x_true, u_true = npzfile['x'], npzfile['u']
    x, u = simulate(nmpc_model_id=nmpc_model_id,
                    sim_model_id=sim_model_id,
                    plot_type=plot_type,
                    no_viz=no_viz)
    print_error(x, x_true, u, u_true)
    assert np.allclose(x, x_true, atol=atol) and np.allclose(u, u_true, atol=atol), "Test failed!"


class TestTiltQd(unittest.TestCase):
    def test_tilt_qd_no_servo(self):
        run_simulation_test(
            npz_filename='nmpc_NMPCTiltQdNoServo_model_NMPCTiltQdServoThrust.npz',
            nmpc_model_id=0,
            sim_model_id=0,
            atol=1e-1,
            plot_type=1,
            no_viz=True
        )

    def test_tilt_qd_servo(self):
        run_simulation_test(
            npz_filename='nmpc_NMPCTiltQdServo_model_NMPCTiltQdServoThrust.npz',
            nmpc_model_id=1,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True
        )

    def test_tilt_qd_thrust(self):
        run_simulation_test(
            npz_filename='nmpc_NMPCTiltQdThrust_model_NMPCTiltQdServoThrust.npz',
            nmpc_model_id=2,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True
        )

    def test_tilt_qd_servo_thrust(self):
        run_simulation_test(
            npz_filename='nmpc_NMPCTiltQdServoThrust_model_NMPCTiltQdServoThrust.npz',
            nmpc_model_id=3,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True
        )

    def test_tilt_qd_servo_dist(self):
        run_simulation_test(
            npz_filename='nmpc_NMPCTiltQdServoDist_model_NMPCTiltQdServoThrust.npz',
            nmpc_model_id=21,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True
        )

    def test_tilt_qd_servo_thrust_dist(self):
        run_simulation_test(
            npz_filename='nmpc_NMPCTiltQdServoThrustDist_model_NMPCTiltQdServoThrust.npz',
            nmpc_model_id=22,
            sim_model_id=0,
            atol=1e-3,
            plot_type=1,
            no_viz=True
        )

    def test_tilt_qd_servo_thrust_drag_sim_model(self):
        run_simulation_test(
            npz_filename='nmpc_NMPCTiltQdServo_model_NMPCTiltQdServoThrustDrag.npz',
            nmpc_model_id=1,
            sim_model_id=1,
            atol=1e-3,
            plot_type=1,
            no_viz=True
        )


if __name__ == '__main__':
    unittest.main()
