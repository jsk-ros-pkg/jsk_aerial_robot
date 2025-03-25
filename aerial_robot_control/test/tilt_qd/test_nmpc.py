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


class TestTiltQd(unittest.TestCase):
    def test_tilt_qd_no_servo(self):
        os.chdir(current_path)

        npzfile = np.load('nmpc_NMPCTiltQdNoServo_model_NMPCTiltQdServoThrust.npz')
        x_true, u_true = npzfile['x'], npzfile['u']
        x, u = simulate(nmpc_model_id=0, sim_model_id=0, plot_type=1, no_viz=True)  # tilt_qd_no_servo

        print_error(x, x_true, u, u_true)
        self.assertTrue(np.allclose(x, x_true, atol=1e-1) and np.allclose(u, u_true, atol=1e-1))

    def test_tilt_qd_servo(self):
        os.chdir(current_path)

        npzfile = np.load('nmpc_NMPCTiltQdServo_model_NMPCTiltQdServoThrust.npz')
        x_true, u_true = npzfile['x'], npzfile['u']
        x, u = simulate(nmpc_model_id=1, sim_model_id=0, plot_type=1, no_viz=True)  # tilt_qd_servo

        print_error(x, x_true, u, u_true)
        self.assertTrue(np.allclose(x, x_true, atol=1e-3) and np.allclose(u, u_true, atol=1e-3))


if __name__ == '__main__':
    unittest.main()
